#include "smart_apartment_controller.h"
#include "mcp_server.h"
#include "cjson_utils.h"
#include "board.h"
#include "display.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>
#include <cstring>
#include <cstdio>

#define TAG "SmartApartment"

SmartApartmentController::SmartApartmentController()
    : light_cocina_{APARTMENT_LIGHT_COCINA_GPIO, false},
      light_sala_{APARTMENT_LIGHT_SALA_GPIO, false},
      light_habitacion_{APARTMENT_LIGHT_HABITACION_GPIO, false},
      light_bano_{APARTMENT_LIGHT_BANO_GPIO, false},
      light_terraza_{APARTMENT_LIGHT_TERRAZA_GPIO, false},
      door_principal_{APARTMENT_SERVO_PRINCIPAL_GPIO, LEDC_CHANNEL_0, false, APARTMENT_DOOR_CLOSED_ANGLE},
      door_habitacion_{APARTMENT_SERVO_HABITACION_GPIO, LEDC_CHANNEL_1, false, APARTMENT_DOOR_CLOSED_ANGLE},
      door_bano_{APARTMENT_SERVO_BANO_GPIO, LEDC_CHANNEL_2, false, APARTMENT_DOOR_CLOSED_ANGLE},
      door_terraza_{APARTMENT_SERVO_TERRAZA_GPIO, LEDC_CHANNEL_3, false, APARTMENT_DOOR_CLOSED_ANGLE} {}

void SmartApartmentController::Initialize() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_) {
        return;
    }

    ESP_LOGI(TAG, "Inicializando controlador de Apartamento Inteligente (Maqueta)...");
    SetupLights();
    SetupServos();
    RegisterMcpTools();

    initialized_ = true;
    ESP_LOGI(TAG, "Apartamento Inteligente inicializado correctamente con MCP.");
}

void SmartApartmentController::SetupLights() {
    const gpio_num_t pins[] = {
        light_cocina_.pin,
        light_sala_.pin,
        light_habitacion_.pin,
        light_bano_.pin,
        light_terraza_.pin
    };

    for (gpio_num_t pin : pins) {
        if (pin == GPIO_NUM_NC) {
            continue;
        }

        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        ApplyLightHardware(pin, false); // Apagado por defecto
    }
}

void SmartApartmentController::SetupServos() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = APARTMENT_SERVO_SPEED_MODE,
        .duty_resolution = APARTMENT_SERVO_RESOLUTION,
        .timer_num = APARTMENT_SERVO_TIMER,
        .freq_hz = APARTMENT_SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar timer LEDC para servos: %s", esp_err_to_name(err));
        return;
    }

    const DoorState* doors[] = {
        &door_principal_,
        &door_habitacion_,
        &door_bano_,
        &door_terraza_
    };

    for (const DoorState* door : doors) {
        if (door->pin == GPIO_NUM_NC) {
            continue;
        }

        ledc_channel_config_t channel_conf = {
            .gpio_num = door->pin,
            .speed_mode = APARTMENT_SERVO_SPEED_MODE,
            .channel = door->channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = APARTMENT_SERVO_TIMER,
            .duty = 0,
            .hpoint = 0
        };
        ledc_channel_config(&channel_conf);
        ApplyServoAngle(door->channel, APARTMENT_DOOR_CLOSED_ANGLE);
    }
}

void SmartApartmentController::ApplyLightHardware(gpio_num_t pin, bool state) {
    if (pin == GPIO_NUM_NC) {
        return;
    }
    int level = state ? (APARTMENT_LIGHTS_ACTIVE_HIGH ? 1 : 0)
                      : (APARTMENT_LIGHTS_ACTIVE_HIGH ? 0 : 1);
    gpio_set_level(pin, level);
}

void SmartApartmentController::ApplyServoAngle(ledc_channel_t channel, int angle) {
    angle = std::max(0, std::min(180, angle));
    float pulse_ms = APARTMENT_SERVO_MIN_PULSE_MS +
                     ((float)angle / 180.0f) * (APARTMENT_SERVO_MAX_PULSE_MS - APARTMENT_SERVO_MIN_PULSE_MS);

    // Con resolución de 13 bits (8191) a 50Hz (período de 20.0 ms):
    uint32_t duty = (uint32_t)((pulse_ms * 8191.0f) / 20.0f);

    ledc_set_duty(APARTMENT_SERVO_SPEED_MODE, channel, duty);
    ledc_update_duty(APARTMENT_SERVO_SPEED_MODE, channel);
}

std::string SmartApartmentController::CleanString(const std::string& str) {
    std::string result;
    result.reserve(str.size());

    for (size_t i = 0; i < str.size(); ++i) {
        unsigned char c = static_cast<unsigned char>(str[i]);
        // Manejar caracteres especiales UTF-8 frecuentes en español
        if (c == 0xC3 && i + 1 < str.size()) {
            unsigned char next = static_cast<unsigned char>(str[i + 1]);
            if (next == 0xA1 || next == 0x81) { result += 'a'; i++; continue; } // á, Á
            if (next == 0xA9 || next == 0x89) { result += 'e'; i++; continue; } // é, É
            if (next == 0xAD || next == 0x8D) { result += 'i'; i++; continue; } // í, Í
            if (next == 0xB3 || next == 0x93) { result += 'o'; i++; continue; } // ó, Ó
            if (next == 0xBA || next == 0x9A) { result += 'u'; i++; continue; } // ú, Ú
            if (next == 0xB1 || next == 0x91) { result += 'n'; i++; continue; } // ñ, Ñ
        }

        if (c == ' ' || c == '_' || c == '-' || c == '.') {
            continue; // Ignorar separadores para comparación flexible
        }
        result += static_cast<char>(std::tolower(c));
    }
    return result;
}

bool SmartApartmentController::ParseAction(const std::string& raw_action, bool& out_state) {
    std::string action = CleanString(raw_action);
    if (action == "on" || action == "encender" || action == "prender" || action == "activar" ||
        action == "open" || action == "abrir" || action == "true" || action == "1" ||
        action == "subir" || action == "enciende" || action == "prende" || action == "abre") {
        out_state = true;
        return true;
    }
    if (action == "off" || action == "apagar" || action == "desactivar" || action == "cerrar" ||
        action == "close" || action == "false" || action == "0" || action == "bajar" ||
        action == "apaga" || action == "cierra") {
        out_state = false;
        return true;
    }
    return false;
}

bool SmartApartmentController::SetLight(const std::string& raw_zone, bool state) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string zone = CleanString(raw_zone);

    if (zone == "todas" || zone == "todos" || zone == "todo" || zone == "toda" ||
        zone == "todaslasluces" || zone == "apartamento" || zone == "apto" ||
        zone == "casa" || zone == "all") {
        return SetAllLights(state);
    }

    bool matched = false;
    std::string zone_display_name;

    if (zone == "cocina" || zone == "kitchen" || zone == "cocineta") {
        light_cocina_.is_on = state;
        ApplyLightHardware(light_cocina_.pin, state);
        zone_display_name = "Cocina";
        matched = true;
    } else if (zone == "sala" || zone == "living" || zone == "livingroom" || zone == "salon" ||
               zone == "comedor" || zone == "estancia" || zone == "greatroom") {
        light_sala_.is_on = state;
        ApplyLightHardware(light_sala_.pin, state);
        zone_display_name = "Sala";
        matched = true;
    } else if (zone == "habitacion" || zone == "habitaciones" || zone == "cuarto" ||
               zone == "cuartos" || zone == "dormitorio" || zone == "dormitorios" ||
               zone == "alcoba" || zone == "bedroom") {
        light_habitacion_.is_on = state;
        ApplyLightHardware(light_habitacion_.pin, state);
        zone_display_name = "Habitación";
        matched = true;
    } else if (zone == "bano" || zone == "toilet" || zone == "sanitario" || zone == "bathroom") {
        light_bano_.is_on = state;
        ApplyLightHardware(light_bano_.pin, state);
        zone_display_name = "Baño";
        matched = true;
    } else if (zone == "terraza" || zone == "balcon" || zone == "patio" || zone == "deck" || zone == "terrace") {
        light_terraza_.is_on = state;
        ApplyLightHardware(light_terraza_.pin, state);
        zone_display_name = "Terraza";
        matched = true;
    }

    if (matched) {
        ESP_LOGI(TAG, "Luz de %s: %s", zone_display_name.c_str(), state ? "ENCENDIDA" : "APAGADA");
        auto display = Board::GetInstance().GetDisplay();
        if (display != nullptr) {
            char buf[64];
            snprintf(buf, sizeof(buf), "Luz %s: %s", zone_display_name.c_str(), state ? "ON" : "OFF");
            display->ShowNotification(buf, 2000);
        }
        return true;
    }

    ESP_LOGW(TAG, "Zona desconocida para luz: %s", raw_zone.c_str());
    return false;
}

bool SmartApartmentController::SetAllLights(bool state) {
    light_cocina_.is_on = state;
    light_sala_.is_on = state;
    light_habitacion_.is_on = state;
    light_bano_.is_on = state;
    light_terraza_.is_on = state;

    ApplyLightHardware(light_cocina_.pin, state);
    ApplyLightHardware(light_sala_.pin, state);
    ApplyLightHardware(light_habitacion_.pin, state);
    ApplyLightHardware(light_bano_.pin, state);
    ApplyLightHardware(light_terraza_.pin, state);

    ESP_LOGI(TAG, "TODAS las luces han sido %s", state ? "ENCENDIDAS" : "APAGADAS");
    auto display = Board::GetInstance().GetDisplay();
    if (display != nullptr) {
        display->ShowNotification(state ? "Todas luces: ON" : "Todas luces: OFF", 2000);
    }
    return true;
}

bool SmartApartmentController::SetDoor(const std::string& raw_door, bool open, int custom_angle) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string door = CleanString(raw_door);

    int target_angle = (custom_angle >= 0 && custom_angle <= 180)
                       ? custom_angle
                       : (open ? APARTMENT_DOOR_OPEN_ANGLE : APARTMENT_DOOR_CLOSED_ANGLE);

    if (door == "todas" || door == "todos" || door == "todaslaspuertas" || door == "all") {
        return SetAllDoors(open, custom_angle);
    }

    bool matched = false;
    std::string door_display_name;

    if (door == "principal" || door == "entrada" || door == "frente" ||
        door == "puertaprincipal" || door == "puertadeentrada" || door == "acceso" ||
        door == "frontdoor" || door == "main" || door == "maindoor") {
        door_principal_.is_open = open;
        door_principal_.current_angle = target_angle;
        ApplyServoAngle(door_principal_.channel, target_angle);
        door_display_name = "Principal";
        matched = true;
    } else if (door == "habitacion" || door == "cuarto" || door == "dormitorio" ||
               door == "alcoba" || door == "bedroom") {
        door_habitacion_.is_open = open;
        door_habitacion_.current_angle = target_angle;
        ApplyServoAngle(door_habitacion_.channel, target_angle);
        door_display_name = "Habitación";
        matched = true;
    } else if (door == "bano" || door == "toilet" || door == "sanitario" || door == "bathroom") {
        door_bano_.is_open = open;
        door_bano_.current_angle = target_angle;
        ApplyServoAngle(door_bano_.channel, target_angle);
        door_display_name = "Baño";
        matched = true;
    } else if (door == "terraza" || door == "balcon" || door == "patio" || door == "terrace") {
        door_terraza_.is_open = open;
        door_terraza_.current_angle = target_angle;
        ApplyServoAngle(door_terraza_.channel, target_angle);
        door_display_name = "Terraza";
        matched = true;
    }

    if (matched) {
        ESP_LOGI(TAG, "Puerta %s: %s (ángulo: %d°)", door_display_name.c_str(), open ? "ABIERTA" : "CERRADA", target_angle);
        auto display = Board::GetInstance().GetDisplay();
        if (display != nullptr) {
            char buf[64];
            snprintf(buf, sizeof(buf), "Puerta %s: %s", door_display_name.c_str(), open ? "ABIERTA" : "CERRADA");
            display->ShowNotification(buf, 2000);
        }
        return true;
    }

    ESP_LOGW(TAG, "Puerta desconocida: %s", raw_door.c_str());
    return false;
}

bool SmartApartmentController::SetAllDoors(bool open, int custom_angle) {
    int target_angle = (custom_angle >= 0 && custom_angle <= 180)
                       ? custom_angle
                       : (open ? APARTMENT_DOOR_OPEN_ANGLE : APARTMENT_DOOR_CLOSED_ANGLE);

    door_principal_.is_open = open;
    door_principal_.current_angle = target_angle;
    ApplyServoAngle(door_principal_.channel, target_angle);

    door_habitacion_.is_open = open;
    door_habitacion_.current_angle = target_angle;
    ApplyServoAngle(door_habitacion_.channel, target_angle);

    door_bano_.is_open = open;
    door_bano_.current_angle = target_angle;
    ApplyServoAngle(door_bano_.channel, target_angle);

    door_terraza_.is_open = open;
    door_terraza_.current_angle = target_angle;
    ApplyServoAngle(door_terraza_.channel, target_angle);

    ESP_LOGI(TAG, "TODAS las puertas han sido %s (ángulo: %d°)", open ? "ABIERTAS" : "CERRADAS", target_angle);
    auto display = Board::GetInstance().GetDisplay();
    if (display != nullptr) {
        display->ShowNotification(open ? "Todas puertas: ABIERTAS" : "Todas puertas: CERRADAS", 2000);
    }
    return true;
}

cJSON* SmartApartmentController::GetStatusJson() {
    std::lock_guard<std::mutex> lock(mutex_);
    cJSON* root = cJSON_CreateObject();
    if (root == nullptr) {
        return nullptr;
    }

    // Luces
    cJSON* lights = cJSON_CreateObject();
    cJSON_AddBoolToObject(lights, "cocina", light_cocina_.is_on);
    cJSON_AddBoolToObject(lights, "sala", light_sala_.is_on);
    cJSON_AddBoolToObject(lights, "habitacion", light_habitacion_.is_on);
    cJSON_AddBoolToObject(lights, "bano", light_bano_.is_on);
    cJSON_AddBoolToObject(lights, "terraza", light_terraza_.is_on);
    cJSON_AddItemToObject(root, "luces", lights);

    // Puertas
    cJSON* doors = cJSON_CreateObject();

    auto add_door_obj = [](cJSON* parent, const char* name, const DoorState& state) {
        cJSON* d = cJSON_CreateObject();
        cJSON_AddBoolToObject(d, "abierta", state.is_open);
        cJSON_AddNumberToObject(d, "angulo", state.current_angle);
        cJSON_AddItemToObject(parent, name, d);
    };

    add_door_obj(doors, "principal", door_principal_);
    add_door_obj(doors, "habitacion", door_habitacion_);
    add_door_obj(doors, "bano", door_bano_);
    add_door_obj(doors, "terraza", door_terraza_);
    cJSON_AddItemToObject(root, "puertas", doors);

    return root;
}

void SmartApartmentController::RegisterMcpTools() {
    auto& mcp_server = McpServer::GetInstance();

    // 1. Herramienta para encender/apagar luces
    mcp_server.AddTool(
        "apartment.control_light",
        "Controla las luces o bombillos de las diferentes partes del apartamento inteligente (maqueta).\n"
        "Usa esta herramienta cuando el usuario pida encender o apagar luces.\n"
        "Zonas disponibles: 'cocina', 'sala', 'habitacion' (o dormitorio/cuarto), 'baño', 'terraza' (o balcón), o 'todas'.\n"
        "Acción: 'on' / 'encender' / true para prender; 'off' / 'apagar' / false para apagar.",
        PropertyList({
            Property("zone", kPropertyTypeString),
            Property("action", kPropertyTypeString)
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            std::string zone = properties["zone"].value<std::string>();
            std::string action = properties["action"].value<std::string>();

            bool state = false;
            if (!ParseAction(action, state)) {
                return "Error: Acción no válida. Usa 'on'/'encender' o 'off'/'apagar'.";
            }

            bool ok = SetLight(zone, state);
            if (!ok) {
                return "Error: Zona no reconocida. Opciones: cocina, sala, habitacion, baño, terraza, todas.";
            }

            std::string msg = "Luz de " + zone + " " + (state ? "encendida" : "apagada") + " con exito.";
            return msg;
        }
    );

    // 2. Herramienta para abrir/cerrar puertas con servomotores
    mcp_server.AddTool(
        "apartment.control_door",
        "Controla los servomotores de 180 grados para abrir o cerrar las puertas del apartamento inteligente.\n"
        "Usa esta herramienta cuando el usuario pida abrir o cerrar una puerta.\n"
        "Puertas disponibles: 'principal' (o entrada), 'habitacion', 'baño', 'terraza', o 'todas'.\n"
        "Acción: 'open' / 'abrir' para abrir la puerta; 'close' / 'cerrar' para cerrarla.\n"
        "Ángulo opcional: 0 a 180 grados (si no se especifica, usa 90° para abierta y 0° para cerrada).",
        PropertyList({
            Property("door", kPropertyTypeString),
            Property("action", kPropertyTypeString),
            Property("angle", kPropertyTypeInteger, -1) // Por defecto -1 indica usar ángulo automático
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            std::string door = properties["door"].value<std::string>();
            std::string action = properties["action"].value<std::string>();
            int angle = properties["angle"].value<int>();

            bool open = false;
            if (!ParseAction(action, open)) {
                return "Error: Acción no válida. Usa 'open'/'abrir' o 'close'/'cerrar'.";
            }

            bool ok = SetDoor(door, open, angle);
            if (!ok) {
                return "Error: Puerta no reconocida. Opciones: principal, habitacion, baño, terraza, todas.";
            }

            std::string msg = "Puerta " + door + " " + (open ? "abierta" : "cerrada") + " con exito.";
            return msg;
        }
    );

    // 3. Herramienta para consultar estado del apartamento
    mcp_server.AddTool(
        "apartment.get_status",
        "Obtiene el estado en tiempo real de todo el apartamento inteligente:\n"
        "Indica qué luces están encendidas o apagadas en cada zona (cocina, sala, habitación, baño, terraza)\n"
        "y el estado de las puertas (abiertas/cerradas y ángulo actual de los servomotores).\n"
        "Usa esta herramienta para responder a preguntas del usuario como: '¿cómo está el apartamento?', "
        "'¿qué luces están prendidas?', '¿la puerta de la habitación está abierta?', etc.",
        PropertyList(),
        [this](const PropertyList& properties) -> ReturnValue {
            return GetStatusJson(); // Puntero cJSON* transferido a ReturnValue
        }
    );

    // 4. Herramienta para autodiagnóstico de hardware
    mcp_server.AddTool(
        "apartment.self_test",
        "Ejecuta una prueba o diagnóstico automático de hardware en la maqueta del apartamento:\n"
        "Enciende y apaga secuencialmente cada una de las 5 luces y abre y cierra cada una de las 4 puertas.\n"
        "Usa esta herramienta cuando el usuario pida 'haz una prueba de hardware', 'revisa los componentes', "
        "'prueba las luces y puertas', 'modo diagnóstico' o similar.",
        PropertyList(),
        [this](const PropertyList& properties) -> ReturnValue {
            StartHardwareSelfTest();
            return "Diagnostico de hardware iniciado. Probando luces y servomotores secuencialmente.";
        }
    );
}

void SmartApartmentController::StartHardwareSelfTest() {
    xTaskCreate(SelfTestTask, "apt_self_test", 4096, this, 5, nullptr);
}

void SmartApartmentController::SelfTestTask(void* arg) {
    auto* self = static_cast<SmartApartmentController*>(arg);
    ESP_LOGI(TAG, "Iniciando diagnóstico secuencial de hardware...");

    const std::string rooms[] = {"cocina", "sala", "habitacion", "bano", "terraza"};
    const std::string doors[] = {"principal", "habitacion", "bano", "terraza"};

    // 1. Probar cada luz individualmente
    for (const auto& room : rooms) {
        self->SetLight(room, true);
        vTaskDelay(pdMS_TO_TICKS(400));
        self->SetLight(room, false);
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    // 2. Probar cada puerta individualmente (abrir a 90° y cerrar a 0°)
    for (const auto& door : doors) {
        self->SetDoor(door, true, 90);
        vTaskDelay(pdMS_TO_TICKS(700));
        self->SetDoor(door, false, 0);
        vTaskDelay(pdMS_TO_TICKS(400));
    }

    // 3. Encender todas las luces y luego apagarlas como señal de finalización
    self->SetAllLights(true);
    vTaskDelay(pdMS_TO_TICKS(500));
    self->SetAllLights(false);

    ESP_LOGI(TAG, "Diagnóstico secuencial de hardware finalizado.");
    vTaskDelete(nullptr);
}

