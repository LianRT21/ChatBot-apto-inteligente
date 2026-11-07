#ifndef _SMART_APARTMENT_CONTROLLER_H_
#define _SMART_APARTMENT_CONTROLLER_H_

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <cJSON.h>

#include "smart_apartment_config.h"

enum class ApartmentZone {
    Cocina,
    Sala,
    Habitacion,
    Bano,
    Terraza,
    All
};

enum class ApartmentDoor {
    Principal,
    Habitacion,
    Bano,
    Terraza,
    All
};

struct LightState {
    gpio_num_t pin;
    bool is_on;
};

struct DoorState {
    gpio_num_t pin;
    ledc_channel_t channel;
    bool is_open;
    int current_angle;
};

class SmartApartmentController {
public:
    static SmartApartmentController& GetInstance() {
        static SmartApartmentController instance;
        return instance;
    }

    // Delete copy constructor and assignment operator
    SmartApartmentController(const SmartApartmentController&) = delete;
    SmartApartmentController& operator=(const SmartApartmentController&) = delete;

    /**
     * Inicializa los GPIOs, los canales PWM LEDC y registra las herramientas MCP.
     */
    void Initialize();

    /**
     * Control de luces individuales o generales.
     * @param zone Nombre de la zona ("cocina", "sala", "habitacion", "baño", "terraza", "todas")
     * @param state true para encender, false para apagar
     */
    bool SetLight(const std::string& zone, bool state);
    bool SetAllLights(bool state);

    /**
     * Control de puertas con servomotor 180°.
     * @param door Nombre de la puerta ("principal", "habitacion", "baño", "terraza", "todas")
     * @param open true para abrir, false para cerrar
     * @param custom_angle Ángulo específico en grados (0 a 180) o -1 para usar el valor por defecto
     */
    bool SetDoor(const std::string& door, bool open, int custom_angle = -1);
    bool SetAllDoors(bool open, int custom_angle = -1);

    /**
     * Obtiene el estado actual de todas las luces y puertas en formato JSON.
     * Quien llama debe liberar el cJSON retornado (cJSON_Delete o CJsonUniquePtr).
     */
    cJSON* GetStatusJson();

private:
    SmartApartmentController();
    ~SmartApartmentController() = default;

    void SetupLights();
    void SetupServos();
    void RegisterMcpTools();

    void ApplyLightHardware(gpio_num_t pin, bool state);
    void ApplyServoAngle(ledc_channel_t channel, int angle);

    static std::string CleanString(const std::string& str);
    static bool ParseAction(const std::string& action, bool& out_state);

    std::mutex mutex_;
    bool initialized_ = false;

    // Estados de luces
    LightState light_cocina_;
    LightState light_sala_;
    LightState light_habitacion_;
    LightState light_bano_;
    LightState light_terraza_;

    // Estados de puertas (servos)
    DoorState door_principal_;
    DoorState door_habitacion_;
    DoorState door_bano_;
    DoorState door_terraza_;
};

#endif // _SMART_APARTMENT_CONTROLLER_H_

