#ifndef _SMART_APARTMENT_CONFIG_H_
#define _SMART_APARTMENT_CONFIG_H_

#include <driver/gpio.h>
#include <driver/ledc.h>

/**
 * ============================================================================
 * CONFIGURACIÓN DE PINES Y PARÁMETROS DEL APARTAMENTO INTELIGENTE (MAQUETA)
 * ============================================================================
 * 
 * Zonas soportadas:
 *  - Cocina
 *  - Sala
 *  - Habitación
 *  - Baño
 *  - Terraza
 * 
 * Puedes cambiar los números de GPIO a continuación según el cableado de tu maqueta.
 * Si alguna luz o servo no está conectado, asígnale GPIO_NUM_NC (-1).
 */

#if CONFIG_IDF_TARGET_ESP32S3
// Pines por defecto para ESP32-S3
#define APARTMENT_LIGHT_COCINA_GPIO     GPIO_NUM_10
#define APARTMENT_LIGHT_SALA_GPIO       GPIO_NUM_11
#define APARTMENT_LIGHT_HABITACION_GPIO GPIO_NUM_12
#define APARTMENT_LIGHT_BANO_GPIO       GPIO_NUM_13
#define APARTMENT_LIGHT_TERRAZA_GPIO    GPIO_NUM_14

#define APARTMENT_SERVO_PRINCIPAL_GPIO  GPIO_NUM_17
#define APARTMENT_SERVO_HABITACION_GPIO GPIO_NUM_18
#define APARTMENT_SERVO_BANO_GPIO       GPIO_NUM_21
#define APARTMENT_SERVO_TERRAZA_GPIO    GPIO_NUM_38

#else
// Pines por defecto para ESP32 estándar (ESP32 DevKit v1 30 pines)
#define APARTMENT_LIGHT_COCINA_GPIO     GPIO_NUM_18  // D18 (el led que ya tienes en el diagrama)
#define APARTMENT_LIGHT_SALA_GPIO       GPIO_NUM_19  // D19
#define APARTMENT_LIGHT_HABITACION_GPIO GPIO_NUM_21  // D21
#define APARTMENT_LIGHT_BANO_GPIO       GPIO_NUM_22  // D22
#define APARTMENT_LIGHT_TERRAZA_GPIO    GPIO_NUM_23  // D23

#define APARTMENT_SERVO_PRINCIPAL_GPIO  GPIO_NUM_13  // D13
#define APARTMENT_SERVO_HABITACION_GPIO GPIO_NUM_16  // RX2
#define APARTMENT_SERVO_BANO_GPIO       GPIO_NUM_17  // TX2
#define APARTMENT_SERVO_TERRAZA_GPIO    GPIO_NUM_5   // D5
#endif

// Lógica de relés / LEDs para las luces:
// true  = Activo en ALTO (3.3V enciende el LED / bombillo)
// false = Activo en BAJO (0V enciende, típico en módulos relé optoacoplados)
#define APARTMENT_LIGHTS_ACTIVE_HIGH    true

// Parámetros de servomotores de 180° (SG90, MG90S, etc.)
#define APARTMENT_SERVO_TIMER           LEDC_TIMER_0
#define APARTMENT_SERVO_SPEED_MODE      LEDC_LOW_SPEED_MODE
#define APARTMENT_SERVO_FREQ_HZ         50
#define APARTMENT_SERVO_RESOLUTION      LEDC_TIMER_13_BIT  // 0 a 8191

// Ángulos de apertura y cierre por defecto (en grados)
#define APARTMENT_DOOR_CLOSED_ANGLE     0   // Puerta cerrada
#define APARTMENT_DOOR_OPEN_ANGLE       90  // Puerta abierta a 90° (puedes cambiar a 180 si tu bisagra lo requiere)

// Rango de pulsos PWM para servos (en milisegundos)
#define APARTMENT_SERVO_MIN_PULSE_MS    0.5f  // Corresponde a 0° (~500 µs)
#define APARTMENT_SERVO_MAX_PULSE_MS    2.5f  // Corresponde a 180° (~2500 µs)

#endif // _SMART_APARTMENT_CONFIG_H_

