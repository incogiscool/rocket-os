#include <stdint.h>

/* Telemetry packet structure - packed so that we can decode the struct on ground station */
typedef struct __attribute__((packed)) {
    uint8_t  device_id;     /* Unique ID for this device - let's keep this const in a config */
    uint16_t seq;
    uint32_t timestamp_ms;
    int32_t  latitude;
    int32_t  longitude;
    int16_t  altitude;      /* Altitude from sensor fusion of Barometer and GPS (and Accelerometer?) */
    uint8_t  fix_quality;   /* GPS fix quality - High four is fix quality and low four bits satellite count */
    uint16_t heading;       /* Magnetometer heading (compass?) */
    int8_t   temperature;
    int16_t  speed;         /* Speed from accelerometer and barometer changes sensor fusion */
    int16_t  acceleration;  /* Acceleration vertically from the accelerometer (change this depending on how the accelerometer is mounted) */
} __attribute__((packed)) telemetry_packet_t;