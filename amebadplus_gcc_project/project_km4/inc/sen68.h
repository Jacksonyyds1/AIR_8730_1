/**
  ******************************************************************************
  * @file    SEN68.h
  * @author  Generated Driver
  * @brief   Header file for SEN68 Environmental Sensor Driver
  ******************************************************************************
  */

#ifndef __SEN68_H
#define __SEN68_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// SEN68 I2C Address (7-bit address 0x6B shifted left by 1)
#define SEN68_I2C_ADDRESS                 0x6B 

// SEN68 Command IDs
#define SEN68_CMD_START_SAMPLE         0x0021
#define SEN68_CMD_STOP_SAMPLE          0x0104
#define SEN68_CMD_GET_DATA_READY            0x0202
#define SEN68_CMD_READ_SAMPLED_VALUES      0x0467
#define SEN68_CMD_READ_RAW_VALUES           0x0455
#define SEN68_CMD_READ_NUMBER_CONCENTRATION 0x0316
#define SEN68_CMD_SET_TEMP_OFFSET_PARAMS    0x60B2
#define SEN68_CMD_SET_TEMP_ACCEL_PARAMS     0x6100
#define SEN68_CMD_GET_PRODUCT_NAME          0xD014
#define SEN68_CMD_GET_SERIAL_NUMBER         0xD033
#define SEN68_CMD_READ_DEVICE_STATUS        0xD206
#define SEN68_CMD_READ_CLEAR_DEVICE_STATUS  0xD210
#define SEN68_CMD_GET_VERSION               0xD100
#define SEN68_CMD_DEVICE_RESET              0xD304
#define SEN68_CMD_START_FAN_CLEANING        0x5607
#define SEN68_CMD_ACTIVATE_SHT_HEATER       0x6765
#define SEN68_CMD_GET_SHT_HEATER_MEAS       0x6790
#define SEN68_CMD_GET_VOC_ALGORITHM_PARAMS  0x60D0
#define SEN68_CMD_SET_VOC_ALGORITHM_PARAMS  0x60D0
#define SEN68_CMD_GET_VOC_ALGORITHM_STATE   0x6181
#define SEN68_CMD_SET_VOC_ALGORITHM_STATE   0x6181
#define SEN68_CMD_GET_NOX_ALGORITHM_PARAMS  0x60E1
#define SEN68_CMD_SET_NOX_ALGORITHM_PARAMS  0x60E1

// CRC Constants
#define SEN68_CRC8_POLYNOMIAL       0x31
#define SEN68_CRC8_INIT             0xFF

// Timing Constants
#define SEN68_STARTUP_TIME_MS       1000
#define SEN68_SAMPLE_INTERVAL_MS 10000

// Command execution times (from datasheet Table 4.8)
#define SEN68_CMD_EXEC_TIME_DEFAULT         20    // Most commands: 20ms
#define SEN68_CMD_EXEC_TIME_START_SAMPLE   50    // Start sample: 50ms  
#define SEN68_CMD_EXEC_TIME_STOP_SAMPLE    1000  // Stop sample: 1000ms

// Status Register Bits
#define SEN68_STATUS_SPEED_WARNING  (1 << 21)
#define SEN68_STATUS_HCHO_ERROR     (1 << 10)
#define SEN68_STATUS_GAS_ERROR      (1 << 7)
#define SEN68_STATUS_RHT_ERROR      (1 << 6)
#define SEN68_STATUS_FAN_ERROR      (1 << 4)

// Return Values
#define SEN68_OK                    0
#define SEN68_ERROR                 1

// State definitions for Handler
typedef enum {
    SEN68_State_Init = 0,
    SEN68_State_Idle,
    SEN68_State_Sample_Starting,
    SEN68_State_Sampling,
    SEN68_State_Error
} sen68_state_t;

// Sensor control structure
typedef struct {
    sen68_state_t state;
    bool sampling;
    uint32_t last_sample_time;
} sen68_control_t;

// Data Structures
typedef struct {
    uint16_t pm1;       // PM1.0 concentration (μg/m³ * 10)
    uint16_t pm2_5;     // PM2.5 concentration (μg/m³ * 10)
    uint16_t pm4;       // PM4.0 concentration (μg/m³ * 10)
    uint16_t pm10;      // PM10.0 concentration (μg/m³ * 10)
    int16_t humi;       // Relative humidity (% * 100)
    int16_t temp;       // Temperature (°C * 200)
    int16_t voc_idx;    // VOC index (* 10)
    int16_t nox_idx;    // NOx index (* 10)
    uint16_t hcho;      // Formaldehyde concentration (ppb * 10)
} sen68_data_t;

// Function Prototypes
sen68_data_t *sen68_get_data(void);
float sen68_get_pm1(void);
float sen68_get_pm2_5(void);
float sen68_get_pm4(void);
float sen68_get_pm10(void);
float sen68_get_humi(void);
float sen68_get_temp(void);
float sen68_get_voc_idx(void);
float sen68_get_nox_idx(void);
float sen68_get_hcho(void);

uint8_t sen68_init(void);
uint8_t sen68_start_sample(void);
uint8_t sen68_stop_sample(void);
bool sen68_is_sampling(void);
void sen68_handler(void);

void sen68_test_product_info(void);
void sen68_start_sampling(void);
void sen68_read_data_simple(void);
void sen68_test(void);



#ifdef __cplusplus
}
#endif

#endif // SEN68_H