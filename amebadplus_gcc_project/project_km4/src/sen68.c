/**
  ******************************************************************************
  * @file    SEN68_simplified.c
  * @author  Generated Driver
  * @brief   SEN68 Environmental Sensor Driver (Simplified Non-blocking)
  *          This file provides firmware functions to manage the SEN68 sensor
  ******************************************************************************
  */

#include <string.h>
#include "IO_driver.h"
#include "logger/logger.h"

#include "sen68_rtl8721dcm.h"

/*
 * =============================================================================
 * SIMPLIFIED NON-BLOCKING DESIGN
 * =============================================================================
 *
 * 1. Remove complex async state machine
 * 2. Keep non-blocking I2C operations
 * 3. Simple timing-based command execution
 * 4. Direct function calls with built-in timing
 * =============================================================================
 */
// Private global variables
static sen68_data_t g_sen68_data;
static sen68_control_t g_sen68_ctrl =
{
    .state = SEN68_State_Init,
    .sampling = false,
    .last_sample_time = 0
};

// Simple command tracking
static struct
{
    uint32_t send_time;     // When command was sent
    uint32_t wait_time;     // How long to wait
    bool pending;           // Command is pending
} g_command_state = {0, 0, false};

/**
  * @brief  Get current sensor data
  * @retval Pointer to sen68_data_t structure
  */
sen68_data_t *sen68_get_data(void)
{
    return &g_sen68_data;
}

float sen68_get_pm1(void)
{
    return g_sen68_data.pm1 / 10.0f;
}

float sen68_get_pm2_5(void)
{
    return g_sen68_data.pm2_5 / 10.0f;
}

float sen68_get_pm4(void)
{
    return g_sen68_data.pm4 / 10.0f;
}

float sen68_get_pm10(void)
{
    return g_sen68_data.pm10 / 10.0f;
}

float sen68_get_humi(void)
{
    return g_sen68_data.humi / 100.0f;
}

float sen68_get_temp(void)
{
    return g_sen68_data.temp / 200.0f;
}

float sen68_get_voc_idx(void)
{
    return g_sen68_data.voc_idx / 10.0f;
}

float sen68_get_nox_idx(void)
{
    return g_sen68_data.nox_idx / 10.0f;
}

float sen68_get_hcho(void)
{
    return g_sen68_data.hcho / 10.0f;
}

// CRC-8-Dallas/Maxim lookup table (polynomial 0x31)
const uint8_t crc8_table[256] = {
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97, 0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
    0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4, 0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
    0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11, 0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
    0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA, 0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
    0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9, 0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C, 0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
    0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F, 0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
    0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED, 0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE, 0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
    0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B, 0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
    0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0, 0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
    0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93, 0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
    0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15, 0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC
};

/**
  * @brief  Calculate CRC-8 for 2 bytes (optimized for SEN68)
  * @param  byte1: first byte
  * @param  byte2: second byte
  * @retval CRC-8 checksum
  */
static inline uint8_t sen68_calculate_crc_2bytes(uint8_t byte1, uint8_t byte2)
{
    uint8_t crc = crc8_table[0xFF ^ byte1];  // Initial value 0xFF
    return crc8_table[crc ^ byte2];
}

/**
  * @brief  Send command to SEN68 (non-blocking)
  * @param  command: 16-bit command ID
  * @param  data: pointer to data buffer
  * @param  data_len: data length in bytes
  * @retval SEN68_OK if success, SEN68_ERROR if failed
  */
static uint8_t sen68_send_command_nb(uint16_t command, uint8_t *data, uint16_t data_len)
{
    uint8_t tx_buffer[32];
    uint16_t tx_len = 0;

    // Add command
    tx_buffer[tx_len++] = (command >> 8) & 0xFF;
    tx_buffer[tx_len++] = command & 0xFF;

    // Add data with CRC if provided
    if(data != NULL && data_len > 0)
    {
        for(uint16_t i = 0; i < data_len; i += 2)
        {
            tx_buffer[tx_len++] = data[i];
            if(i + 1 < data_len)
            {
                tx_buffer[tx_len++] = data[i + 1];
                tx_buffer[tx_len++] = sen68_calculate_crc_2bytes(data[i], data[i + 1]);
            }
            else
            {
                tx_buffer[tx_len++] = 0x00;
                uint8_t temp_data[2] = {data[i], 0x00};
                tx_buffer[tx_len++] = sen68_calculate_crc_2bytes(temp_data[0], temp_data[1]);
            }
        }
    }

    // Send command and track timing
    if(sen68_i2c_send_data(SEN68_I2C_ADDRESS, tx_buffer, tx_len) == SEN68_OK)
    {
        g_command_state.send_time = sen68_get_tick();
        g_command_state.pending = true;
        return SEN68_OK;
    }

    return SEN68_ERROR;
}

/**
  * @brief  Check if enough time has passed to read response
  * @param  wait_ms: minimum wait time in milliseconds
  * @retval true if can read, false if need to wait more
  */
static bool sen68_can_read_response(uint32_t wait_ms)
{
    if(!g_command_state.pending)
    {
        return false;
    }

    return (sen68_get_tick() - g_command_state.send_time) >= wait_ms;
}

/**
  * @brief  Mark command as completed
  * @retval None
  */
static void sen68_command_completed(void)
{
    g_command_state.pending = false;
}

/**
  * @brief  Parse sample data from response buffer
  * @param  buffer: pointer to response buffer (27 bytes)
  * @retval true if successful, false if CRC error
  */
static bool sen68_parse_sample_data(uint8_t *buffer)
{
    // Verify CRC for each data word
    for(uint16_t i = 0; i < 27; i += 3)
    {
        if(sen68_calculate_crc_2bytes(buffer[i], buffer[i + 1]) != buffer[i + 2])
        {
            return false;
        }
    }

    // Parse data
    g_sen68_data.pm1 = ((uint16_t)buffer[0] << 8) | buffer[1];
    g_sen68_data.pm2_5 = ((uint16_t)buffer[3] << 8) | buffer[4];
    g_sen68_data.pm4 = ((uint16_t)buffer[6] << 8) | buffer[7];
    g_sen68_data.pm10 = ((uint16_t)buffer[9] << 8) | buffer[10];
    g_sen68_data.humi = ((int16_t)buffer[12] << 8) | buffer[13];
    g_sen68_data.temp = ((int16_t)buffer[15] << 8) | buffer[16];
    g_sen68_data.voc_idx = ((int16_t)buffer[18] << 8) | buffer[19];
    g_sen68_data.nox_idx = ((int16_t)buffer[21] << 8) | buffer[22];
    g_sen68_data.hcho = ((uint16_t)buffer[24] << 8) | buffer[25];

    // Handle invalid values
    if(g_sen68_data.pm1 == 0xFFFF)
    {
        g_sen68_data.pm1 = 0;
    }
    if(g_sen68_data.pm2_5 == 0xFFFF)
    {
        g_sen68_data.pm2_5 = 0;
    }
    if(g_sen68_data.pm4 == 0xFFFF)
    {
        g_sen68_data.pm4 = 0;
    }
    if(g_sen68_data.pm10 == 0xFFFF)
    {
        g_sen68_data.pm10 = 0;
    }
    if(g_sen68_data.humi == 0x7FFF)
    {
        g_sen68_data.humi = 0;
    }
    if(g_sen68_data.temp == 0x7FFF)
    {
        g_sen68_data.temp = 0;
    }
    if(g_sen68_data.voc_idx == 0x7FFF)
    {
        g_sen68_data.voc_idx = 0;
    }
    if(g_sen68_data.nox_idx == 0x7FFF)
    {
        g_sen68_data.nox_idx = 0;
    }
    if(g_sen68_data.hcho == 0xFFFF)
    {
        g_sen68_data.hcho = 0;
    }

    return true;
}

/**
  * @brief  Handle initializing state
  * @retval None
  */
static void sen68_handle_initializing_state(void)
{
    static uint32_t init_start_time = 0;
    static bool command_sent = false;
    uint32_t tick = sen68_get_tick();

    if(init_start_time == 0)
    {
        init_start_time = tick;
        command_sent = false;
    }

    // Wait for sensor startup
    if(tick - init_start_time > SEN68_STARTUP_TIME_MS)
    {
        if(!command_sent && !g_command_state.pending)
        {
            // Send product name command
            if(sen68_send_command_nb(SEN68_CMD_GET_PRODUCT_NAME, NULL, 0) == SEN68_OK)
            {
                command_sent = true;
            }
        }
        else if(command_sent && sen68_can_read_response(20))
        {
            uint8_t rx_buffer[48];
            if(sen68_i2c_receive_data(SEN68_I2C_ADDRESS, rx_buffer, 48) == SEN68_OK)
            {
                g_sen68_ctrl.state = SEN68_State_Idle;
                init_start_time = 0;
                command_sent = false;
            }
            else
            {
                // Retry after delay
                if(tick - init_start_time > 5000)
                {
                    init_start_time = 0;
                    command_sent = false;
                }
            }
            sen68_command_completed();
        }
    }
}

/**
  * @brief  Handle idle state
  * @retval None
  */
static void sen68_handle_idle_state(void)
{
    static bool start_command_sent = false;

    if(!g_sen68_ctrl.sampling)
    {
        if(!start_command_sent && !g_command_state.pending)
        {
            if(sen68_send_command_nb(SEN68_CMD_START_SAMPLE, NULL, 0) == SEN68_OK)
            {
                start_command_sent = true;
            }
        }
        else if(start_command_sent && sen68_can_read_response(50))
        {
            g_sen68_ctrl.state = SEN68_State_Sample_Starting;
            g_sen68_ctrl.sampling = true;
            g_sen68_ctrl.last_sample_time = sen68_get_tick();
            start_command_sent = false;
            sen68_command_completed();
        }
    }
}

/**
  * @brief  Handle sample starting state
  * @retval None
  */
static void sen68_handle_sample_starting_state(void)
{
    static uint32_t stabilization_start_time = 0;
    uint32_t tick = sen68_get_tick();

    if(stabilization_start_time == 0)
    {
        stabilization_start_time = tick;
    }

    // Wait for sensor stabilization
    if(tick - stabilization_start_time > 1500)
    {
        g_sen68_ctrl.state = SEN68_State_Sampling;
        g_sen68_ctrl.last_sample_time = tick;
        stabilization_start_time = 0;
    }
}

/**
  * @brief  Handle sampling state
  * @retval None
  */
static void sen68_handle_sampling_state(void)
{
    static enum
    {
        STEP_CHECK_READY,
        STEP_READ_DATA
    } sample_step = STEP_CHECK_READY;

    uint32_t tick = sen68_get_tick();

    // Only start new sample cycle after interval has passed
    if(tick - g_sen68_ctrl.last_sample_time >= SEN68_SAMPLE_INTERVAL_MS)
    {
        switch(sample_step)
        {
        case STEP_CHECK_READY:
            if(!g_command_state.pending)
            {
                sen68_send_command_nb(SEN68_CMD_GET_DATA_READY, NULL, 0);
            }
            else if(sen68_can_read_response(20))
            {
                uint8_t rx_buffer[3];
                if(sen68_i2c_receive_data(SEN68_I2C_ADDRESS, rx_buffer, 3) == SEN68_OK)
                {
                    if(sen68_calculate_crc_2bytes(rx_buffer[0], rx_buffer[1]) == rx_buffer[2] && rx_buffer[1] == 1)
                    {
                        sample_step = STEP_READ_DATA;
                    }
                    else
                    {
                        // Data not ready, reset timer and wait for next interval
                        g_sen68_ctrl.last_sample_time = tick;
                        sample_step = STEP_CHECK_READY;
                    }
                }
                else
                {
                    // Read failed, reset timer and wait for next interval
                    g_sen68_ctrl.last_sample_time = tick;
                    sample_step = STEP_CHECK_READY;
                }
                sen68_command_completed();
            }
            break;

        case STEP_READ_DATA:
            if(!g_command_state.pending)
            {
                sen68_send_command_nb(SEN68_CMD_READ_SAMPLED_VALUES, NULL, 0);
            }
            else if(sen68_can_read_response(20))
            {
                uint8_t data_buffer[27];
                if(sen68_i2c_receive_data(SEN68_I2C_ADDRESS, data_buffer, 27) == SEN68_OK)
                {
                    sen68_parse_sample_data(data_buffer);
                    // LOGI("SEN68 Sampled Data: ");
                    // LOGI("PM1.0: %.1f, PM2.5: %.1f, PM4.0: %.1f, PM10.0: %.1f",
                    //      g_sen68_data.pm1 / 10.0, g_sen68_data.pm2_5 / 10.0,
                    //      g_sen68_data.pm4 / 10.0, g_sen68_data.pm10 / 10.0);
                    // LOGI("Humidity: %.1f, Temperature: %.1f, VOC: %.1f, NOx: %.1f, HCHO: %.1f",
                    //      g_sen68_data.humi / 100.0, g_sen68_data.temp / 200.0,
                    //      g_sen68_data.voc_idx / 10.0, g_sen68_data.nox_idx / 10.0,
                    //      g_sen68_data.hcho / 10.0);
                }
                sample_step = STEP_CHECK_READY;
                // Only update timer after completing full sample cycle
                g_sen68_ctrl.last_sample_time = tick;
                sen68_command_completed();
            }
            break;
        }
    }
}

/**
  * @brief  Handle error state
  * @retval None
  */
static void sen68_handle_error_state(void)
{
    static uint32_t error_recovery_time = 0;
    static bool reset_sent = false;
    uint32_t tick = sen68_get_tick();

    if(error_recovery_time == 0)
    {
        error_recovery_time = tick;
        g_sen68_ctrl.sampling = false;
        reset_sent = false;
    }

    if(tick - error_recovery_time > 5000)
    {
        if(!reset_sent && !g_command_state.pending)
        {
            if(sen68_send_command_nb(SEN68_CMD_DEVICE_RESET, NULL, 0) == SEN68_OK)
            {
                reset_sent = true;
            }
        }
        else if(reset_sent && sen68_can_read_response(1200))
        {
            g_sen68_ctrl.state = SEN68_State_Init;
            error_recovery_time = 0;
            reset_sent = false;
            sen68_command_completed();
        }
        else if(tick - error_recovery_time > 15000)
        {
            error_recovery_time = 0;
            reset_sent = false;
        }
    }
}

/**
  * @brief  Process SEN68 state machine
  * @retval None
  */
static void sen68_state_process(void)
{
    switch(g_sen68_ctrl.state)
    {
    case SEN68_State_Init:
        sen68_handle_initializing_state();
        break;
    case SEN68_State_Idle:
        sen68_handle_idle_state();
        break;
    case SEN68_State_Sample_Starting:
        sen68_handle_sample_starting_state();
        break;
    case SEN68_State_Sampling:
        sen68_handle_sampling_state();
        break;
    case SEN68_State_Error:
        sen68_handle_error_state();
        break;
    default:
        g_sen68_ctrl.state = SEN68_State_Init;
        break;
    }
}

/**
  * @brief  Initialize SEN68 sensor
  * @retval SEN68_OK if success, SEN68_ERROR if failed
  */
uint8_t sen68_init(void)
{
    g_sen68_ctrl.state = SEN68_State_Init;
    g_sen68_ctrl.sampling = false;
    memset(&g_sen68_data, 0, sizeof(g_sen68_data));
    g_sen68_ctrl.last_sample_time = sen68_get_tick();
    g_command_state.pending = false;

    return SEN68_OK;
}

/**
  * @brief  Start continuous sample
  * @retval SEN68_OK if success, SEN68_ERROR if failed
  */
uint8_t sen68_start_sample(void)
{
    if(g_sen68_ctrl.sampling)
    {
        return SEN68_OK;
    }

    // Will be handled by state machine
    g_sen68_ctrl.sampling = true;
    g_sen68_ctrl.state = SEN68_State_Sampling;
    return SEN68_OK;
}

/**
  * @brief  Stop sample (non-blocking)
  * @retval SEN68_OK if success, SEN68_ERROR if failed
  */
uint8_t sen68_stop_sample(void)
{
    if(!g_sen68_ctrl.sampling)
    {
        return SEN68_OK;
    }

    // Send stop command if no other command pending
    if(!g_command_state.pending)
    {
        if(sen68_send_command_nb(SEN68_CMD_STOP_SAMPLE, NULL, 0) == SEN68_OK)
        {
            g_sen68_ctrl.sampling = false;
            g_sen68_ctrl.state = SEN68_State_Idle;
            return SEN68_OK;
        }
    }

    return SEN68_ERROR;
}

/**
  * @brief  Check if sample is running
  * @retval true if sampling, false otherwise
  */
bool sen68_is_sampling(void)
{
    return g_sen68_ctrl.sampling;
}

/**
  * @brief  SEN68 Handler function - call this periodically in main loop
  * @retval None
  */
void sen68_handler(void)
{
    //if(user_get_runtime()->on)
   // {
        sen68_power_on();
        sen68_state_process();
   // }
    //else
    //{
        if(g_sen68_ctrl.sampling)
        {
            sen68_stop_sample();
        }

        sen68_power_off();
        g_sen68_ctrl.state = SEN68_State_Init;
        g_command_state.pending = false;
        memset(&g_sen68_data, 0, sizeof(g_sen68_data));
   // }
}
/************************************************test**************************************** */
void sen68_test_product_info(void) {
    uint8_t rx_buffer[48];
    
    // 读取产品名称
    if(sen68_send_command_nb(SEN68_CMD_GET_PRODUCT_NAME, NULL, 0) == SEN68_OK) {
        rtos_time_delay_ms(20);  // 等待命令执行
        if(sen68_i2c_receive_data(SEN68_I2C_ADDRESS, rx_buffer, 48) == SEN68_OK) {
            printf("Product Name: ");
            for(int i = 0; i < 48; i += 3) {
                if(rx_buffer[i] != 0 && rx_buffer[i+1] != 0) {
                    printf("%c%c", rx_buffer[i], rx_buffer[i+1]);
                }
            }
            printf("\n");
        }
    }
    
    // 读取序列号
    rtos_time_delay_ms(100);
    if(sen68_send_command_nb(SEN68_CMD_GET_SERIAL_NUMBER, NULL, 0) == SEN68_OK) {
        rtos_time_delay_ms(20);
        if(sen68_i2c_receive_data(SEN68_I2C_ADDRESS, rx_buffer, 48) == SEN68_OK) {
            printf("Serial Number: ");
            for(int i = 0; i < 48; i += 3) {
                if(rx_buffer[i] != 0 && rx_buffer[i+1] != 0) {
                    printf("%c%c", rx_buffer[i], rx_buffer[i+1]);
                }
            }
            printf("\n");
        }
    }
    
    // 读取版本信息
    rtos_time_delay_ms(100);
    if(sen68_send_command_nb(SEN68_CMD_GET_VERSION, NULL, 0) == SEN68_OK) {
        rtos_time_delay_ms(20);
        if(sen68_i2c_receive_data(SEN68_I2C_ADDRESS, rx_buffer, 12) == SEN68_OK) {
            printf("Firmware Version: %d.%d\n", rx_buffer[0], rx_buffer[1]);
            printf("Hardware Version: %d.%d\n", rx_buffer[3], rx_buffer[4]);
            printf("Protocol Version: %d.%d\n", rx_buffer[6], rx_buffer[7]);
        }
    }
}
void sen68_start_sampling(void) {
    printf("Starting SEN68 sampling...\n");
    
    if(sen68_start_sample() == SEN68_OK) {
        printf("Sampling started successfully\n");
    } else {
        printf("Failed to start sampling\n");
    }
}
void sen68_read_data_simple(void) {
    uint8_t rx_buffer[27];
    
    // 检查数据是否准备好
    if(sen68_send_command_nb(SEN68_CMD_GET_DATA_READY, NULL, 0) == SEN68_OK) {
        rtos_time_delay_ms(20);
        uint8_t ready_buffer[3];
        if(sen68_i2c_receive_data(SEN68_I2C_ADDRESS, ready_buffer, 3) == SEN68_OK) {
            if(ready_buffer[1] == 1) {  // 数据准备好了
                // 读取采样数据
                rtos_time_delay_ms(50);
                if(sen68_send_command_nb(SEN68_CMD_READ_SAMPLED_VALUES, NULL, 0) == SEN68_OK) {
                    rtos_time_delay_ms(20);
                    if(sen68_i2c_receive_data(SEN68_I2C_ADDRESS, rx_buffer, 27) == SEN68_OK) {
                        // 解析并显示数据
                        sen68_parse_sample_data(rx_buffer);
                        
                        printf("=== SEN68 Environmental Data ===\n");
                        printf("PM1.0:  %.1f μg/m³\n", sen68_get_pm1());
                        printf("PM2.5:  %.1f μg/m³\n", sen68_get_pm2_5());
                        printf("PM4.0:  %.1f μg/m³\n", sen68_get_pm4());
                        printf("PM10.0: %.1f μg/m³\n", sen68_get_pm10());
                        printf("Humidity: %.1f %%\n", sen68_get_humi());
                        printf("Temperature: %.1f °C\n", sen68_get_temp());
                        printf("VOC Index: %.1f\n", sen68_get_voc_idx());
                        printf("NOx Index: %.1f\n", sen68_get_nox_idx());
                        printf("HCHO: %.1f ppb\n", sen68_get_hcho());
                        printf("===============================\n");
                    }
                }
            } else {
                printf("Data not ready yet\n");
            }
        }
    }
}

void sen68_test(void){

    gpio_init(&sen68_power_gpio, SEN68_POWER_PIN);
    gpio_dir(&sen68_power_gpio, PIN_OUTPUT);
    gpio_mode(&sen68_power_gpio, PullNone);
     sen68_power_on();
    rtos_time_delay_ms(1000);
    printf("SEN68 power status: %d\n", gpio_read(&sen68_power_gpio));
    
    if(sen68_i2c_init() != SEN68_OK) {
        printf("I2C init failed\n");
        return;
    }
    printf("I2C init successful\n");
    
    if(sen68_init() != SEN68_OK) {
        printf("SEN68 init failed\n");
        return;
    }
    printf("SEN68 init successful\n");
    
    // 2. 读取产品信息
    rtos_time_delay_ms(1000);  // 等待传感器稳定
    sen68_test_product_info();
    
    // 3. 启动采样
    rtos_time_delay_ms(1000);
    sen68_start_sampling();
    
    // 4. 等待并读取数据（循环几次）
    for(int i = 0; i < 10; i++) {
        rtos_time_delay_ms(2000);  // 等待2秒
        printf("\n--- Reading %d ---\n", i+1);
        sen68_read_data_simple();
    }
}
