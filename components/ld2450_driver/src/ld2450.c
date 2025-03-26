/**
 * @file ld2450.c
 * @brief Main implementation of the HLK-LD2450 driver
 * 
 * This file implements the main driver functionality for the HLK-LD2450 radar sensor,
 * including initialization, deinitialization, and core operations.
 * 
 * @author NieRVoid
 * @date 2025-03-12
 * @license MIT
 */

#include <string.h>
#include <inttypes.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ld2450.h"
#include "ld2450_private.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_timer.h"

static const char *TAG = LD2450_LOG_TAG;

// Global driver instance (singleton)
static ld2450_state_t s_ld2450_state = {0};

/**
 * @brief Get driver instance
 * 
 * @return Pointer to driver state structure
 */
ld2450_state_t *ld2450_get_instance(void)
{
    return &s_ld2450_state;
}

/**
 * @brief Initialize the LD2450 radar driver
 * 
 * @param config Pointer to driver configuration structure
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_init(const ld2450_config_t *config)
{
    ld2450_state_t *instance = ld2450_get_instance();
    esp_err_t ret = ESP_OK;
    
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (instance->initialized) {
        ESP_LOGW(TAG, "LD2450 driver already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Zero out the state structure
    memset(instance, 0, sizeof(ld2450_state_t));
    
    // Store configuration
    instance->uart_port = config->uart_port;
    instance->rx_pin = config->uart_rx_pin;
    instance->tx_pin = config->uart_tx_pin;
    instance->baud_rate = config->uart_baud_rate;
    instance->auto_processing = config->auto_processing;
    
    // Initialize logging configuration
    instance->log_level = config->log_level;
    instance->data_log_interval_ms = config->data_log_interval_ms;
    instance->last_data_log_time = 0;
    instance->last_frame_valid = false;
    
    // Create mutex for thread safety
    instance->mutex = xSemaphoreCreateMutex();
    if (!instance->mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = config->uart_baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Install UART driver
    ret = uart_driver_install(config->uart_port, LD2450_UART_RX_BUF_SIZE * 2, 
                              0, 20, &instance->uart_queue, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        vSemaphoreDelete(instance->mutex);
        return ret;
    }
    
    // Configure UART parameters
    ret = uart_param_config(config->uart_port, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        uart_driver_delete(config->uart_port);
        vSemaphoreDelete(instance->mutex);
        return ret;
    }
    
    // Set UART pins
    ret = uart_set_pin(config->uart_port, config->uart_tx_pin, config->uart_rx_pin, 
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        uart_driver_delete(config->uart_port);
        vSemaphoreDelete(instance->mutex);
        return ret;
    }
    
    // Configure GPIO pull-up for reliability
    gpio_set_pull_mode(config->uart_rx_pin, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(config->uart_tx_pin, GPIO_PULLUP_ONLY);
    
    // Initialize frame buffer index and sync state
    instance->frame_idx = 0;
    instance->frame_synced = false;
    instance->in_config_mode = false;
    
    // Initialize header pattern for fast detection
    static frame_header_t header_pattern;
    memcpy(header_pattern.bytes, LD2450_DATA_FRAME_HEADER, 4);
    
    // Set initialized flag
    instance->initialized = true;
    
    ESP_LOGI(TAG, "LD2450 driver initialized on UART%" PRIu32 " (RX: GPIO%" PRIu32 ", TX: GPIO%" PRIu32 ", baud: %" PRIu32 ")",  
        (uint32_t)instance->uart_port, (uint32_t)instance->rx_pin, (uint32_t)instance->tx_pin, instance->baud_rate);
    
    // Start processing task if auto-processing is enabled
    if (config->auto_processing) {
        xTaskCreate(ld2450_processing_task, "ld2450_task", LD2450_TASK_STACK_SIZE,
                    NULL, config->task_priority, &instance->task_handle);
                    
        if (!instance->task_handle) {
            ESP_LOGE(TAG, "Failed to create processing task");
            ld2450_deinit();
            return ESP_ERR_NO_MEM;
        }
        
        ESP_LOGI(TAG, "Auto-processing enabled with task priority %d", config->task_priority);
    }
    
    return ESP_OK;
}

/**
 * @brief Deinitialize the LD2450 radar driver and release resources
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_deinit(void)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Make sure we exit configuration mode if active
    if (instance->in_config_mode) {
        ld2450_exit_config_mode();
    }
    
    // Delete task if it was created
    if (instance->task_handle) {
        instance->initialized = false;  // Signal task to exit
        vTaskDelay(pdMS_TO_TICKS(100)); // Give task time to exit
        
        // If task still running, delete it
        if (eTaskGetState(instance->task_handle) != eDeleted) {
            vTaskDelete(instance->task_handle);
        }
        
        instance->task_handle = NULL;
    }
    
    // Delete UART driver
    uart_driver_delete(instance->uart_port);
    
    // Delete mutex
    if (instance->mutex) {
        vSemaphoreDelete(instance->mutex);
        instance->mutex = NULL;
    }
    
    // Reset state
    memset(instance, 0, sizeof(ld2450_state_t));
    
    ESP_LOGI(TAG, "LD2450 driver deinitialized");
    
    return ESP_OK;
}

/**
 * @brief Register a callback function for target data
 * 
 * @param callback Function pointer to call when new target data is available
 * @param user_ctx User context pointer passed to the callback function
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_register_target_callback(ld2450_target_cb_t callback, void *user_ctx)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(instance->mutex, portMAX_DELAY) == pdTRUE) {
        instance->target_callback = callback;
        instance->user_ctx = user_ctx;
        xSemaphoreGive(instance->mutex);
        
        ESP_LOGI(TAG, "Target callback %sregistered", callback ? "" : "un");
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

/**
 * @brief Process a radar data frame manually
 * 
 * @param data Raw frame data buffer
 * @param length Length of the data buffer in bytes
 * @param frame Pointer to frame structure to store parsed results
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_process_frame(const uint8_t *data, size_t length, ld2450_frame_t *frame)
{
    if (!data || !frame) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (length != LD2450_DATA_FRAME_SIZE) {
        ESP_LOGW(TAG, "Invalid frame size: %zu bytes (expected %d)", length, LD2450_DATA_FRAME_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }
    
    return ld2450_parse_frame(data, length, frame);
}

/**
 * @brief Processing task for radar data
 * 
 * This task continuously processes UART events to handle incoming radar data.
 * 
 * @param arg Task argument (not used)
 */
void ld2450_processing_task(void *arg)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance || !instance->initialized) {
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "LD2450 processing task started");
    
    // Initialize for adaptive delay
    instance->idle_count = 0;
    static uint8_t data_buffer[LD2450_UART_RX_BUF_SIZE];
    
    while (instance->initialized) {
        bool processed_data = false;
        
        // Skip processing if in configuration mode
        if (instance->in_config_mode) {
            // Give longer delay while in config mode to not interfere with config commands
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // Process UART events
        uart_event_t event;
        if (xQueueReceive(instance->uart_queue, &event, 0)) {
            switch (event.type) {
                case UART_DATA:
                {
                    // Read data from UART
                    int len = uart_read_bytes(instance->uart_port, data_buffer, 
                                             MIN(event.size, LD2450_UART_RX_BUF_SIZE),
                                             pdMS_TO_TICKS(10));
                    
                    if (len > 0) {
                        // Process the received data using optimized handler
                        ld2450_uart_event_handler(data_buffer, len);
                        processed_data = true;
                    }
                    break;
                }
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "UART FIFO overflow detected");
                    uart_flush_input(instance->uart_port);
                    xQueueReset(instance->uart_queue);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART buffer full");
                    uart_flush_input(instance->uart_port);
                    xQueueReset(instance->uart_queue);
                    break;
                case UART_BREAK:
                case UART_FRAME_ERR:
                case UART_PARITY_ERR:
                case UART_DATA_BREAK:
                case UART_PATTERN_DET:
                    // Log and ignore these events
                    ESP_LOGD(TAG, "UART event: %d", event.type);
                    break;
                default:
                    ESP_LOGD(TAG, "Unhandled UART event: %d", event.type);
                    break;
            }
        }
        
        // Adaptive delay based on activity
        if (processed_data) {
            instance->idle_count = 0;
            // Yield immediately to process more data
            taskYIELD();
        } else {
            instance->idle_count++;
            // Adaptive delay based on activity (max 50ms)
            uint32_t delay_ms = MIN(instance->idle_count, 10) * 5;
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
    
    ESP_LOGI(TAG, "LD2450 processing task stopped");
    vTaskDelete(NULL);
}

/**
 * @brief Format and log a radar data frame based on current log level
 * 
 * @param frame Pointer to frame data to log
 * @param force Force logging regardless of interval
 */
void ld2450_log_radar_frame(const ld2450_frame_t *frame, bool force)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance || !instance->initialized || !frame) {
        return;
    }
    
    // Check if logging is enabled
    if (instance->log_level < LD2450_LOG_VERBOSE && !force) {
        return;
    }
    
    // Check interval if not forced
    if (!force) {
        int64_t current_time = esp_timer_get_time() / 1000; // Convert to ms
        if ((current_time - instance->last_data_log_time) < instance->data_log_interval_ms) {
            return;
        }
        instance->last_data_log_time = current_time;
    }
    
    // Log frame data
    ESP_LOGI(TAG, "--- RADAR FRAME DATA [t=%lld] ---", frame->timestamp / 1000);
    ESP_LOGI(TAG, "Targets detected: %d", frame->count);
    
    // mm
    for (int i = 0; i < frame->count; i++) {
        const ld2450_target_t *target = &frame->targets[i];
        if (target->valid) {
            ESP_LOGI(TAG, "Target #%d: pos=(%d,%d)mm dist=%dmm angle=%.1f° speed=%dcm/s res=%dmm", 
                     i + 1, 
                     target->x, target->y,
                     (int)(target->distance),
                     target->angle, 
                     target->speed,                   // cm/s
                     target->resolution);
        }
    }
    
    // // cm
    // for (int i = 0; i < frame->count; i++) {
    //     const ld2450_target_t *target = &frame->targets[i];
    //     if (target->valid) {
    //         ESP_LOGI(TAG, "Target #%d: pos=(%d,%d)cm dist=%dcm angle=%.1f° speed=%dcm/s res=%dmm", 
    //                  i + 1, 
    //                  target->x / 10, target->y / 10,  // Convert mm to cm
    //                  (int)(target->distance / 10),    // Convert mm to cm without decimals
    //                  target->angle, 
    //                  target->speed,                   // Already in cm/s
    //                  target->resolution);             // Keep in mm
    //     }
    // }
    
    ESP_LOGI(TAG, "-------------------------------");
}

/**
 * @brief Set log verbosity level
 * 
 * @param level New log level
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_set_log_level(ld2450_log_level_t level)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance || !instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (level > LD2450_LOG_VERBOSE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(instance->mutex, portMAX_DELAY) == pdTRUE) {
        instance->log_level = level;
        xSemaphoreGive(instance->mutex);
        
        ESP_LOGI(TAG, "Log level set to %d", level);
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

/**
 * @brief Get current log verbosity level
 * 
 * @param level Pointer to store current log level
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_get_log_level(ld2450_log_level_t *level)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance || !instance->initialized || !level) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(instance->mutex, portMAX_DELAY) == pdTRUE) {
        *level = instance->log_level;
        xSemaphoreGive(instance->mutex);
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

/**
 * @brief Set interval between radar data logs
 * 
 * @param interval_ms Interval in milliseconds (0 to disable periodic logging)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_set_data_log_interval(uint32_t interval_ms)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance || !instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(instance->mutex, portMAX_DELAY) == pdTRUE) {
        instance->data_log_interval_ms = interval_ms;
        xSemaphoreGive(instance->mutex);
        
        ESP_LOGI(TAG, "Data log interval set to %" PRIu32 " ms", interval_ms);
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

/**
 * @brief Log current radar frame data (can be called anytime)
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t ld2450_log_frame_data(void)
{
    ld2450_state_t *instance = ld2450_get_instance();
    
    if (!instance || !instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(instance->mutex, portMAX_DELAY) == pdTRUE) {
        if (instance->last_frame_valid) {
            ld2450_frame_t frame_copy = instance->last_frame;
            xSemaphoreGive(instance->mutex);
            
            // Log the frame with force=true to bypass interval check
            ld2450_log_radar_frame(&frame_copy, true);
            return ESP_OK;
        } else {
            xSemaphoreGive(instance->mutex);
            ESP_LOGW(TAG, "No radar frame available to log");
            return ESP_ERR_NOT_FOUND;
        }
    }
    
    return ESP_FAIL;
}

/* 
 * The following functions are implemented in ld2450_config.c and are already declared
 * in ld2450.h. We don't need to re-implement them here, as they are accessible through
 * the public API header.
 * 
 * - ld2450_set_tracking_mode
 * - ld2450_get_tracking_mode
 * - ld2450_get_firmware_version
 * - ld2450_set_baud_rate
 * - ld2450_restore_factory_settings
 * - ld2450_restart_module
 * - ld2450_set_bluetooth
 * - ld2450_get_mac_address
 * - ld2450_set_region_filter
 * - ld2450_get_region_filter
 */