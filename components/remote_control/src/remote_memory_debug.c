/**
 * @file remote_memory_debug.c
 * @brief Memory debugging utilities for Remote Control component
 */

#include <stdio.h>
#include "esp_log.h"
#include "remote_control.h"

static const char *TAG = "remote-memory";

void remote_control_print_memory_usage(void)
{
    size_t total = remote_control_get_memory_usage();
    
    ESP_LOGI(TAG, "Remote Control Memory Usage: %u bytes", total);
    
    // Get MQTT Manager memory for comparison
    size_t mqtt_handlers_mem = 0;
    size_t mqtt_topics_mem = 0;
    size_t mqtt_props_mem = 0;
    
    size_t mqtt_total = mqtt_manager_get_memory_usage(&mqtt_handlers_mem, &mqtt_topics_mem, &mqtt_props_mem);
    
    ESP_LOGI(TAG, "MQTT Manager Memory: %u bytes", mqtt_total);
    ESP_LOGI(TAG, "Total Smart Stay Memory: %u bytes", mqtt_total + total);
}
