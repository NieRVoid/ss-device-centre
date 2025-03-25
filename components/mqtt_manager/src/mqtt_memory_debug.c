/**
 * @file mqtt_memory_debug.c
 * @brief Memory debugging utilities for MQTT manager
 */

#include <stdio.h>
#include "esp_log.h"
#include "mqtt_manager.h"

static const char *TAG = "mqtt-memory";

void mqtt_manager_print_memory_usage(void)
{
    size_t handlers_mem = 0;
    size_t topics_mem = 0;
    size_t props_mem = 0;
    
    size_t total = mqtt_manager_get_memory_usage(&handlers_mem, &topics_mem, &props_mem);
    
    ESP_LOGI(TAG, "MQTT Manager Memory Usage:");
    ESP_LOGI(TAG, "  Handlers: %u bytes", handlers_mem);
    ESP_LOGI(TAG, "  Topics:   %u bytes", topics_mem);
    ESP_LOGI(TAG, "  Props:    %u bytes", props_mem);
    ESP_LOGI(TAG, "  TOTAL:    %u bytes", total);
}
