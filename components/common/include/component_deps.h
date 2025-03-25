/**
 * @file component_deps.h
 * @brief Component dependency management utilities
 * 
 * Provides functions and macros to manage dependencies between components.
 * 
 * @date 2025-03-20
 */

#pragma once

#include "esp_err.h"
#include "esp_log.h"

/**
 * @brief Check if a required component is initialized
 * 
 * @param component Component name (for logging)
 * @param init_check Function that checks if the component is initialized
 * @return ESP_OK if initialized, ESP_ERR_INVALID_STATE otherwise
 */
#define REQUIRE_COMPONENT_INIT(component, init_check) do { \
    if (!(init_check)) { \
        ESP_LOGE(TAG, component " must be initialized first"); \
        return ESP_ERR_INVALID_STATE; \
    } \
} while (0)
