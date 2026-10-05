
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "cJSON.h"
cJSON *api_config_build_json(void);
void wifi_web_start(void);
void wifi_apply_from_lcd(bool on, uint8_t mode, const char *password);
