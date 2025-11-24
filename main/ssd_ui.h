#ifndef __SSD1306_UI_H__
#define __SSD1306_UI_H__

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "bt_app_core.h"
#include "bt_app_av.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#include "ssd1306.h"
#include "font8x8_basic.h"

#define SSD_TAG		"SSD1306"

#define CENTER		(1)
#define TOP		(1)
#define BOTTOM		(4)

extern const char local_device_name[];
extern SSD1306_t dev;

void ssd_ui_show_init_start(void);
void ssd_ui_show_init_finished(void);

void ssd_ui_show_title(char *str, int len);
void ssd_ui_show_device_name(void);

// bt event notification
void ssd_ui_show_connected(void);
void ssd_ui_show_disconnected(void);
void ssd_ui_show_connecting(void);

void ssd_ui_show_volume(uint8_t volume);
void ssd_ui_show_error(void);
void ssd_ui_show_paused(void);

#endif
