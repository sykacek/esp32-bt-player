#include "ssd_ui.h"

extern SSD1306_t dev;
static char local_title[128];


void ssd_ui_show_init_start(void){
	ESP_LOGI(SSD_TAG, "INTERFACE is i2c");
	ESP_LOGI(SSD_TAG, "CONFIG_SDA_GPIO=%d",CONFIG_SDA_GPIO);
	ESP_LOGI(SSD_TAG, "CONFIG_SCL_GPIO=%d",CONFIG_SCL_GPIO);
	ESP_LOGI(SSD_TAG, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
	i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

	ESP_LOGI(SSD_TAG, "Panel is 128x64");
	ssd1306_init(&dev, 128, 64);

	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	for(int i = 0; i < 4; ++i){
		ssd1306_display_text_x3(&dev, 0, ".", 1, false);
		vTaskDelay(500 / portTICK_PERIOD_MS);
		ssd1306_clear_screen(&dev, false);
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

void ssd_ui_show_init_finished(void){
	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	ssd1306_display_text(&dev, 4, "   ESP_SPEAKER", 14, false);
	ESP_LOGI(SSD_TAG, "Init finished");
}

void ssd_ui_show_title(char *title, int len){
	memset(local_title, 0x00, 64);
	ESP_LOGI(SSD_TAG, "changed title to %s", title);
	memcpy(local_title, title, 14);
	memset(local_title + 14, 0x20, 2);

	// Horizontal Scroll
	ssd1306_hardware_scroll(&dev, SCROLL_STOP);
	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	ssd1306_display_text(&dev, 4, local_title, strlen((const char *)local_title), false);
	ssd1306_hardware_scroll(&dev, SCROLL_RIGHT);

}

void ssd_ui_show_device_name(void){
	ssd1306_hardware_scroll(&dev, SCROLL_STOP);
	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	ssd1306_display_text(&dev, 4, local_device_name, 16, false);
}

void ssd_ui_show_connected(void){
	ssd1306_hardware_scroll(&dev, SCROLL_STOP);
	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	ssd1306_display_text(&dev, 4, "   Connected", 12, false);
	ESP_LOGI(SSD_TAG, "Connected");
}

void ssd_ui_show_disconnected(void){
	ssd1306_hardware_scroll(&dev, SCROLL_STOP);
	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	ssd1306_display_text(&dev, 4, "  Disconnected", 14, false);
	ESP_LOGI(SSD_TAG, "Disconnected");
}

void ssd_ui_show_connecting(void){
	ssd1306_hardware_scroll(&dev, SCROLL_STOP);
	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	ssd1306_display_text(&dev, 4, "  Connecting", 12, false);
	ESP_LOGI(SSD_TAG, "Connecting...");
}

void ssd_ui_show_paused(void){
	ssd1306_hardware_scroll(&dev, SCROLL_STOP);
	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	ssd1306_display_text(&dev, 4, "     PAUSED", 11, false);
	ESP_LOGI(SSD_TAG, "Paused");
}

void ssd_ui_show_volume(uint8_t volume){
	char buf[20];
	sprintf(buf, "      %d%c       ", volume, '%');
	ESP_LOGI(SSD_TAG, "volume chnaged to %d%", volume);

	ssd1306_hardware_scroll(&dev, SCROLL_STOP);
	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	ssd1306_display_text(&dev, 4, buf, 16, false);

	vTaskDelay(500 / portTICK_PERIOD_MS);

	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	ssd1306_display_text(&dev, 4, local_title, strlen((const char *)local_title), false);
	ssd1306_hardware_scroll(&dev, SCROLL_RIGHT);

}

void ssd_ui_show_error(void){
	ssd1306_display_text(&dev, 6, "Error", 5, false);
}
