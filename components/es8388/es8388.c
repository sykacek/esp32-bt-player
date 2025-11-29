// Michal Sykacek 2025

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "es8388.h"

#define REG_SIZE		(0x35)

const uint8_t reg_defaults[] = {
	0x06,	// reg 0x00
	0x5c,	// reg 0x01
	0xc3,	// reg 0x02
	0xfc,	// reg 0x03
	0xc0,	// reg 0x04
	0x00,	// reg 0x05
	0x00,	// reg 0x06
	0x7c,	// reg 0x07
	0x80,	// reg 0x08
	0x00,	// reg 0x09
	0x00,	// reg 0x0a
	0x02,	// reg 0x0b
	0x00,	// reg 0x0c
	0x06,	// reg 0x0d
	0x30,	// reg 0x0e
	0x20,	// reg 0x0f

	0xc0,	// reg 0x10
	0xc0,	// reg 0x11
	0x38,	// reg 0x12
	0xb0,	// reg 0x13
	0x32,	// reg 0x14
	0x06,	// reg 0x15
	0x00,	// reg 0x16
	0x00,	// reg 0x17
	0x06,	// reg 0x18
	0x22,	// reg 0x19
	0xc0,	// reg 0x1a
	0xc0,	// reg 0x1b
	0x08,	// reg 0x1c
	0x00,	// reg 0x1d
	0x1f,	// reg 0x1e
	0xf7,	// reg 0x1f

	0xfd,	// reg 0x20
	0xff,	// reg 0x21
	0x1f,	// reg 0x22
	0xf7,	// reg 0x23
	0xfd,	// reg 0x24
	0xff,	// reg 0x25
	0x00,	// reg 0x26
	0x38,	// reg 0x27
	0x28,	// reg 0x28
	0x28,	// reg 0x29
	0x38,	// reg 0x2a
	0x00,	// reg 0x2b
	0x00,	// reg 0x2c
	0x00,	// reg 0x2d
	0x00,	// reg 0x2e
	0x00,	// reg 0x2f

	0x00,	// reg 0x30
	0x00,	// reg 0x31
	0x00,	// reg 0x32
	0xaa,	// reg 0x33
	0xaa,	// reg 0x34
};


const uint8_t reg_copied[] = {
	0x12,
	0x50,
	0x0,
	0x9,
	0x3c,
	0x0,
	0x0,
	0x7c,
	0x0,
	0xbb,

	0x0,
	0x2,
	0xc,
	0x2,
	0x30,
	0x20,
	0x0,
	0x0,
	0x38,
	0xb0,

	0x32,
	0x6,
	0x0,
	0x18,
	0x2,
	0x0,
	0x2a,
	0x2a,
	0x8,
	0x0,

	0x1f,
	0xf7,
	0xfd,
	0xff,
	0x1f,
	0xf7,
	0xfd,
	0xff,
	0x0,
	0x90,

	0x28,
	0x28,
	0x90,
	0x80,
	0x0,
	0x0,
	0x1e,
	0x1e,
	0x0,
	0x0,
};

static const char *ES_TAG = "ES8388_DRIVER";
static i2c_master_bus_handle_t i2c_handle;
static i2c_master_dev_handle_t dev_handle;

extern i2s_chan_handle_t tx_chan;

/* 
* STATIC FUNCTION DEFINITIONS
*/

static esp_err_t es_write_reg(uint8_t reg_add, uint8_t data){
	const uint8_t write_buf[2] = {reg_add, data};
    esp_err_t ret = i2c_master_transmit(dev_handle, write_buf, 2, 1000);

	return ret;
}

static esp_err_t es_read_reg(uint8_t reg_add, uint8_t *data){
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_add, 1, data, 1, 1000);

	return ret;
}

static esp_err_t es_add_bits(uint8_t reg, uint8_t mask){
	uint8_t data;
    esp_err_t ret = es_read_reg(reg, &data);
	data |= mask;
	ret |= es_write_reg(reg, data);

	return ret;
}

static esp_err_t es_remove_bits(uint8_t reg, uint8_t mask){
	uint8_t data;
    esp_err_t ret = es_read_reg(reg, &data);
	data &= ~mask;
	ret |= es_write_reg(reg, data);

	return ret;
}

static esp_err_t reset_regs(void){
	esp_err_t ret = 0;
	for(uint8_t i = 0; i < REG_SIZE; ++i){
		ret |= es_write_reg(i, reg_defaults[i]);
	}

	return ret;
}

static int i2c_init(){
    int res = 0;
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_33,
        .scl_io_num = GPIO_NUM_32,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        //.device_address = (ES8388_ADDR >> 1),
        .device_address = 0x10,

        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_handle, &dev_config, &dev_handle));

    return res;
}

/* 
* FUNCTION DEFINITIONS
*/

void es8388_read_all(){
    for (int i = 0; i < 0x35; i++) {
        uint8_t reg = 0;
        es_read_reg(i, &reg);
        //ESP_LOGI(ES_TAG, "%x: %x", i, reg);
		printf("reg %2d: 0x%2x,\n", i, reg);
		vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

esp_err_t es8388_write_reg(uint8_t reg_add, uint8_t data){
    return es_write_reg(reg_add, data);
}

esp_err_t es8388_read_reg(uint8_t reg_add, uint8_t *data){
    return es_read_reg(reg_add, data);
}

esp_err_t es8388_init(void){
    esp_err_t res = ESP_OK;
    res |= i2c_init();

	res |= reset_regs(); // reset all registers to their defaults
	res |= es_add_bits(0x00, 0x10); // dsiable VREF volateg?
	res |= es_write_reg(0x01, 0x50); // same ?
	//res |= es_write_reg(0x02, 0x02);
	res |= es_write_reg(0x02, 0x02); // power up ADC too

	// 0x03 ADC 
	res |= es_write_reg(0x04, 0x3c);
	// 0x05 LOW POWER
	// 0x06 LOW POWER
	// 0x07 ANALOG V MANAGEMENT
	res |= es_write_reg(0x08, 0x00);
	// 0x09 - 0x16 ADC CTL

	res |= es_write_reg(0x17, 0x18); // format - SAME
	res |= es_write_reg(0x18, 0x02); // DAC DIV - SAME

	//res |= es_write_reg(0x19, 0x0a); // enable gain
	res |= es_write_reg(0x19, 0x00); // DONT CHANGE ANYTHING

	//res |= es_write_reg(0x1a, 0x00); // GAIN -0 dB
	res |= es_write_reg(0x1a, 0x2a); // GAIN -21 dB

	//res |= es_write_reg(0x1b, 0x00); // GAIN -0 dB
	res |= es_write_reg(0x1b, 0x2a); // GAIN -21 dB

	res |= es_write_reg(0x1d, 0x00); // ??

	// 0x1c DEEMPH
	// 0x1d SE ?
	// 0x1e  - 0x25 shelving filters
	// 0x26 mix routing
	//res |= es_write_reg(0x26, 0x09); // routing LRIN 2
	res |= es_write_reg(0x26, 0x00); // routing LRIN 1 (default) we dont use LIN and RIN

	res |= es_write_reg(0x27, 0x90); // LDAC to mixer enable 0dB

	res |= es_write_reg(0x2a, 0x90); // RDAC to mixer enable 0dB

	res |= es_write_reg(0x2b, 0x80); // DACLRC = ADCLRC
	// 0x2b ADCLRC and DACLRC
	// 0x2c DC OFFSET
	res |= es_write_reg(0x2e, 0x1e); // LOUT1 -0 dB
	res |= es_write_reg(0x2f, 0x1e); // ROUT1 -0 dB
	res |= es_write_reg(0x30, 0x1e); // LOUT2 -0 dB
	res |= es_write_reg(0x31, 0x1e); // ROUT2 -0 dB

	res |= es_write_reg(0x1a, 0x00); // GAIN -0 dB
	res |= es_write_reg(0x1b, 0x00); // GAIN -0 dB

	vTaskDelay(10 / portTICK_PERIOD_MS);

	gpio_reset_pin(PA_PIN);
	gpio_set_direction(PA_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(PA_PIN, 0);

    return res;
}

esp_err_t es8388_deinit(void){
    esp_err_t res = ESP_OK;

    res = es_write_reg(ES8388_CHIPPOWER, 0xFF);  //reset and stop es8388
    i2c_del_master_bus(i2c_handle);

    return res;
}

esp_err_t es8388_start(es_module_t mode){
    esp_err_t res = ESP_OK;

    return res;
}

esp_err_t es8388_stop(es_module_t mode){
    esp_err_t res = ESP_OK;

    return res;
}

esp_err_t es8388_pa_enable(bool en){
	if(en)
		gpio_set_level(PA_PIN, 1);
	else
		gpio_set_level(PA_PIN, 0);


	return ESP_OK;
}
