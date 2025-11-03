#include "proximity_sensor.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char* TAG = "ProximitySensor";

// I2C timeout in milliseconds
#define I2C_TIMEOUT_MS 1000

// Helper function declarations
static bool proximity_sensor_write_register(ProximitySensor* sensor, uint8_t reg, uint8_t value);
static bool proximity_sensor_read_register(ProximitySensor* sensor, uint8_t reg, uint8_t* value);
static bool proximity_sensor_write_register_bit(ProximitySensor* sensor, uint8_t reg, uint8_t bit_mask, bool value);

// Public function implementations

ProximitySensor* proximity_sensor_create(uint8_t int_pin, uint8_t threshold, bool verbose) {
    ProximitySensor* sensor = (ProximitySensor*)malloc(sizeof(ProximitySensor));
    if (sensor == NULL) {
        return NULL;
    }

    sensor->i2c_port = I2C_NUM_0;
    sensor->int_pin = int_pin;
    sensor->threshold = threshold;
    sensor->verbose = verbose;
    sensor->connected = false;

    return sensor;
}

void proximity_sensor_destroy(ProximitySensor* sensor) {
    if (sensor == NULL) {
        return;
    }

    if (sensor->connected) {
        i2c_driver_delete(sensor->i2c_port);
    }

    free(sensor);
}

bool proximity_sensor_begin(ProximitySensor* sensor, i2c_port_t i2c_port, int sda_pin, int scl_pin, uint32_t clk_speed) {
    if (sensor == NULL) {
        return false;
    }

    sensor->i2c_port = i2c_port;

    // Configure I2C
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_pin;
    conf.scl_io_num = scl_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = clk_speed;
    conf.clk_flags = 0;

    esp_err_t err = i2c_param_config(i2c_port, &conf);
    if (err != ESP_OK) {
        if (sensor->verbose) {
            ESP_LOGE(TAG, "Failed to configure I2C parameters: %s", esp_err_to_name(err));
        }
        return false;
    }

    err = i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        if (sensor->verbose) {
            ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(err));
        }
        return false;
    }

    // Give the sensor time to power up
    vTaskDelay(pdMS_TO_TICKS(50));

    // Check device ID
    uint8_t id = 0;
    if (!proximity_sensor_read_register(sensor, APDS9960_ID, &id)) {
        if (sensor->verbose) {
            ESP_LOGE(TAG, "Failed to read device ID");
        }
        return false;
    }

    if (id != APDS9960_ID_1 && id != APDS9960_ID_2) {
        if (sensor->verbose) {
            ESP_LOGE(TAG, "Invalid device ID: 0x%02X (expected 0x%02X or 0x%02X)",
                     id, APDS9960_ID_1, APDS9960_ID_2);
        }
        return false;
    }

    if (sensor->verbose) {
        ESP_LOGI(TAG, "Device ID: 0x%02X", id);
    }

    // Disable all features initially
    if (!proximity_sensor_write_register(sensor, APDS9960_ENABLE, 0x00)) {
        if (sensor->verbose) {
            ESP_LOGE(TAG, "Failed to disable features");
        }
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    // Power on the device
    if (!proximity_sensor_write_register(sensor, APDS9960_ENABLE, APDS9960_PON)) {
        if (sensor->verbose) {
            ESP_LOGE(TAG, "Failed to power on device");
        }
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(10));



    // Set default proximity gain (4x)
    if (!proximity_sensor_set_gain(sensor, APDS9960_PGAIN_4X)) {
        if (sensor->verbose) {
            ESP_LOGE(TAG, "Failed to set proximity gain");
        }
        return false;
    }

    // Set default LED drive (100mA)
    if (!proximity_sensor_set_led_drive(sensor, APDS9960_LED_DRIVE_100MA)) {
        if (sensor->verbose) {
            ESP_LOGE(TAG, "Failed to set LED drive");
        }
        return false;
    }

    // Set default LED boost (100%)
    uint8_t config2 = 0;
    if (!proximity_sensor_read_register(sensor, APDS9960_CONFIG2, &config2)) {
        return false;
    }
    config2 = (config2 & 0xCF) | APDS9960_LED_BOOST_100;
    if (!proximity_sensor_write_register(sensor, APDS9960_CONFIG2, config2)) {
        return false;
    }

    // Set default proximity pulse (8 pulses, 8us length)
    if (!proximity_sensor_set_pulse(sensor, APDS9960_PULSE_LEN_8US, 8)) {
        if (sensor->verbose) {
            ESP_LOGE(TAG, "Failed to set proximity pulse");
        }
        return false;
    }

    // Enable proximity mode
    uint8_t enable = 0;
    if (!proximity_sensor_read_register(sensor, APDS9960_ENABLE, &enable)) {
        return false;
    }
    enable |= (APDS9960_PON | APDS9960_PEN);
    if (!proximity_sensor_write_register(sensor, APDS9960_ENABLE, enable)) {
        if (sensor->verbose) {
            ESP_LOGE(TAG, "Failed to enable proximity");
        }
        return false;
    }

    // Set the interrupt threshold (low threshold = 0, high threshold = THRESHOLD)
    if (!proximity_sensor_write_register(sensor, APDS9960_PILT, 0)) {
        return false;
    }
    if (!proximity_sensor_write_register(sensor, APDS9960_PIHT, sensor->threshold)) {
        if (sensor->verbose) {
            ESP_LOGE(TAG, "Failed to set interrupt threshold");
        }
        return false;
    }

    if (sensor->verbose) {
        ESP_LOGI(TAG, "Interrupt when value < %d", sensor->threshold);
    }

    // Enable proximity interrupt
    proximity_sensor_enable_interrupt(sensor);
    proximity_sensor_clear_interrupt(sensor);

    if (sensor->verbose) {
        ESP_LOGI(TAG, "✓ Proximity sensor ready...");
    }

    sensor->connected = true;
    return true;
}

bool proximity_sensor_is_connected(const ProximitySensor* sensor) {
    if (sensor == NULL) {
        return false;
    }
    return sensor->connected;
}

uint8_t proximity_sensor_read(ProximitySensor* sensor) {
    if (sensor == NULL) {
        return 0;
    }

    uint8_t proximity = 0;
    if (!proximity_sensor_read_register(sensor, APDS9960_PDATA, &proximity)) {
        if (sensor->verbose) {
            ESP_LOGE(TAG, "Failed to read proximity data");
        }
        return 0;
    }

    if (sensor->verbose) {
        ESP_LOGI(TAG, "Proximity value: %d", proximity);
    }

    return proximity;
}

void proximity_sensor_clear_interrupt(ProximitySensor* sensor) {
    if (sensor == NULL) {
        return;
    }

    // Clear interrupt by writing to PICLEAR register
    // This is a special transaction - just address the register
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (APDS9960_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, APDS9960_AICLEAR, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(sensor->i2c_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
}

void proximity_sensor_enable_interrupt(ProximitySensor* sensor) {
    if (sensor == NULL) {
        return;
    }

    uint8_t enable = 0;
    if (proximity_sensor_read_register(sensor, APDS9960_ENABLE, &enable)) {
        enable |= APDS9960_PIEN;
        enable &= ~APDS9960_AIEN;
        proximity_sensor_write_register(sensor, APDS9960_ENABLE, enable);
        proximity_sensor_write_register(sensor, APDS9960_PERS, 0x40); 
    }
}

void proximity_sensor_disable_interrupt(ProximitySensor* sensor) {
    if (sensor == NULL) {
        return;
    }

    uint8_t enable = 0;
    if (proximity_sensor_read_register(sensor, APDS9960_ENABLE, &enable)) {
        enable &= ~APDS9960_PIEN;
        proximity_sensor_write_register(sensor, APDS9960_ENABLE, enable);
    }
}

bool proximity_sensor_set_gain(ProximitySensor* sensor, uint8_t gain) {
    if (sensor == NULL) {
        return false;
    }

    uint8_t control = 0;
    if (!proximity_sensor_read_register(sensor, APDS9960_CONTROL, &control)) {
        return false;
    }

    // Clear proximity gain bits and set new value
    control = (control & 0xF3) | (gain & 0x0C);
    control = (control & 0xFC) | (APDS9960_AGAIN_4X & 0x03);
    return proximity_sensor_write_register(sensor, APDS9960_CONTROL, control);
}

bool proximity_sensor_set_led_drive(ProximitySensor* sensor, uint8_t drive) {
    if (sensor == NULL) {
        return false;
    }

    uint8_t control = 0;
    if (!proximity_sensor_read_register(sensor, APDS9960_CONTROL, &control)) {
        return false;
    }

    // Clear LED drive bits and set new value
    control = (control & 0x3F) | (drive & 0xC0);
    return proximity_sensor_write_register(sensor, APDS9960_CONTROL, control);
}

bool proximity_sensor_set_pulse(ProximitySensor* sensor, uint8_t pulse_length, uint8_t pulse_count) {
    if (sensor == NULL) {
        return false;
    }

    if (pulse_count < 1 || pulse_count > 64) {
        return false;
    }

    // Pulse count is encoded as count - 1
    uint8_t ppulse = (pulse_length & 0xC0) | ((pulse_count - 1) & 0x3F);
    return proximity_sensor_write_register(sensor, APDS9960_PPULSE, ppulse);
}

// Helper function implementations

static bool proximity_sensor_write_register(ProximitySensor* sensor, uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (APDS9960_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(sensor->i2c_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK && sensor->verbose) {
        ESP_LOGE(TAG, "Failed to write register 0x%02X: %s", reg, esp_err_to_name(ret));
    }

    return (ret == ESP_OK);
}

static bool proximity_sensor_read_register(ProximitySensor* sensor, uint8_t reg, uint8_t* value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (APDS9960_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (APDS9960_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(sensor->i2c_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK && sensor->verbose) {
        ESP_LOGE(TAG, "Failed to read register 0x%02X: %s", reg, esp_err_to_name(ret));
    }

    return (ret == ESP_OK);
}

static bool proximity_sensor_write_register_bit(ProximitySensor* sensor, uint8_t reg, uint8_t bit_mask, bool value) {
    uint8_t reg_value = 0;
    if (!proximity_sensor_read_register(sensor, reg, &reg_value)) {
        return false;
    }

    if (value) {
        reg_value |= bit_mask;
    } else {
        reg_value &= ~bit_mask;
    }

    return proximity_sensor_write_register(sensor, reg, reg_value);
}