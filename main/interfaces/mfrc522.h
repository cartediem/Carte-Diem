#ifndef MFRC522_H
#define MFRC522_H

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MFRC522_MAX_UID_LEN 10

typedef struct {
    spi_device_handle_t spi;
    gpio_num_t cs_pin;
    gpio_num_t rst_pin;
    uint8_t uid[MFRC522_MAX_UID_LEN];
    uint8_t uid_len;
} mfrc522_t;

/**
 * @brief Initialize MFRC522 over SPI
 * @param dev pointer to mfrc522_t structure
 * @param host SPI host (e.g., SPI2_HOST)
 * @param miso, mosi, sck, cs, rst GPIO pins
 * @return esp_err_t
 */
esp_err_t mfrc522_init(mfrc522_t *dev, spi_host_device_t host,
                       gpio_num_t miso, gpio_num_t mosi, gpio_num_t sck,
                       gpio_num_t cs, gpio_num_t rst);

/**
 * @brief Reads the 4-, 7-, or 10-byte NUID (UID) of the nearest card
 * @param dev pointer to initialized mfrc522_t
 * @param uid output buffer
 * @param uid_len output length (4–10)
 * @return ESP_OK if tag was detected
 */
esp_err_t mfrc522_read_uid(mfrc522_t *dev, uint8_t *uid, uint8_t *uid_len);

/**
 * @brief Perform a soft reset of the MFRC522
 */
void mfrc522_reset(mfrc522_t *dev);

#ifdef __cplusplus
}
#endif

#endif // MFRC522_H
