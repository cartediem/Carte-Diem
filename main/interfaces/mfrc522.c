#include "mfrc522.h"
#include "esp_log.h"
#include "freertos/task.h"

#define TAG "MFRC522"

// Common MFRC522 commands and registers
#define PCD_IDLE        0x00
#define PCD_TRANSCEIVE  0x0C
#define PCD_AUTHENT     0x0E
#define PCD_SOFTRESET   0x0F
#define PCD_CALCCRC     0x03

#define CommandReg      0x01
#define FIFODataReg     0x09
#define FIFOLevelReg    0x0A
#define BitFramingReg   0x0D
#define ComIrqReg       0x04
#define ErrorReg        0x06
#define ModeReg         0x11
#define TxModeReg       0x12
#define RxModeReg       0x13
#define TxControlReg    0x14
#define TModeReg        0x2A
#define TPrescalerReg   0x2B
#define TReloadRegH     0x2C
#define TReloadRegL     0x2D

#define PICC_REQIDL     0x26
#define PICC_ANTICOLL   0x93

// Helper macros
#define MFRC_CMD(addr, read) (((read) ? 0x80 : 0x00) | ((addr) & 0x7E))

static void write_reg(mfrc522_t *dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { MFRC_CMD(reg, 0), val };
    uint8_t rx[2] = {0};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = buf,
        .rx_buffer = rx
    };
    gpio_set_level(dev->cs_pin, 0);
    esp_err_t err = spi_device_transmit(dev->spi, &t);
    gpio_set_level(dev->cs_pin, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "write_reg failed: reg=0x%02X val=0x%02X err=%d", reg, val, err);
    }
    ESP_LOGD(TAG, "write_reg: reg=0x%02X val=0x%02X cmd=0x%02X rx[0]=0x%02X rx[1]=0x%02X",
             reg, val, buf[0], rx[0], rx[1]);
}

static uint8_t read_reg(mfrc522_t *dev, uint8_t reg) {
    // MFRC522 SPI read protocol: send address byte, then send dummy byte
    // The register data comes back during the SECOND byte (dummy byte phase)
    // Both bytes must be sent in a SINGLE SPI transaction to keep CS LOW throughout

    uint8_t tx[2] = { MFRC_CMD(reg, 1), 0x00 };  // Address byte + dummy byte
    uint8_t rx[2] = {0};

    spi_transaction_t t = {
        .length = 16,  // 2 bytes = 16 bits
        .tx_buffer = tx,
        .rx_buffer = rx
    };

    gpio_set_level(dev->cs_pin, 0);
    esp_err_t err = spi_device_transmit(dev->spi, &t);
    gpio_set_level(dev->cs_pin, 1);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "read_reg failed: reg=0x%02X err=%d", reg, err);
        return 0;
    }

    // Data is in rx[0] - the MFRC522 returns data one byte earlier than expected
    // This is due to SPI protocol timing in the ESP-IDF driver
    ESP_LOGD(TAG, "read_reg: reg=0x%02X rx[0]=0x%02X rx[1]=0x%02X",
             reg, rx[0], rx[1]);

    return rx[0];
}

static void set_bit_mask(mfrc522_t *dev, uint8_t reg, uint8_t mask) {
    uint8_t tmp = read_reg(dev, reg);
    write_reg(dev, reg, tmp | mask);
}

static void clear_bit_mask(mfrc522_t *dev, uint8_t reg, uint8_t mask) {
    uint8_t tmp = read_reg(dev, reg);
    write_reg(dev, reg, tmp & (~mask));
}

void mfrc522_reset(mfrc522_t *dev) {
    write_reg(dev, CommandReg, PCD_SOFTRESET);
    vTaskDelay(pdMS_TO_TICKS(50));
}

esp_err_t mfrc522_init(mfrc522_t *dev, spi_host_device_t host,
                       gpio_num_t miso, gpio_num_t mosi, gpio_num_t sck,
                       gpio_num_t cs, gpio_num_t rst)
{
    // --- SPI setup ---
    ESP_LOGI(TAG, "Initializing SPI with MISO=%d MOSI=%d SCK=%d CS=%d RST=%d", miso, mosi, sck, cs, rst);
    spi_bus_config_t buscfg = {
        .miso_io_num = miso,
        .mosi_io_num = mosi,
        .sclk_io_num = sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    esp_err_t ret = spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus initialization failed: %d (0x%x)", ret, ret);
        if (ret == ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "  → SPI host already initialized. Check if SPI2_HOST is used elsewhere.");
        }
        return ret;
    }
    ESP_LOGI(TAG, "SPI bus initialized successfully");

    // Try with 1 MHz clock for better signal integrity
    // MFRC522 max clock is 10 MHz, but 1 MHz is safer for longer wires
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,  // 1 MHz (was 4 MHz)
        .mode = 0,
        .spics_io_num = -1,   // manual CS
        .queue_size = 1,
    };
    ret = spi_bus_add_device(host, &devcfg, &dev->spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device addition failed: %d (0x%x)", ret, ret);
        return ret;
    }
    ESP_LOGI(TAG, "SPI device added successfully with 1 MHz clock (reduced from 4 MHz for reliability)");

    dev->cs_pin  = cs;
    dev->rst_pin = rst;
    gpio_set_direction(cs, GPIO_MODE_OUTPUT);
    gpio_set_level(cs, 1);
    gpio_set_direction(rst, GPIO_MODE_OUTPUT);
    gpio_set_level(rst, 1);
    ESP_LOGI(TAG, "CS and RST pins configured and set to HIGH");
    vTaskDelay(pdMS_TO_TICKS(10));

    // Perform hard reset sequence
    ESP_LOGI(TAG, "Performing hard reset: RST LOW → HIGH");
    gpio_set_level(rst, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(rst, 1);
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait 100ms for chip to power up

    // Test basic SPI communication before soft reset
    ESP_LOGI(TAG, "Testing basic SPI read after hard reset...");
    for (int i = 0; i < 3; i++) {
        uint8_t test_val = read_reg(dev, 0x37); // VersionReg
        ESP_LOGI(TAG, "  Test read %d: VersionReg = 0x%02X", i+1, test_val);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // --- Arduino-style PCD_Init sequence ---
    ESP_LOGI(TAG, "Performing soft reset...");
    write_reg(dev, CommandReg, PCD_SOFTRESET);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Wait for PowerDown bit to clear with timeout
    uint8_t cmd_reg;
    int timeout = 100; // ~500ms timeout (100 * 5ms)
    while (timeout-- > 0) {
        cmd_reg = read_reg(dev, CommandReg);
        if (!(cmd_reg & (1 << 4))) {
            break; // PowerDown bit cleared
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (timeout <= 0) {
        ESP_LOGW(TAG, "PowerDown bit did not clear after reset. CommandReg=0x%02X", cmd_reg);
    } else {
        ESP_LOGI(TAG, "PowerDown bit cleared. CommandReg=0x%02X", cmd_reg);
    }

    ESP_LOGI(TAG, "Configuring registers...");
    write_reg(dev, TModeReg, 0x80);
    write_reg(dev, TPrescalerReg, 0xA9);
    write_reg(dev, TReloadRegH, 0x03);
    write_reg(dev, TReloadRegL, 0xE8);
    write_reg(dev, TxModeReg, 0x00);
    write_reg(dev, RxModeReg, 0x00);
    write_reg(dev, ModeReg, 0x3D);

    // Turn on antenna driver pins (TX1, TX2)
    set_bit_mask(dev, TxControlReg, 0x03);

    ESP_LOGI(TAG, "Reading chip version...");
    uint8_t version = read_reg(dev, 0x37); // VersionReg
    ESP_LOGI(TAG, "Chip version: 0x%02X", version);

    if (version == 0x00) {
        ESP_LOGE(TAG, "ERROR: Chip version is 0x00 - SPI communication failure!");
        ESP_LOGE(TAG, "Check: 1) Power supply, 2) Wiring, 3) SPI pins, 4) CS/RST pins");
        return ESP_FAIL;
    }

    // Accept known MFRC522 versions: 0x92, 0x91 (official), 0x3F (compatible/clone variant)
    if (version != 0x92 && version != 0x91 && version != 0x3F) {
        ESP_LOGW(TAG, "WARNING: Unexpected chip version 0x%02X (expected 0x91, 0x92, or 0x3F)", version);
    } else {
        ESP_LOGI(TAG, "Chip version 0x%02X is supported", version);
    }

    return ESP_OK;
}


static esp_err_t picc_request(mfrc522_t *dev, uint8_t *tag_type) {
    write_reg(dev, BitFramingReg, 0x07);
    uint8_t req_cmd = PICC_REQIDL;
    write_reg(dev, CommandReg, PCD_IDLE);
    write_reg(dev, FIFOLevelReg, 0x80);
    write_reg(dev, FIFODataReg, req_cmd);
    write_reg(dev, CommandReg, PCD_TRANSCEIVE);
    set_bit_mask(dev, BitFramingReg, 0x80);

    vTaskDelay(pdMS_TO_TICKS(5));
    clear_bit_mask(dev, BitFramingReg, 0x80);
    uint8_t irq = read_reg(dev, ComIrqReg);
    ESP_LOGD(TAG, "picc_request: ComIrqReg=0x%02X", irq);
    if (!(irq & 0x30)) {
        return ESP_FAIL;
    }

    tag_type[0] = read_reg(dev, FIFODataReg);
    tag_type[1] = read_reg(dev, FIFODataReg);
    return ESP_OK;
}

static esp_err_t picc_anticoll(mfrc522_t *dev, uint8_t *uid, uint8_t *uid_len) {
    write_reg(dev, BitFramingReg, 0x00);
    write_reg(dev, CommandReg, PCD_IDLE);
    write_reg(dev, FIFOLevelReg, 0x80);

    uint8_t cmd = PICC_ANTICOLL;
    write_reg(dev, FIFODataReg, cmd);
    write_reg(dev, FIFODataReg, 0x20);
    write_reg(dev, CommandReg, PCD_TRANSCEIVE);
    set_bit_mask(dev, BitFramingReg, 0x80);

    vTaskDelay(pdMS_TO_TICKS(5));
    clear_bit_mask(dev, BitFramingReg, 0x80);

    uint8_t irq = read_reg(dev, ComIrqReg);
    ESP_LOGD(TAG, "picc_anticoll: ComIrqReg=0x%02X", irq);
    if (!(irq & 0x30)) {
        return ESP_FAIL;
    }

    *uid_len = read_reg(dev, FIFOLevelReg);
    ESP_LOGI(TAG, "picc_anticoll: uid_len=%d", *uid_len);
    for (uint8_t i = 0; i < *uid_len && i < MFRC522_MAX_UID_LEN; i++) {
        uid[i] = read_reg(dev, FIFODataReg);
    }
    return ESP_OK;
}

esp_err_t mfrc522_read_uid(mfrc522_t *dev, uint8_t *uid, uint8_t *uid_len) {
    uint8_t tag_type[2];
    if (picc_request(dev, tag_type) != ESP_OK) {
        ESP_LOGD(TAG, "mfrc522_read_uid: picc_request failed");
        return ESP_FAIL;
    }
    if (picc_anticoll(dev, uid, uid_len) != ESP_OK) {
        ESP_LOGD(TAG, "mfrc522_read_uid: picc_anticoll failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "UID read successfully: %d bytes", *uid_len);
    return ESP_OK;
}
