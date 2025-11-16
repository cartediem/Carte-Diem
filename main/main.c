#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

#include "interfaces/barcode.h"
#include "interfaces/proximity_sensor.h"
#include "interfaces/mfrc522.h"
#include "interfaces/ble_barcode_nimble.h"
#include "interfaces/loadcells.h"
#include "interfaces/cart_rfid.h"
#include "interfaces/imu.h"
#include "driver/i2c_master.h"

// I2C: Proximity Sensor, IMU, 
#define SCL_PIN GPIO_NUM_9
#define SDA_PIN GPIO_NUM_8

#define PROXIMITY_INT_PIN GPIO_NUM_15
#define PROXIMITY_THRESHOLD 30

// SPI: Payment
#define MOSI_PIN GPIO_NUM_11
#define SCK_PIN  GPIO_NUM_12
#define MISO_PIN GPIO_NUM_13
#define PAYMENT_CS_PIN   GPIO_NUM_10
#define PAYMENT_RST_PIN  GPIO_NUM_14

#define TOP_LOAD_DATA_PIN GPIO_NUM_38
#define TOP_LOAD_CLK_PIN GPIO_NUM_39

#define BOTTOM_LOAD_DATA_PIN GPIO_NUM_40
#define BOTTOM_LOAD_CLK_PIN GPIO_NUM_41

// UART: Barcode scanner, Item RFID, Customer RFID
#define BARCODE_TX_PIN GPIO_NUM_1
#define BARCODE_RX_PIN GPIO_NUM_2

#define CART_RFID_UART_PORT UART_NUM_2
#define CART_RFID_TX_PIN GPIO_NUM_5
#define CART_RFID_RX_PIN GPIO_NUM_16

//Other
#define BUTTON_PIN GPIO_NUM_37
#define TAG "MAIN"

static barcode_t scanner;
static ProximitySensor* proximity_sensor = NULL;
static cart_rfid_reader_t* cart_reader = NULL;
static ICM20948_t imu_sensor;
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static mfrc522_t paymenter;

static LoadCell* produce_load_cell = NULL;
static LoadCell* cart_load_cell = NULL;

static QueueHandle_t button_evt_queue = NULL;
static QueueHandle_t proximity_evt_queue = NULL;
static QueueHandle_t imu_idle_evt_queue = NULL;

static TaskHandle_t payment_task_handle = NULL;

static bool continuous_mode = false;
static bool payment_mode = false;

static void IRAM_ATTR button_isr(void *arg)
{
    uint32_t evt = 1;
    xQueueSendFromISR(button_evt_queue, &evt, NULL);
    ESP_EARLY_LOGI(TAG, "Button interrupt triggered");
}

static void IRAM_ATTR proximity_isr(void *arg)
{
    uint32_t evt = 1;
    xQueueSendFromISR(proximity_evt_queue, &evt, NULL);
    ESP_EARLY_LOGI(TAG, "Proximity interrupt triggered");
}

void on_cart_scan_complete(const cart_rfid_tag_t *tags, int count) {
    ESP_LOGI(TAG, "Found %d items in cart", count);

    // Get cart weight
    float cart_weight = load_cell_display_pounds(produce_load_cell);

    // Build verification string: "weight, num_tags, tag1, tag2, tag3, ..."
    char verification_msg[512] = {0};
    int offset = snprintf(verification_msg, sizeof(verification_msg), "%.4f,%d", cart_weight, count);

    // Add each tag to the message
    for (int i = 0; i < count && offset < (int)sizeof(verification_msg) - 1; i++) {
        offset += snprintf(verification_msg + offset,
                          sizeof(verification_msg) - offset,
                          ",%s",
                          tags[i].tag);
    }

    // Send via BLE
    esp_err_t send_ret = ble_send_item_verification(verification_msg);
    if (send_ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Cart verification sent via BLE: %s", verification_msg);
    } else {
        ESP_LOGW(TAG, "✗ Failed to send cart verification via BLE");
    }
}

// ===== BLE Receive Handler =====

static void handle_ble_command(const char *data, uint16_t len)
{
    ESP_LOGI(TAG, "BLE command received: %s (len=%d)", data, len);

    if (len == 0) {
        return;
    }

    switch (data[0]) {
        case 'T':  // "TARE_" commands
            if(strcmp("TARE_PRODUCE_WEIGHT", data) == 0) {
                ESP_LOGI(TAG, "BLE Command: Taring produce load cell");
                load_cell_tare(produce_load_cell);
                break;
            }
            else if(strcmp("TARE_CART_WEIGHT", data) == 0) {
                ESP_LOGI(TAG, "BLE Command: Taring cart load cell");
                // load_cell_tare(cart_load_cell);
                break;
            }
        
        case 'M': // "MEASURE_" commands
            if(strcmp("MEASURE_PRODUCE_WEIGHT", data) == 0) {
                ESP_LOGI(TAG, "BLE Command: Measuring produce weight");
                float weight = load_cell_display_pounds(produce_load_cell);
                char weight_str[32];
                snprintf(weight_str, sizeof(weight_str), "%.4f", weight);
                ble_send_produce_weight(weight_str);
                break;
            }
            else if(strcmp("MEASURE_CART_WEIGHT", data) == 0) {
                ESP_LOGI(TAG, "BLE Command: Measuring cart weight");
                // float weight = load_cell_display_pounds(cart_load_cell);
                // char weight_str[32];
                // snprintf(weight_str, sizeof(weight_str), "[Cart Load Cell] Weight: %.4f", weight);
                // ble_send_misc_data(weight_str);
                break;
            }

        case 'P':  // Payment module activation
            ESP_LOGI(TAG, "BLE Command: Checking payment status - enabling payment module");
            payment_mode = true;
            ESP_LOGI(TAG, "Payment task: Waiting for card...");
            break;

        case 'C':  // Cart RFID:
            ESP_LOGI(TAG, "BLE Command: Triggering cart scan");
            cart_rfid_scan(cart_reader);
            break;

        case 'I': // "IMU_" commands
            if(strcmp("IMU_CHECK_ACTIVITY", data) == 0) {
                ESP_LOGI(TAG, "BLE Command: Checking IMU activity");
                if(icm20948_is_moving(&imu_sensor)) {
                    ESP_LOGI(TAG, "IMU reports: Cart is moving");
                    ble_send_misc_data("[IMU] MOVING");
                } else {
                    ESP_LOGI(TAG, "IMU reports: Cart is idle");
                    ble_send_misc_data("[IMU] IDLE");
                }
            }
            else if(strcmp("IMU_GET_HEADING", data) == 0) {
                ESP_LOGI(TAG, "BLE Command: Getting IMU heading");
                float heading = icm20948_compute_heading(&imu_sensor);
                char heading_str[32];
                snprintf(heading_str, sizeof(heading_str), "[IMU] HEADING:%.2f", heading);
                ble_send_misc_data(heading_str);
            }
            break;

        default:
            ESP_LOGW(TAG, "BLE Command: Unknown command '%c' (full: %s)", data[0], data);
            ble_send_barcode("ERR_UNKNOWN_CMD");
            break;
    }
}

// ===== IMU Idle Event Handler (called from main loop, not from timer context) =====

static void handle_imu_idle_event(void)
{
    ESP_LOGI(TAG, "⏱ IMU: Cart idle for 5 minutes - no motion detected");
    ble_send_misc_data("[IMU] IDLE");
}

// ===== Individual Setup Functions =====

static void i2c_setup(void)
{
    ESP_LOGI(TAG, "Initializing I2C bus...");
    i2c_master_bus_config_t i2c_bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
        },
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_handle));
    ESP_LOGI(TAG, "I2C master bus initialized");
}

static void ble_setup(void)
{
    ESP_LOGI(TAG, "Initializing BLE...");
    esp_err_t ble_ret = ble_init("Carte_Diem");
    if (ble_ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE initialization failed, but continuing...");
    } else {
        ESP_LOGI(TAG, "BLE barcode service initialized (NimBLE)");
        ble_register_rx_callback(handle_ble_command);
    }
}

static void barcode_setup(void)
{
    ESP_LOGI(TAG, "Initializing barcode scanner...");
    barcode_init(&scanner, UART_NUM_1, BARCODE_TX_PIN, BARCODE_RX_PIN, true);
    // Note: barcode_set_manual_mode is called within barcode_init
    ESP_LOGI(TAG, "Barcode scanner ready in manual mode");
}

static void button_setup(void)
{
    ESP_LOGI(TAG, "Initializing button...");
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);

    button_evt_queue = xQueueCreate(4, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr, NULL);
    ESP_LOGI(TAG, "Button ready on GPIO %d", BUTTON_PIN);
}

static void proximity_setup(void)
{
    ESP_LOGI(TAG, "Initializing proximity sensor...");
    proximity_sensor = proximity_sensor_create(PROXIMITY_INT_PIN, PROXIMITY_THRESHOLD, false);
    if (proximity_sensor == NULL || !proximity_sensor_begin(proximity_sensor, i2c_bus_handle)) {
        ESP_LOGE(TAG, "Failed to initialize proximity sensor");
        return;
    }
    proximity_sensor_enable_interrupt(proximity_sensor);
    ESP_LOGI(TAG, "Proximity sensor ready with threshold %d", PROXIMITY_THRESHOLD);
}

static void imu_setup(void){
    ESP_LOGI(TAG, "Initializing IMU...");
    icm20948_init(&imu_sensor, i2c_bus_handle);
    ESP_LOGI(TAG, "IMU initialized successfully");

    imu_idle_evt_queue = xQueueCreate(4, sizeof(uint32_t));
    if (imu_idle_evt_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU idle event queue");
        return;
    }

    // Start IMU monitor first
    // icm20948_start_activity_monitor(&imu_sensor, imu_idle_evt_queue);

    // Try accel read (but don't exit)
    if(icm20948_read_accel(&imu_sensor) != ESP_OK){
        ESP_LOGW(TAG, "IMU WARNING: accel read failed. System will still run but IMU will be inaccurate");
    } else {
        ESP_LOGI(TAG, "IMU accelerometer verified working");
    }

    // Create dedicated IMU monitoring task (1 second interval)
    xTaskCreate(icm20948_monitor_task, "imu_monitor", 2048, &imu_sensor, 5, NULL);
    ESP_LOGI(TAG, "IMU monitoring task created (5-minute idle timeout)");
}


static void proximity_interrupt_setup(void)
{
    ESP_LOGI(TAG, "Initializing proximity interrupt...");
    gpio_config_t prox_io_conf = {
        .pin_bit_mask = 1ULL << PROXIMITY_INT_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&prox_io_conf);

    proximity_evt_queue = xQueueCreate(4, sizeof(uint32_t));
    gpio_isr_handler_add(PROXIMITY_INT_PIN, proximity_isr, NULL);
    ESP_LOGI(TAG, "Proximity interrupt ready on GPIO %d", PROXIMITY_INT_PIN);
}

static void payment_setup(void)
{
    ESP_LOGI(TAG, "Initializing MFRC522 payment card reader...");
    esp_err_t mfrc_result = mfrc522_init(&paymenter, SPI2_HOST, MISO_PIN, MOSI_PIN, SCK_PIN, PAYMENT_CS_PIN, PAYMENT_RST_PIN);
    if (mfrc_result != ESP_OK) {
        ESP_LOGE(TAG, "MFRC522 initialization failed! Payment card reader will not work.");
    } else {
        ESP_LOGI(TAG, "MFRC522 initialized successfully!");
    }
}

static void produce_loadcell_setup(void)
{
    ESP_LOGI(TAG, "Initializing load cell...");
    produce_load_cell = load_cell_create(TOP_LOAD_CLK_PIN, TOP_LOAD_DATA_PIN, 25, false);
    load_cell_begin(produce_load_cell);
    load_cell_tare(produce_load_cell);
    ESP_LOGI(TAG, "Load cell initialized and tared.");
}

static void cart_loadcell_setup(void)
{
    ESP_LOGI(TAG, "Initializing load cell...");
    cart_load_cell = load_cell_create(BOTTOM_LOAD_CLK_PIN, BOTTOM_LOAD_DATA_PIN, 25, true);
    load_cell_begin(cart_load_cell);
    load_cell_tare(cart_load_cell);
    ESP_LOGI(TAG, "Load cell initialized and tared.");
}

static void cart_rfid_setup(void)
{
    ESP_LOGI(TAG, "Initializing cart RFID reader...");
    cart_reader = cart_rfid_init(
        CART_RFID_UART_PORT,
        CART_RFID_TX_PIN,
        CART_RFID_RX_PIN,
        on_cart_scan_complete
    );
    ESP_LOGI(TAG, "Cart RFID reader initialized");
}

static void setup(void)
{
    ESP_LOGI(TAG, "Starting system initialization...");

    // Initialize I2C bus first (used by proximity and IMU)
    i2c_setup();

    ble_setup();
    barcode_setup();
    button_setup();

    proximity_setup();
    proximity_interrupt_setup();
    imu_setup();
    payment_setup();
    produce_loadcell_setup();
    // cart_loadcell_setup();
    cart_rfid_setup();

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "System initialization complete");
    ESP_LOGI(TAG, "Ready: press button on GPIO %d to trigger scan or approach proximity sensor.", BUTTON_PIN);
    ESP_LOGI(TAG, "BLE Status: %s", ble_is_connected() ? "Connected" : "Waiting for connection...");
}

void app_main(void)
{
    setup();

    // --- Main task loop ---
    uint8_t uid[10], uid_len = 0;
    char buf[128];
    float produce_weight = 0;
    char *produce_weight_str = buf;
    uint8_t authorized_uid[] = {0x1A, 0x83, 0x26, 0x03, 0xBC};

    while (1)
    {
        uint32_t evt;

        // button press
        if (xQueueReceive(button_evt_queue, &evt, pdMS_TO_TICKS(10)))
        {
            if (!continuous_mode) {
                ESP_LOGI(TAG, "Button press detected → triggering manual scan");
                barcode_trigger_scan(&scanner);
            }
        }

        // proximity interrupt
        if (xQueueReceive(proximity_evt_queue, &evt, pdMS_TO_TICKS(10)))
        {
            uint8_t proximity_value = proximity_sensor_read(proximity_sensor);
            ESP_LOGI(TAG, "Proximity interrupt → value: %d", proximity_value);

            if (proximity_value > PROXIMITY_THRESHOLD && !continuous_mode) {
                ESP_LOGI(TAG, "Proximity threshold exceeded → switching to continuous scan mode");
                barcode_set_continuous_mode(&scanner);
                continuous_mode = true;
            }

            proximity_sensor_clear_interrupt(proximity_sensor);
        }

        // IMU idle for 5 minutes
        if (xQueueReceive(imu_idle_evt_queue, &evt, pdMS_TO_TICKS(10)))
        {
            handle_imu_idle_event();
        }

        // barcode reading
        if (barcode_read_line(&scanner, buf, sizeof(buf)))
        {
            ESP_LOGI(TAG, "Scanned: %s", buf);

            // Send barcode data over BLE
            if (ble_is_connected()) {
                esp_err_t send_ret = ble_send_barcode(buf);
                if (send_ret == ESP_OK) {
                    ESP_LOGI(TAG, "✓ Barcode sent via BLE");
                } else {
                    ESP_LOGW(TAG, "✗ Failed to send barcode via BLE");
                }
            } else {
                ESP_LOGW(TAG, "⚠ BLE not connected - barcode not sent");
            }

            if (continuous_mode) {
                ESP_LOGI(TAG, "Barcode read → switching back to manual scan mode");
                barcode_set_manual_mode(&scanner);
                continuous_mode = false;
            }
        }

        // Payment processing
        if (payment_mode) {

            if (mfrc522_read_uid(&paymenter, uid, &uid_len) == ESP_OK && uid_len > 0) {
                printf("[MAIN] Payment card detected: ");
                for (int i = 0; i < uid_len; i++) {
                    printf("%02X ", uid[i]);
                }
                printf("\n");

                bool match = (uid_len == 5);
                for (int i = 0; i < 5 && match; i++) {
                    if (uid[i] != authorized_uid[i]) match = false;
                }

                if (match) {
                    printf("💳 Payment Successful!\n");
                    ble_send_payment_status("1");
                } else {
                    printf("🚫 Payment Declined. Try another card.\n");
                    ble_send_payment_status("0");
                }

                payment_mode = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}