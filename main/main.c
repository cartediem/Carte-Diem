#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "interfaces/barcode.h"
#include "interfaces/proximity_sensor.h"

#define SCL_PIN GPIO_NUM_9
#define SDA_PIN GPIO_NUM_8
#define BARCODE_TX_PIN GPIO_NUM_38
#define BARCODE_RX_PIN GPIO_NUM_39

#define BUTTON_PIN GPIO_NUM_45
#define PROXIMITY_INT_PIN GPIO_NUM_47
#define PROXIMITY_THRESHOLD 30
#define TAG "MAIN"

static barcode_t scanner;
static ProximitySensor* proximity_sensor = NULL;
static QueueHandle_t button_evt_queue = NULL;
static QueueHandle_t proximity_evt_queue = NULL;
static bool continuous_mode = false;

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
    // ESP_EARLY_LOGI(TAG, "Proximity interrupt triggered");
}


void app_main(void)
{
    barcode_init(&scanner, UART_NUM_1, BARCODE_TX_PIN, BARCODE_RX_PIN, true);
    barcode_set_manual_mode(&scanner);
    ESP_LOGI(TAG, "Barcode scanner ready in manual mode");

    proximity_sensor = proximity_sensor_create(PROXIMITY_INT_PIN, PROXIMITY_THRESHOLD, false);
    if (proximity_sensor == NULL || !proximity_sensor_begin(proximity_sensor, I2C_NUM_0, SDA_PIN, SCL_PIN, 100000)) {
        ESP_LOGE(TAG, "Failed to initialize proximity sensor");
        return;
    }
    proximity_sensor_enable_interrupt(proximity_sensor);
    ESP_LOGI(TAG, "Proximity sensor ready with threshold %d", PROXIMITY_THRESHOLD);

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE, 
    };
    gpio_config(&io_conf);

    gpio_config_t prox_io_conf = {
        .pin_bit_mask = 1ULL << PROXIMITY_INT_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE, 
    };
    gpio_config(&prox_io_conf);

    button_evt_queue = xQueueCreate(4, sizeof(uint32_t));
    proximity_evt_queue = xQueueCreate(4, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr, NULL);
    gpio_isr_handler_add(PROXIMITY_INT_PIN, proximity_isr, NULL);

    ESP_LOGI(TAG, "Ready: press button on GPIO %d to trigger scan or approach proximity sensor.", BUTTON_PIN);

    // --- Main task loop ---
    char buf[128];
    while (1)
    {
        uint32_t evt;
        
        if (xQueueReceive(button_evt_queue, &evt, pdMS_TO_TICKS(10)))
        {
            if (!continuous_mode) {
                ESP_LOGI(TAG, "Button press detected → triggering manual scan");
                barcode_trigger_scan(&scanner);
            }
        }

        if (xQueueReceive(proximity_evt_queue, &evt, pdMS_TO_TICKS(10)))
        {
            uint8_t proximity_value = proximity_sensor_read(proximity_sensor);
            // ESP_LOGI(TAG, "Proximity interrupt → value: %d", proximity_value);

            if (proximity_value > PROXIMITY_THRESHOLD && !continuous_mode) {
                ESP_LOGI(TAG, "Proximity threshold exceeded → switching to continuous scan mode");
                barcode_set_continuous_mode(&scanner);
                continuous_mode = true;
            }

            proximity_sensor_clear_interrupt(proximity_sensor);
        }

        // ESP_LOGI(TAG, "Proximity: %d", proximity_sensor_read(proximity_sensor));

        if (barcode_read_line(&scanner, buf, sizeof(buf)))
        {
            ESP_LOGI(TAG, "Scanned: %s", buf);
            
            if (continuous_mode) {
                ESP_LOGI(TAG, "Barcode read → switching back to manual scan mode");
                barcode_set_manual_mode(&scanner);
                continuous_mode = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
