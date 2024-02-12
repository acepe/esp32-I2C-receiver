#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_SLAVE_SCL_IO     2                /*!< gpio number for I2C slave clock */
#define I2C_SLAVE_SDA_IO     1                /*!< gpio number for I2C slave data  */
#define I2C_SLAVE_ADDR       0x27             /*!< I2C slave address for this example */
#define I2C_SLAVE_NUM       I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_SLAVE_RX_BUF_LEN  1024             /*!< I2C slave buffer size for receive data */
#define I2C_SLAVE_TX_BUF_LEN  1024             /*!< I2C slave buffer size for send data */

static const char *TAG = "I2C_SLAVE";

/**
 * @brief Task to process I2C slave events
 */
static void i2c_slave_task(void *arg) {
    ESP_LOGI(TAG, "Task started");

    uint8_t *data = (uint8_t *)malloc(I2C_SLAVE_RX_BUF_LEN);
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for receive buffer");
        vTaskDelete(NULL);
    }

    int size = 0;
    while (1) {      
        size = i2c_slave_read_buffer(I2C_SLAVE_NUM, data, I2C_SLAVE_RX_BUF_LEN, 10 / portTICK_PERIOD_MS);
        if (size > 0) {
            // ESP_LOGI(TAG, "Received data:");
            ESP_LOG_BUFFER_CHAR(TAG, data, size);
        }
    }

    free(data);
    vTaskDelete(NULL);
}

void app_main() {
    i2c_config_t conf = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave = {
            .addr_10bit_en = 0,
            .slave_addr = I2C_SLAVE_ADDR
        }
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_SLAVE_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_SLAVE_NUM, conf.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0));

    xTaskCreate(i2c_slave_task, "i2c_slave_task", 4096, NULL, 10, NULL);
}
