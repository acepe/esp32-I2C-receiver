#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "i2c_slave";

#define I2C_SLAVE_SCL_IO           2        /*!< GPIO number for I2C slave clock  */
#define I2C_SLAVE_SDA_IO           1        /*!< GPIO number for I2C slave data   */
#define I2C_SLAVE_NUM              I2C_NUM_0 /*!< I2C slave port number for slave dev */
#define I2C_SLAVE_ADDR             0x27      /*!< I2C slave address for the slave device */
#define I2C_SLAVE_RX_BUF_LEN       1024      /*!< I2C slave buffer size */

uint8_t i2c_slave_rx_buffer[I2C_SLAVE_RX_BUF_LEN] = {0};

void i2c_slave_init() {
  i2c_config_t conf;
  conf.mode = I2C_MODE_SLAVE;
  conf.sda_io_num = I2C_SLAVE_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_SLAVE_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.slave.addr_10bit_en = 0;
  conf.slave.slave_addr = I2C_SLAVE_ADDR;
  conf.clk_flags = 0;

  i2c_param_config(I2C_SLAVE_NUM, &conf);
  i2c_driver_install(I2C_SLAVE_NUM, conf.mode, I2C_SLAVE_RX_BUF_LEN, 0, 0);
  i2c_reset_rx_fifo(I2C_SLAVE_NUM);
}

void i2c_slave_receive_hello() {
  int len = i2c_slave_read_buffer(
      I2C_SLAVE_NUM, i2c_slave_rx_buffer, sizeof(i2c_slave_rx_buffer),
      500 / portTICK_PERIOD_MS);  // Use a timeout of 1000 ms
  
  if (len > 0) {
    ESP_LOGI(TAG, "Received message from master: %.*s", len,
             i2c_slave_rx_buffer);
  } else {
    ESP_LOGI(TAG, "No message received from master.");
  }
}



void app_main() {
    ESP_LOGI(TAG, "I2C Slave Task Started");
    i2c_slave_init();
    while (1) {
        i2c_slave_receive_hello();
    }
}
