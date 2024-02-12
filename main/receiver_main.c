#include <stdio.h>
#include <stdlib.h>

#include "class/hid/hid_device.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"

#define I2C_SLAVE_SCL_IO 2      /*!< gpio number for I2C slave clock */
#define I2C_SLAVE_SDA_IO 1      /*!< gpio number for I2C slave data  */
#define I2C_SLAVE_ADDR 0x27     /*!< I2C slave address for this example */
#define I2C_SLAVE_NUM I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_SLAVE_RX_BUF_LEN                                             \
  1024                            /*!< I2C slave buffer size for receive \
                                     data*/
#define I2C_SLAVE_TX_BUF_LEN 1024 /*!< I2C slave buffer size for send data */

static const char *TAG = "I2C_SLAVE";

/************* TinyUSB descriptors ****************/

#define TUSB_DESC_TOTAL_LEN \
  (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

/**
 * @brief HID report descriptor
 *
 * In this example we implement Keyboard + Mouse HID device,
 * so we must define both report descriptors
 */
const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD))};

/**
 * @brief String descriptor
 */
const char *hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},     // 0: is supported language is English (0x0409),
                              // 0x0407 is for german
    "TinyUSB",                // 1: Manufacturer
    "TinyUSB Device",         // 2: Product
    "123456",                 // 3: Serials, should use chip ID
    "Example HID interface",  // 4: HID
};

/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and 1
 * HID interface
 */
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length,
    // attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP
    // In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16,
                       10),
};

/********* TinyUSB HID callbacks ***************/

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long
// enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
  // We use only one interface and one HID report descriptor, so we can ignore
  // parameter 'instance'
  return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
  (void)instance;
  (void)report_id;
  (void)report_type;
  (void)buffer;
  (void)reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize) {}

static void app_send_hid_demo(uint8_t *keycode) {
  ESP_LOGI(TAG, "Sending Keyboard report");

  tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode);
  vTaskDelay(pdMS_TO_TICKS(10));

  tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);
  vTaskDelay(pdMS_TO_TICKS(10));
}

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
    size = i2c_slave_read_buffer(I2C_SLAVE_NUM, data, I2C_SLAVE_RX_BUF_LEN,
                                 10 / portTICK_PERIOD_MS);
    if (size > 0) {
      ESP_LOGI(TAG, "Received data: %d", size);
      ESP_LOG_BUFFER_HEX(TAG, data, size);

      if (tud_mounted()) {
        app_send_hid_demo(data);
      }
    }
  }

  free(data);
  vTaskDelete(NULL);
}

void initI2C() {
  i2c_config_t conf = {
      .mode = I2C_MODE_SLAVE,
      .sda_io_num = I2C_SLAVE_SDA_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_io_num = I2C_SLAVE_SCL_IO,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .slave = {.addr_10bit_en = 0, .slave_addr = I2C_SLAVE_ADDR}};

  ESP_ERROR_CHECK(i2c_param_config(I2C_SLAVE_NUM, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(
      I2C_SLAVE_NUM, conf.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0));
}

void initUSB() {
  ESP_LOGI(TAG, "USB initialization");
  const tinyusb_config_t tusb_cfg = {
      .device_descriptor = NULL,
      .string_descriptor = hid_string_descriptor,
      .string_descriptor_count =
          sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
      .external_phy = false,
      .configuration_descriptor = hid_configuration_descriptor,
  };

  ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
  ESP_LOGI(TAG, "USB initialization DONE");
}

void app_main() {
  initUSB();
  initI2C();

  xTaskCreate(i2c_slave_task, "i2c_slave_task", 4096, NULL, 10, NULL);
}
