#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <freertos/FreeRTOS.h>

#include <driver/uart.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>

#define GPIO_CMD_MODE 0

#define UART_NUM_IRDA UART_NUM_2
#define GPIO_IRDA_RX 27
#define GPIO_IRDA_TX 14

#define UART_NUM_SERIAL UART_NUM_0
#define GPIO_SERIAL_RX UART_PIN_NO_CHANGE
#define GPIO_SERIAL_TX UART_PIN_NO_CHANGE

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

struct mcp2120_baudrate_map {
  uint8_t cmd;
  uint32_t baudrate;
};

struct mcp2120_baudrate_map baudrate_map[] = {
  { 0x87, 9600 },
  { 0x88, 19200 },
  { 0x85, 38400 },
  { 0x83, 57600 },
  { 0x81, 115200 },
  { 0, 0 }
};

#define MCP2120_CMD_COMMIT 0x11
volatile struct {
  volatile uint8_t cmd_mode:1;
  uint32_t next_baudrate;
} state;

static void IRAM_ATTR cmd_mode_isr(void* priv) {
  state.cmd_mode = gpio_get_level(GPIO_CMD_MODE) ? 0 : 1;
};

static void irda_set_tx_enable(uint8_t state) {
  switch(UART_NUM_IRDA) {
    case UART_NUM_0:
      UART0.conf0.irda_tx_en = state;
      break;
    case UART_NUM_1:
      UART1.conf0.irda_tx_en = state;
      break;
    case UART_NUM_2:
      UART2.conf0.irda_tx_en = state;
      break;
    default:
      ESP_ERROR_CHECK(ESP_FAIL);
  }
}

static inline void irda_tx_enable() {
  irda_set_tx_enable(1);
}

static inline void irda_tx_disable() {
  irda_set_tx_enable(0);
}

static inline size_t irda_bytes_available() {
  size_t bytes_available;
  ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_IRDA, &bytes_available));
  return bytes_available;
}

static inline size_t serial_bytes_available() {
  size_t bytes_available;
  ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_SERIAL, &bytes_available));
  return bytes_available;
}

static inline uint8_t serial_read_byte() {
  uint8_t byt;
  ESP_ERROR_CHECK(uart_read_bytes(UART_NUM_SERIAL, &byt, 1, portMAX_DELAY));
  return byt;
};

static inline void serial_write_byte(uint8_t byt) {
  ESP_ERROR_CHECK(uart_write_bytes(UART_NUM_SERIAL, (char*)&byt, 1));
};

static uint32_t find_baudrate(uint8_t cmd) {
  struct mcp2120_baudrate_map* entry = baudrate_map;
  while(entry->baudrate) {
    if(entry->cmd == cmd) {
      return entry->baudrate;
    }
  }
  return 0;
}

static uint8_t data_buf[256];
int app_main() {
  // Initialize cmd mode pin
  gpio_config_t cmd_mode_gpio_conf = {
    .intr_type = GPIO_PIN_INTR_ANYEDGE,
    .pin_bit_mask = 1ULL << GPIO_CMD_MODE,
    .mode         = GPIO_MODE_INPUT,
  };
  ESP_ERROR_CHECK(gpio_isr_register(cmd_mode_isr, NULL, 0, NULL));
  ESP_ERROR_CHECK(gpio_config(&cmd_mode_gpio_conf));

  // Initialize serials
  uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_SERIAL, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_SERIAL, GPIO_SERIAL_TX, GPIO_SERIAL_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_SERIAL, 256, 0, 0, NULL, 0));

  ESP_ERROR_CHECK(uart_param_config(UART_NUM_IRDA, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_IRDA, GPIO_IRDA_TX, GPIO_IRDA_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_IRDA, 256, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_set_mode(UART_NUM_IRDA, UART_MODE_IRDA));

  while(1) {
    size_t bytes_available;
    while((bytes_available = serial_bytes_available())) {
      if(state.cmd_mode) {
        ESP_LOGI("TX", "Cmd mode");
        uint8_t cmd = serial_read_byte();
        serial_write_byte(cmd);
        if(cmd == MCP2120_CMD_COMMIT && state.next_baudrate != 0) {
          ESP_LOGI("BAUD", "Setting new baudrate %u", state.next_baudrate);
          ESP_ERROR_CHECK(uart_set_baudrate(UART_NUM_IRDA, state.next_baudrate));
          ESP_ERROR_CHECK(uart_set_baudrate(UART_NUM_SERIAL, state.next_baudrate));
          continue;
        }
        state.next_baudrate = find_baudrate(cmd);
      } else {
        int res = uart_read_bytes(UART_NUM_SERIAL, data_buf, min(bytes_available, sizeof(data_buf)), 0);
        ESP_LOGI("TX", "Txing %d bytes", res);
        if(res < 0) {
          ESP_ERROR_CHECK(-res);
        }
        irda_tx_enable();
        uart_write_bytes(UART_NUM_IRDA, (char*)data_buf, res);
        irda_tx_disable();
      }
    }

    while((bytes_available = irda_bytes_available())) {
        int res = uart_read_bytes(UART_NUM_IRDA, data_buf, min(bytes_available, sizeof(data_buf)), 0);
        ESP_LOGI("RX", "Rxing %d bytes", res);
        if(res < 0) {
          ESP_ERROR_CHECK(-res);
        }
        uart_write_bytes(UART_NUM_SERIAL, (char*)data_buf, res);
    }
    vTaskDelay(1);
  }

  return 0;
}
