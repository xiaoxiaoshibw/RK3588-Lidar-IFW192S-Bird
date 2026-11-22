#include "replay_wire.hpp"

#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include <algorithm>

namespace {

constexpr int kRxBytes = 32768;
#if CONFIG_BIRD_USB_SERIAL_JTAG
constexpr int kUsbTxBytes = 4096;
#endif

bool read_exact(void* data, size_t size) {
    auto* next = static_cast<uint8_t*>(data);
    while (size > 0) {
#if CONFIG_BIRD_USB_SERIAL_JTAG
        const int got = usb_serial_jtag_read_bytes(next, size, portMAX_DELAY);
#else
        const int got = uart_read_bytes(UART_NUM_0, next, size, portMAX_DELAY);
#endif
        if (got <= 0) {
            return false;
        }
        next += got;
        size -= got;
    }
    return true;
}

bool write_reply(const char* data, size_t size) {
#if CONFIG_BIRD_USB_SERIAL_JTAG
    while (size > 0) {
        const size_t chunk = std::min<size_t>(size, kUsbTxBytes);
        if (usb_serial_jtag_write_bytes(data, chunk, pdMS_TO_TICKS(2000)) != static_cast<int>(chunk)) {
            return false;
        }
        data += chunk;
        size -= chunk;
    }
    return usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(2000)) == ESP_OK;
#else
    return uart_write_bytes(UART_NUM_0, data, size) == static_cast<int>(size);
#endif
}

}  // namespace

extern "C" void app_main() {
#if CONFIG_BIRD_USB_SERIAL_JTAG
    usb_serial_jtag_driver_config_t config{kUsbTxBytes, kRxBytes};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&config));
#else
    const uart_config_t config{
        .baud_rate = CONFIG_BIRD_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {},
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, kRxBytes, 0, 0, nullptr, 0));
#endif
    bird::run_replay({
        read_exact, write_reply, esp_timer_get_time,
        [](size_t size) { return heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); },
        esp_psram_get_size(),
    });
}
