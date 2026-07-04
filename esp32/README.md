# `ts_link` — ESP32 (ESP-IDF) side of the STM32 motor-controller UART link

Drop-in ESP-IDF component that talks to the STM32 (`Src/esp_link.c`) over UART using
the **same wire format** (SLIP framing + CRC16, shared `protocol/trainsonic_link.h`), so
neither side needs changing.

## Install
1. Copy `esp32/ts_link/` into your ESP-IDF project's `components/` folder.
2. Make the shared header reachable (see `CMakeLists.txt`): either add the STM32 repo's
   `protocol/` dir to `INCLUDE_DIRS`, or copy `trainsonic_link.h` next to `ts_link.c`
   (prefer a submodule/shared path to avoid drift).

## Wire it
- STM32 is on **USART2 = PA2 (TX) / PA3 (RX)**. Connect ESP32-TX → PA3, ESP32-RX → PA2,
  common GND. Use any ESP32 UART **except UART0** (console).
- Match the baud (STM32 is 115200 today).

## Use it
```c
#include "ts_link.h"

void app_main(void) {
    ts_link_init(UART_NUM_1, /*tx*/17, /*rx*/16, 115200);  // pick your pins
    ts_link_send_hello();                                   // version handshake

    while (1) {
        // 50 Hz control: keep-alive + live setpoint (fire-and-forget)
        ts_link_send_heartbeat(/*torque_mA*/ my_torque, /*speed_rpm*/ 0);

        // read the full motor telemetry any time:
        tsl_status_t s;
        if (ts_link_get_status(&s)) {
            // s.speed_rpm, s.torque_mA, s.id_mA, s.state, s.fault_flags ...
        }
        if (!ts_link_up()) { /* link down -> UI warning; STM32 self-safes */ }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
```
Reliable commands: `ts_link_set_mode()`, `ts_link_enable()`, `ts_link_estop()`,
`ts_link_ack_fault()`. (These are sent once today; the reliable ACK/retransmit layer is
the pending decision — see `ESP_LINK_STATUS.md`.)

## Health
`ts_link_age_ms()`, `ts_link_up()`, `ts_link_rx_frames()`, `ts_link_crc_errors()`,
`ts_link_peer_version()` for link diagnostics on the UI.

> Not compiled in this repo (needs the ESP-IDF toolchain). Written against ESP-IDF v5.x
> UART APIs; build it in your ESP32 project.
