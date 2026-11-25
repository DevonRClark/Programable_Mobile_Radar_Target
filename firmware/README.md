# Firmware for adjustable return speed

This package contains the two matched sketches for the cart
(onboard-log-33) and dongle (onboard-log-29). It adds return to home up
to 2 m/s. The desktop app enables fast returns when both boards advertise support.

## Upload on Windows

Close the cart UI before uploading and disconnect motor power for the firmware
checks. Use the Espressif Arduino ESP32 board package 3.3.12. Open and upload the sketches separately.

| Board | Sketch | Arduino IDE settings |
| --- | --- | --- |
| Cart ESP32 | `CartStaged/CartStaged.ino` | ESP32 Dev Module |
| USB dongle ESP32-C3 | `Dongle/Dongle.ino` | ESP32C3 Dev Module, USB CDC On Boot enabled |

Keep each sketch folder and its `src` folder together. If your working sketches
have different peer addresses or hardware settings, copy those settings into
these sketches before uploading. The cart's peer address is in
`CartStaged/src/Settings.h` and the dongle's is in `Dongle/Dongle.ino`.

Reconnect the UI with motor power still disconnected. Verify that it receives
cart status and acknowledges STOP. You can exercise the interface
and inspect the setting in Demo mode as well.


