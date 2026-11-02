# Programmable Mobile Radar Target

Devon Clark · BSc Engineering (Mechatronics) · University of Cape Town · EEE4022S · 2026

I designed and built a track-guided cart to carry a lightweight radar target repeatably along a known path. By programming the cart's speed, an operator can vary the target's motion between runs while keeping the track geometry fixed. This was my individual final-year engineering project.

## How it works

An ESP32 on the cart reads wheel pulses, estimates speed and runs a PI controller. It sends throttle commands to the electronic speed controller. A second board, an ESP32-C3 connected to the operator's computer by USB, exchanges commands and telemetry with the cart over ESP-NOW. The firmware supports programmed velocity profiles and onboard run logs, so the requested motion can be compared with what the cart measured.

## Firmware

| Board | Sketch | Role |
| --- | --- | --- |
| Cart ESP32 | [CartStaged.ino](firmware/CartStaged/CartStaged.ino) | Speed control, wheel and track sensing, profile execution and logging |
| USB dongle ESP32-C3 | [Dongle.ino](firmware/Dongle/Dongle.ino) | Serial command interface, wireless telemetry and log retrieval |


## Mechanical CAD

[Open the cart and track CAD in Onshape](https://cad.onshape.com/documents/64ca6ddc38676766799ed6b8/w/bc4ebbca6c69d7bca39da27d/e/f28f6204cea985993fedeecc)
