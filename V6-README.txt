V6 — confirmed hardware: LEFT AO barcode-only; MIDDLE DO + RIGHT DO steering.

V5's left-sensor steering is removed. Do not use V5 for this layout.
LEFT AO GP26 feeds the original barcode decoder and cross-junction evidence;
it never supplies steering error. Middle DO is configured on GP7. Right DO
is sampled through the existing GP27 ADC path; its logic is unchanged.

Left rolling correction: after centre-only, when middle and right become
white, request +600 (the opposite of right-only's -600), through the existing
forward wheel-speed and PWM ramps. The left curve no longer switches into
a fan/pivot search after a timer. The previous right-side tracking/recovery
and v4 motor controller are retained. Positive/left curvature still eases
out when the middle sensor reacquires the line.

The original barcode decoder, adaptive threshold, ADC driver and barcode
pause/feed logic are unchanged. V3/V4 cross-junction fixes are retained.
A physical flat floor or a missing line cannot be distinguished from a
left bend by two white digital sensors alone; the existing line-lost stop
still triggers after 10 seconds of driving without middle/right detection.

Validation: 81 subsystem checks pass, plus motion simulations, right/left
line-task tests, 5-second left rolling correction, barcode-only isolation,
10-second line-lost fault, normal/skewed crosses, barcode then cross,
pivot then cross and weak AO barcode then cross. Controller mapped action,
centre landing, MQTT golden vector and sensor-bench tests pass.
No physical-car confirmation of smoothness has been performed.

Build: Pico W / RP2040, SMP=0 NET=lwip MQTT=1 TEST=0.
Pico SDK 2.2.0; xPack Arm GCC 15.2.1-1.1.
See README.md for SDK/submodule/toolchain setup.
Source ZIP omits personal config/wifi_secrets.h. Copy that file from your
original project (or copy config/wifi_secrets.example.h and fill in settings)
before rebuilding. Supplied UF2 preserves your existing Wi-Fi/MQTT settings.
Send/confirm your barcode mapping on the dashboard after flashing.
UF2 SHA256: b26bb9f20f9fe3d7589ac351b30a4e2517eda91770c19509a0a98c894de7d99f
