# Agent Guidelines

- Keep UI rendering and audio streaming responsive; avoid introducing long blocking sections on the loop task when adding new code.
- Prefer cooperative multitasking helpers (e.g. `vTaskDelay`, `taskYIELD`, or non-blocking waits) when adding work to FreeRTOS tasks so the radar and audio remain smooth.
- Maintain the separation between the radar render task, the audio streaming task, and the aircraft data fetch task. New work that touches those subsystems should respect their dedicated cores and synchronisation so the three services stay independent.
- No automated tests are available for this project; if checks cannot be run because they require hardware, note that clearly when reporting results.
