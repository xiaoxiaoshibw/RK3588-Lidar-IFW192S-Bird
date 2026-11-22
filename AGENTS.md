# Repository Guidelines

- `firmware/esp32s3_ifw192s/` contains the ESP-IDF IFW192S/W5500 firmware. `firmware/esp32s3_bird_core/` contains the live tracker. `firmware/esp32s3_bird/` contains the earlier serial replay firmware and host protocol test.
- `deploy/s3_rk3588/` contains the serial-to-HTTP bridge and systemd unit. `web/` contains the viewer and offline replay tools. `bird_audit/raw_capture.py` reads and writes BIRDRAW captures. `tests/` contains the Python regression tests.
- From the repository root, run `python -m pip install -r requirements.txt` and `python -B -W error -m unittest discover -s tests -v`. See the firmware and deployment READMEs for build commands and hardware prerequisites.
- Use four-space Python indentation, `snake_case` functions, and `PascalCase` classes. Follow the surrounding C++ and JavaScript styles. Add regression coverage for changes to tracking, replay, the API or the viewer.
- Keep sensor subscriptions read-only and reuse the shared connection for recording. Do not change board services implicitly. Keep recordings, generated outputs, firmware binaries and device-specific calibration under ignored local paths. Preserve current measured-point provenance and `UNKNOWN` states.
- Use a fresh output filename when replaying or capturing. Synthetic tests and same-scene replay do not establish independent field performance.
- Use imperative commits with prefixes such as `refactor:` or `chore:`. Summarize behavior, prerequisites and verification in pull requests. Include screenshots for visible UI changes.

## Multi-Model Collaboration Workflow

- Keep frame-by-frame results deterministic, explainable from geometry, and traceable to measurements.
- Delegate repetitive verification, standard replay, tests, rendering, and aggregation to Gemini 4 Hydrogen.
- Delegate complex algorithmic diagnosis and decisions about P0/P1/P2 to Claude Opus 5.5.
