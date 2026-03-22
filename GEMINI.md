# GEMINI.md - Project Context

## Project Overview
This is an embedded systems project for the **PES Board**, a custom robotics hardware platform used at ZHAW (Zurich University of Applied Sciences). The project is built on **Mbed OS 6** and targets the **ST Nucleo-F446RE** (STM32F446RE) microcontroller.

The codebase provides a modular driver library for various sensors and actuators, including DC motors with encoders, servos, ultrasonic sensors, infrared distance sensors, and an IMU.

### Key Technologies & Frameworks
- **Framework:** Mbed OS 6 (C++)
- **Microcontroller:** ST Nucleo-F446RE
- **Build Systems:** PlatformIO (recommended) or Mbed Studio
- **Core Libraries:** 
  - `DCMotor`: Velocity and rotation control for brushed DC motors.
  - `EncoderCounter`: Hardware-timer based quadrature encoder reading.
  - `FastPWM`: High-resolution PWM output.
  - `IMU/LSM9DS1`: Driver for the 9-axis motion sensor.
  - `SerialStream`: High-speed (2 Mbaud) data streaming to MATLAB/Python.
  - `Eigen`: Linear algebra library for kinematics and filtering.

## Building and Running

### PlatformIO (VS Code)
- **Build:** `pio run`
- **Upload:** `pio run -t upload`
- **Serial Monitor:** `pio device monitor` (Default speed: 115200)

### Mbed Studio
- **Build/Flash:** Use the GUI buttons (**Hammer** for build, **Play** for flash).
- **Profile:** Typically uses the **Develop** profile.

### Manual Flashing
- The compiled `.bin` file is located in the `BUILD/` directory.
- Flash by dragging and dropping the `.bin` file onto the `NODE_F446RE` drive mapped to your computer.

## Development Conventions

### Pin Mapping
All hardware pin assignments must be defined in `include/PESBoardPinMap.h`. Never hardcode pin names (e.g., `PA_0`) directly in drivers or `main.cpp`; use the `PB_` prefixed aliases (e.g., `PB_PWM_M1`).

### Serial Communication
- **Standard Debugging:** 115,200 baud (defined in `mbed_app.json`).
- **High-Speed Streaming:** 2,000,000 baud (used by `SerialStream` library).
- **Floating Point Support:** Requires `-Wl,-u,_printf_float` in linker flags (configured in `platformio.ini`).

### Time & Concurrency
- **Mbed OS 6:** Uses `std::chrono` for all time-related functions (e.g., `milliseconds`, `microseconds`).
- **Main Loop:** `src/main.cpp` typically implements a non-blocking periodic loop using a `Timer` and `thread_sleep_for`. The default period is 20ms (50Hz).
- **Drivers:** Often utilize their own `Thread` or `Ticker` for high-frequency control tasks (e.g., `DCMotor` runs a control loop at 2kHz).

### Coding Style
- Follow the existing convention of using descriptive variable names (e.g., `gear_ratio_M1`, `voltage_max`).
- Keep `main.cpp` organized with sections for object instantiation, configuration, and the periodic state machine/loop.
- Use `DigitalOut enable_motors(PB_ENABLE_DCMOTORS)` to control the power stage of the DC motors.

## Directory Structure
- `src/`: Main application logic (`main.cpp`).
- `lib/`: Modular hardware drivers and filters.
- `include/`: Centralized headers like `PESBoardPinMap.h`.
- `docs/`: Extensive documentation and tutorials in Markdown.
- `test/`: Project-specific test files.
