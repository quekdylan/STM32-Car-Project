# STM32 Firmware Documentation

This document provides a comprehensive overview of the STM32 firmware for the MDP Group 19 robot, including tunable parameters and important file descriptions.

## Table of Contents
- [Tunable Parameters](#tunable-parameters)
- [Important Files Overview](#important-files-overview)
- [Notes and Guidelines](#notes-and-guidelines)

## Tunable Parameters

The following table lists all configurable parameters in the codebase:

| Parameter | Description | Location | Default Value |
|-----------|-------------|----------|---------------|
| **PID Control** |
| `PID_KP` | Proportional gain for speed control PID | `MDP/Core/Src/control.c:20` | `2.0f` |
| `PID_KI` | Integral gain for speed control PID | `MDP/Core/Src/control.c:21` | `0.5f` |
| `PID_KD` | Derivative gain for speed control PID | `MDP/Core/Src/control.c:22` | `0.01f` |
| `PID_MAX_OUT` | Maximum PID output (PWM percent) | `MDP/Core/Src/control.c:14` | `100.0f` |
| `PID_MAX_IOUT` | Integral output clamp to prevent windup | `MDP/Core/Src/control.c:15` | `40.0f` |
| `PID_MAX_DOUT` | Derivative output clamp to reduce noise | `MDP/Core/Src/control.c:16` | `40.0f` |
| `CONTROL_DT_S` | PID control loop period (seconds) | `MDP/Core/Src/control.c:11` | `0.01f` |
| **Motor Control** |
| `MIN_MOTOR_SPEED_PERCENT` | Minimum PWM to overcome motor stiction | `MDP/Core/Src/motor.c:13` | `54.0f` |
| **Movement System** |
| `V_MAX_TICKS_PER_DT` | Maximum speed during cruise (ticks/10ms) | `MDP/Core/Src/movements.c:22` | `24.0f` |
| `V_MIN_TICKS_PER_DT` | Minimum speed to overcome friction | `MDP/Core/Src/movements.c:23` | `4.0f` |
| `YAW_KP_TICKS_PER_DEG` | Heading correction gain | `MDP/Core/Src/movements.c:29` | `3.0f` |
| `g_accel_dist_cm` | Acceleration distance (cm) | `MDP/Core/Src/movements.c:16` | `10.0f` |
| `g_decel_dist_cm` | Deceleration distance (cm) | `MDP/Core/Src/movements.c:17` | `15.0f` |
| **Turn Control** |
| `TURN_BASE_TICKS_PER_DT` | Base speed for turns (ticks/10ms) | `MDP/Core/Src/movements.c:33` | `12` |
| `TURN_YAW_TARGET_DEG` | Target turn angle (degrees) | `MDP/Core/Src/movements.c:34` | `90.0f` |
| `TURN_YAW_TOLERANCE_DEG` | Turn completion tolerance | `MDP/Core/Src/movements.c:35` | `0.5f` |
| `TURN_YAW_SLOW_BAND_DEG` | Deceleration zone for turns | `MDP/Core/Src/movements.c:36` | `10.0f` |
| `TURN_MIN_TICKS_PER_DT` | Minimum turn speed | `MDP/Core/Src/movements.c:37` | `4` |
| `g_turn_spinup_ticks` | Turn startup delay (100Hz ticks) | `MDP/Core/Src/movements.c:39` | `100` |
| **Robot Geometry** |
| `WHEELBASE_CM` | Distance between front and rear axles | `MDP/Core/Src/movements.c:42` | `14.5f` |
| `TRACK_WIDTH_CM` | Distance between left and right wheels | `MDP/Core/Src/movements.c:43` | `16.5f` |
| `STEER_MAX_DEG` | Maximum steering angle | `MDP/Core/Src/movements.c:45` | `30.0f` |
| `STEER_ANGLE_MAG` | Servo command magnitude for turns | `MDP/Core/Src/movements.c:48` | `100.0f` |
| `WHEEL_CIRC_CM` | Wheel circumference | `MDP/Core/Src/movements.c:55` | `20.73f` |
| `COUNTS_PER_REV` | Encoder counts per wheel revolution | `MDP/Core/Src/movements.c:56` | `1560.0f` |
| **IMU Settings** |
| `IMU_DT_S` | IMU update period (seconds) | `MDP/Core/Src/imu.c:34` | `0.01f` |
| `GZ_DEADBAND_DPS` | Gyro noise deadband (deg/s) | `MDP/Core/Src/imu.c:37` | `0.4f` |
| **AHRS Filter** |
| `s_beta` | Madgwick filter gain (convergence rate) | `MDP/Core/Src/madgwick.c:8` | `0.1f` |
| `s_dt` | Madgwick sample period (seconds) | `MDP/Core/Src/madgwick.c:9` | `0.01f` |
| **Sensor Configuration** |
| `SPEED_OF_SOUND_CM_PER_US` | Sound speed for ultrasonic sensor | `MDP/Core/Src/sensor.c:12` | `0.0343f` |
| **Timer Configurations** |
| TIM5 Prescaler | Control loop timer prescaler | `MDP/Core/Src/main.c:513` | `1599` |
| TIM5 Period | Control loop timer period | `MDP/Core/Src/main.c:515` | `99` |
| TIM4 Period | Left motor PWM period | `MDP/Core/Src/main.c:453` | `799` |
| TIM9 Period | Right motor PWM period | `MDP/Core/Src/main.c:635` | `799` |
| TIM8 Prescaler | Servo PWM prescaler | `MDP/Core/Src/main.c:560` | `15` |
| TIM8 Period | Servo PWM period | `MDP/Core/Src/main.c:562` | `19999` |
| **Servo Configuration** |
| Servo Min Pulse | Minimum servo pulse width (μs) | `MDP/Core/Src/main.c:835` | `1050` |
| Servo Center Pulse | Center servo pulse width (μs) | `MDP/Core/Src/main.c:835` | `1500` |
| Servo Max Pulse | Maximum servo pulse width (μs) | `MDP/Core/Src/main.c:835` | `2600` |
| **OLED Display** |
| OLED Update Delay | Display refresh period (ms) | Various tasks in `main.c` | `100` |
| **FreeRTOS Tasks** |
| Default Task Stack | Default task stack size (words) | `MDP/Core/Src/main.c:72` | `512` |
| Motor Task Stack | Motor task stack size (words) | `MDP/Core/Src/main.c:79` | `512` |
| Encoder Task Stack | Encoder display task stack size | `MDP/Core/Src/main.c:86` | `512` |
| Default Task Priority | Main task priority level | `MDP/Core/Src/main.c:73` | `osPriorityNormal` |
| Motor Task Priority | Motor control task priority | `MDP/Core/Src/main.c:80` | `osPriorityAboveNormal` |
| Encoder Task Priority | Display task priority | `MDP/Core/Src/main.c:87` | `osPriorityLow` |

## Important Files Overview

### Core Control Files

#### `MDP/Core/Src/control.c`
**Purpose**: Implements the main speed control system using PID controllers for both wheels.

**Key Functions**:
- `control_init()`: Initializes PID controllers with configured gains and limits
- `control_tick()`: Main control loop called at 100 Hz, reads encoders and applies PID control
- `control_set_target_ticks_per_dt()`: Sets target speeds in encoder ticks per 10ms
- `HAL_TIM_PeriodElapsedCallback()`: Timer interrupt handler that triggers control loop

**Important Features**:
- Dual PID controllers for independent left/right wheel speed control
- Encoder-based speed measurement using differential tick counting
- Configurable gains, output limits, and anti-windup protection

#### `MDP/Core/Src/pid.c`
**Purpose**: Generic PID controller implementation with time-step awareness.

**Key Functions**:
- `PID_init_with_dt()`: Initializes PID structure with gains, limits, and timestep
- `PID_calc_with_dt()`: Computes PID output with proper time scaling
- `PID_clear()`: Resets PID internal state
- `PID_SPEED_1()`, `PID_SPEED_2()`, `PID_SPEED_T()`: Specialized PID functions for different speed modes

**Important Features**:
- Time-step compensated integral and derivative terms
- Configurable output clamping for each PID component
- Support for multiple PID instances

### Motor and Actuation Files

#### `MDP/Core/Src/motor.c`
**Purpose**: Low-level motor control with PWM generation and encoder reading.

**Key Functions**:
- `motor_init()`: Initializes PWM channels and encoder timers
- `motor_set_speeds()`: Sets motor speeds with deadzone compensation
- `motor_stop()`: Immediately stops both motors
- `motor_brake_ms()`: Applies active braking for specified duration
- `motor_get_left_encoder_counts()`, `motor_get_right_encoder_counts()`: Read raw encoder values
- `motor_reset_encoders()`: Zeros encoder counters

**Important Features**:
- Automatic PWM scaling to overcome motor deadzone
- Independent left/right motor control
- Support for forward/reverse operation with proper H-bridge control

#### `MDP/Core/Src/servo.c`
**Purpose**: Servo motor control for steering system.

**Key Functions**:
- `Servo_Attach()`: Associates servo with timer channel and pulse width limits
- `Servo_Start()`, `Servo_Stop()`: Enable/disable servo PWM output
- `Servo_WriteUS()`: Sets servo position by pulse width in microseconds
- `Servo_WriteAngle()`: Sets servo position by angle (-100 to +100)
- `Servo_Center()`: Moves servo to center position

**Important Features**:
- Flexible pulse width mapping for different servo types
- Angle-based interface for intuitive steering control
- Automatic pulse width clamping for safety

### Navigation and Movement Files

#### `MDP/Core/Src/movements.c`
**Purpose**: High-level movement primitives with trajectory planning and IMU feedback.

**Key Functions**:
- `move_start_straight()`: Initiates straight-line movement with trapezoidal velocity profile
- `move_start_turn()`: Initiates Ackermann steering turns with IMU feedback
- `move_tick_100Hz()`: Main movement state machine, called at 100 Hz
- `move_abort()`: Emergency stop function
- `move_is_active()`: Returns movement status

**Important Features**:
- Trapezoidal velocity profiles for smooth acceleration/deceleration
- IMU-based heading correction for straight-line driving
- Ackermann steering model for realistic turning behavior
- Configurable motion parameters for different robot configurations

#### `MDP/Core/Src/imu.c`
**Purpose**: IMU sensor interface with gyroscope integration and magnetometer support.

**Key Functions**:
- `imu_init()`: Initializes IMU sensor and calibrates gyroscope bias
- `imu_update_yaw_100Hz()`: Integrates gyroscope data to estimate heading
- `imu_get_yaw()`: Returns current heading estimate
- `imu_zero_yaw()`: Resets heading reference
- `imu_calibrate_mag_*()`: Magnetometer calibration functions

**Important Features**:
- Automatic gyroscope bias calibration on startup
- Continuous heading integration with wraparound handling
- Magnetometer calibration support for absolute heading
- Noise filtering and deadband processing

### Sensor and I/O Files

#### `MDP/Core/Src/sensor.c`
**Purpose**: Ultrasonic distance sensor interface using input capture.

**Key Functions**:
- `ultrasonic_init_ex()`: Configures ultrasonic sensor on specified timer
- `ultrasonic_trigger()`: Initiates distance measurement
- `ultrasonic_get_distance_cm()`: Returns measured distance
- `HAL_TIM_IC_CaptureCallback()`: Input capture interrupt handler

**Important Features**:
- Non-blocking distance measurement using timer input capture
- Automatic echo pulse width calculation
- Configurable trigger GPIO and capture timer

#### `MDP/Core/Src/oled.c`
**Purpose**: OLED display driver for status information and debugging.

**Key Functions**:
- `OLED_Init()`: Initializes OLED display hardware
- `OLED_Clear()`: Clears display buffer
- `OLED_ShowString()`, `OLED_ShowNumber()`: Display text and numbers
- `OLED_Refresh_Gram()`: Updates physical display from buffer

**Important Features**:
- Buffered display updates for smooth rendering
- Text and numeric display functions
- SPI-based communication with display controller

#### `MDP/Core/Src/userButton.c`
**Purpose**: Simple user button interface for manual control.

**Key Functions**:
- `user_is_pressed()`: Returns current button state

**Important Features**:
- Debounced button reading
- Used for manual movement control and testing

#### `MDP/Core/Src/ICM20948.c`
**Purpose**: Low-level driver for ICM20948 9-DOF IMU sensor.

**Key Functions**:
- `ICM20948_Init()`: Initializes the ICM20948 sensor over I2C
- `ICM20948_ReadGyro()`: Reads raw gyroscope data (deg/s)
- `ICM20948_ReadAccel()`: Reads raw accelerometer data (g)
- `ICM20948_ReadMag()`: Reads magnetometer data (μT)
- `ICM20948_SetBank()`: Switches register banks for configuration

**Important Features**:
- I2C communication with configurable address (0x68/0x69)
- Raw sensor data conversion to engineering units
- Multi-bank register interface for full feature access
- Configurable sample rates and sensitivity ranges

#### `MDP/Core/Src/madgwick.c`
**Purpose**: Madgwick AHRS algorithm for sensor fusion and orientation estimation.

**Key Functions**:
- `madgwick_init()`: Initializes filter with sample rate and gain
- `madgwick_update_imu()`: Updates orientation using gyro/accel data
- `madgwick_get_euler()`: Returns roll, pitch, yaw angles
- `madgwick_get_quaternion()`: Returns quaternion representation

**Important Features**:
- Gradient descent algorithm for orientation estimation
- Gyroscope drift compensation using accelerometer
- Configurable filter gain (`s_beta = 0.1f`) for tuning convergence vs. noise
- Quaternion-based calculations for singularity-free rotation handling

**Tunable Parameters**:
- `s_beta`: Algorithm gain (higher = faster convergence, more noise sensitivity)
- `s_dt`: Sample period in seconds (should match IMU update rate)

### System Files

#### `MDP/Core/Src/main.c`
**Purpose**: Main application entry point with hardware initialization and FreeRTOS tasks.

**Key Functions**:
- `main()`: System initialization and RTOS startup
- `SystemClock_Config()`: Clock configuration
- Timer initialization functions (`MX_TIM*_Init()`)
- FreeRTOS task functions for different subsystems

**Important Features**:
- Complete hardware peripheral initialization
- FreeRTOS task definitions for concurrent operation
- Timer configurations for PWM, encoders, and control loops
- Servo initialization with calibrated pulse widths

## Notes and Guidelines

### Units and Scaling
- **Speed targets**: Expressed as encoder ticks per 10ms control period
- **PWM outputs**: Percentage values from -100 to +100
- **Distances**: Centimeters for movement commands
- **Angles**: Degrees for heading and steering

### Tuning Recommendations

#### PID Controller Tuning
1. Start with conservative gains: `Kp=2.0`, `Ki=0.5`, `Kd=0.01`
2. Increase `Kp` until system oscillates, then reduce by 20-30%
3. Add `Ki` to eliminate steady-state error (watch for integral windup)
4. Add small `Kd` to reduce overshoot (watch for noise amplification)
5. Adjust `PID_MAX_IOUT` and `PID_MAX_DOUT` to prevent excessive integral/derivative action

#### Movement System Tuning
1. Calibrate `WHEEL_CIRC_CM` and `COUNTS_PER_REV` for accurate distance measurement
2. Adjust `V_MAX_TICKS_PER_DT` and `V_MIN_TICKS_PER_DT` based on motor characteristics
3. Tune `YAW_KP_TICKS_PER_DEG` for straight-line tracking (start low to avoid oscillation)
4. Set `g_accel_dist_cm` and `g_decel_dist_cm` based on desired motion smoothness

#### Motor Tuning
1. Determine `MIN_MOTOR_SPEED_PERCENT` by finding minimum PWM for reliable motor startup
2. Ensure motors can start reliably at this percentage
3. Increase if motors stall, decrease if they start too aggressively

#### AHRS Filter Tuning
1. Start with default `s_beta = 0.1f` for balanced performance
2. Increase `s_beta` for faster convergence to true orientation (more noise sensitive)
3. Decrease `s_beta` for smoother filtering (slower convergence)
4. Ensure `s_dt` matches actual IMU update rate for accurate integration

### System Architecture
- **Control Loop**: 100 Hz PID control provides responsive speed regulation
- **Movement Planning**: High-level trajectory generation with sensor feedback
- **Sensor Fusion**: IMU and encoder data combined for accurate navigation
- **Modular Design**: Clear separation between low-level control and high-level planning

### Debugging Tips
- Use OLED display to monitor real-time values during tuning
- Button-driven test sequences in `main.c` for systematic parameter testing
- Encoder tick counts provide direct feedback on motion accuracy
- IMU heading values help diagnose turning and straight-line performance