## STM32 

## Parameters To Tweak

### PID Gains
- `PID_KP`, `PID_KI`, `PID_KD`: main speed-loop gains.
  - File: `MDP/Core/Src/control.c`
  - Lines: `PID_KP` (around 20), `PID_KI` (21), `PID_KD` (22)
  - Start near: `Kp=2.0`, `Ki=0.5`, `Kd=0.01` at 100 Hz, then tune.

### Anti‑Windup / Noise Clamps
- `PID_MAX_IOUT`: integral clamp (typ. 20–60).
- `PID_MAX_DOUT`: derivative clamp (typ. 20–60).
  - File: `MDP/Core/Src/control.c`
  - Lines: `PID_MAX_IOUT` (around 15), `PID_MAX_DOUT` (16)

### Output Limit
- `PID_MAX_OUT`: overall PWM limit in percent (keep 100 unless you want to cap).
  - File: `MDP/Core/Src/control.c:14`

### Control Period (dt)
- `CONTROL_DT_S`: PID loop dt (seconds). Keep `0.01f` for 100 Hz.
  - File: `MDP/Core/Src/control.c:11`

### Timer Frequency (TIM5 @ 100 Hz)
- `Prescaler = 1599`, `Period = 99` (yields 100 Hz with HSI defaults).
  - File: `MDP/Core/Src/main.c:425` and `MDP/Core/Src/main.c:427`
  - If you change these, update `CONTROL_DT_S` accordingly.

### Motor Dead‑Zone
- `MIN_MOTOR_SPEED_PERCENT`: minimum effective PWM to overcome stiction (e.g., `54.0f`).
  - File: `MDP/Core/Src/motor.c:11`
  - Lower if motors can spin reliably at smaller duty; raise if they stall.

### Step Sequence for Tuning
- `seq[]`: button‑driven targets (ticks per 10 ms) sequence.
  - File: `MDP/Core/Src/main.c:682`
  - Edit values/order to test different speeds/directions.

### OLED Refresh Rate
- UI update delay (ms) in the encoder display task.
  - File: `MDP/Core/Src/main.c:653` → `osDelay(100)`
  - Lower for faster updates during tuning.

## Notes
- Units: target/measured are “encoder ticks per 10 ms”; output is “PWM percent.”
- Scaling check: if 100% ≈ 40 ticks/10 ms on your robot, then 10–20 ticks/10 ms are moderate speeds.
- Tuning order: increase Kp to just below oscillation → add Ki to remove steady‑state error (watch `PID_MAX_IOUT`) → add small Kd to reduce overshoot (watch `PID_MAX_DOUT`).
