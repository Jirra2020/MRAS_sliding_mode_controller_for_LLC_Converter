# MRAS Sliding Mode Controller — LLC Resonant DC-DC Converter

Embedded C implementation of a **Model Reference Adaptive System (MRAS) Sliding Mode** controller for an LLC resonant DC-DC converter. Auto-generated from MATLAB/Simulink R2025b using Embedded Coder.

---

## Hardware Target

| Item | Detail |
|------|--------|
| Board | STM32 Nucleo-F446RE |
| MCU | STM32F446RE — ARM Cortex-M4 @ 180 MHz |
| FPU | Yes — hardware double precision |
| IDE | STM32CubeIDE |
| Sample time | 1 µs (1 MHz control loop via TIM2 interrupt) |

---

## Pin Mapping

| Pin | Signal | Description |
|-----|--------|-------------|
| PA0 | ADC IN0 | Output voltage Vo measurement |
| PA1 | ADC IN1 | Output current io measurement |
| PA4 | DAC OUT1 | u_control → VCO frequency input |
| PA2/PA3 | UART2 TX/RX | Debug output at 115200 baud |
| PC13 | LED | Heartbeat indicator |

---

## Controller Signals

| Signal | Direction | Description |
|--------|-----------|-------------|
| `Vo` | Input | Measured LLC output voltage |
| `io` | Input | Measured LLC output current |
| `Vref` | Input | Reference voltage setpoint (4000 V) |
| `Verror` | Input | Voltage error: Vref − Vo |
| `Ierror` | Input | Current error: io_ref − io |
| `u_control` | Output | Frequency command to VCO (~59 kHz centre) |

---

## Control Law

```
s = −0.02 × (Verror − ∫Verror) − 0.03 × (Ierror − ∫Ierror)

u = k11 × (0.3×Vref² + 0.3×io² + 0.0009×Vo)
      − (80×k12 + 100×k2) × tanh(s)
```

### Adaptive Parameter Update Laws

| Parameter | State | Update Law |
|-----------|-------|------------|
| k11 | `Integrator1_DSTATE` | dk11 = Ts × s |
| k12 | `Integrator4_DSTATE` | dk12 = Ts × s |
| k2 | `Integrator_DSTATE` | dk2 = Ts × s × √(Vref² + io²) |

---

## File Structure

```
MRAS_Controller2_ert_rtw/
├── MRAS_Controller2.c          # Core controller — step, init, terminate
├── MRAS_Controller2.h          # Structs, extern declarations
├── MRAS_Controller2_data.c     # Constant signals (Sign2 = -1.0)
├── MRAS_Controller2_private.h  # Internal declarations
├── MRAS_Controller2_types.h    # Type definitions
├── rtwtypes.h                  # Embedded Coder base types (real_T etc.)
├── ert_main.c                  # Generated main template (reference only)
└── main.c                      # STM32 HAL integration (ADC, DAC, Timer)
```

> **Note:** `ert_main.c` is a reference template only. `main.c` is the actual STM32 HAL implementation.

---

## STM32CubeIDE Setup

1. **New project** → Board selector → `NUCLEO-F446RE` → C project
2. Copy source files into `Core/Src/`:
   - `MRAS_Controller2.c`
   - `MRAS_Controller2_data.c`
   - `main.c`
3. Copy headers into `Core/Inc/`:
   - `MRAS_Controller2.h`
   - `MRAS_Controller2_private.h`
   - `MRAS_Controller2_types.h`
   - `rtwtypes.h`
4. Adjust scaling constants at the top of `main.c`:

```c
#define VO_SCALE         (5000.0f / 3.3f)   // Voltage sensor range
#define IO_SCALE         (30.0f / 3.3f)     // Current sensor range
#define U_CONTROL_MIN    -1000.0f            // VCO input min
#define U_CONTROL_MAX     1000.0f            // VCO input max
#define VREF_SETPOINT     4000.0f            // Target output voltage (V)
```

5. **Build** → **Run As** → **STM32 Cortex-M C/C++ Application**

---

## Debug Output

Connect USB cable to Nucleo board and open any serial terminal (PuTTY, Tera Term, CoolTerm) at **115200 baud**. Output is printed every 500 ms:

```
MRAS Controller Started
Vo=3998.2 V  io=1.243 A  u=0.0312
Vo=4001.1 V  io=1.251 A  u=0.0298
```

---

## Code Generation Info

| Item | Detail |
|------|--------|
| Tool | MATLAB Simulink Embedded Coder 25.2 (R2025b) |
| Target file | `ert.tlc` — Embedded Real-Time |
| Language | C |
| Solver | Fixed-step, ode4 |
| Sample time | 1×10⁻⁶ s |
| Generated | Mon Apr 27 2026 |

---

## License

MSc Thesis Project — Academic use only.
