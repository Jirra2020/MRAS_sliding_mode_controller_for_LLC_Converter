/*
 * MRAS Controller for LLC Resonant DC-DC Converter
 * MSc Thesis - Embedded C Implementation
 * Generated: Simulink Coder 25.2 (R2025b)
 * Sample Time: 1 microsecond (Ts = 1e-6 s)
 *
 * INPUTS:
 *   Vo      - LLC output voltage (measured)
 *   io      - LLC output current (measured)
 *   Vref    - Reference voltage setpoint
 *   Verror  - Voltage error: Verror = Vref_rm - Vo
 *   Ierror  - Current error: Ierror = io_rm - io
 *
 * OUTPUT:
 *   u_control - Frequency control signal to VCO (center 59kHz)
 *
 * ADAPTIVE PARAMETERS (estimated online):
 *   k11 = Integrator1_DSTATE  - Voltage gain (learning rate 0.02)
 *   k12 = Integrator4_DSTATE  - Auxiliary gain  (gain 80)
 *   k2  = Integrator_DSTATE   - Current gain (gain 100)
 *
 * CONTROL LAW:
 *   s     = -0.02*(Verror - Int2) - 0.03*(Ierror - Int3)
 *   u     = k11*Vref^2 + k11*io^2*0.3 + k11*Vo*0.0009
 *           - (80*k12 + 100*k2) * tanh(s)
 *
 * PARAMETER UPDATE LAWS (Lyapunov-based):
 *   dk11/dt = s
 *   dk12/dt = s
 *   dk2/dt  = s * sqrt(Vref^2 + io^2)
 */

#include "MRAS_Controller2.h"
#include <math.h>
#include "rtwtypes.h"

/* ============================================
 * STATE VARIABLES (adaptive parameter estimates)
 * ============================================ */
DW_MRAS_Controller2_T MRAS_Controller2_DW;

/* ============================================
 * INPUTS
 * ============================================ */
ExtU_MRAS_Controller2_T MRAS_Controller2_U;

/* ============================================
 * OUTPUTS
 * ============================================ */
ExtY_MRAS_Controller2_T MRAS_Controller2_Y;

static RT_MODEL_MRAS_Controller2_T MRAS_Controller2_M_;
RT_MODEL_MRAS_Controller2_T *const MRAS_Controller2_M = &MRAS_Controller2_M_;

/* ============================================
 * STEP FUNCTION - called every 1 microsecond
 * ============================================ */
void MRAS_Controller2_step(void)
{
  /* --- Local variables --- */
  real_T k11_phi;    /* k11 * regressor */
  real_T k12_Vref;   /* k12 * Vref component */
  real_T k12_io;     /* k12 * io component */
  real_T k2_Vo;      /* k2 * Vo component */
  real_T s;          /* sliding surface / error signal */
  real_T tanh_s;     /* tanh(s) - bounded nonlinearity */
  real_T k12_val;    /* adapted gain k12 = 80 * Integrator4 */
  real_T k2_val;     /* adapted gain k2  = 100 * Integrator */
  real_T norm_phi;   /* normalisation: sqrt(Vref^2 + io^2) */
  real_T ds;         /* integrator update: Ts * s */

  /* --- Step 1: Compute k11 (sign=-1 * Integrator1) --- */
  k11_phi = -(MRAS_Controller2_ConstB.Sign2 *
              MRAS_Controller2_DW.Integrator1_DSTATE);

  /* --- Step 2: Compute regressor terms --- */
  k12_Vref = k11_phi * MRAS_Controller2_U.Vref * 0.3;
  k12_io   = k11_phi * MRAS_Controller2_U.io   * 0.3;
  k2_Vo    = k11_phi * MRAS_Controller2_U.Vo   * 0.0009;

  /* --- Step 3: Compute sliding surface s --- */
  /* s = -0.02*(Verror - Int2) - 0.03*(Ierror - Int3) */
  real_T fcn_v = (MRAS_Controller2_U.Verror -
                  MRAS_Controller2_DW.Integrator2_DSTATE) * -0.02;
  real_T fcn_i = (MRAS_Controller2_U.Ierror -
                  MRAS_Controller2_DW.Integrator3_DSTATE) * -0.03;
  s = fcn_v + fcn_i;

  /* --- Step 4: Bounded nonlinearity --- */
  tanh_s = tanh(s);

  /* --- Step 5: Adapted gains --- */
  k12_val = 80.0  * MRAS_Controller2_DW.Integrator4_DSTATE;
  k2_val  = 100.0 * MRAS_Controller2_DW.Integrator_DSTATE;

  /* --- Step 6: Control law output --- */
  /* u = k11*(0.3*Vref^2 + 0.3*io^2 + 0.0009*Vo^2) */
  /* u = u - (k12 + k2) * tanh(s)                   */
  MRAS_Controller2_Y.u_control =
    ((k12_Vref * MRAS_Controller2_U.Vref +
      k12_io   * MRAS_Controller2_U.io) +
      k2_Vo    * MRAS_Controller2_U.Vo) -
    (k12_val + k2_val) * tanh_s;

  /* --- Step 7: Parameter update laws (Forward Euler) --- */
  /* Normalisation for k2 update */
  norm_phi = MRAS_Controller2_U.Vref * MRAS_Controller2_U.Vref +
             MRAS_Controller2_U.io   * MRAS_Controller2_U.io;

  ds = 1.0E-6 * s;   /* Ts * s */

  /* k11 update: dk11 = Ts * s */
  MRAS_Controller2_DW.Integrator1_DSTATE += ds;

  /* k12 update: dk12 = Ts * s */
  MRAS_Controller2_DW.Integrator4_DSTATE += ds;

  /* Verror integrator update */
  MRAS_Controller2_DW.Integrator2_DSTATE += 1.0E-6 * MRAS_Controller2_U.Verror;

  /* Ierror integrator update */
  MRAS_Controller2_DW.Integrator3_DSTATE += 1.0E-6 * MRAS_Controller2_U.Ierror;

  /* k2 update: dk2 = Ts * s * sqrt(Vref^2 + io^2) */
  MRAS_Controller2_DW.Integrator_DSTATE  += s * sqrt(norm_phi) * 1.0E-6;
}

/* ============================================
 * INITIALISE - call once at startup
 * ============================================ */
void MRAS_Controller2_initialize(void)
{
  /* Zero all adaptive parameter states */
  MRAS_Controller2_DW.Integrator1_DSTATE = 0.0;  /* k11 */
  MRAS_Controller2_DW.Integrator4_DSTATE = 0.0;  /* k12 */
  MRAS_Controller2_DW.Integrator_DSTATE  = 0.0;  /* k2  */
  MRAS_Controller2_DW.Integrator2_DSTATE = 0.0;  /* Verror integrator */
  MRAS_Controller2_DW.Integrator3_DSTATE = 0.0;  /* Ierror integrator */
}

/* ============================================
 * TERMINATE - call at shutdown
 * ============================================ */
void MRAS_Controller2_terminate(void)
{
  /* No cleanup required */
}

/* [EOF] */
