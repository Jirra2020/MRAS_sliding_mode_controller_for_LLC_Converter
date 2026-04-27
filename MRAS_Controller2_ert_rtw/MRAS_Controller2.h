/*
 * File: MRAS_Controller2.h
 *
 * Code generated for Simulink model 'MRAS_Controller2'.
 *
 * Model version                  : 1.2
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Mon Apr 27 14:40:08 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef MRAS_Controller2_h_
#define MRAS_Controller2_h_
#ifndef MRAS_Controller2_COMMON_INCLUDES_
#define MRAS_Controller2_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* MRAS_Controller2_COMMON_INCLUDES_ */

#include "MRAS_Controller2_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Integrator1_DSTATE;           /* '<S1>/Integrator1' */
  real_T Integrator3_DSTATE;           /* '<S1>/Integrator3' */
  real_T Integrator2_DSTATE;           /* '<S1>/Integrator2' */
  real_T Integrator4_DSTATE;           /* '<S1>/Integrator4' */
  real_T Integrator_DSTATE;            /* '<S1>/Integrator' */
} DW_MRAS_Controller2_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real_T Sign2;                  /* '<S1>/Sign2' */
} ConstB_MRAS_Controller2_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Vo;                           /* '<Root>/Vo' */
  real_T io;                           /* '<Root>/io' */
  real_T Vref;                         /* '<Root>/Vref' */
  real_T Verror;                       /* '<Root>/Verror' */
  real_T Ierror;                       /* '<Root>/Ierror' */
} ExtU_MRAS_Controller2_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T u_control;                    /* '<Root>/u_control' */
} ExtY_MRAS_Controller2_T;

/* Real-time Model Data Structure */
struct tag_RTM_MRAS_Controller2_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_MRAS_Controller2_T MRAS_Controller2_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_MRAS_Controller2_T MRAS_Controller2_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_MRAS_Controller2_T MRAS_Controller2_Y;
extern const ConstB_MRAS_Controller2_T MRAS_Controller2_ConstB;/* constant block i/o */

/* Model entry point functions */
extern void MRAS_Controller2_initialize(void);
extern void MRAS_Controller2_step(void);
extern void MRAS_Controller2_terminate(void);

/* Real-time Model object */
extern RT_MODEL_MRAS_Controller2_T *const MRAS_Controller2_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'MRAS_Controller2'
 * '<S1>'   : 'MRAS_Controller2/ASM controller'
 */
#endif                                 /* MRAS_Controller2_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
