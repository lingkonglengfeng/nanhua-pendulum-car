#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H
#include <stdint.h>
#define BAL_TURN_DUTY_LIMIT 0.12f
#define BAL_OVERSPEED_DWELL_MS 200U

/* Single pending ACTIVE-exit event. Logger peeks/acks under its task critical
 * section. Snapshot survives STOP/ARM/ZERO; pending is cleared after TX posts. */
typedef struct {
  uint32_t ms, run_ms, overspeed_ms;
  float peak_rpm, max_rpm, angle_deg;
  uint16_t coalesced;
  uint8_t reason, pending;
} BalanceExitEvent;
uint8_t Balance_PeekExit(BalanceExitEvent *event);
void Balance_AckExit(uint32_t ms);

enum { BAL_OFF=0, BAL_ZEROING, BAL_WAIT_TOP, BAL_HOLD_TOP, BAL_ACTIVE, BAL_TRIPPED, BAL_SWING };
enum { BAL_REASON_NONE=0, BAL_REASON_STOP, BAL_REASON_ANGLE, BAL_REASON_SENSOR,
       BAL_REASON_TIMING, BAL_REASON_ZERO, BAL_REASON_CONFLICT, BAL_REASON_CONFIG,
       BAL_REASON_SPEED, BAL_REASON_RUN_TIME, BAL_REASON_TRAVEL };
enum { BAL_SEQ_NONE=0, BAL_SEQ_OUT, BAL_SEQ_PAUSE, BAL_SEQ_RETURN, BAL_SEQ_HOME, BAL_SEQ_HOLD, BAL_SEQ_SPIN,
       BAL_SEQ_STEP_WAIT, BAL_SEQ_STEP_MOVE, BAL_SEQ_STEP_END };

/* ControlTask owns all updates; volatile also permits live Watch inspection. */
extern volatile uint8_t g_bal_enable, g_bal_state, g_bal_reason;
extern volatile float g_bal_kp, g_bal_ki, g_bal_kd, g_bal_integral;
extern volatile float g_bal_theta, g_bal_theta_dot, g_bal_output;
extern volatile float g_bal_entry_deg, g_bal_exit_deg, g_bal_limit;
extern volatile int8_t g_bal_sign;
extern volatile uint32_t g_bal_hold_ms, g_bal_dwell_ms, g_bal_fault;
/* Independent commissioning cutoffs; neither is a speed controller or brake.
 * g_bal_max_rpm=0 disables overspeed exit; max_run_ms=0 disables only the timer. */
extern volatile float g_bal_max_rpm, g_bal_guard_rpm;
extern volatile uint32_t g_bal_max_run_ms, g_bal_run_ms, g_bal_overspeed_ms;
/* Zero-speed outer PI: degrees/RPM, degrees/(RPM*s), maximum reference degrees.
 * It adjusts the angle reference; it never drives motors independently. */
extern volatile uint8_t g_bal_vel_enable;
extern volatile float g_bal_vel_kp, g_bal_vel_ki, g_bal_vel_limit_deg;
extern volatile float g_bal_vel_rpm, g_bal_vel_integral_deg, g_bal_ref_deg;
extern volatile float g_bal_vel_slew_deg_s;
/* Before 100ms LP: SPIN uses all-wheel mean (same weights as odometry);
 * other modes retain the middle-pair outlier rejection. Legacy log name. */
extern volatile float g_bal_robust_rpm;
/* Manual upright reference offset in degrees; never changes sensor zero/guards. */
extern volatile float g_bal_trim_deg;
/* Motion requests need ACTIVE balance. STOP/ARM/ZERO reset to zero speed. */
extern volatile float g_bal_vel_target_rpm, g_bal_vel_command_rpm;
extern volatile float g_bal_position_rev, g_bal_seq_goal_rev;
extern volatile uint8_t g_bal_seq_state;
extern volatile uint32_t g_bal_seq_ms;
/* Position outer P: RPM per wheel-shaft revolution, speed ceiling in RPM. */
extern volatile float g_bal_pos_kp, g_bal_pos_max_rpm;
uint8_t Balance_SetSpeed(float rpm);
uint8_t Balance_SetPosition(float turns);
/* zero_origin=0: hold current location; 1: redefine it as encoder position 0. */
uint8_t Balance_HoldPosition(uint8_t zero_origin);
/* Hold signed mean travel at the entry location while requesting wheel yaw.
 * Repeated updates and neutral keep the same anchor; no heading angle given. */
uint8_t Balance_StartSpin(float rpm);
/* Positive turn = right: left wheels faster. Unit is (left-right)/2 RPM.
 * No heading sensor: this controls differential wheel speed, not yaw degrees. */
extern volatile float g_bal_turn_target_rpm, g_bal_turn_command_rpm;
extern volatile float g_bal_turn_rpm, g_bal_turn_output, g_bal_turn_kp;
extern volatile float g_bal_turn_ki, g_bal_turn_integral;
uint8_t Balance_SetTurn(float rpm);
float Balance_MotorOutput(uint8_t wheel);
uint8_t Balance_StartSequence(float rpm, float turns, uint32_t pause_ms);
/* Effective rolling diameter in mm; 0 means not measured/configured yet. */
extern volatile float g_bal_wheel_diameter_mm;
/* Hold here 5s, move signed metres, hold there 5s, then keep the end anchor.
 * Requires ACTIVE balance, configured wheels and neutral turn request/ramp. */
uint8_t Balance_StartStep(float metres, float max_rpm);
void Balance_AddPosition(float delta_rev);
/* Explicit automatic swing-up, only from a valid calibrated stationary bottom.
 * Model/gains are commissioning estimates, not identified plant parameters. */
extern volatile float g_swing_limit, g_swing_travel_rev, g_swing_gain, g_swing_omega2;
extern volatile float g_swing_energy, g_swing_capture_rate, g_swing_capture_deg;
extern volatile uint32_t g_swing_max_ms;
extern volatile int8_t g_swing_kick_sign;
extern volatile uint8_t g_swing_phase; /* 0 idle,1 kick,2 pump,3 caught,4 aborted */
uint8_t Balance_StartSwing(uint8_t calibrated,uint8_t valid,float wheel_peak_rpm);

void Balance_Stop(uint8_t reason);
void Balance_BeginZero(void);
void Balance_ZeroFailed(void);
uint8_t Balance_Arm(uint8_t calibrated);
/* theta is filtered; guard_theta is the latest unfiltered upright error.
 * Every invalid sample/long gap breaks the uninterrupted dwell interval. */
void Balance_Update(float theta, float rate, float guard_theta, uint8_t valid,
                    uint8_t calibrated, uint8_t conflict, uint32_t now_ms);
/* Call after Update and before applying motor output, with fresh mapped RPM. */
void Balance_CheckMotion(const volatile float *wheel_rpm, uint32_t now_ms);
/* Raw count speeds still guard overspeed. Feedback uses the hybrid estimate. */
void Balance_CheckMotionFeedback(const volatile float *raw_rpm,
                                const volatile float *feedback_rpm, uint32_t now_ms);
#endif
