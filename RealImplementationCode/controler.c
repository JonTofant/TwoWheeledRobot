/*
 * LQR_Controller.c
 *
 *  Created on: Mar 18, 2025
 *      Author: tofan
 */


#include "controler.h"

// PITCH CONTROLLER (PID)
volatile float Kp_pitch = 0.0f;
volatile float Ki_pitch = 0.0f;
volatile float Kd_pitch = 0.0f;


float K_GAINS[2] = {100.0f,   10.0f};
float K_I_THETA = -18.0f; // Integral gain for theta


float Kp_pos_chasis = 1.2f;
float Kd_pos_chasis = -0.15f;
float Ki_pos_chasis = 0.6f;

float position_integral_L;
float position_integral_R;

// Position control gains for fall strategy
 float Kp_pos = 0.13f;
 float Kd_pos = -0.015f;
 float Ki_pos = 0.25f;

 float desired_v_left=0;
 float desired_v_right=0;

 //Target chasis position
 float target_x_chasis = 10.8/2;
 float target_y_chasis = -13.0;


 // Default values for DDSM115 motor current is 0 [A]
 float current_motor1_out= 0.0f;
 float current_motor2_out= 0.0f;
 float total_torque_out= 0.0f;
 float total_force_out = 0.0f;



volatile float pitch_integral = 0.0f;
volatile float previous_pitch_error = 0.0f;

volatile float DESIRED_PITCH_RAD = 0.0f;

float MAX_PITCH_HEIGHT_ADJUSTMENT = 20.0f;
extern float base_target_y;
extern float final_y_left;
extern float final_y_right;

void update_pitch_leveling_controller(float current_pitch_rad, float dt)
{
    // 1. Calculate the error
    float pitch_error = DESIRED_PITCH_RAD - current_pitch_rad;

    // 2. Update the integral term (with anti-windup)
    pitch_integral += pitch_error * dt;

    if (Ki_pitch > 0.0f) {
        const float max_integral = MAX_PITCH_HEIGHT_ADJUSTMENT / Ki_pitch;
        if (pitch_integral > max_integral) pitch_integral = max_integral;
        if (pitch_integral < -max_integral) pitch_integral = -max_integral;
    }

    // 3. Derivative term
    float pitch_derivative = 0.0f;
    if (dt > 0.0f) {
        pitch_derivative = (pitch_error - previous_pitch_error) / dt;
    }
    previous_pitch_error = pitch_error;

    // 4. Calculate the controller output (PID)
    float height_adjustment = (Kp_pitch * pitch_error) +
                              (Ki_pitch * pitch_integral) +
                              (Kd_pitch * pitch_derivative);

    // 5. Clamp the final adjustment to prevent sudden, large movements
    if (height_adjustment > MAX_PITCH_HEIGHT_ADJUSTMENT) height_adjustment = MAX_PITCH_HEIGHT_ADJUSTMENT;
    if (height_adjustment < -MAX_PITCH_HEIGHT_ADJUSTMENT) height_adjustment = -MAX_PITCH_HEIGHT_ADJUSTMENT;

    // 6. Apply the adjustment differentially to the base height
    final_y_left  = base_target_y + height_adjustment;
    final_y_right = base_target_y - height_adjustment;

    // 7. Final safety clamp to ensure legs stay within physical limits
    const float LEG_Y_MAX = -15.0f; // Lowest position
    const float LEG_Y_MIN = -27.0f; // Highest position

    if (final_y_left > LEG_Y_MAX) final_y_left = LEG_Y_MAX;
    if (final_y_left < LEG_Y_MIN) final_y_left = LEG_Y_MIN;

    if (final_y_right > LEG_Y_MAX) final_y_right = LEG_Y_MAX;
    if (final_y_right < LEG_Y_MIN) final_y_right = LEG_Y_MIN;
}


void calculate_cascaded_motor_currents(float x_target_left, float x_target_right,
                                       float* current_motor1_out,
                                       float* current_motor2_out,
                                       float* total_force_out)
{
    // --- [STATE] Persistent Integrators ---
    static float position_integral_L = 0.0f;
    static float position_integral_R = 0.0f;
    static float theta_error_integral_L = 0.0f;
    static float theta_error_integral_R = 0.0f;
    static uint32_t last_time_ms = 0;

    // --- [TIMING] Compute dt ---
    uint32_t current_time_ms = HAL_GetTick();
    float dt = (last_time_ms == 0) ? 0.01f : (current_time_ms - last_time_ms) / 1000.0f;
    last_time_ms = current_time_ms;

    // --- [SENSOR INPUT] Shared IMU (theta, theta_dot) ---
    float temp_theta     = roll_esp32 - 0.0280328498f;  // Corrected offset
    float temp_theta_dot = gx_esp32;

    // --- [SENSOR INPUT] Per-Motor Wheel States ---
    float x_L     = -DDSM115MotorList[0].x;
    float x_dot_L = -DDSM115MotorList[0].x_dot;
    float x_ddot_L = -DDSM115MotorList[0].x_ddot;

    float x_R     =  DDSM115MotorList[1].x;
    float x_dot_R =  DDSM115MotorList[1].x_dot;
    float x_ddot_R = DDSM115MotorList[1].x_ddot;



    ////////////////////////////////////////////////////////////////////////////////
    // -------------------- OUTER LOOP: POSITION CONTROL -------------------------
    ////////////////////////////////////////////////////////////////////////////////

    // --- [Outer Loop Errors] ---
    float x_err_L = x_dot_L - x_target_left;
    float x_err_R = x_dot_R - x_target_right;

    // --- [Outer Loop Integration (optional)] ---
    if (Ki_pos > 0.0f) {
        position_integral_L += x_err_L * dt;
        position_integral_R += x_err_R * dt;

        const float MAX_POS_INTEGRAL = 0.2f;
        if (position_integral_L > MAX_POS_INTEGRAL) position_integral_L = MAX_POS_INTEGRAL;
        if (position_integral_L < -MAX_POS_INTEGRAL) position_integral_L = -MAX_POS_INTEGRAL;
        if (position_integral_R > MAX_POS_INTEGRAL) position_integral_R = MAX_POS_INTEGRAL;
        if (position_integral_R < -MAX_POS_INTEGRAL) position_integral_R = -MAX_POS_INTEGRAL;
    }

    // --- [Theta Desired from Outer Loop PD] ---
    float theta_des_L = -(Kp_pos * x_err_L + Kd_pos * x_ddot_L + Ki_pos * position_integral_L);
    float theta_des_R = -(Kp_pos * x_err_R + Kd_pos * x_ddot_R + Ki_pos * position_integral_R);

    theta_des_l_telemetry = theta_des_L;
    theta_des_r_telemetry = theta_des_R;

    // --- [Clamp Desired Angle] ---
    const float MAX_THETA_DES = 0.244346095;  // ≈ 14 degrees
    if (theta_des_L > MAX_THETA_DES) theta_des_L = MAX_THETA_DES;
    if (theta_des_L < -MAX_THETA_DES) theta_des_L = -MAX_THETA_DES;
    if (theta_des_R > MAX_THETA_DES) theta_des_R = MAX_THETA_DES;
    if (theta_des_R < -MAX_THETA_DES) theta_des_R = -MAX_THETA_DES;

    ////////////////////////////////////////////////////////////////////////////////
    // -------------------- INNER LOOP: ANGLE TRACKING LQI -----------------------
    ////////////////////////////////////////////////////////////////////////////////

    float temp_theta_l_plus_varphi = temp_theta + delta_varphi_l;
    float temp_theta_r_plus_varphi = temp_theta + delta_varphi_r;
    // --- [Inner Loop Errors] ---
    float theta_error_L = temp_theta_l_plus_varphi - theta_des_L;
    float theta_error_R = temp_theta_r_plus_varphi - theta_des_R;

    // --- [Inner Loop Integration (LQI)] ---
    theta_error_integral_L += theta_error_L * dt;
    theta_error_integral_R += theta_error_R * dt;

    const float MAX_THETA_I = 0.2f;
    if (theta_error_integral_L > MAX_THETA_I) theta_error_integral_L = MAX_THETA_I;
    if (theta_error_integral_L < -MAX_THETA_I) theta_error_integral_L = -MAX_THETA_I;
    if (theta_error_integral_R > MAX_THETA_I) theta_error_integral_R = MAX_THETA_I;
    if (theta_error_integral_R < -MAX_THETA_I) theta_error_integral_R = -MAX_THETA_I;

    // --- [Compute Force from LQI] ---
    float force_L = -(K_GAINS[0] * theta_error_L + K_GAINS[1] * temp_theta_dot + K_I_THETA * theta_error_integral_L);
    float force_R = -(K_GAINS[0] * theta_error_R + K_GAINS[1] * temp_theta_dot + K_I_THETA * theta_error_integral_R);

    // --- [Optional Logging: average total force] ---
    if (total_force_out) {
        *total_force_out = 0.5f * (force_L + force_R);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // -------------------- ACTUATOR: CURRENT COMPUTATION ------------------------
    ////////////////////////////////////////////////////////////////////////////////

    float current_L = (force_L * WHEEL_RADIUS_R) / MOTOR_TORQUE_CONSTANT_KT;
    float current_R = (force_R * WHEEL_RADIUS_R) / MOTOR_TORQUE_CONSTANT_KT;

    // --- [Deadzone Compensation] ---
    if (current_L > 0) current_L += DZ_LEFT_POS;
    else if (current_L < 0) current_L -= DZ_LEFT_NEG;

    if (current_R > 0) current_R += DZ_RIGHT_POS;
    else if (current_R < 0) current_R -= DZ_RIGHT_NEG;

    // --- [Final Motor Current Outputs] ---
    *current_motor1_out = -current_R;  // Motor 0x10 (Right)
    *current_motor2_out =  current_L;  // Motor 0x11 (Left)
}


void posture_controler()
{
	float x_L = -DDSM115MotorList[0].x;
	float x_dot_L = -DDSM115MotorList[0].x_dot;
	float x_ddot_R = -DDSM115MotorList[0].x_ddot;

	float x_R     =  DDSM115MotorList[1].x;
	float x_dot_R =  DDSM115MotorList[1].x_dot;
	float x_ddot_L = DDSM115MotorList[1].x_ddot;

	float v_L = x_dot_L;
	float v_R = x_dot_R;

	// Izračun želenega xc z pd regulatorjem
	// --- [Outer Loop Errors] ---
	float x_err_L = v_L - desired_v_left;
	float x_err_R = v_R - desired_v_right;

	// --- [Outer Loop Integration (optional)] ---
	if (Ki_pos > 0.0f) {
		position_integral_L += x_err_L * 0.015; // * dt
		position_integral_R += x_err_R * 0.015; // * dt

		const float MAX_POS_INTEGRAL = 0.2f;
		if (position_integral_L > MAX_POS_INTEGRAL) position_integral_L = MAX_POS_INTEGRAL;
		if (position_integral_L < -MAX_POS_INTEGRAL) position_integral_L = -MAX_POS_INTEGRAL;
		if (position_integral_R > MAX_POS_INTEGRAL) position_integral_R = MAX_POS_INTEGRAL;
		if (position_integral_R < -MAX_POS_INTEGRAL) position_integral_R = -MAX_POS_INTEGRAL;
	}

			// --- [Theta Desired from Outer Loop PD] ---
			xc_des_l = -(Kp_pos_chasis * x_err_L + Kd_pos_chasis * x_ddot_L + Ki_pos_chasis * position_integral_L);
			xc_des_r = (Kp_pos_chasis * x_err_R + Kd_pos_chasis * x_ddot_R + Ki_pos_chasis * position_integral_R);

			// --- [Clamp Desired Angle] ---
			const float MAX_POS_DES = 1.0;
			if (xc_des_l > MAX_POS_DES) xc_des_l = MAX_POS_DES;
			if (xc_des_l < -MAX_POS_DES) xc_des_l = -MAX_POS_DES;
			if (xc_des_r > MAX_POS_DES) xc_des_r = MAX_POS_DES;
			if (xc_des_r < -MAX_POS_DES) xc_des_r = -MAX_POS_DES;


			float xc_des_l_translated = xc_des_l + 10.8/2;
			float xc_des_r_translated = xc_des_r + 10.8/2;

		bool success_right = set_leg_foot_position(
			&MOTOR_CG_RF,       // The "right" motor of the right leg
			&MOTOR_CG_RB,       // The "left" motor of the right leg
			&leg_state_rf,
			xc_des_r_translated,
			base_target_y
		);

		// Call the kinematics for the left leg
		bool success_left = set_leg_foot_position(
			&MOTOR_CG_LF,       // The "right" motor of the left leg
			&MOTOR_CG_LB,       // The "left" motor of the left leg
			&leg_state_lf,
			xc_des_l_translated,
			base_target_y
		);

		LegGeometryList[0].x_c = xc_des_l_translated;
		LegGeometryList[0].y_c = -base_target_y;


		LegGeometryList[1].x_c = xc_des_r_translated;
		LegGeometryList[1].y_c = -base_target_y;



		calculate_L_and_theta(&LegGeometryList[0]);
		calculate_L_and_theta(&LegGeometryList[1]);



	delta_varphi_l = -LegGeometryList[0].theta;
	delta_varphi_r = LegGeometryList[1].theta;

}

void disable_controler(){
	Kp_pitch = 0.0f;
	Ki_pitch = 0.0f;
	Kd_pitch = 0.0f;

	K_GAINS[0] = 0.0f;
	K_GAINS[1] = 0.0f;
	K_I_THETA = 0.0f;

	Kp_pos_chasis = 0.0f;
	Kd_pos_chasis = 0.0f;
	Ki_pos_chasis = 0.0f;

	position_integral_L = 0.0f;
	position_integral_R = 0.0f;

	Kp_pos = 0.0f;
	Kd_pos = 0.0f;
	Ki_pos = 0.0f;

	current_motor1_out= 0.0f;
	current_motor2_out= 0.0f;
	total_torque_out= 0.0f;
	total_force_out = 0.0f;
}

void controler_defaults(){
	Kp_pitch = 0.0f;
	Ki_pitch = 0.0f;
	Kd_pitch = 0.0f;

	K_GAINS[0] = 100.0f;
	K_GAINS[1] = 10.0f;
	K_I_THETA = -18.0f;

	Kp_pos_chasis = 1.2f;
	Kd_pos_chasis = -0.15f;
	Ki_pos_chasis = 0.6f;

	position_integral_L = 0.0f;
	position_integral_R = 0.0f;

	Kp_pos = 0.13f;
	Kd_pos = -0.015f;
	Ki_pos = 0.25f;

	current_motor1_out= 0.0f;
	current_motor2_out= 0.0f;
	total_torque_out= 0.0f;
	total_force_out = 0.0f;
}

void MIT_controler_gain_schedule_Jump(){
	CyberGearMotorList[0].kp = 4.0f;
	CyberGearMotorList[0].kd = 0.2f;
}
void MIT_controler_gain_schedule_Normal(){
	CyberGearMotorList[0].kp = 3.5f;
	CyberGearMotorList[0].kd = 0.1f;

}
