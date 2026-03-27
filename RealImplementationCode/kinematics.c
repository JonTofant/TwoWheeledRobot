/*
 * kinematics.c
 *
 *  Created on: Mar 10, 2025
 *      Author: tofan
 */

#include "kinematics.h"
#include <math.h>


//Inverzna kinematika
const float L1_C = 10.8f;
const float L2_C = 10.0f;
const float L3_C = 20.0f;
const float PI_C = 3.14159265359f;

// Header Kinematika ?
float base_target_y = -15.0f;
float final_y_left = -15.0f;
float final_y_right = -15.0f;

// Header kinematika ?
float xc_des_l =0;
float xc_des_r = 0;

float delta_varphi_l;
float delta_varphi_r;


// Global leg states
LegState leg_state_rf = {0};
LegState leg_state_lf = {0};
LegState leg_state_rb = {0};
LegState leg_state_lb = {0};

// Leg geometry definition
LegGeometry LegGeometryList[MAX_NUM_LEGS] = {
    {
        .l0 = 10.8f, .l1 = 10.0f, .l2 = 20.0f,
        .l = 0.0f, .l_dot = 0.0f, .r_eff = 0.0f,
        .l_prev = 0.0f, .x_c = 0.0f, .y_c = 0.0f,
        .theta = 0.0f, .l_eq = 0.13f, .l_dot_desired = 0.0f,
		.theta_ref = 0.0f, .theta_dot = 0.0f, .theta_prev = 0.0f,
        .l_ddot = 0.0f, .l_dot_prev = 0.0f,
        .J = {{0.0f, 0.0f}, {0.0f, 0.0f}},
        .J_T = {{0.0f, 0.0f}, {0.0f, 0.0f}}
    },
    {
        .l0 = 10.8f, .l1 = 10.0f, .l2 = 20.0f,
        .l = 0.0f, .l_dot = 0.0f, .r_eff = 0.0f,
        .l_prev = 0.0f, .x_c = 0.0f, .y_c = 0.0f,
		.theta_ref = 0.0f, .theta_dot = 0.0f, .theta_prev = 0.0f,
        .theta = 0.0f, .l_eq = 0.13f, .l_dot_desired = 0.0f,
        .l_ddot = 0.0f, .l_dot_prev = 0.0f,
        .J = {{0.0f, 0.0f}, {0.0f, 0.0f}},
        .J_T = {{0.0f, 0.0f}, {0.0f, 0.0f}}
    }
};


// Defintion of the r_eff function
// r_eff = l1 * sin(theta1) + l2 * sin(theta1 + theta2)
void calculate_r_eff(float theta1, float theta2, LegGeometry* legGeometry) {
	// theta 1 is offset by 90 degrees
	float theta1_c = theta1;
	float theta2_c = theta2;

	// Calculate the effective leg length
	legGeometry->r_eff = legGeometry->l1 * sinf(theta1_c) + legGeometry->l2 * sinf(theta1_c + theta2_c);
}

// Function to calculate L and L_dot

void calculate_L_and_L_dot(float theta1, float theta2, LegGeometry* legGeometry, float dt) {
	float theta1_c =theta1;
	float theta2_c =theta2;
    // Calculate x_c and y_c based on the angles and leg segments.
    legGeometry->x_c = legGeometry->l1 * cosf(theta1_c) + legGeometry->l2 * cosf(theta1_c + theta2_c);
    legGeometry->y_c = legGeometry->l1 * sinf(theta1_c) + legGeometry->l2 * sinf(theta1_c + theta2_c);

    // Correct calculation of L: square the difference and add y_c squared, then take the square root.
    legGeometry->l = sqrtf((legGeometry->x_c - (legGeometry->l0 / 2.0f)) * (legGeometry->x_c - (legGeometry->l0 / 2.0f)) + legGeometry->y_c * legGeometry->y_c);

    // Compute the derivative of L, making sure dt is positive.
    legGeometry->l_dot = (dt > 0.0f) ? (legGeometry->l - legGeometry->l_prev) / dt : 0.0f;

    // Update l_prev for the next iteration.
    legGeometry->l_prev = legGeometry->l;
}


// Function to calculate Xc and Yc
void calculate_Xc_Yc(float phi1, float phi2, LegGeometry* legGeometry) {
	// Calculate x_c and y_c based on the angles and leg segments.
	phi1 =  phi1;
	phi2 = phi2;
	float l0 = legGeometry->l0;
	float l1 = legGeometry->l1;
	float l2 = legGeometry->l2;
	float x_c = legGeometry->x_c;
	float y_c = legGeometry->y_c;

	float PHI = sqrtf((4*l2*l2)/(l0*l0-2*l1*l1*cosf(phi1-phi2)+2*l1*l1-2*l0*l1*(cosf(phi1)-cosf(phi2)))-1);
	x_c =  0.5f * (
		    l0
		    + l1 * (cosf(phi1) + cosf(phi2))
		    + l1 * PHI * (sinf(phi1) - sinf(phi2))
		);
	y_c = 0.5f * (
	    l1 * (sinf(phi1) + sinf(phi2))   	  // l1(sin(phi1) + sin(phi2))
	  + l0 * PHI                        	  // + l0 * PHI
	  - l1 * PHI * (cosf(phi1) + cosf(phi2))  // - l1*PHI*(cos(phi1)+cos(phi2))
	);

	// Write the results back to the leg geometry struct
	legGeometry->x_c = x_c;
	legGeometry->y_c = y_c;

}
// This is the corrected version of your function.
// It fixes the typo (0.05f -> 0.5f) and uses the safe atan2f function.
void calculate_L_and_theta(LegGeometry* legGeometry) {
	// --- Input values from the struct ---
	// We don't need to declare local variables like 'l' and 'theta'
	// because we will write directly to the struct at the end.
	const float l0 = legGeometry->l0;   // Base link distance (e.g., 0.129)
	const float x_c = legGeometry->x_c; // Absolute X position of the foot
	const float y_c = legGeometry->y_c; // Absolute Y position of the foot

	// --- Define the origin of the virtual leg ---
	// This is the midpoint of the base link.
	//const float virtual_leg_base_x = l0 / 2.0f;
	//const float virtual_leg_base_y = 0.0f;

	// --- Calculate the vector from the virtual leg's base to the foot ---
	//float delta_x = x_c - virtual_leg_base_x;
	//float delta_y = y_c - virtual_leg_base_y;

	// --- Calculate and write the results back to the leg geometry struct ---

	// Calculate the virtual leg length 'l'
	// Calculate the virtual leg angle 'theta' using the correct math
	legGeometry->theta = atanf((x_c - 0.5f*l0)/y_c);
}


void init_leg_state(LegState* state) {
    state->is_initialized = false;
    state->prev_B_x = 0.0f;
    state->prev_B_y = 0.0f;
    state->prev_C_x = 0.0f;
    state->prev_C_y = 0.0f;
}


bool set_leg_foot_position(CyberGear* motor_right, CyberGear* motor_left, LegState* leg_state, float xf, float yf) {
    const float O0_x = 0.0f, O0_y = 0.0f;
    const float A_x = L1_C, A_y = 0.0f;

    float B_x1, B_y1, B_x2, B_y2;
    float C_x1, C_y1, C_x2, C_y2;
    float chosen_B_x, chosen_B_y;
    float chosen_C_x, chosen_C_y;

    // Solve for all possible knee positions for both legs independently
    bool reach_B = solveIKTwoSolutions_c(A_x, A_y, xf, yf, L2_C, L3_C, &B_x1, &B_y1, &B_x2, &B_y2);
    bool reach_C = solveIKTwoSolutions_c(O0_x, O0_y, xf, yf, L2_C, L3_C, &C_x1, &C_y1, &C_x2, &C_y2);

    if (!(reach_B && reach_C)) {
        return false; // Position is unreachable
    }

    // --- CHANGE: Always enforce outward configuration by x-coordinate ---
    // For the right knee (B), choose the solution with the LARGER x-coordinate.
    chosen_B_x = (B_x1 > B_x2) ? B_x1 : B_x2;
    chosen_B_y = (B_x1 > B_x2) ? B_y1 : B_y2;

    // For the left knee (C), choose the solution with the SMALLER x-coordinate.
    chosen_C_x = (C_x1 < C_x2) ? C_x1 : C_x2;
    chosen_C_y = (C_x1 < C_x2) ? C_y1 : C_y2;

    // --- Keep this: Update the state for the next iteration (optional, but harmless) ---
    leg_state->prev_B_x = chosen_B_x;
    leg_state->prev_B_y = chosen_B_y;
    leg_state->prev_C_x = chosen_C_x;
    leg_state->prev_C_y = chosen_C_y;

    // --- CHANGE: No longer needed, but you can remove this flag entirely if desired ---
    leg_state->is_initialized = true;

    // --- Calculate mathematical angles from the chosen knee points ---
    float calculated_alpha1_rad = atan2f(chosen_B_y - A_y, chosen_B_x - A_x);
    float calculated_alpha2_rad = atan2f(chosen_C_y - O0_y, chosen_C_x - O0_x);

    // --- Apply YOUR original, physically-tuned final transformation ---
    float physical_angle_right = -calculated_alpha1_rad;
    float physical_angle_left  = -(calculated_alpha2_rad + PI_C);

    // --- Assign to motors ---
    motor_right->desired_angle = physical_angle_right;
    motor_left->desired_angle = physical_angle_left;

    return true;
}


bool solveIKTwoSolutions_c(float base_x, float base_y, float foot_x, float foot_y,
                                  float L_upper, float L_lower,
                                  float *out_x1, float *out_y1, float *out_x2, float *out_y2)
{
    float dx = foot_x - base_x;
    float dy = foot_y - base_y;
    float d_sq = dx * dx + dy * dy;
    float d = sqrtf(d_sq);

    if (d > (L_upper + L_lower) || d < fabsf(L_upper - L_lower)) {
        return false;
    }

    float a = (L_upper * L_upper - L_lower * L_lower + d_sq) / (2.0f * d);
    float h_sq = L_upper * L_upper - a * a;
    float h = (h_sq > 0) ? sqrtf(h_sq) : 0;
    float x2 = base_x + a * dx / d;
    float y2 = base_y + a * dy / d;
    float rx = -dy * (h / d);
    float ry =  dx * (h / d);

    *out_x1 = x2 + rx; *out_y1 = y2 + ry;
    *out_x2 = x2 - rx; *out_y2 = y2 - ry;
    return true;
}

void chooseContinuousSolution_c(float prev_x, float prev_y,
                                       float x1, float y1, float x2, float y2,
                                       float *out_x, float *out_y)
{
    float dist1_sq = (x1 - prev_x) * (x1 - prev_x) + (y1 - prev_y) * (y1 - prev_y);
    float dist2_sq = (x2 - prev_x) * (x2 - prev_x) + (y2 - prev_y) * (y2 - prev_y);
    if (dist1_sq < dist2_sq) {
        *out_x = x1;
        *out_y = y1;
    } else {
        *out_x = x2;
        *out_y = y2;
    }
}
