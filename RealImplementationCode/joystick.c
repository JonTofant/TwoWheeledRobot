/*
 * joystick.c
 *
 *  Created on: Oct 22, 2025
 *      Author: jon
 */

#include "joystick.h"

// CONTROLLER VARIABLES
uint8_t uart3_controller_byte;
uint8_t uart3_controller_buf[UART3_CONTROLLER_PACKET_LEN];
uint8_t uart3_controller_index = 0;
uint8_t uart3_controller_packet_ready = 0;

int16_t axisLX;
uint16_t throttle;
uint16_t brake;
uint8_t xPressed;
uint8_t dpadUp;
uint8_t dpadDown;
uint8_t dpadLeft;
uint8_t dpadRight;
uint8_t startPressed;
uint8_t sharePressed;
uint8_t PSButtonPressed;
uint8_t MicButtonPressed;

float desired_x_dualshock = 0.0f;
float desired_angle_dualshock = 0.0f; // Desired angle from the dualshock controller

float scale_speed = 35;

// Function for handling joystick input

void process_joystick_input()
{
    // === 1. Calculate Checksum for Safety ===
    // Sum bytes 1 through 9 (all data bytes excluding SOF and the Checksum itself)
    uint8_t calculated_checksum = 0;
    for (int i = 1; i <= 9; i++) {
        calculated_checksum += uart3_controller_buf[i];
    }

    // Compare with the received checksum (Byte 10)
    if (calculated_checksum != uart3_controller_buf[10]) {
        // Optional: Increment an error counter here for debugging
        return; // DATA CORRUPTED: Exit early without updating robot state
    }

    // === 2. Unpack Validated Data ===
    axisLX       = (int16_t)(uart3_controller_buf[1] | (uart3_controller_buf[2] << 8));
    throttle     = (uint16_t)(uart3_controller_buf[3] | (uart3_controller_buf[4] << 8));
    brake        = (uint16_t)(uart3_controller_buf[5] | (uart3_controller_buf[6] << 8));
    xPressed     = uart3_controller_buf[7];
    uint8_t dpad_raw = uart3_controller_buf[8]; // Changed to uint8_t for cleaner bit comparison
    uint8_t misc_raw = uart3_controller_buf[9]; // Changed to uint8_t for cleaner bit comparison

    dpadUp   = (dpad_raw == 0x01) ? 1 : 0;
    dpadDown = (dpad_raw == 0x02) ? 1 : 0;
    dpadLeft = (dpad_raw == 0x08) ? 1 : 0;
    dpadRight= (dpad_raw == 0x04) ? 1 : 0;

    startPressed = (misc_raw == 0x04) ? 1 : 0;
    sharePressed = (misc_raw == 0x02) ? 1 : 0;
    PSButtonPressed = (misc_raw == 0x01) ? 1 : 0;
    MicButtonPressed = (misc_raw == 0x08) ? 1 : 0;

    // === 3. Motion Logic (remains the same) ===
    float delta_time = 0.02f; // 20 ms cycle
    float scale_factor = 1.0f / 1020.0f;

    static float throttle_filtered = 0.0f;
    static float brake_filtered = 0.0f;
    const float alpha = 0.1f;
    const float max_change = 10.0f;

    float throttle_target = (float)throttle;
    float brake_target    = (float)brake;

    throttle_filtered += alpha * (throttle_target - throttle_filtered);
    brake_filtered    += alpha * (brake_target - brake_filtered);

    // Rate limiting
    float delta_throttle = throttle_filtered - throttle_target;
    if (delta_throttle >  max_change) throttle_filtered = throttle_target + max_change;
    if (delta_throttle < -max_change) throttle_filtered = throttle_target - max_change;

    float delta_brake = brake_filtered - brake_target;
    if (delta_brake >  max_change) brake_filtered = brake_target + max_change;
    if (delta_brake < -max_change) brake_filtered = brake_target - max_change;

    // Behavior modes
    if ((throttle_filtered > 20.0f) || (brake_filtered > 20.0f)) {
        isSTATIC = false;
        isLOCOMOTION = true;
    } else {
        isSTATIC = true;
        isLOCOMOTION = false;
    }

    // Inputs
    float input = (throttle_filtered - brake_filtered);
    float steering = (float)axisLX / 512.0f;
    if (steering > 1.0f) steering = 1.0f;
    if (steering < -1.0f) steering = -1.0f;

    const float turn_gain      = 0.7f;
    const float pivot_gain_max = 450.0f;
    float pivot_blend = 1.0f - fminf(fabsf(input) / 400.0f, 1.0f);
    float steer_mag = fabsf(steering);
    float pivot_strength = steer_mag * pivot_gain_max * pivot_blend;

    float v_left  = input + steering * (turn_gain * fabsf(input) + pivot_strength);
    float v_right = input - steering * (turn_gain * fabsf(input) + pivot_strength);

    desired_v_left  = v_left  * scale_factor * delta_time * scale_speed;
    desired_v_right = v_right * scale_factor * delta_time * scale_speed;

    // D-pad body control
    if (dpadUp == 1)   base_target_y -= 0.1f;
    if (dpadDown == 1) base_target_y += 0.1f;

    if (base_target_y <= -24.0f) base_target_y = -24.0f;
    if (base_target_y >= -13.0f) base_target_y = -13.0f;
}

