/*
 * StateEstimator.c
 *
 *  Created on: Oct 22, 2025
 *      Author: jon
 */

#include "StateEstimator.h"
#include "stdint.h"
#include "stdbool.h"

volatile float estimated_theta_rad;
volatile float estimated_theta_dot_rad_s;
volatile float estimated_phi_dot_rad_s; // Assuming this is also global

float yaw_esp32 ;
float pitch_esp32  ;
float roll_esp32   ;
float gx_esp32   ;
float gy_esp32  ;
float gz_esp32   ;


