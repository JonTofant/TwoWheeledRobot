/*
 * JumpStrategy.c
 *
 *  Created on: Jan 12, 2026
 *      Author: jon
 *  Updated on: Jan 21, 2026
 */

#include "JumpStrategy.h"

bool isJumpStrategy = false;

// Parameters
#include "controler.h"
#include "cybergear.h"

#define JUMP_EXTEND_Y           -22.0f
#define JUMP_RETRACT_Y          -17.0f

// --- Cycle Configurations ---
#define JUMP_HOLD_CYCLES        2       // N: Cycles to hold extension FF
#define JUMP_RETRACT_CYCLES     1       // M: Cycles to inject negative retraction FF

// --- Torque Configurations ---
#define JUMP_FF_TORQUE          12.0f   // Torque for the Jump (Extension)
#define JUMP_RETRACT_FF_TORQUE  4.0f   // Torque for the Retraction (Negative injection)

typedef enum {
    JUMP_IDLE = 0,
    JUMP_EXTEND_HOLD,    // Holding positive FF (N cycles)
    JUMP_RETRACT_FF_PHASE, // Injecting negative FF (M cycles)
    JUMP_CLEANUP         // Restoring normal operation
} JumpState_t;

static JumpState_t jump_state = JUMP_IDLE;
static uint32_t jump_counter = 0;

void jump_strategy_control()
{
    switch(jump_state)
    {
        case JUMP_IDLE:
            // --- STEP 1: START JUMP (Injection) ---
            base_target_y = JUMP_EXTEND_Y;

            // Set the feed-forward torque for jump (Extension)
            CyberGearMotorList[0].desired_torque_ff = JUMP_FF_TORQUE;  // Left leg
            CyberGearMotorList[1].desired_torque_ff = -JUMP_FF_TORQUE; // Right leg
            CyberGearMotorList[2].desired_torque_ff = JUMP_FF_TORQUE;  // Left leg
            CyberGearMotorList[3].desired_torque_ff = -JUMP_FF_TORQUE; // Right leg
            CyberGearMotorList[0].desired_velocity = 30.0f;
            CyberGearMotorList[1].desired_velocity = -30.0f;
            CyberGearMotorList[2].desired_velocity = 30.0f;
            CyberGearMotorList[3].desired_velocity = -30.0f;


            disable_controler(); // Disable other controllers during jump
            MIT_controler_gain_schedule_Jump(); // Set gains for jump

            // Prepare for N cycles hold
            jump_counter = 0;
            jump_state = JUMP_EXTEND_HOLD;
            break;

        case JUMP_EXTEND_HOLD:
            // --- STEP 2: HOLD EXTENSION FF (N Cycles) ---
            if(jump_counter < JUMP_HOLD_CYCLES) {
                jump_counter++;
            } else {
                // N cycles finished, setup Retraction Phase

                // Change target to retracted position
                base_target_y = JUMP_RETRACT_Y;

                // INJECT NEGATIVE RETRACTION FF
                // (Signs are inverted relative to the Jump phase)
                CyberGearMotorList[0].desired_torque_ff = -JUMP_RETRACT_FF_TORQUE;
                CyberGearMotorList[1].desired_torque_ff = JUMP_RETRACT_FF_TORQUE;
                CyberGearMotorList[2].desired_torque_ff = -JUMP_RETRACT_FF_TORQUE;
                CyberGearMotorList[3].desired_torque_ff = JUMP_RETRACT_FF_TORQUE;
                CyberGearMotorList[0].desired_velocity = 0.0f;
                CyberGearMotorList[1].desired_velocity = 0.0f;
                CyberGearMotorList[2].desired_velocity = 0.0f;
                CyberGearMotorList[3].desired_velocity = 0.0f;


                // Reset counter for the M cycles
                jump_counter = 0;
                jump_state = JUMP_RETRACT_FF_PHASE;
            }
            break;

        case JUMP_RETRACT_FF_PHASE:
            // --- STEP 3: HOLD NEGATIVE RETRACTION FF (M Cycles) ---
            if(jump_counter < JUMP_RETRACT_CYCLES) {
                jump_counter++;
            } else {
                // M cycles finished, move to cleanup
                jump_state = JUMP_CLEANUP;
            }
            break;

        case JUMP_CLEANUP:
            // --- STEP 4: RESTORE NORMAL OPERATION ---
            controler_defaults(); // Re-enable normal controllers
            MIT_controler_gain_schedule_Normal(); // Restore normal gains

            // Zero out the feed-forward torque
            CyberGearMotorList[0].desired_torque_ff = 0.0f;
            CyberGearMotorList[1].desired_torque_ff = 0.0f;
            CyberGearMotorList[2].desired_torque_ff = 0.0f;
            CyberGearMotorList[3].desired_torque_ff = 0.0f;

            jump_state = JUMP_IDLE;
            isJumpStrategy = false;  // Jump finished
            break;

        default:
            jump_state = JUMP_IDLE;
            break;
    }
}
