/*
 * state_machine.c
 *
 *  Created on: Nov 6, 2025
 *      Author: jon
 */

#include "state_machine.h"

volatile bool isCANReady = false;
volatile bool isDDSM115Ready = false;
volatile bool isControllerReady = false;
volatile bool isCYBERGEARReady = false;
volatile bool isTELEMETRYReady = true;
volatile bool isLOCOMOTION = false;
volatile bool isSTATIC = true;
volatile bool isJUMP = false;
volatile bool uartSynced = false;
volatile bool isDEMO = true;


// Function to check if robot is fallen based on pitch and roll angles returns true if fallen

bool isFallen()
{
    static bool fallen = false;  // remembers the previous state

    if (roll_esp32 > 0.60 || roll_esp32 < -0.60)
        fallen = true;
    else if (roll_esp32 > -0.0872664626 && roll_esp32 < 0.0872664626)
        fallen = false;

    return fallen;
}
