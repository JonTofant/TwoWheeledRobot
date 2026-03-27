/*
 * StartupStrategy.c
 *
 *  Created on: Oct 22, 2025
 *      Author: jon
 */
#include "StartupStrategy.h"

uint8_t attempt_for_amount_of_samples = 45;
bool isStartupStrategy = false;
bool isStartupStategySuccess = false;

void startup_strategy_control()
{
				// TODO instead of hardcoded gains store a memery of previous gains and set them after startup strategy finishes
				K_GAINS[0] = 180.0f;
				K_GAINS[1] = 10.0f;
				 attempt_for_amount_of_samples -=1;
				 if (attempt_for_amount_of_samples <=0 || (roll_esp32 > -0.0872664626 && roll_esp32 < 0.0872664626)){

					 if (roll_esp32 > -0.1872664626 && roll_esp32 < 0.1872664626)
					 {
					     isStartupStategySuccess = true;
					     isStartupStrategy = false;
					     attempt_for_amount_of_samples = 45;
						    K_GAINS[0] = 100.0f;
						    K_GAINS[1] = 10.0f;

					 }
					 else{
						 isStartupStategySuccess = false;
						 isStartupStrategy = false;
						 attempt_for_amount_of_samples = 55;
						    K_GAINS[0] = 100.0f;
						    K_GAINS[1] = 10.0f;
					 }

				 }
}
