/*
 * system_init.c
 *
 *  Created on: Apr 13, 2025
 *      Author: tofan
 */


#include "system_init.h"
#include "cybergear.h"

#define MOTOR_CG_LB CyberGearMotorList[0] // Phi 2
#define MOTOR_CG_LF CyberGearMotorList[1] // Phi 1
#define MOTOR_CG_RF CyberGearMotorList[2]
#define MOTOR__CG_RB CyberGearMotorList[3]
// Function that initializes the system
void System_Init(void)
{
	// Set the cybergear motor to angle mode
	  // 2) Enable motor
	  motorEnable(MOTOR_CG_LF.hostID, MOTOR_CG_LF.motorID);
	  HAL_Delay(50);

	  motorEnable(MOTOR_CG_LB.hostID, MOTOR_CG_LB.motorID);

	  HAL_Delay(50);

	  motorEnable(MOTOR_CG_RF.hostID, MOTOR_CG_RF.motorID);
	  HAL_Delay(50);

	  motorEnable(MOTOR__CG_RB.hostID, MOTOR__CG_RB.motorID);
	  HAL_Delay(50);



	  // Zero the motor
	  setMechanicalZero(MOTOR_CG_LF.hostID, MOTOR_CG_LF.motorID);
	  HAL_Delay(50);

	  setMechanicalZero(MOTOR_CG_LB.hostID, MOTOR_CG_LB.motorID);
	  HAL_Delay(50);

	  setMechanicalZero(MOTOR_CG_RF.hostID, MOTOR_CG_RF.motorID);
	  HAL_Delay(50);

	  setMechanicalZero(MOTOR__CG_RB.hostID, MOTOR__CG_RB.motorID);
	  HAL_Delay(50);

	  // DDSM115 change ID
	  //DDSM115ChangeID(0xAA, 0x10); // Change ID of first motor to 0x10

	  //2) Put motor in position mode
	  	  // TODO change this into a function
	  	  /*uint8_t runMode = 1; // 1 => position mode, 3 => current mode
	  	  writeParameter(0x7005, &runMode, MOTOR_CG_LF.hostID, MOTOR_CG_LF.motorID);
	  	  HAL_Delay(50);
	  	  writeParameter(0x7005, &runMode, MOTOR_CG_LB.hostID, MOTOR_CG_LB.motorID);
	  	  HAL_Delay(50);
	  	  writeParameter(0x7005, &runMode, MOTOR_CG_RF.hostID, MOTOR_CG_RF.motorID);
	  	  HAL_Delay(50);
	  	  writeParameter(0x7005, &runMode, MOTOR__CG_RB.hostID, MOTOR__CG_RB.motorID);
	  	  HAL_Delay(50);

	  	  writeParameter(0x7017, &MOTOR_CG_LF.max_velocity, MOTOR_CG_LF.hostID, MOTOR_CG_LF.motorID);
	  	  HAL_Delay(50);

	  	  writeParameter(0x7017, &MOTOR_CG_LB.max_velocity, MOTOR_CG_LB.hostID, MOTOR_CG_LB.motorID);
	  	  HAL_Delay(50);

	  	  writeParameter(0x7017, &MOTOR_CG_RF.max_velocity, MOTOR_CG_RF.hostID, MOTOR_CG_RF.motorID);
	  	  HAL_Delay(50);

	  	  writeParameter(0x7017, &MOTOR__CG_RB.max_velocity, MOTOR__CG_RB.hostID, MOTOR__CG_RB.motorID);
	  	  HAL_Delay(50);
*/


	  	  // ------------------DEMO INICIALIZACIJA----------------

	  	  // Set the angle for both for one motor to 50deg and the other to -50deg

	  	  // Decide target angle for each motor
	  	/*  MOTOR_CG_LF.target_angle = 0.785398163f;

	  	  MOTOR_CG_LB.target_angle = -0.785398163f;

	  	  MOTOR_CG_RF.target_angle = -0.785398163f;

	  	  MOTOR__CG_RB.target_angle = 0.785398163f;*/

	  	  // DEBUG MODE
	  	  /*
	  	  MOTOR_CG_LF.target_angle = 0.523598776f;

	  	  MOTOR_CG_LB.target_angle = -0.523598776f;

	  	  MOTOR_CG_RF.target_angle = -0.523598776f;

	  	  MOTOR__CG_RB.target_angle = 0.523598776f;
	  	  */
	  	  MOTOR_CG_LF.target_angle = 0.0;

	  	  MOTOR_CG_LB.target_angle = -0.0;

	  	  MOTOR_CG_RF.target_angle = -0.0;

	  	  MOTOR__CG_RB.target_angle = 0.0;


	  	  // Send the target angle to each motor
	  	//Motor_SendMITCommand(&MOTOR_CG_LF, 1.0);
	  	//HAL_Delay(70);
	  	//Motor_SendMITCommand(&MOTOR_CG_LB, 1.0);
	  	//HAL_Delay(70);
	  	//Motor_SendMITCommand(&MOTOR_CG_RF, 1.0);
	  	//HAL_Delay(70);
	  	//Motor_SendMITCommand(&MOTOR__CG_RB, 1.0);
	  	//HAL_Delay(70);

	  	/*
	  	  Motor_SendAngle(&MOTOR_CG_LF);
	  	  HAL_Delay(70);
	  	  Motor_SendAngle(&MOTOR_CG_LB);
	  	  HAL_Delay(70);
	  	  Motor_SendAngle(&MOTOR_CG_RF);
	  	  HAL_Delay(70);
	  	  Motor_SendAngle(&MOTOR__CG_RB);
	  	  HAL_Delay(70);
*/
	  	  /* Ta Mechanical zero ne dela nic
		  // Zero the motor
		  setMechanicalZero(MOTOR_CG_LF.hostID, MOTOR_CG_LF.motorID);
		  HAL_Delay(50);

		  setMechanicalZero(MOTOR_CG_LB.hostID, MOTOR_CG_LB.motorID);
		  HAL_Delay(50);

		  setMechanicalZero(MOTOR_CG_RF.hostID, MOTOR_CG_RF.motorID);
		  HAL_Delay(50);

		  setMechanicalZero(MOTOR__CG_RB.hostID, MOTOR__CG_RB.motorID);
		  HAL_Delay(50);

		  HAL_Delay(3000);*/

	  	  // Set DDSM115 motor to current mode
	  	  DDMS115setMode(0x10, 0x01);  // 0x01 => current mode
	  	  HAL_Delay(10);
	  	  DDMS115setMode(0x11, 0x01);  // 0x01 => current mode

	  	  MIT_controler_gain_schedule_Normal();


}
