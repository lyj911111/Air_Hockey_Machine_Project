/*
 * khy.c
 *
 *  Created on: 2018. 10. 18.
 *      Author: lwj
 */

#include "khy.h"
#include "lwj.h"


int Math_Mapping(int val, int from_min, int from_max, int to_min, int to_max)
{
	return (val - from_min) * (to_max - to_min)/(from_max - from_min) + to_min;
}



int Reverse_Mapping(int map_val, int max, int min)
{
	return (max-map_val)+min;
}


void Mapped_Value_Controller(int value1, int value2, int low_boundary, int high_boundary)
{


	if(value1 >=low_boundary && value1 <=high_boundary)
		  {
			  if(value2 < low_boundary) // (¡è)
			  {
				  Step_Motor_Control(MOTOR_LEFT,CLK_WISE, 90, 1);
				  Step_Motor_Control(MOTOR_RIGHT, CNT_CLK_WISE, 90, 1);
				  osDelay(1);
			  }
			  else if(value2 > high_boundary) // (¡é)
			  {
				  Step_Motor_Control(MOTOR_LEFT, CNT_CLK_WISE, 90, 1);
				  Step_Motor_Control(MOTOR_RIGHT, CLK_WISE, 90, 1);
				  osDelay(1);
			  }
			  else if(value2 >=low_boundary && value2 <=high_boundary)// STOP
			  {

				  Step_Motor_Control(MOTOR_LEFT, CNT_CLK_WISE, 270, 0);
				  Step_Motor_Control(MOTOR_RIGHT, CLK_WISE, 270, 0);
				  osDelay(1);
			  }
		  }

		  else if(value1 < low_boundary)
		  {
			  if(value2 >=low_boundary && value2 <=high_boundary) // (¡ç)
			  {
				  Step_Motor_Control(MOTOR_LEFT, CLK_WISE, 90, 1);
				  Step_Motor_Control(MOTOR_RIGHT, CLK_WISE, 90, 1);
				  osDelay(1);
			  }
			  else if(value2 > high_boundary) // (¢×)
			  {
				  Step_Motor_Control(MOTOR_LEFT, CLK_WISE, 270, 0);
				  Step_Motor_Control(MOTOR_RIGHT, CLK_WISE, 90, 1);
				  osDelay(1);
			  }
			  else if(value2 < low_boundary) // (¢Ø)
			  {
				  Step_Motor_Control(MOTOR_LEFT, CLK_WISE, 90, 1);
				  Step_Motor_Control(MOTOR_RIGHT, CLK_WISE, 270, 0);
				  osDelay(1);
			  }
		  }

		  else if(value1 > high_boundary)
		  {
			  if(value2 < low_boundary) // (¢Ö)
			  {
				  Step_Motor_Control(MOTOR_LEFT, CLK_WISE, 270, 0);
				  Step_Motor_Control(MOTOR_RIGHT, CNT_CLK_WISE, 90, 1);
				  osDelay(1);
			  }
			  else if(value2 > high_boundary) // (¢Ù)
			  {
				  Step_Motor_Control(MOTOR_LEFT, CNT_CLK_WISE, 90, 1);
				  Step_Motor_Control(MOTOR_RIGHT, CLK_WISE, 270, 0);
				  osDelay(1);
			  }
			  else if(value2 >=low_boundary && value2 <=high_boundary) // (¡æ)
			  {
				  Step_Motor_Control(MOTOR_LEFT, CNT_CLK_WISE, 90, 1);
				  Step_Motor_Control(MOTOR_RIGHT, CNT_CLK_WISE, 90, 1);
				  osDelay(1);
			  }
		  }
}
