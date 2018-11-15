/*
 * lwj.c
 *
 *  Created on: 2018. 10. 18.
 *      Author: lwj
 */

#include "../AHockey_Inc/lwj.h"

#include "../AHockey_Inc/khy.h"

extern TIM_HandleTypeDef htim1;
int cnt = 0;


//////////////////////////////////////////////////////////////
/*
 *  ���ܸ��� ���� �����ϴ� �Լ�.
 *  �Լ� ���� : �¿���ϸ� ���� ������ �� �ֵ��� ��.
 *
 *	Step_Stop()            : ����.
 *  Step_GoingRight()      : ���������� �̵�.(��)
 *  Step_GoingLeft()       : ���ʷ� �̵�.(��)
 *  Step_GoingDown()       : �Ʒ��� �̵�.(��)
 *  Step_GoingUp()         : ���� �̵�.(��)
 *  Step_GoingRight_Up()   : ������ ���� �̵�.(��)
 *  Step_GoingLeft_Up()    : �������� �̵�.(��)
 *  Step_GoingRight_Down() : ������ �Ʒ��� �̵�.(��)
 *  Step_GoingLeft_Down()  : ���� �Ʒ��� �̵�.(��)
 *
 */
//////////////////////////////////////////////////////////////

void Step_Stop(void)
{
	Step_Motor_Control(MOTOR_LEFT, CNT_CLK_WISE, 80, MOVE_OFF);
	Step_Motor_Control(MOTOR_RIGHT, CNT_CLK_WISE, 80, MOVE_OFF);
}

void Step_GoingRight(void)
{
	Step_Motor_Control(MOTOR_LEFT, CNT_CLK_WISE, 85, MOVE_ON);
	Step_Motor_Control(MOTOR_RIGHT, CNT_CLK_WISE, 85, MOVE_ON);
}

void Step_GoingLeft(void)
{
	Step_Motor_Control(MOTOR_LEFT, CLK_WISE, 85, MOVE_ON);
	Step_Motor_Control(MOTOR_RIGHT, CLK_WISE, 85, MOVE_ON);
}

void Step_GoingDown(void)
{
	Step_Motor_Control(MOTOR_LEFT, CNT_CLK_WISE, 85, MOVE_ON);
	Step_Motor_Control(MOTOR_RIGHT, CLK_WISE, 85, MOVE_ON);
}

void Step_GoingUp(void)
{
	Step_Motor_Control(MOTOR_LEFT, CLK_WISE, 85, MOVE_ON);
	Step_Motor_Control(MOTOR_RIGHT, CNT_CLK_WISE, 85, MOVE_ON);
}

void Step_GoingRight_Up(void)
{
	Step_Motor_Control(MOTOR_LEFT, CLK_WISE, 85, MOVE_OFF);
	Step_Motor_Control(MOTOR_RIGHT, CNT_CLK_WISE, 85, MOVE_ON);
}

void Step_GoingLeft_Up(void)
{
	Step_Motor_Control(MOTOR_LEFT, CLK_WISE, 85, MOVE_ON);
	Step_Motor_Control(MOTOR_RIGHT, CNT_CLK_WISE, 85, MOVE_OFF);
}

void Step_GoingRight_Down(void)
{
	Step_Motor_Control(MOTOR_LEFT, CNT_CLK_WISE, 85, MOVE_ON);
	Step_Motor_Control(MOTOR_RIGHT, CNT_CLK_WISE, 85, MOVE_OFF);
}

void Step_GoingLeft_Down(void)
{
	Step_Motor_Control(MOTOR_LEFT, CNT_CLK_WISE, 85, MOVE_OFF);
	Step_Motor_Control(MOTOR_RIGHT, CLK_WISE, 85, MOVE_ON);
}

//////////////////////////////////////////////////////////////
/*
 *  ���ܸ��� ���� �Լ� �޴� ���� 4��.
 *  �Լ� ���� : [���ͼ���] / [����(�ð�, �ð�ݴ�)] / [Pulse�ֱ⸦ �޾� ���ܸ��� ����] / [0:����, 1:������].
 *	�����ǻ���, Motor_Speed �ּҰ�: 90 (��������ӵ�), �ִ밪: 270 (���� ���� �ӵ�)
 *		========================================
 *		���ʸ���     �����ʸ���     |   ���
 *		========================================
 *		�ð����     �ð����        |  (��)���� �̵�
 *		�ݽð����  �ݽð����     |  (��)������ �̵�
 *		�ð����     �ݽð����     |  (��)���� �̵�
 *		�ݽð����  �ð����        |  (��)�Ʒ��� �̵�
 *
 *		����     	 �ð����         |  (��)���� �Ʒ� �̵�
 *		����     	 �ݽð����      |  (��)������ �� �̵�
 *		�ݽð�       ����              |  (��)������ �Ʒ� �̵�
 *		�ð����    ����              |  (��)���� �� �̵�


 */
//////////////////////////////////////////////////////////////

void Step_Motor_Control(const uint8_t Motor_Select, const uint8_t Direction, uint32_t Motor_Speed, uint8_t Motor_Stop)
{
	switch(Motor_Select)		//	���� ���� (���� �Ǵ� ������)
	{
	case MOTOR_LEFT:
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, Direction);	//	�ð�, �ð�ݴ� ���� ����.
		TIM1->ARR = Motor_Speed;							//	����  Auto-Reload Register�� ����.
		TIM1->CCR1 = TIM1->ARR/2;							//	��Ƽ�� 50%�� ����.
		if(Motor_Stop == MOVE_OFF)
		{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);	//	MOVE_OFF �Է½� ���� ����.
		}
		break;

	case MOTOR_RIGHT:
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, Direction);
		TIM1->ARR = Motor_Speed;
		TIM1->CCR2 = TIM1->ARR/2;
		if(Motor_Stop == MOVE_OFF)
		{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
		}
		break;
	}
}


uint16_t pulse_count1=0;
uint16_t pulse_count2=0;
uint8_t step_flag=1;

uint8_t Step_pulse(uint16_t pulse_num,uint16_t motor_select,uint16_t direction)
{
  if(pulse_count1 == 0 && motor_select == MOTOR_LEFT){
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, direction);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 0);
    pulse_count1 = pulse_num*2;
  }
  else if(pulse_count2 == 0 && motor_select == MOTOR_RIGHT){
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, direction);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 0);
    pulse_count2 = pulse_num*2;
  }
  else
    return 1;
  return 0;
}

void pulse_start()
{
  if(step_flag == 1){
    HAL_TIM_Base_Start_IT(&htim1);
    step_flag = 0;
  }
}



//////////////////////////////////////////////////////////////
/*
 *  WatchDog �Լ�.
 *  �Լ� ���� : 0.5���̻� ������ ���� ��, �ý����� ������ϴ� �Լ�.
 *            WatchDog�� �ʿ��� ���� �Լ��� ������ �ȴ�.
 *            ���۵����� ���� �ý��� ����ý� ������ ������ LED�� ���� ���´�. (��, Watchdog�� ���� �����)
 */
//////////////////////////////////////////////////////////////

void Watch_Dog(void)
{
	//__HAL_IWDG_RELOAD_COUNTER(&hiwdg);   //  System�� Reset���� �ʵ��� �������� ī��Ʈ�Ѵ�. (0.5���̻�Ǹ� ����)

	// �ý����� ����� �Ǿ����� Ȯ���ϴ� �Լ�.
	if(RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);  // ������ LED�� ����� ���� Ȯ��.
		__HAL_RCC_CLEAR_RESET_FLAGS();
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	}
}
