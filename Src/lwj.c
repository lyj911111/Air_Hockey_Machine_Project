/*
 * lwj.c
 *
 *  Created on: 2018. 10. 18.
 *      Author: lwj
 */

#include "lwj.h"

//////////////////////////////////////////////////////////////
/*
 *  ���ܸ��� ���� �Լ�
 *  �Լ� ���� : ���ͼ���, ����, PWM��ȣ�� �޾� ���ܸ��� ����
 *
 *
 */
//////////////////////////////////////////////////////////////

void Step_Motor(const uint8_t Motor_Select, const uint8_t Direction, uint32_t Motor_PWM)
{

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
	__HAL_IWDG_RELOAD_COUNTER(&hiwdg);   //  System�� Reset���� �ʵ��� �������� ī��Ʈ�Ѵ�. (0.5���̻�Ǹ� ����)

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
