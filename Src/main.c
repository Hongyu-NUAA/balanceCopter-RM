/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "CAN_receive.h"
#include "bsp_can.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "pid.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"


#ifdef __GNUC__									//�����ض���
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif 
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

#define IR_IN1 HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9) // ����pwm�ź����벶���ΪPE9,��ʱ��T1_CH1
#define IR_IN2 HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) // ����pwm�ź����벶���ΪPC6,��ʱ��T8_CH1

uint8_t time_up_flag_1 = 0;                        // ���������ر�־λ
uint8_t time_up_flag_2 = 0;
uint8_t time_dowm_flag_1 = 0;                      // �����½��ر�־λ
uint8_t time_dowm_flag_2 = 0;

uint32_t time_up_num_1 = 0;   // �����ؼ���
uint32_t time_up_num_2 = 0;
uint32_t time_dowm_num_1 = 0; // �½��ؼ���
uint32_t time_dowm_num_2 = 0;

float pulse_width_1;      // pwm����
float pulse_width_2;
float pulse_width_temp_1; // ��ʱ����洢����pmw�źŽ��й���
float pulse_width_temp_2;

RC_Ctl_t RC_Ctl;                     // ����ң�������ݽṹ��
uint8_t sbus_rx_buffer[18];          // ����ң������������
pid_type_def motor_pid_1;            // ����PID���ݽṹ��
pid_type_def motor_pid_2;
const motor_measure_t* motor_data_1; // ��������ṹ��ָ��
const motor_measure_t* motor_data_2;

int set_speed_1 = 0; // Ŀ���ٶ�
int set_speed_2 = 0;

const fp32 PID[3] = { 3, 0.1, 0.1 }; // P,I,D����

void SystemClock_Config(void);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{ 
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();

		can_filter_init();
    HAL_UART_Receive_DMA(&huart3, sbus_rx_buffer, 18);
    PID_init(&motor_pid_1, PID_POSITION, PID, 16000, 2000); // PID�ṹ�壬PID����ģʽ��PID���������ֵ�����Iֵ
    PID_init(&motor_pid_2, PID_POSITION, PID, 16000, 2000); // PID�ṹ�壬PID����ģʽ��PID���������ֵ�����Iֵ
    motor_data_1 = get_chassis_motor_measure_point(0);      // ��ȡIDΪ1�ŵĵ������ָ��
    motor_data_2 = get_chassis_motor_measure_point(1);      // ��ȡIDΪ2�ŵĵ������ָ��

    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);             // ʹ�ܶ�ʱ��1��ͨ��1���벶���ܣ���ʹ����Ӧ���ж�
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);             // ʹ�ܶ�ʱ��1��ͨ��1���벶���ܣ���ʹ����Ӧ���ж�

  while (1)
  {
				HAL_TIM_IC_CaptureCallback(&htim1); // ѭ���������벶����������ʱ��ͨ��pwm�ź�ֵ
        HAL_TIM_IC_CaptureCallback(&htim8);

        pulse_width_1 = (float)time_dowm_num_1 - (float)time_up_num_1; // ���㶨ʱ��1ͨ��1��PWM����ֵ
        pulse_width_2 = (float)time_dowm_num_2 - (float)time_up_num_2; // ���㶨ʱ��8ͨ��1��PWM����ֵ

                                                                       //		if ( pulse_width_1 >= 2000 )																													//����������޷����𵽹��˵�����
        //		{ pulse_width_1 = pulse_width_temp[0];		}
        //		else if ( pulse_width_1 <= 1000 )
        //		{ pulse_width_1 = pulse_width_temp[0];		}
        //
        //		if ( pulse_width_2 >= 2000 )
        //		{ pulse_width_2 = pulse_width_temp[1];		}
        //		else if ( pulse_width_2 <= 1000 )
        //		{ pulse_width_2 = pulse_width_temp[1];		}

        //		pulse_width_temp[0] = pulse_width_1;																									//����ǰ����ֵ������ʱ�洢����
        //		pulse_width_temp[1] = pulse_width_2;

        pulse_width_1 = (pulse_width_1 / 10 + 4000) / 4; // ����Χӳ��Ϊ-500~500֮��
        pulse_width_2 = (pulse_width_2 / 10 + 4000) / 4;

        //		if(pulse_width_1)

        set_speed_1 = pulse_width_1 * 50; // �ٶȷ�ΧΪ-25000~25000
        set_speed_2 = pulse_width_2 * 50;

        PID_calc(&motor_pid_1, motor_data_1->speed_rpm, set_speed_1); // ������pid�����PID�ṹ�壬ʵ���ٶȣ��趨�ٶ�
        PID_calc(&motor_pid_2, motor_data_2->speed_rpm, set_speed_2); // ������pid�����PID�ṹ�壬ʵ���ٶȣ��趨�ٶ�

        CAN_cmd_chassis(motor_pid_1.out, motor_pid_2.out, 0, 0);      // ���ͼ����Ŀ��Ƶ��������1�͵��2�����3��4������Ϊ0

        HAL_Delay(2);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }


  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) // �����жϻص�������ÿ�β����źžͻ���뵽�������
{

    if (htim->Instance == TIM1) // �ж��Ƿ��Ƕ�ʱ��1���ⲿ�����1
    {
        if (htim->Channel == TIM_CHANNEL_1) {
            if (IR_IN1 && time_up_flag_1 == 0)                                                          // �ж�IO���ǵ͵�ƽ�������ر�־λ��0�����ǵ�һ������
            {
                time_up_flag_1 = 1;                                                                     // �����ر�־λ��1
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING); // �ı䲶����Ϊ�½��ز���
                __HAL_TIM_SET_COUNTER(&htim1, 0);                                                       // �������㣬��ͷ��ʼ����
            } else if (IR_IN1 == 0 && time_dowm_flag_1 == 0)                                            // �ж�IO��Ϊ�͵�ƽ���½��ر�־λ��0�����ǵ�һ���½�
            {
                time_dowm_num_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);                       // ��ȡ������������ʱ�伴Ϊ�����س���ʱ��
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);  // �ı䲶����Ϊ�����ز���
                time_dowm_flag_1 = 1;                                                                   // �½��ر�־λ��1
            } else if (IR_IN1 && time_dowm_flag_1 == 1)                                                 // �ж�IO��Ϊ�ߵ�ƽ���½��ر�־λΪ1�����ǵڶ�������
            {
                time_up_num_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);                         // ��ȡ������������ʱ�伴Ϊ������ʱ��
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING); // �ı䲶����Ϊ�½��ز���
                time_dowm_flag_1 = 0;                                                                   // �½��ر�־λ��0
                __HAL_TIM_SET_COUNTER(&htim1, 0);                                                       // �������㣬��ͷ��ʼ����
            }
        }
    }
    if (htim->Instance == htim8.Instance) {
        if (htim->Channel == TIM_CHANNEL_1)                                                             // �ж��Ƿ��Ƕ�ʱ��8���ⲿ�����1
        {
            if (IR_IN2 && time_up_flag_2 == 0)                                                          // �ж�IO���ǵ͵�ƽ�������ر�־λ��0�����ǵ�һ������
            {
                time_up_flag_2 = 1;                                                                     // �����ر�־λ��1
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING); // �ı䲶����Ϊ�½��ز���
                __HAL_TIM_SET_COUNTER(&htim8, 0);                                                       // �������㣬��ͷ��ʼ����
            } else if (IR_IN2 == 0 && time_dowm_flag_2 == 0)                                            // �ж�IO��Ϊ�͵�ƽ���½��ر�־λ��0�����ǵ�һ���½�
            {
                time_dowm_num_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);                       // ��ȡ������������ʱ�伴Ϊ�����س���ʱ��
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);  // �ı䲶����Ϊ�����ز���
                time_dowm_flag_2 = 1;                                                                   // �½��ر�־λ��1
            } else if (IR_IN2 && time_dowm_flag_2 == 1)                                                 // �ж�IO��Ϊ�ߵ�ƽ���½��ر�־λΪ1�����ǵڶ�������
            {
                time_up_num_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);                         // ��ȡ������������ʱ�伴Ϊ������ʱ��
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING); // �ı䲶����Ϊ�½��ز���
                time_dowm_flag_2 = 0;                                                                   // �½��ر�־λ��0
                __HAL_TIM_SET_COUNTER(&htim8, 0);                                                       // �������㣬��ͷ��ʼ����
            }
        }
    }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef* UartHandle)
{
    RC_Ctl.rc.ch0 = (sbus_rx_buffer[0] | (sbus_rx_buffer[1] << 8)) & 0x07ff;
    RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;
    RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;
    RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;
    RC_Ctl.rc.s1 = ((sbus_rx_buffer[5] >> 4) & 0x000C) >> 2;
    RC_Ctl.rc.s2 = ((sbus_rx_buffer[5] >> 4) & 0x0003);

    RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);   //!< Mouse X axis
    RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);   //!< Mouse Y axis
    RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8); //!< Mouse Z axis
    RC_Ctl.mouse.press_l = sbus_rx_buffer[12];                       //!< Mouse Left Is Press
    RC_Ctl.mouse.press_r = sbus_rx_buffer[13];                       //!< Mouse Right Is Press
    RC_Ctl.key.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8);   //!< KeyBoard value
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif /* USE_FULL_ASSERT */
