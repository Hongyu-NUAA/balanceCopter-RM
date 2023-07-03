#include "bsp_uart.h"
#include "main.h"
#include "stdio.h"
#include "string.h"

float motor_target_f[BALANCEBOT_MOTOR_NUM];
float servo_target_f[BALANCEBOT_SERVO_NUM];

uint8_t UART1_RX_BUF[RM_UART_MAX_LEN]; // ��������
uint8_t UART1_RX_LEN;                  // �������鳤��

union rmuart_t rmuart;
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
void bsp_uart1_init(void)
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);                  // �������ڿ����жϣ��������
    HAL_UART_Receive_DMA(&huart1, UART1_RX_BUF, RM_UART_MAX_LEN); // ����DMA����
}

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;

void bsp_uart1_handler(void)
{
    if ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);                           // �����־λ
        HAL_UART_DMAStop(&huart1);                                    // ֹͣDMA���գ���ֹ���ݳ���

        UART1_RX_LEN = 200 - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);  // ��ȡDMA�д�������ݸ���

        memcpy(rmuart.bits, UART1_RX_BUF, UART1_RX_LEN);              // ��������

        HAL_UART_Receive_DMA(&huart1, UART1_RX_BUF, RM_UART_MAX_LEN); // ��DMA���գ����ݴ��� UART1_RX_BUF ������
    }
}

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
void bsp_uart1_update()
{
    if (rmuart.rmuart_s.header[0] != 0xAA) {
        return;
    }

    if (rmuart.rmuart_s.header[1] != 0xAF) {
        return;
    }

    if (rmuart.rmuart_s.len != RM_UART_STRUCT_LEN) {
        return;
    }

    uint8_t i = 0;

    // �����ʽ����[-1~1], Ȼ��������ת��, �õ�ʵ��Ŀ��ת��
    for (i = 0; i < BALANCEBOT_MOTOR_NUM; i++) {
        motor_target_f[i] = (float)(rmuart.rmuart_s.motor[i] - 1000) / 1000.0f * MOTOR_MAX_SPEED;
    }

    // for (i = 0; i < BALANCEBOT_SERVO_NUM; i++) {
    //     motor_target_f[i] = (float)(rmuart.rmuart_s.servo[i] - 1000) / 1000.0f;
    // }
}
