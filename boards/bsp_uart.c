#include "bsp_uart.h"
#include "main.h"
#include "stdio.h"
#include "string.h"

float motor_target_f[BALANCEBOT_MOTOR_NUM];
float servo_target_f[BALANCEBOT_SERVO_NUM];

uint8_t UART1_RX_BUF[RM_UART_MAX_LEN]; // 缓存数组
uint8_t UART1_RX_LEN;                  // 缓存数组长度

union rmuart_t rmuart_rx;
union ardupilot_t ardupilot_tx;

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
void bsp_uart1_init(void)
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);                  // 开启串口空闲中断，必须调用
    HAL_UART_Receive_DMA(&huart1, UART1_RX_BUF, RM_UART_MAX_LEN); // 启动DMA接收
}

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;

void bsp_uart1_handler(void)
{
    if ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);                           // 清除标志位
        HAL_UART_DMAStop(&huart1);                                    // 停止DMA接收，防止数据出错

        UART1_RX_LEN = 200 - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);  // 获取DMA中传输的数据个数

        memcpy(rmuart_rx.bits, UART1_RX_BUF, UART1_RX_LEN);           // 更新数据

        HAL_UART_Receive_DMA(&huart1, UART1_RX_BUF, RM_UART_MAX_LEN); // 打开DMA接收，数据存入 UART1_RX_BUF 数组中
    }
}

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
void bsp_uart1_rx()
{
    if (rmuart_rx.rmuart_s.header[0] != 0xAA) {
        return;
    }

    if (rmuart_rx.rmuart_s.header[1] != 0xAF) {
        return;
    }

    if (rmuart_rx.rmuart_s.len != RM_UART_STRUCT_LEN) {
        return;
    }

    uint8_t i = 0;

    // 电机格式化到[-1~1], 然后乘最大电机转速, 得到实际目标转速
    for (i = 0; i < BALANCEBOT_MOTOR_NUM; i++) {
        motor_target_f[i] = (float)(rmuart_rx.rmuart_s.motor[i] - 1000) / 1000.0f * MOTOR_MAX_SPEED;
    }

    // for (i = 0; i < BALANCEBOT_SERVO_NUM; i++) {
    //     motor_target_f[i] = (float)(rmuart_rx.rmuart_rx_s.servo[i] - 1000) / 1000.0f;
    // }
}

void bsp_uart1_tx(void)
{
    ardupilot_tx.ardupilot_s.header[0] = 0xAA;
    ardupilot_tx.ardupilot_s.header[1] = 0xAC;

    ardupilot_tx.ardupilot_s.len = sizeof(struct ardupilot_struct);

    HAL_UART_Transmit(&huart1, ardupilot_tx.bits, sizeof(struct ardupilot_struct), 0xff);
}

void bsp_uart1_set_wheel_speed(int16_t wheel1, int16_t wheel2)
{
    ardupilot_tx.ardupilot_s.wheel_speed[0] = wheel1;
    ardupilot_tx.ardupilot_s.wheel_speed[1] = wheel2;
}
