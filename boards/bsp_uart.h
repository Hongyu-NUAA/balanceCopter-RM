#ifndef BSP_UART_H
#define BSP_UART_H

#include "usart.h"

#define RM_UART_MAX_LEN    50
#define RM_UART_STRUCT_LEN 15

#define BALANCEBOT_MOTOR_NUM 2
#define BALANCEBOT_SERVO_NUM 2

#define MOTOR_MAX_SPEED 1000                  // 此处为最大的电机转速

extern uint8_t UART1_RX_BUF[RM_UART_MAX_LEN]; // 缓存数组
extern uint8_t UART1_RX_LEN;                  // 缓存数组长度

void bsp_uart1_handler(void);
void bsp_uart1_init(void);
void bsp_uart1_update(void);

union rmuart_t {
#pragma pack(1) /*指定按1字节对齐*/
    struct rmuart_struct {
        uint8_t header[2];
        uint8_t len;
        uint32_t timestamp_ms;
        uint16_t motor[BALANCEBOT_MOTOR_NUM];
        uint16_t servo[BALANCEBOT_SERVO_NUM];
    } rmuart_s;
#pragma pack() /*取消指定对齐，恢复缺省对齐*/

    uint8_t bits[sizeof(struct rmuart_struct)];
};

extern float motor_target_f[BALANCEBOT_MOTOR_NUM];
extern float servo_target_f[BALANCEBOT_SERVO_NUM];

#endif
