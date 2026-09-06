#ifndef __MYTASK_H
#define __MYTASK_H

void MyTask_Init(void);
void Task_Key_Get(void *pvParameters);
void Task_Led_Display(void *pvParameters);
void Task_OLED_Display(void *pvParameters);
void Task_UART_Service(void *pvParameters);
void Task_BlueSerial_Service(void *pvParameters);
void Task_NRF24L01_Service(void *pvParameters);
void TasK_Test(void *pvParemeters);
#endif
