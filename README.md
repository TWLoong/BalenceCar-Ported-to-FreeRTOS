## BalanceCar‑Ported‑to‑FreeRTOS
STM32两轮平衡小车，裸机工程移植到FreeRTOS

## 项目简介
本项目基于江协科技的裸机两轮倒立摆平衡小车工程移植到 FreeRTOS 实时操作系统。

江协科技平衡车资料获取网站：https://jiangxiekeji.com/index.html

## 软件平台
1.Keil 5

2.标准库

3.FreeRTOS

## 硬件平台
1.主控芯片：STM32F103

2.姿态传感器：MPU6050

3.动力单元：带编码器直流减速电机

4.通信模块：HC‑05蓝牙模块、NRF24L01无线模块

5.外设：OLED显示屏、LED指示灯、独立按键

## 待优化
1.移植FreeRtos后小车平衡效果不如裸机版本

2.蓝牙前进后退快速控制效果不佳，慢速尚可，无线遥控对小车的控制延迟较大

<img width="1280" height="961" alt="0c1814e9bf1f8b4d42c7478becdf80a6_720" src="https://github.com/user-attachments/assets/19db1770-5b54-46ae-8c5a-2eb73544fbea" />

