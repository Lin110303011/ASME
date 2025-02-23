# ASME

## 底盤控制
### 底盤架構圖  
![底盤架構](image.png)
### 函式庫
1. ToF 讀取 *(尚未測試)* https://github.com/Squieler/VL53L0X---STM32-HAL.git
2. 馬達驅動(L298N) &#10004; 
3. PID控制 &#10004;
4. IMU讀取 *(尚未整理)*

### 底盤整合 version 1.0(2025/02/24)
以 stm32 Nucleo f446RE 為主控制器，整合馬達控制、ToF感測器、PID控制，建立ASME_Chassis。
編譯與執行以vscode為主，尚未實際測試。
