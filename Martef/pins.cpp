// PINS.CPP
// Configuration of IO pins	
// COPYRIGHT 2012 Sasha Chrichov

#include "stm32h7xx.h"

#include "pins.h"

struct IoConfStruct {
  unsigned int Register : 5;
  unsigned int Bit : 5;
  unsigned int Mode : 2;        // 0-input, 1-output, 2-mapped, 3-analog
  unsigned int Mapping : 4;     // Up to 16 alternate functions 
  unsigned int OpenDrain : 1;   // 0-pushpull, 1-opendrain (output only)
  unsigned int Speed : 3;       // 0-2 Mhz, 1-25 MHz, 2-50 MHz, 3-100 MHz (output only)
  unsigned int UpDown : 2;      // 0-none, 1-pullup, 2-pulldown
  unsigned int Initial : 1;     // Initial state for output
}; 
 
/*
// For AB-07
#define nIoPins   51
struct IoConfStruct IoConf[nIoPins] = {
  { 0,  9,  2,  7,  0,  2,  0,  0},     // A9 to USART1 TX, push-pull, 50 MHz, no pull-up/down
  { 0, 10,  2,  7,  0,  2,  0,  0},     // A10 to USART1 RX, push-pull, 50 MHz, no pull-up/down
  { 1,  6,  2,  4,  1,  2,  0,  0},     // B6 to I2C1 SCL, open-drain, 50 MHz, no pull-up/down
  { 1,  7,  2,  4,  1,  2,  0,  0},     // B7 to I2C1 SDA, open-drain, 50 MHz, no pull-up/down
  { 1,  3,  2,  5,  0,  2,  0,  0},     // B3 to SPI1 SCK, push-pull, 50 MHz, no pull-up/down
  { 1,  4,  2,  5,  0,  2,  0,  0},     // B4 to SPI1 MISO, push-pull, 50 MHz, no pull-up/down
  { 1,  5,  1,  0,  0,  0,  0,  0},     // B5 output, 2 MHz (Dummy SPI1 MOSI)
  { 0, 15,  1,  0,  0,  2,  0,  0},     // A15 output, 50 MHz (SPI1 NSS)
  { 2,  9,  1,  0,  0,  2,  0,  0},     // C9 output, 50 MHz (USART6 CS)
  { 2,  8,  2,  8,  0,  2,  0,  0},     // C8 to USART6 CK, push-pull, 50 MHz, no pull-up/down
  { 2,  6,  2,  8,  0,  2,  0,  0},     // C6 to USART6 TX, push-pull, 50 MHz, no pull-up/down
  { 4,  8,  2,  1,  0,  3,  0,  0},     // E8 to TIM1 CH1N, push-pull, 100 MHz, no pull-up/down (PWM2B)
  { 4,  9,  2,  1,  0,  3,  0,  0},     // E9 to TIM1 CH1, push-pull, 100 MHz, no pull-up/down (PWM2A)
  { 4, 10,  2,  1,  0,  3,  0,  0},     // E10 to TIM1 CH2N, push-pull, 100 MHz, no pull-up/down (PWM1B)
  { 4, 11,  2,  1,  0,  3,  0,  0},     // E11 to TIM1 CH2, push-pull, 100 MHz, no pull-up/down (PWM1A)
  { 4, 12,  2,  1,  0,  3,  0,  0},     // E12 to TIM1 CH3N, push-pull, 100 MHz, no pull-up/down (PWM3B-pushpull)
  { 4, 13,  2,  1,  0,  3,  0,  0},     // E13 to TIM1 CH3, push-pull, 100 MHz, no pull-up/down (PWM3A-pushpull)
  { 3,  0,  0,  0,  0,  0,  1,  0},     // D0 input, 2 MHz, pull-up (ENC_PWR_FAULT)
  { 3,  1,  0,  0,  0,  0,  1,  0},     // D1 input, 2 MHz, pull-up (ENCODER_PWR_EN)
  { 3,  3,  0,  0,  0,  0,  1,  0},     // D3 input, 2 MHz, pull-up (MOTOR_CONNECTED)
  { 3,  4,  0,  0,  0,  0,  1,  0},     // D4 input, 2 MHz, pull-up (AB5_SET_OFFSET_MCU)
  { 3,  5,  0,  0,  0,  0,  1,  0},     // D5 input, 2 MHz, pull-up (AB1/AB5_MODE)
  { 3,  6,  0,  0,  0,  0,  1,  0},     // D6 input, 2 MHz, pull-up (UHR_MODE)
  { 3,  7,  0,  0,  0,  0,  1,  0},     // D7 input, 2 MHz, pull-up (DC_MODE)
  { 3,  8,  0,  0,  0,  0,  1,  0},     // D8 input, 2 MHz, pull-up (LIMIT_LEFT_MCU)
  { 1,  8,  0,  0,  0,  0,  1,  0},     // B8 input, 2 MHz, pull-up (LIMIT_RIGHT_MCU)
  { 3,  9,  0,  0,  0,  0,  1,  0},     // D9 input, 2 MHz, pull-up (ENABLE)
  { 3, 10,  0,  0,  0,  0,  1,  0},     // D10 input, 2 MHz, pull-up (EMERGANCY_STOP)
  { 3, 12,  1,  0,  0,  1,  0,  0},     // D12 output, 25 MHz (DIGITAL_OUT)
  { 3, 13,  1,  0,  0,  1,  0,  0},     // D13 output, 25 MHz (DC_ENABLE)
  { 3, 14,  1,  0,  0,  1,  0,  0},     // D14 output, 25 MHz (24V_DC_ON)
  { 3, 15,  1,  0,  0,  1,  0,  0},     // D15 output, 25 MHz (Black_SW)
  { 4,  0,  1,  0,  0,  1,  0,  0},     // E0 output, 25 MHz (White_SW)
  { 4,  1,  1,  0,  0,  1,  0,  0},     // E1 output, 25 MHz (Red_SW)
  { 2, 13,  1,  0,  0,  1,  0,  1},     // C13 output, 25 MHz (24V_ISO_Disable)
  { 1,  9,  1,  0,  0,  1,  0,  0},     // B9 output, 25 MHz (FAULT), 0 = No fault
  { 4,  3,  0,  0,  0,  0,  0,  0},     // E3 input, 2 MHz (LIMIT_LEFT_ENC)
  { 4,  4,  0,  0,  0,  0,  0,  0},     // E4 input, 2 MHz (LIMIT_RIGHT_ENC,  0)
  { 2,  4,  3,  0,  0,  0,  0,  0},     // C4 to analog input (User)
  { 0,  2,  3,  0,  0,  0,  0,  0},     // A2 to analog input (ID)
  { 0,  3,  3,  0,  0,  0,  0,  0},     // A3 to analog input (OV1)
  { 0,  4,  3,  0,  0,  0,  0,  0},     // A4 to analog output (DAC1)
  { 0,  5,  3,  0,  0,  0,  0,  0},     // A5 to analog input (24V_CHECK)
  { 0,  6,  3,  0,  0,  0,  0,  0},     // A6 to analog input (3V3_CHECK)
  { 0,  7,  3,  0,  0,  0,  0,  0},     // A7 to analog input (5V_CHECK)
  { 2,  5,  3,  0,  0,  0,  0,  0},     // C5 to analog input (12V_CHECK)
  { 2,  2,  3,  0,  0,  0,  0,  0},     // C2 to analog input (24VCM)
  { 2,  3,  3,  0,  0,  0,  0,  0},     // C3 to analog input (24VDC_CM)
  { 1,  0,  3,  0,  0,  0,  0,  0},     // B0 to analog input (ADC_AIN_backup)
  { 1,  1,  3,  0,  0,  0,  0,  0},     // B1 to analog input (ADC_0.5_VREF)
  { 2,  1,  0,  0,  0,  0,  1,  0},     // C1 input, 2 MHz, pull-up (2.	LOG_P_S_HR/SE)
};
*/

#define nIoPins   4
struct IoConfStruct IoConf[nIoPins] = {
  { 3,  8,  2,  7,  0,  2,  0,  0},     // D8 to USART3 TX, push-pull, 50 MHz, no pull-up/down
  { 3,  9,  2,  7,  0,  2,  0,  0},     // D9 to USART3 RX, push-pull, 50 MHz, no pull-up/down
  { 2,  2,  1,  0,  1,  0,  0,  1},     // C2 - Red LED
  { 2,  3,  1,  0,  1,  0,  0,  1},     // C2 - Green LED
};

void PinSet(struct IoConfStruct IoC) {
  RCC->AHB4ENR |= ((uint32_t)0x00000001)<<IoC.Register;             // Enable GPIOn clock
  GPIO_TypeDef* GPIOn = (GPIO_TypeDef*)(GPIOA_BASE+0x0400*IoC.Register);
  if (IoC.Bit<8) {
    GPIOn->AFR[0] = (GPIOn->AFR[0] & ~(0x0000000F<<(4*IoC.Bit))) | (IoC.Mapping<<(4*IoC.Bit));
  } else {
    GPIOn->AFR[1] = (GPIOn->AFR[1] & ~(0x0000000F<<(4*(IoC.Bit-8)))) | (IoC.Mapping<<(4*(IoC.Bit-8)));
  }  
  GPIOn->ODR = (GPIOn->ODR & ~(0x00000003<<IoC.Bit)) | (IoC.Initial<<IoC.Bit);
  GPIOn->MODER = (GPIOn->MODER & ~(0x00000003<<(2*IoC.Bit))) | (IoC.Mode<<(2*IoC.Bit));         
  GPIOn->OSPEEDR = (GPIOn->OSPEEDR & ~(0x00000003<<(2*IoC.Bit))) | (IoC.Speed<<(2*IoC.Bit));     
  GPIOn->PUPDR = (GPIOn->PUPDR & ~(0x00000003<<(2*IoC.Bit))) | (IoC.UpDown<<(2*IoC.Bit));
  GPIOn->OTYPER = (GPIOn->OTYPER & ~(0x00000001<<(IoC.Bit))) | (IoC.OpenDrain<<(IoC.Bit));
}

void PinsInit() {
  for (int i=0; i<nIoPins; i++) {
    PinSet(IoConf[i]);
  }
}
