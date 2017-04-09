/**
 * unsigned char  uint8_t
 * unsigned short uint16_t
 * unsigned int   uint32_t
 */
#include "stm32f0xx.h"
//#include "arm_math.h"
#include "stdlib.h"

#define CLOCK_SPEED 16000000
#define TIMER14_PERIOD 24
#define TIMER14_PRESCALER 0xFFFF
#define TIMER14_TACTS_PER_SECOND (CLOCK_SPEED / TIMER14_PERIOD / TIMER14_PRESCALER)

#define TIMER14_100MS 1
#define TIMER14_200MS 2
#define TIMER14_500MS 5
#define TIMER14_1S 10
#define TIMER14_2S 20
#define TIMER14_3S 30
#define TIMER14_5S 50
#define TIMER14_10S 102
#define TIMER14_30S 305
#define TIMER14_60S 610
#define TIMER14_10MIN 6103

volatile unsigned int general_flags;
volatile unsigned short spi_sender_counter;

void Clock_Config();
void Pins_Config();
void TIMER14_Confing();
void SPI_Config();
void set_flag(unsigned int *flags, unsigned int flag_value);
void reset_flag(unsigned int *flags, unsigned int flag_value);
unsigned char read_flag_state(unsigned int *flags, unsigned int flag_value);

void TIM14_IRQHandler() {
   TIM_ClearITPendingBit(TIM14, TIM_IT_Update);

   spi_sender_counter++;
}

void SPI1_IRQHandler(void) {
   if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) == SET) {
      SPI_SendData8(SPI1, 0xAA);
      SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
   }
}

int main() {
   Clock_Config();
   Pins_Config();
   TIMER14_Confing();
   SPI_Config();

   while (1) {
      if (spi_sender_counter >= TIMER14_10S) {
         spi_sender_counter = 0;
         // Enable the Tx buffer empty interrupt
         SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
      }
   }
}

void Clock_Config() {
   RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
   RCC_PLLCmd(DISABLE);
   while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == SET);
   RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_4); // 8MHz / 2 * 4 = 16MHz
   RCC_PCLKConfig(RCC_HCLK_Div1);
   RCC_PLLCmd(ENABLE);
   while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
   RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
}

void Pins_Config() {
   // Connect BOOT0 directly to ground, RESET to VDD with a resistor

   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

   GPIO_InitTypeDef ports_default_config;
   ports_default_config.GPIO_Pin = GPIO_Pin_All & ~(GPIO_Pin_13 | GPIO_Pin_14); // PA13, PA14 - Debugger pins
   ports_default_config.GPIO_Mode = GPIO_Mode_IN;
   ports_default_config.GPIO_Speed = GPIO_Speed_Level_1; // 2 MHz
   ports_default_config.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_Init(GPIOA, &ports_default_config);

   ports_default_config.GPIO_Pin = GPIO_Pin_All;
   GPIO_Init(GPIOB, &ports_default_config);

   GPIO_Init(GPIOF, &ports_default_config);
}

/**
 * 0.0983s with 16MHz clock
 */
void TIMER14_Confing() {
   DBGMCU_APB1PeriphConfig(DBGMCU_TIM14_STOP, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

   TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
   TIM_TimeBaseStructure.TIM_Period = TIMER14_PERIOD;
   TIM_TimeBaseStructure.TIM_Prescaler = TIMER14_PRESCALER;
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

   TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
   NVIC_EnableIRQ(TIM14_IRQn);
   TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);

   TIM_Cmd(TIM14, ENABLE);
}

void SPI_Config() {
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

   GPIO_InitTypeDef pins_config;
   pins_config.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
   pins_config.GPIO_Mode = GPIO_Mode_AF;
   pins_config.GPIO_Speed = GPIO_Speed_Level_3; // 10 MHz
   pins_config.GPIO_OType = GPIO_OType_PP;
   pins_config.GPIO_PuPd = GPIO_PuPd_DOWN;
   GPIO_Init(GPIOA, &pins_config);

   // YES! Firstly pins MUST be configured and only after their alternate config. Not like in doc of driver.
   //GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_0);
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0);
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0); // SPI1_MOSI

   SPI_InitTypeDef spi_config;
   spi_config.SPI_Direction = SPI_Direction_1Line_Tx;
   spi_config.SPI_Mode = SPI_Mode_Master;
   spi_config.SPI_DataSize = SPI_DataSize_8b;
   spi_config.SPI_CPOL = SPI_CPOL_Low;
   spi_config.SPI_CPHA = SPI_CPHA_2Edge;
   spi_config.SPI_NSS = SPI_NSS_Soft; // When SPI_NSS_Hard, the pin MUST be connected to Vcc
   spi_config.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; // 16MHz / 2Mhz of ESP8266
   spi_config.SPI_FirstBit = SPI_FirstBit_MSB;
   SPI_Init(SPI1, &spi_config);

   NVIC_InitTypeDef NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

   SPI_Cmd(SPI1, ENABLE);
}

void set_flag(unsigned int *flags, unsigned int flag_value) {
   *flags |= flag_value;
}

void reset_flag(unsigned int *flags, unsigned int flag_value) {
   *flags &= ~(*flags & flag_value);
}

unsigned char read_flag_state(unsigned int *flags, unsigned int flag_value) {
   return *flags & flag_value;
}
