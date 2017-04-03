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

#define TIMER3_PERIOD_MS 0.13f
#define TIMER3_PERIOD_S (TIMER3_PERIOD_MS / 1000)
#define TIMER3_250MS (unsigned short)(250 / TIMER3_PERIOD_MS)

unsigned int general_flags;

void Clock_Config();
void Pins_Config();
void TIMER14_Confing();
void SPI_Config();
void set_flag(unsigned int *flags, unsigned int flag_value);
void reset_flag(unsigned int *flags, unsigned int flag_value);
unsigned char read_flag_state(unsigned int *flags, unsigned int flag_value);

void TIM14_IRQHandler() {
   TIM_ClearITPendingBit(TIM14, TIM_IT_Update);


}

int main() {
   Clock_Config();
   Pins_Config();
   TIMER14_Confing();
   SPI_Config();

   while (1) {
      SPI_SendData8(SPI1, 0xCC);
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
   pins_config.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7;
   pins_config.GPIO_Mode = GPIO_Mode_AF;
   pins_config.GPIO_Speed = GPIO_Speed_Level_2; // 10 MHz
   pins_config.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_Init(GPIOA, &pins_config);

   GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_0);
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0);
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0); // SPI1_MOSI

   SPI_InitTypeDef spi_config;
   spi_config.SPI_Direction = SPI_Direction_1Line_Tx;
   spi_config.SPI_Mode = SPI_Mode_Master;
   spi_config.SPI_DataSize = SPI_DataSize_8b;
   spi_config.SPI_CPOL = SPI_CPOL_Low;
   spi_config.SPI_CPHA = SPI_CPHA_2Edge;
   spi_config.SPI_NSS = SPI_NSS_Hard;
   spi_config.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; // 16MHz / 2Mhz of ESP8266
   spi_config.SPI_FirstBit = SPI_FirstBit_MSB;
   SPI_Init(SPI1, &spi_config);

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
