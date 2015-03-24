#include "ws2812.h"

uint8_t led_len;
uint16_t led_buffer[24*10];

void ws2812_set_pixel(uint16_t pixel, uint8_t R, uint8_t G, uint8_t B){
	int j;
	int base = 24*pixel;
	if(pixel >= led_len)return;

	for(j=0;j<8;j++){
		led_buffer[base + j ] 	= ((G<<j ) & 0x80) ? 17: 9;
		led_buffer[base+8 +j ] 	= ((B<<j ) & 0x80) ? 17: 9;
		led_buffer[base+16+j]	= ((R<<j ) & 0x80) ? 17: 9;
	}
}

void ws2812_init(uint8_t len) {
	if(len > 10)len = 10;
	int i;
	led_len = len;
	for(i=0;i<len;i++)ws2812_set_pixel(i,0,0,0);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* TIM1 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);

	/* GPIOA clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	/* DMA2 clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* GPIOA Configuration: PA8(TIM1 CH1) as alternate function push-pull ------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect TIM pins to AF2 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_2);
	/* Time base configuration */
	/* -----------------------------------------------------------------------
	 TIM1 Configuration: generate 1 PWM signal using the DMA burst mode:

	 TIM1 input clock (TIM1CLK) is set to APB2 clock (PCLK2)
	 TIM1CLK = PCLK2
	 PCLK2 = HCLK
	 => TIM1CLK = HCLK = SystemCoreClock

	 To get TIM1 counter clock at 24 MHz, the prescaler is computed as follows:
	 Prescaler = (TIM1CLK / TIM1 counter clock) - 1
	 Prescaler = (SystemCoreClock /24 MHz) - 1

	 TIM1 Frequency = TIM1 counter clock/(ARR + 1)
	 = 24 MHz / 4096 = 5.85 KHz
	 TIM1 Channel1 duty cycle = (TIM1_CCR1/ TIM1_ARR)* 100 = 33.33%

	 Note:
	 SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f0xx.c file.
	 Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
	 function to update SystemCoreClock variable value. Otherwise, any configuration
	 based on this variable will be incorrect.
	 ----------------------------------------------------------------------- */

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_OCStructInit(&TIM_OCInitStructure);

	TIM_TimeBaseStructure.TIM_Period = 30 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler =
			(uint16_t) (SystemCoreClock / 24000000) - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);

	/* TIM Configuration in PWM Mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC1Init(TIM16, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM16, TIM_OCPreload_Enable);

	/* TIM1 DMAR Base register and DMA Burst Length Config*/
	//TIM_DMAConfig( TIM16, TIM_DMABase_CCR1, TIM_DMABurstLength_1Transfer);
	/* TIM1 DMA Update enable */
	TIM_DMACmd(TIM16, TIM_DMA_Update, ENABLE);
	TIM_CtrlPWMOutputs(TIM16, ENABLE);

	/* TIM1 enable */
	//TIM_Cmd(TIM16, ENABLE);
	/* TIM1 PWM Outputs Enable */
	//TIM_CtrlPWMOutputs(TIM16, ENABLE);
	DMA_InitTypeDef DMA_InitStructure;

	/* DeInitialize the DMA1 Stream5 */
	DMA_DeInit(DMA1_Channel3);

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &TIM16->CCR1;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) led_buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 4;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
	//DMA_ITConfig(DMA1_Channel3, DMA_IT_HT, ENABLE);

}


void ws2812_send() {
	int i,j;

//	for(i=0;i<len;i++){
//		int base = 24*i;
//		uint8_t *c = color[i];
//		for(j=0;j<8;j++){
//			led_buffer[base + j ] 	= ((c[1]<<j ) & 0x80) ? 17: 9;
//			led_buffer[base+8 +j ] 	= ((c[0]<<j ) & 0x80) ? 17: 9;
//			led_buffer[base+16+j]	= ((c[2]<<j ) & 0x80) ? 17: 9;
//		}
//
//	}

	TIM3->CCR1 = 0;
	DMA1_Channel3->CNDTR = 24*led_len;
	DMA_Cmd(DMA1_Channel3, ENABLE);
	TIM_Cmd(TIM16, ENABLE);
}
//
//void ws2812DmaIsr(void) {
//	//portBASE_TYPE xHigherPriorityTaskWoken;
//	uint8_t * buffer;
//	int i;
//
//	if (total_led == 0) {
//		TIM_Cmd(TIM16, DISABLE);
//		DMA_Cmd(DMA1_Channel2, DISABLE);
//	}
//
//	if (DMA_GetITStatus(DMA1_IT_HT2)) {
//		DMA_ClearITPendingBit(DMA1_IT_HT2);
//		buffer = led_dma.begin;
//	}
//
//	if (DMA_GetITStatus(DMA1_IT_TC2)) {
//		DMA_ClearITPendingBit(DMA1_IT_TC2);
//		buffer = led_dma.end;
//	}
//
//	for (i = 0; (i < LED_PER_HALF) && (current_led < total_led + 2);
//			i++, current_led++) {
//		if (current_led < total_led)
//			fillLed(buffer + (24 * i), color_led[current_led]);
//		else
//			bzero(buffer + (24 * i), 24);
//	}
//
//	if (current_led >= total_led + 2) {
//		//	xSemaphoreGiveFromISR(allLedDone, &xHigherPriorityTaskWoken);
//
//		TIM_Cmd(TIM16, DISABLE); 					// disable Timer 1
//		DMA_Cmd(DMA1_Channel2, DISABLE); 			// disable DMA channel 2
//
//		total_led = 0;
//	}
//
//}

void DMA1_Channel2_3_IRQHandler() {

	//ws2812DmaIsr();

	if (DMA_GetITStatus(DMA1_IT_TC3)) {
		TIM16->CCR1 = 0;
		TIM_Cmd(TIM16, DISABLE);
		DMA_Cmd(DMA1_Channel3, DISABLE);
		DMA_ClearITPendingBit(DMA1_IT_TC3);

	}


	//TIM_CtrlPWMOutputs(TIM16, DISABLE);
	//GPIO_ResetBits(GPIOB,GPIO_Pin_8);

}

