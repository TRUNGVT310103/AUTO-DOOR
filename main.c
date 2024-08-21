#include "stm32f10x.h"                  
#include "stm32f10x_rcc.h"  
#include "stm32f10x_tim.h"             
#include "stm32f10x_gpio.h" 
#include "stm32f10x_exti.h"    
#include "stm32f10x_usart.h"          

#include "lcd20x4.h"
/**/
#define TIM2_CCR1_address 0x40000034
#define TIM2_CCR2_address ((uint32_t)0x40012C38)

/**/
uint8_t mang[] = "ATD0339371753;\r\n";
static uint32_t CCR1_value;
uint32_t CCR2_value[1];
static uint32_t A = 0;
static uint32_t B = 0;
double S;
void GPIO_Tim_config(void);
void TIM1_config(void);
void HC_SR04_trigger( void);
double HC_SR04_Get_Dis(void);
void DMA_CCRx( uint32_t *CCRx_address, uint32_t *CCx_value);
void TIM4_config(void );
void Servo_Init(void);
void Delay_ms(uint32_t ms);
void Delay_us(uint32_t us);
void Servo_x( uint8_t x);
void EXIT0_config(void);
void GPIO_config(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void USART_config(void);
void USART_Send_String( uint8_t *Data, uint8_t Len);
void SendByte(uint8_t byte);
void USART3_IRQHandler(void);
/**/
int main(void){
	GPIO_config();
	EXIT0_config();
	GPIO_Tim_config();
	LCD20X4_Init();
	TIM1_config();
	Servo_Init();
	HC_SR04_trigger();
	USART_config();
	LCD20X4_Gotoxy(0,0);
	LCD20X4_PutString("TUNG");
	while(1){
		}
}
/*
**@ cau hinh chan 
    ** PA0 : HC_SR04_trig
    ** PA8 : HC_SR04_echo 
*/
void GPIO_Tim_config(void){
	/**/
	GPIO_InitTypeDef GPIO_InitStruct;
	/**/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	/*PA8 INput Push PULL*/
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	/*PA alternate push pull*/
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}
/*
**@ HC_SR04_trigger
**@ frequency: tan so lay mau
*/
void HC_SR04_trigger( void){
	RCC->APB1ENR |= 0x01;/*enable clock for time 2*/
	TIM2->PSC =72-1;/*counter clock = 1MHZ*/
	TIM2->ARR =10000-1; /*reset counter value*/
	TIM2-> CCMR1 |= 0x60;/*pwm mode 1*/
	TIM2->CCR1 = 10;
	TIM2->CCER |= 0x01;
	TIM2->CR1 |= 0x01;
	TIM2->EGR |= 0x01;
	
	}
/*
	**@: config TIM1 PWM_input
	*/
void TIM1_config(void){
	/**/
	TIM_TimeBaseInitTypeDef TimeBaseInitStruct;
	/**/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	/* TimeBaseInit*/
	TimeBaseInitStruct.TIM_ClockDivision = 1;
	TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	
	TimeBaseInitStruct.TIM_Prescaler = 72-1;/*counter clock  = 1MHz*/
	TimeBaseInitStruct.TIM_Period = 0xFFFF;
	TIM_TimeBaseInit(TIM1,&TimeBaseInitStruct);
	/**/
	TIM1->CCMR1 |= 0x01;
	TIM1->CCMR1 |= 0x01<<9;
	/**/
	TIM1->CCER |= 0x31;
	/**/
	TIM1->SMCR |= 0x54;
	/* TIM2 DMA config enable*/
	TIM1->DIER |= 0x0700;
	TIM1->EGR = 0x05;
	/**/
	TIM1->CNT = 0u;
	/* enable counter*/
	TIM1->CR1 = 0x01;
}
double HC_SR04_Get_Dis(void){
	return (TIM1->CCR2*171)/10000;
}
/**/
void DMA_CCRx( uint32_t *CCRx_address, uint32_t *CCx_value){
	/*enable clock for DMA*/
	RCC->AHBENR = 0x01;
	/*number of data tranfer*/
	DMA1_Channel2->CNDTR |= 0x01;
	/**/
	DMA1_Channel2->CCR |= 0x0A20;
	/**/
	DMA1_Channel2->CCR &= ~(1U<<4);
	/**/
	DMA1_Channel2->CPAR = (uint32_t)CCRx_address;
	/**/
	DMA1_Channel2->CMAR = (uint32_t)CCx_value;
	/**/
	DMA1_Channel2->CCR |= 0x01;
	
}
void Servo_Init(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	RCC->APB1ENR |=0x02;
	TIM3->PSC = 1440-1;
	TIM3->ARR = 1000-1;
	
	TIM3-> CCMR1 |= 0x60;/*pwm mode 1*/
	TIM3->CCR1 = 0;
	TIM3->CCER |= 0x01;
	
	TIM3->EGR |= 0x01;
	TIM3->CR1 |= 0x01;
}
void Servo_x( uint8_t x){
	TIM3->CCR1 = x;
	}
	
/*
**@: TIM4_TimeBaseInit
*/

void TIM4_config(void ){
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 72-1;
	TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
	TIM_Cmd(TIM4, ENABLE);
}

void Delay_ms(uint32_t ms){
	TIM4_config();
	while(ms>0){
		TIM4->CNT = 0U;
		while(TIM4->CNT <1000);
		ms--;
		}
	}
void Delay_us( uint32_t us){
	TIM4_config();
	while(us>0){
		TIM4->CNT = 0U;
		while(TIM4->CNT <1);
		us--;
		}
	}
/**/
void GPIO_config(void){
	GPIO_InitTypeDef My_GPIO;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOA,ENABLE);
  /* config pin Pb0 EXTO*/
	My_GPIO.GPIO_Pin = GPIO_Pin_0;
	My_GPIO.GPIO_Mode = GPIO_Mode_IPU;
	My_GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &My_GPIO);
	/**/
	My_GPIO.GPIO_Pin = GPIO_Pin_1;
	My_GPIO.GPIO_Mode = GPIO_Mode_IPU;
	My_GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &My_GPIO);
	/**/
	My_GPIO.GPIO_Pin = GPIO_Pin_13;
	My_GPIO.GPIO_Mode = GPIO_Mode_Out_PP;
	My_GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &My_GPIO);
	}
/**/
void EXIT0_config(void){
	/**/
  
	/*EXTI config*/
	EXTI_InitTypeDef EXTI_InitStruc;
	NVIC_InitTypeDef NVIC_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	/**/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource0);
	/**/
	EXTI_InitStruc.EXTI_Line = EXTI_Line0;
	EXTI_InitStruc.EXTI_LineCmd = ENABLE;
	EXTI_InitStruc.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruc.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStruc);
	/**/
	EXTI_InitStruc.EXTI_Line = EXTI_Line1;
	EXTI_InitStruc.EXTI_LineCmd = ENABLE;
	EXTI_InitStruc.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruc.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStruc);
	/*clear pending bit*/
	EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI_ClearITPendingBit(EXTI_Line1);
	/*NVIC config*/
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
	/**/
	NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
	}
void EXTI0_IRQHandler(void){
	if((EXTI_GetITStatus(EXTI_Line0))!=RESET){
		/* clear pending bit cho lan tiep theo*/
		EXTI_ClearITPendingBit(EXTI_Line0);
		static int dem = 0;
        dem++;
        
        // Kiểm tra nếu số lần ngắt là số chẵn thì bật LED, ngược lại tắt LED
        if (dem % 2 == 0) {
            GPIOC->ODR |= (1 << 13); // Bật LED
        } else {
            GPIOC->ODR &= ~(1 << 13); // Tắt LED
        }
		}
		
	}
void EXTI1_IRQHandler(void){
	if((EXTI_GetITStatus(EXTI_Line1))!=RESET){
		EXTI_ClearITPendingBit(EXTI_Line1);
		A = 1;
		B = 0;
		}
	}
void USART_config(void){
	/* khoi tao cac bien cau truc*/
	NVIC_InitTypeDef NVIC_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	/**/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);
	/**/
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	/* config PB 10 for Tx*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	/**/
	
	/**/
	USART_ClearFlag(USART3,USART_FLAG_RXNE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	/* cho phep USART hoat dong*/
	
	/*USART2 config*/
	USART_InitStruct.USART_BaudRate = 9600;
	USART_InitStruct.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = 1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART3,&USART_InitStruct);USART_Cmd(USART3,ENABLE);
	/**/
	NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
	
	
	}
void SendByte(uint8_t byte){
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==0);
	USART_SendData(USART3, byte);
	}
void USART_Send_String( uint8_t *Data, uint8_t Len){
	uint8_t i;
	for(i = 0; i<Len; i++){
		
		while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==0);
		USART_SendData(USART3, *Data);
		Data++;
		}
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==0);
	}

void USART3_IRQHandler(void){
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET){
	}
	USART_ClearFlag(USART3,USART_FLAG_RXNE);
	
}