#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#define LCD_RS GPIO_Pin_13
#define LCD_RW GPIO_Pin_14
#define LCD_EN GPIO_Pin_15
#define LCD_D4 GPIO_Pin_8
#define LCD_D5 GPIO_Pin_9
#define LCD_D6 GPIO_Pin_10
#define LCD_D7 GPIO_Pin_11

void Delay_ms_1(uint16_t _time);
void Input(void);
void Led_Init(void);
void Relay_Init(void);
void DelayInit(void);
void DelayUs(uint32_t us);
void DelayMs(uint32_t ms);
void GPIO_LCD_Config(void);
void LCD_Enable(void);
void LCD_Send4Bit(unsigned char Data);
void LCD_SendCommand(unsigned char command);
void LCD_Clear();
void LCD_Init();
void LCD_Gotoxy(unsigned char x, unsigned char y);
void LCD_PutChar(unsigned char Data);
void LCD_Puts(char *s);
static __IO uint32_t usTicks;

int main(void)
{
		Input();
		Led_Init();
		Relay_Init();
		DelayInit();
		LCD_Init();
		int gasdetected = 0;
		int flamedetected = 0;
		int lcd_clear = 1;
		int check = 0;
    while(1)
    {		
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET && gasdetected == 0)	// Flame xuat tin hieu Low(phat hien lua)
        {		
						flamedetected = 1;
						check = 1;
						if(lcd_clear)
						{
							LCD_SendCommand(0x01); // xoa toan bo khung hinh
							lcd_clear = 0;
						}
						LCD_Gotoxy(0,0);
						LCD_Puts("WARNING");
						LCD_Gotoxy(0,1);
						LCD_Puts("FLAME DETECTED");
						GPIOC->BRR = (1<<14);	// kich hoat coi
						GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_RESET); // kich hoat relay
						while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET)
						{	// Led nhay canh bao
							GPIOC->BRR = (1<<13);
							Delay_ms_1(50);
							GPIOC->BSRR = (1<<13);
							Delay_ms_1(50);
							if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_RESET)
							{
									gasdetected = 1;
									lcd_clear = 1;
									break;
							}
						}
        }
				else if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_RESET && flamedetected == 0) // MQ2 xuat tin hieu Low(phat hien khoi)
				{			
							gasdetected = 1;
							check = 1;
							if(lcd_clear)
							{
								LCD_SendCommand(0x01); // xoa toan bo khung hinh
								lcd_clear = 0;
							}
							LCD_Gotoxy(0, 0);
							LCD_Puts("WARNING");
							LCD_Gotoxy(0, 1);
							LCD_Puts("GAS DETECTED");
							GPIOC->BRR = (1<<14);	// kich hoat coi
							while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_RESET)
							{	// Led nhay canh bao
								GPIOC->BRR = (1<<13);
								Delay_ms_1(50);
								GPIOC->BSRR = (1<<13);
								Delay_ms_1(50);
								if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET)
								{
										flamedetected = 1;
										lcd_clear = 1;
										break;
								}
							}
				}
				else if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET && GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_RESET)
				{		
						check = 1;
						if(lcd_clear)
						{
							LCD_SendCommand(0x01); // xoa toan bo khung hinh
							lcd_clear = 0;
						}
            LCD_Gotoxy(0, 0);
            LCD_Puts("GAS DETECTED");
            LCD_Gotoxy(0, 1);
            LCD_Puts("FLAME DETECTED");
						GPIOC->BRR = (1<<14);	// kich hoat coi
						GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_RESET); // kich hoat relay
						while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET && GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_RESET)
						{	// Led nhay canh bao
							GPIOC->BRR = (1<<13);
							Delay_ms_1(50);
							GPIOC->BSRR = (1<<13);
							Delay_ms_1(50);
						}
				}
        else
        {
            GPIOC->BSRR = (1<<13); // Tat led
						GPIOC->BSRR = (1<<14);// Tat coi
						GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_SET); // Ngat relay
						gasdetected = 0;
						flamedetected = 0;
						lcd_clear = 1;
						if(check){
							LCD_SendCommand(0x01); // Clear the LCD screen
							check = 0;
						}
            LCD_Gotoxy(0, 0);
            LCD_Puts("BTL NHOM_7");
            LCD_Gotoxy(0, 1);
            LCD_Puts("NO DETECTED");
        }
    }
}

void Delay_ms_1(uint16_t _time){
	volatile uint16_t i,j;
	for(i = 0; i < _time; i++){
		for(j = 0; j < 0x2AFF; j++){}	//601
	}
}

void Input(void){
		GPIO_InitTypeDef input;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		input.GPIO_Mode = GPIO_Mode_IPU;
		input.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
		input.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&input);
}

void Relay_Init(void){
		GPIO_InitTypeDef relay;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		relay.GPIO_Mode = GPIO_Mode_Out_PP;
		relay.GPIO_Pin = GPIO_Pin_1;
		relay.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB,&relay);
}

void Led_Init(void){
	GPIO_InitTypeDef led_init;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	led_init.GPIO_Mode = GPIO_Mode_Out_PP;
	led_init.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;
	led_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&led_init);	
}

void SysTick_Handler()
{
	if (usTicks != 0)
	{
		usTicks--;
	}
} 

void DelayInit()
{
	// Update SystemCoreClock value
	SystemCoreClockUpdate();
	// Configure the SysTick timer to overflow every 1 us
	SysTick_Config(SystemCoreClock / 1000000);
} 

void DelayUs(uint32_t us)
{
	// Reload us value
	usTicks = us;
	// Wait until usTick reach zero
	while (usTicks);
} 

void DelayMs(uint32_t ms)
{
	// Wait until ms reach zero
	while (ms--)
	{
		// Delay 1ms
		DelayUs(1000);
	}
}

void GPIO_LCD_Config(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_LCD_InitStruction;
	GPIO_WriteBit(GPIOA, LCD_D4|LCD_D5|LCD_D6|LCD_D7, 0);
	GPIO_WriteBit(GPIOB, LCD_RS|LCD_EN|LCD_RW, 0);
	GPIO_LCD_InitStruction.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_LCD_InitStruction.GPIO_Pin = LCD_D4|LCD_D5|LCD_D6|LCD_D7;
	GPIO_LCD_InitStruction.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_LCD_InitStruction);
	GPIO_LCD_InitStruction.GPIO_Pin = LCD_EN|LCD_RS|LCD_RW;
	GPIO_Init(GPIOB, &GPIO_LCD_InitStruction);
}

void LCD_Enable(void)
{
	GPIO_SetBits(GPIOB, LCD_EN);
	DelayMs(1);
	GPIO_ResetBits(GPIOB, LCD_EN);
	DelayMs(1);
}

void LCD_Send4Bit(unsigned char Data)
{
GPIO_WriteBit(GPIOA, LCD_D4, Data & 0x01);
GPIO_WriteBit(GPIOA, LCD_D5, (Data>>1)&1);
GPIO_WriteBit(GPIOA, LCD_D6, (Data>>2)&1);
GPIO_WriteBit(GPIOA, LCD_D7, (Data>>3)&1);
}

void LCD_SendCommand(unsigned char command)
{
	LCD_Send4Bit(command >> 4);
	LCD_Enable();
	LCD_Send4Bit(command);
	LCD_Enable();
} 

void LCD_Clear()
{
	LCD_SendCommand(0x01);
	DelayUs(10);
}

void LCD_Init()
{
	GPIO_LCD_Config();
	LCD_Send4Bit(0x00);
	GPIO_WriteBit(GPIOB, LCD_RS, 0);
	LCD_Send4Bit(0x03);
	LCD_Enable();
	LCD_Enable();
	LCD_Enable();
	LCD_Send4Bit(0x02);
	LCD_Enable();
	LCD_SendCommand(0x28); // giao thuc 4 bit, hien thi 2 hang, ki tu 5x8
	LCD_SendCommand(0x0C); // cho phep hien thi man hinh
	LCD_SendCommand(0x06); // tang ID, khong dich khung hinh
	LCD_SendCommand(0x01); // xoa toan bo khung hinh
}

void LCD_Gotoxy(unsigned char x, unsigned char y)
{
	unsigned char address;
	if(y == 0)address=(0x80+x);
	else if(y == 1) address=(0xc0+x);
	LCD_SendCommand(address);
}

void LCD_PutChar(unsigned char Data)
{
	GPIO_SetBits(GPIOB, LCD_RS);
	LCD_SendCommand(Data);
	GPIO_ResetBits(GPIOB, LCD_RS);
}

void LCD_Puts(char *s)
{
	while (*s)
	{
		LCD_PutChar(*s);
		s++;
	}
}