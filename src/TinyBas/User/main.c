/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : Stan6314
 * Version            : V1.0
 * Date               : 2025/07/02
 * Description        : TinyBasic for CH32V003
 *      + character constants are in charstable.h
 *      + oscillator settings mus be written to systemv00x.c
 * The program contains 3 parts:
 *      -> initialization, interrupt and auxiliary functions
 *      -> Tiny BASIC commands
 *      -> main loop
*********************************************************************************
*******************************************************************************/
// When using an external 24MHz crystal, uncomment the following #define line and
// change the settings #define SYSCLK_FREQ_48MHZ_HSI 48000000 to #define SYSCLK_FREQ_48MHz_HSE 48000000
// in the file system_ch32v00x.c
// For RC oscillator line must be commented and #define SYSCLK_FREQ_48MHZ_HSI 48000000 in the file system_ch32v00x.c
#define SYSCLK_EXT_CRYSTAL

#include "debug.h"
#include "ch32v00x_dma.h"
#include "charstable.h"

// Global variables for VGA display
u_int16_t nVGAline;                // VGA displayed line counter (0 - 525)
volatile u_int8_t DMABuff[47];      // Buffer for DMA to SPI transfer
volatile u_int8_t VideoRAM[800];    // VideoRAM for 25 * 32 chars
volatile u_int16_t nDispChar=0;    // Which character (line) will be displayed
volatile u_int16_t nProcesChar=0;   // Which character is prepared for display
volatile u_int16_t rowPointer;      // Pointer to char table for rows 0..7
volatile u_int32_t basTimer;      // Internal timer for Tiny BASIC TIME command incremented with vertical sync

// Global variables for Tiny BASIC
static uint16_t dispPointer = 0;      // Pointer to current display position
volatile uint8_t RunMode = 0;     // If it is 1, then basic program is in run
uint8_t currentLine=0;    // Actual line to proccess
uint8_t noOfMemory = 0;  // Number of section in program memory or "file" (4 KB sections)
uint8_t compFlags = 0;  // Bit7 = Is DISK1, Bit6 = Is PCF857x, Bits3-0 = Low Addres of EEPROM (Disk adr)
uint8_t linePointer=0;    // Pointer to line memory buffer
uint8_t line_buffer[33];  // buffer for data read/write
uint8_t stack_buffer[8];  // buffer for GOSUB stack
uint8_t stackPointer=0;    // Pointer to GOSUB buffer
int charVariable[26]; // Variables A..Z
int arrayVariable[100]; // Variables in array @0..99
int evalNumber;     // Value of number for evaluation in BASIC line

/*********************************************************************
    Functions for keyboard
 *********************************************************************/
// Assignment input pins (DATA and CLOCK) for keyboard
#define KBD_CLOCK_PORT    GPIOD
#define KBD_CLOCK_PIN     GPIO_Pin_1
#define KBD_DATA_PORT     GPIOC
#define KBD_DATA_PIN      GPIO_Pin_7
#define KBD_INT_PORT_SRC  GPIO_PortSourceGPIOD
#define KBD_INT_PIN_SRC   GPIO_PinSource1
#define KBD_INT_LINE      EXTI_Line1

// HW keyboard initialization - pins and interrupt
void kbd_init() {
  // Pins of Port configuration 
	GPIO_InitTypeDef Config = {0};
	Config.GPIO_Pin = KBD_CLOCK_PIN;
	Config.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	Config.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(KBD_CLOCK_PORT, &Config);
	Config.GPIO_Pin = KBD_DATA_PIN;
	GPIO_Init(KBD_DATA_PORT, &Config);
  // Keyboard interrupt configuration
	EXTI_InitTypeDef kbd_exti = {0};
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_EXTILineConfig(KBD_INT_PORT_SRC, KBD_INT_PIN_SRC);
  kbd_exti.EXTI_Line = KBD_INT_LINE;
  kbd_exti.EXTI_Mode = EXTI_Mode_Interrupt;
  kbd_exti.EXTI_Trigger = EXTI_Trigger_Falling;
  kbd_exti.EXTI_LineCmd = ENABLE;
  EXTI_Init(&kbd_exti);

	NVIC_InitTypeDef kbd_nvic = {0};
	kbd_nvic.NVIC_IRQChannel = EXTI7_0_IRQn;
  kbd_nvic.NVIC_IRQChannelPreemptionPriority = 0;
  kbd_nvic.NVIC_IRQChannelSubPriority = 1;
  kbd_nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&kbd_nvic);
}

// Keyboard variables
static uint16_t KbdBuffer = 0;      // Buffer for received data
static uint8_t KbdReady = 0;        // Flag full buffer (11 bits received)
static uint8_t KbdBitCnt = 0;       // Counter received bits from keyboard
volatile uint8_t KbdData = 0;       // Keyboard code output (bits in reverse order)
volatile uint8_t KbdStatus = 0;     // Status flags for keyboard
// Bit 3 = Key released UP, Bit 2 = RShift, Bit 1 = LShift, Bit 0 = Ctrl

// Function translate keyboard code from KbdData to ASCII or statement character
uint8_t kbdRead() {
  uint8_t RetKey = 0;
  uint8_t KeyCode = KbdData; // Strobe volatile data to auxiliary variable
	if(KeyCode) {   // Is key code in reverse order, so translate it
        KbdData = 0;
        // Test letter codes and and adjust capital letters
        switch (KeyCode) {
            case 0x38: RetKey = 'a'; break;
            case 0x4C: RetKey = 'b'; break;
            case 0x84: RetKey = 'c'; break;
            case 0xC4: RetKey = 'd'; break;
            case 0x24: RetKey = 'e'; break;
            case 0xD4: RetKey = 'f'; break;
            case 0x2C: RetKey = 'g'; break;
            case 0xCC: RetKey = 'h'; break;
            case 0xC2: RetKey = 'i'; break;
            case 0xDC: RetKey = 'j'; break;
            case 0x42: RetKey = 'k'; break;
            case 0xD2: RetKey = 'l'; break;
            case 0x5C: RetKey = 'm'; break;
            case 0x8C: RetKey = 'n'; break;
            case 0x22: RetKey = 'o'; break;
            case 0xB2: RetKey = 'p'; break;
            case 0xA8: RetKey = 'q'; break;
            case 0xB4: RetKey = 'r'; break;
            case 0xD8: RetKey = 's'; break;
            case 0x34: RetKey = 't'; break;
            case 0x3C: RetKey = 'u'; break;
            case 0x54: RetKey = 'v'; break;
            case 0xB8: RetKey = 'w'; break;
            case 0x44: RetKey = 'x'; break;
            case 0xAC: RetKey = 'y'; break;
            case 0x58: RetKey = 'z'; break;
        }
        // Shift moves letter to upper case
        if(KbdStatus & 0x06) RetKey &= 0xDF;
        // Test number codes and and special characters
        switch (KeyCode) {
            case 0x68: RetKey = (KbdStatus & 0x06) ? '!' : '1'; break;
            case 0x78: RetKey = (KbdStatus & 0x06) ? '@' : '2'; break;
            case 0x64: RetKey = (KbdStatus & 0x06) ? '#' : '3'; break;
            case 0xA4: RetKey = (KbdStatus & 0x06) ? '$' : '4'; break;
            case 0x74: RetKey = (KbdStatus & 0x06) ? '%' : '5'; break;
            case 0x6C: RetKey = (KbdStatus & 0x06) ? '^' : '6'; break;
            case 0xBC: RetKey = (KbdStatus & 0x06) ? '&' : '7'; break;
            case 0x7C: RetKey = (KbdStatus & 0x06) ? '*' : '8'; break;
            case 0x62: RetKey = (KbdStatus & 0x06) ? '(' : '9'; break;
            case 0xA2: RetKey = (KbdStatus & 0x06) ? ')' : '0'; break;
            // Num keys
            case 0x8E: RetKey = '.'; break;    // .
            case 0x0E: RetKey = '0'; break;    // 0
            case 0x96: RetKey = '1'; break;    // 1
            case 0x4E: RetKey = '2'; break;    // 2
            case 0x5E: RetKey = '3'; break;    // 3
            case 0xD6: RetKey = '4'; break;    // 4
            case 0xCE: RetKey = '5'; break;    // 5
            case 0x2E: RetKey = '6'; break;    // 6
            case 0x36: RetKey = '7'; break;    // 7
            case 0xAE: RetKey = '8'; break;    // 8
            case 0xBE: RetKey = '9'; break;    // 9
            case 0x9E: RetKey = '+'; break;    // +
            case 0xDE: RetKey = '-'; break;    // -
            case 0x3E: RetKey = '*'; break;    // *
            case 0x52: RetKey = '/'; break;    // /
        }
        // And remaining special characters
        switch (KeyCode) {
            case 0x94: RetKey = ' '; break;
            case 0x5A: RetKey = 0x0A; break;    // Enter
            case 0xB0: RetKey = 0x09; break;    // Tab
            case 0x66: RetKey = 0x08; break;    // Backspace
            case 0x72: RetKey = (KbdStatus & 0x06) ? '_' : '-'; break;
            case 0xAA: RetKey = (KbdStatus & 0x06) ? '+' : '='; break;
            case 0x2A: RetKey = (KbdStatus & 0x06) ? '{' : '['; break;
            case 0xDA: RetKey = (KbdStatus & 0x06) ? '}' : ']'; break;
            case 0x32: RetKey = (KbdStatus & 0x06) ? ':' : ';'; break;
            case 0x4A: RetKey = (KbdStatus & 0x06) ? '"' : 0x27; break;
            case 0xBA: RetKey = (KbdStatus & 0x06) ? '|' : 0x5C; break;
            case 0x82: RetKey = (KbdStatus & 0x06) ? '<' : ','; break;
            case 0x92: RetKey = (KbdStatus & 0x06) ? '>' : '.'; break;
            case 0x52: RetKey = (KbdStatus & 0x06) ? '?' : '/'; break;
        }
	}
  if(KbdStatus & 0x01) {
    // Ctrl changes character to KEYWORD code
    switch (RetKey) {
      case 'p': RetKey = 0xA0; break; // PRINT
      case 'u': RetKey = 0xA1; break; // IF
      case 't': RetKey = 0xA2; break; // THEN
      case 'g': RetKey = 0xA3; break; // GOTO
      case 'i': RetKey = 0xA4; break; // INPUT
      case 'l': RetKey = 0xA5; break; // LET
      case 'h': RetKey = 0xA6; break; // GOSUB
      case 'y': RetKey = 0xA7; break; // RETURN
      case 'x': RetKey = 0xA8; break; // CLEAR
      case 'k': RetKey = 0xA9; break; // LIST
      case 'r': RetKey = 0xAA; break; // RUN
      case 'n': RetKey = 0xAB; break; // END

      case 'v': RetKey = 0xAC; break; // TIME
      case 'w': RetKey = 0xAD; break; // CURSOR
      case 'c': RetKey = 0xAE; break; // PUTCH
      case ',': RetKey = 0xAF; break; // GETCH
      case 's': RetKey = 0xB0; break; // INKEY
      case 'f': RetKey = 0xB1; break; // FILE
      case 'd': RetKey = 0xB2; break; // DISK
      case 'm': RetKey = 0xB3; break; // DELAY
      case 'z': RetKey = 0xB4; break; // BEEP
      case 'e': RetKey = 0xB5; break; // REM

      case 'a': RetKey = 0xB6; break; // AINP
      case 'b': RetKey = 0xB7; break; // DINP
      case 'o': RetKey = 0xB8; break; // DOUT
      case 'q': RetKey = 0xB9; break; // I2CW
      case 'j': RetKey = 0xBA; break; // I2CR
      case '.': RetKey = 0xBB; break; // COPY
    }
  }
	return RetKey;
}

// Wait for keystroke and blink cursor on the corresponding position
uint8_t GetKeyChar()
{
    uint8_t saveChar, retChar;
    saveChar = VideoRAM[dispPointer];  // Save char on the cursor position (for cursor blink)
    do{
        if(basTimer & 0x2000) VideoRAM[dispPointer] = saveChar; else VideoRAM[dispPointer] = 0x60;  // blink
        retChar = kbdRead();
        Delay_Ms(1);
    } while(retChar == 0);
    VideoRAM[dispPointer] = saveChar;  // Restore position
    return retChar;
}

/*********************************************************************
    Functions for display
 *********************************************************************/
// Display character 0x00 - 0xFF to VGA on cursor position
void WriteChar(uint8_t dispChar)
{
    switch (dispChar) {
        case 0x0C: {      // FormFeed
            for(int i=0;i<800;i++) VideoRAM[i]=0; 
            dispPointer = 0;
        } break;
        case 0x0A: {    // Enter / New line
            dispPointer = ((dispPointer >> 5) + 1) << 5;
        } break;
        case 0x09: {    // Tab
            dispPointer = ((dispPointer >> 3) + 1) << 3;
        } break;
        default: {  // Remaining characters
            if((dispChar>0x1F) && (dispChar<0xA0)) VideoRAM[dispPointer++] = dispChar-0x20; // Printable
            else if((dispChar>0x9F) && (dispChar<0xBC)) { // Tiny BASIC Keywords
              int pKeyWord = ((dispChar - (uint8_t)0xA0)) << 3;
              for(int j=0; j<KeyWordTab[pKeyWord];j++){
                VideoRAM[dispPointer++] = KeyWordTab[pKeyWord+j+1] - 0x20;
                // Test pointer and roll display if it is over last line
                if(dispPointer>799) {
                    for(int i=0;i<768;i++) VideoRAM[i]=VideoRAM[i+32];  // Roll it up
                    for(int i=768;i<800;i++) VideoRAM[i]=0; // Clear last line
                    dispPointer = 768;
                }
              }
            }
            else if(dispChar!=0x08) VideoRAM[dispPointer++] = 0x0E; // Dot for non printable - except backspace
        }; break;
    }
    // Test pointer and roll display if it is over last line
    if(dispPointer>799) {
        for(int i=0;i<768;i++) VideoRAM[i]=VideoRAM[i+32];  // Roll it up
        for(int i=768;i<800;i++) VideoRAM[i]=0; // Clear last line
        dispPointer = 768;
    }
}

// Print text (and second version with new line)
void WriteText(char *pointChar)
{
    while(*pointChar) WriteChar(*pointChar++);
}
void WriteTextln(const char *pointChar)
{
    while(*pointChar) WriteChar(*pointChar++);
    WriteChar(0x0A);
}
// Display integer number (32bits) on VGA
void WriteNum(int writNumber)
{
  uint8_t digits[10];     // Integer number has 10 digits max
  uint8_t Signum = 0; 
  uint8_t i = 0;
  if(writNumber) {
    if(writNumber < 0) {Signum = 1; writNumber = -writNumber;}
    while(writNumber) {
      digits[i] = (uint8_t) (writNumber%10) | 0x30;
      writNumber /= 10;
      i++;
    }
    if(Signum) WriteChar('-');
    while(i) WriteChar(digits[--i]);
  } else  WriteChar('0');
}

// Function for error message
void WriteErrMsg(const char *pointChar)
{
    WriteChar(0x0A);WriteChar('L');   // Write L (aka Line)
    WriteNum(currentLine);      // Number of line with error
    WriteChar(0x20);
    while(*pointChar) WriteChar(*pointChar++);  // and error message
    WriteChar(0x0A);
    RunMode = 0;    // Error == stop program
}

/*********************************************************************
    Functions for initialization and interrupts
 *********************************************************************/
/*********************************************************************
 * @fn      DMA_Tx_Init
 * @brief   Initializes the DMAy Channelx configuration.
 * @param   DMA_CHx - x can be 1 to 7.
 *          ppadr - Peripheral base address.
 *          memadr - Memory base address.
 *          bufsize - DMA channel buffer size.
 * @return  none
 */
void DMA_Tx_Init(DMA_Channel_TypeDef *DMA_CHx, u32 ppadr, u32 memadr, u8 bufsize)
{
    DMA_InitTypeDef DMA_InitStructure = {0};
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);
}


/*********************************************************************
 * @fn      GPIO_InOut_INIT
 * @brief   Initializes GPIO
 */
void GPIO_InOut_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    // Settings digital pins 1 and 2 (5 and 6 on GPIOD)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    // Settings digital pins 3 and 4 (1 and 2 on GPIOA) - only if not used crystal
#ifndef SYSCLK_EXT_CRYSTAL
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif
    // Settings digital pins on GPIOC
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_4;  // VGA signals
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_0;  // VGA signal + repro
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*********************************************************************
 * Function for handling keyboard interrupt
 * @brief   In KbdData is saved keyboard scan code (bit reverse order) after last clock
 *          also handles program termination with END command
 */

void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler(void) {
    if (!(KBD_CLOCK_PORT->INDR & KBD_CLOCK_PIN)) {  // This condition eliminates oscillation on rising edge
        if (KbdReady == 0) {
            KbdBitCnt++;
            KbdBuffer <<= 1;
            if (KBD_DATA_PORT->INDR & KBD_DATA_PIN) {
                KbdBuffer |= 1;
            }
            KbdReady = (KbdBitCnt == 11);
            if(KbdReady) {  // Process keyboard code
                uint8_t AuxCode = (KbdBuffer >> 2) & 0xFF;  // Shift code to 8 bit (without parity and stop check)
                if(AuxCode == 0xF) KbdStatus |= 0x08;    // Is "key up" code - set flag "UP"
                else if(AuxCode == 0x48) { // Is "L shift" code - set or clear flag "LSHIFT"
                    if(KbdStatus & 0x08) KbdStatus &= 0xF5; else KbdStatus |= 0x02;
                }
                else if(AuxCode == 0x9A) { // Is "R shift" code - set or clear flag "RSHIFT"
                    if(KbdStatus & 0x08) KbdStatus &= 0xF3; else KbdStatus |= 0x04;
                }
                else if(AuxCode == 0x28) { // Is "L ctrl" code - set or clear flag "CTRL"
                    if(KbdStatus & 0x08) KbdStatus &= 0xF6; else KbdStatus |= 0x01;
                } 
                else if(KbdStatus & 0x08) // Normal code (no Shift, Ctrl or Up)
                KbdStatus &= 0xF7; else KbdData = AuxCode;  // Discard code and clear UP flag / forward key code
                KbdBuffer = 0;
                KbdBitCnt = 0;
                KbdReady = 0;
                // Check if END keys (Ctrl + N) is pressed - so stop program execution
                if((KbdData == 0x8C) && (KbdStatus & 0x01)) RunMode = 0;
            }
        }
    }
	EXTI->INTFR = KBD_INT_LINE;
}

/*********************************************************************
 * @fn      TIM1_UP_IRQHandler
 * @brief   This function handles TIM1 UP interrupt
 *          generates data and signals for VGA display and update TIME counter
 */
void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_UP_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM1, TIM_IT_Update)==SET)
    {
        // Prepare data for VGA line od display (takes 1 to 16 usec)
        if((nVGAline < 400) && !(nVGAline & 0x0001)) {
            if(nDispChar >= 800) nDispChar=0;    // All video RAM is displayed, return to begin
            nProcesChar=nDispChar;
            rowPointer = (nVGAline & 0x000E) << 6;   // Set pointer to char table on line base
            for(u_int16_t i=0; i<32; i++) DMABuff[i] = CharTable[rowPointer + (u16)VideoRAM[nProcesChar+i]];
            if(rowPointer >= 0x380) nDispChar += 32;
        }
        if(nVGAline == 400) for(u_int16_t i=0; i<32; i++) DMABuff[i] = 0x00;   // Clear buffer
        nVGAline++;     // Update VGA lines counter
        if(nVGAline >= 525) nVGAline = 0;       // VGA has 525 lines total
        if(nVGAline == 460) GPIOC->BSHR = 0x00080000;   // Start vertical sync pulse
        if(nVGAline == 462) GPIOC->BSHR = 0x00000008;   // Stop vertical sync pulse
        basTimer++;     // Increment timer for Basic
    }
    TIM_ClearITPendingBit( TIM1, TIM_IT_Update );
}

/*********************************************************************
 * @fn      TIM1_INT_Init
 * @brief   Initializes SPI module and TIM1 module.
 * @return  none
 */
void TIM1_INT_Init()
{
    SPI_InitTypeDef SPI_InitStruct={0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    SPI_InitStruct.SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPI1,&SPI_InitStruct);
    SPI1->CTLR1 |= 0x0100; SPI1->CTLR1 |= 0x0004;
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
    SPI_Cmd(SPI1,ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};
    TIM_OCInitTypeDef TIM_OCInitStructure={0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE );
    TIM_TimeBaseInitStructure.TIM_Period = 1503;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

    TIM_ClearITPendingBit( TIM1, TIM_IT_Update );
    TIM_ClearITPendingBit( TIM1, TIM_IT_CC2 );

    NVIC_InitStructure.NVIC_IRQChannel =TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;
    NVIC_InitStructure.NVIC_IRQChannelCmd =ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM1, TIM_IT_Update , ENABLE);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 180;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC4Init( TIM1, &TIM_OCInitStructure );

    TIM_CtrlPWMOutputs(TIM1, ENABLE );
    TIM_OC4PreloadConfig( TIM1, TIM_OCPreload_Disable );
    TIM_ARRPreloadConfig( TIM1, ENABLE );
}

/*********************************************************************
 * Initialize adc for polling (taken from ch32fun example - I was too lazy to think of something)
 */
void adc_init( void )
{
	// ADCCLK = 24 MHz => RCC_ADCPRE = 0: divide by 2
	RCC->CFGR0 &= ~(0x1F<<11);
	// Enable GPIOD and ADC
	RCC->APB2PCENR |= RCC_APB2Periph_ADC1;
	// PD4 is analog input chl 7
	GPIOD->CFGLR &= ~(0xf<<(4*4));	// CNF = 00: Analog, MODE = 00: Input
	// Reset the ADC to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;
	// Set up single conversion on chl 7
	ADC1->RSQR1 = 0;
	ADC1->RSQR2 = 0;
	ADC1->RSQR3 = 7;	// 0-9 for 8 ext inputs and two internals
	// set sampling time for chl 7
	ADC1->SAMPTR2 &= ~(ADC_SMP0<<(3*7));
	ADC1->SAMPTR2 |= 7<<(3*7);	// 0:7 => 3/9/15/30/43/57/73/241 cycles
	// turn on ADC and set rule group to sw trig
	ADC1->CTLR2 |= ADC_ADON | ADC_EXTSEL;
  // Reset calibration
	ADC1->CTLR2 |= ADC_RSTCAL;
	while(ADC1->CTLR2 & ADC_RSTCAL);
	// Calibrate
	ADC1->CTLR2 |= ADC_CAL;
	while(ADC1->CTLR2 & ADC_CAL);
	// should be ready for SW conversion now
}

/*********************************************************************
    Functions for I2C module 
 *********************************************************************/
// Initialization of I2C module - code from examples in Mounriver Studio
void IIC_Init(u32 bound, u16 address)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    I2C_InitTypeDef I2C_InitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    I2C_InitStructure.I2C_ClockSpeed = bound;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;
    I2C_InitStructure.I2C_OwnAddress1 = address;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init( I2C1, &I2C_InitStructure );

    I2C_Cmd( I2C1, ENABLE );
}

// Check I2C address for connected device
int CheckI2CDevice(u8 addr)
{
  while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET ) {};    // Wait until I2C module busy
  I2C_GenerateSTART( I2C1, ENABLE );    // Start I2C communication
  while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );   // Wait for ready send data
  I2C_Send7bitAddress( I2C1, addr, I2C_Direction_Transmitter );   // Send tested address
  int startI2C = basTimer;      // Prepare timeout timer
  //Once sent, we need to have an response from the slave device, we need an "timeout" because most of addresses wont have a valid device
  while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) ){
    if (basTimer > (startI2C+100)){
      I2C_GenerateSTOP( I2C1, ENABLE );  //Make sure to close the i2c
      return 0;   // No such device
    }
  }
  I2C_GenerateSTOP( I2C1, ENABLE );
  return 1;   // Yes, device is connected
}

// Write address of line of program - command + 2 bytes of address (address = lineNumber*32)
void I2C_write_address(unsigned char lineNumber)
{
  while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET ) {};    // Wait until I2C module busy
  I2C_GenerateSTART( I2C1, ENABLE );    // Start I2C communication
  while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );   // Wait for ready send data
  I2C_Send7bitAddress( I2C1, 0xA0 | (compFlags & 0x0F), I2C_Direction_Transmitter );  // Send 24Cxxx memory address + DISK number
  while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );   // Wait for ready send upper adr. byte
  I2C_SendData( I2C1, (lineNumber >> 3) | (noOfMemory << 4) );  // Send upper adr. byte
  while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );  // Wait for ready send lower adr. byte
  I2C_SendData( I2C1, (lineNumber << 5) );  // Send lower adr. byte
}

// Write 1 line of program - 32 bytes
void I2C_write_line(unsigned char lineNumber) 
{
  I2C_write_address(lineNumber);    // Write memory address of requested line
  for(int j=0; j<32; j++) {
    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );   // Wait for ready send data
    I2C_SendData( I2C1, line_buffer[linePointer++] );   // Send data
  }
  while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );   // Wait for ready to stop
  I2C_GenerateSTOP( I2C1, ENABLE );   // And stop transmition
}

// Read 1 line of program - 32 bytes
void I2C_read_line(unsigned char lineNumber) 
{
  int j=0;  // Counter for received data
  I2C_write_address(lineNumber);    // Write memory address of requested line
  while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );   // Wait for ready to stop and change direct
  I2C_GenerateSTOP( I2C1, ENABLE );
  while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET )    // Wait for ready to start receiving data
   ;
  I2C_GenerateSTART( I2C1, ENABLE );
  while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );   // Wait for ready send address device
  I2C_Send7bitAddress( I2C1, 0xA0 | (compFlags & 0x0F), I2C_Direction_Receiver );  // Send 24Cxxx memory address + DISK number
  while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) );   // Wait for ready read data
  I2C_AcknowledgeConfig( I2C1, ENABLE ); 
  do
  {
    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED ) );   // Wait for ready read data
    line_buffer[j++] = I2C_ReceiveData( I2C1 );
  } while(( j < 32 ) && (line_buffer[j-1] != (uint8_t)0xFF));   // Save byte to buffer until 32 bytes or EOL
  I2C_GenerateSTOP( I2C1, ENABLE );   // And stop transmition
}
/*********************************************************************
    END Functions for I2C module END
 *********************************************************************/

/*********************************************************************
    Auxiliary functions for Tiny BASIC
 *********************************************************************/
// Function for LISTing of BASIC program line
uint8_t PrintBasLine(unsigned int lineProgr)
{
  I2C_read_line(lineProgr);
  if(line_buffer[0] == (unsigned char)0xFF) return 0;
  int j=0;
  // Number of line at beginning - right justified
  if(lineProgr <10) WriteChar(' ');
  if(lineProgr <100) WriteChar(' ');
  WriteNum(lineProgr); WriteChar(' ');
  // Now print program line
  while(line_buffer[j] != (unsigned char)0xFF) WriteChar(line_buffer[j++]);   // 0xFF means end of line text
  WriteChar(0x0A);
  return 1;
}

// Remove white spaces (and version moving to next position)
void RemoveWhtSP()
{
  while(line_buffer[linePointer] == ' ') { linePointer++; }
}
void RemoveNextWhtSP()
{
  linePointer++;
  while(line_buffer[linePointer] == ' ') { linePointer++; }
}

// Return pointer to number evaluated from line or NULL if no number
int* isNumberNext()
{
  evalNumber=0;
  if(line_buffer[linePointer] == 'b') {   // Binary number can be on the line
    linePointer++;
    if((line_buffer[linePointer] < '0') || (line_buffer[linePointer] > '1')) return(NULL);  // 0 or 1 must be next
    do{
      evalNumber = (evalNumber<<1) + (0x01 & line_buffer[linePointer]); linePointer++;  // Use LSB for filling number
    }while((line_buffer[linePointer] == '0') || (line_buffer[linePointer] == '1'));  // while 0 or 1
    return(&evalNumber);
  } else {    // Next is test for decadic number
    if((line_buffer[linePointer] < '0') || (line_buffer[linePointer] > '9')) return(NULL);  // 0 to 9 must be first
    do{
      evalNumber = evalNumber*10 + (0x0F & line_buffer[linePointer]); linePointer++;  // Use value for filling number
    }while((line_buffer[linePointer] >= '0') && (line_buffer[linePointer] <= '9'));  // repeat until 0 or 1
    return(&evalNumber);
  }
}

// Return pointer to variable evaluated from line or NULL if no variable
int* isVariableNext()
{
  int aux = line_buffer[linePointer];
  if(line_buffer[linePointer] == '@') // Array variable
    { linePointer++; 
      if((line_buffer[linePointer] >= 'A') && (line_buffer[linePointer] <= 'Z'))      // index is in variable A-Z
      { aux = charVariable[line_buffer[linePointer] - 'A'];   // Get value of index from variable
        linePointer++;
        // Test range of array index 0..99
        if((aux >= 0) && (aux<100)) return(arrayVariable + aux); else {WriteErrMsg("Out of array!"); return(NULL); }
      }
      else {      // Is it direct number?
        if((line_buffer[linePointer] >= '0') && (line_buffer[linePointer] <= '9')) {      // first digit of array index
          aux = 0x0F & line_buffer[linePointer]; linePointer++;
          if((line_buffer[linePointer] >= '0') && (line_buffer[linePointer] <= '9'))      // second digit of array index
            {aux = aux*10 + (0x0F & line_buffer[linePointer]); linePointer++;}
          return(arrayVariable + aux);
        }
      }
      WriteErrMsg("Index error!"); return(NULL); 
    }
  else
  if((line_buffer[linePointer] >= 'A') && (line_buffer[linePointer] <= 'Z')) 
    { linePointer++; return(charVariable + (aux - 'A')); }
  else return(NULL);
}

// Return pointer to factor evaluated from line (number or variable)
int* isFactor()
{
  int* numReturned;
  if((numReturned = isVariableNext()) != NULL) return(numReturned);
  else return(isNumberNext());
}

// Return pointer to term evaluated from line (term with multiplikation or division)
int* isTerm()
{
  int auxValue;
  int* numReturned;
  unsigned char isDiv = 0;
  if((numReturned = isFactor()) != NULL) auxValue = *numReturned; else return(NULL);
  RemoveWhtSP();
  while((line_buffer[linePointer] == '/') || (line_buffer[linePointer] == '*')) {
    if(line_buffer[linePointer] == '/') isDiv=1; else isDiv=0;
    RemoveNextWhtSP();
    if((numReturned = isFactor()) != NULL) {
      if(isDiv) {if(*numReturned) auxValue /= *numReturned; else WriteErrMsg("Zero div!");}
      else auxValue *= *numReturned;
      RemoveWhtSP();
    } else {WriteTextln("Missing operand!"); return(NULL); }
  }
  evalNumber = auxValue;
  return(&evalNumber);
}

// Return pointer to members with addition or subtraction
int* isExpression()
{
  int auxValue;
  int* numReturned;
  unsigned char isMinus = 0;
  if(line_buffer[linePointer] == '-') { linePointer++; isMinus=1; RemoveWhtSP(); }
  if(line_buffer[linePointer] == '+') { linePointer++; RemoveWhtSP(); }
  if((numReturned = isTerm()) != NULL) auxValue = *numReturned; else return(NULL);
  if(isMinus) auxValue = -auxValue;
  RemoveWhtSP();
  while((line_buffer[linePointer] == '-') || (line_buffer[linePointer] == '+')) {
    if(line_buffer[linePointer] == '-') isMinus=1; else isMinus=0;
    RemoveNextWhtSP();
    if((numReturned = isTerm()) != NULL) {
      if(isMinus) auxValue -= *numReturned; else auxValue += *numReturned;
      RemoveWhtSP();
    } else {WriteTextln("Missing operand!"); return(NULL); }
  }
  evalNumber = auxValue;
  return(&evalNumber);
}
/*********************************************************************
    Most impotant function for Tiny BASIC interpreter - executes commands
 *********************************************************************/
void Statement()
{
  // Function for executing commands. Command is on the line_buffer[] pointed with linePointer
  int* numOnLine;
  // "Tiny Basic" commands can be added here:
  switch (line_buffer[linePointer]) {
    case (unsigned char)0xFF: break;  // Empty line
    // ...........................................................
    case 0xA9:    // LIST
      RemoveNextWhtSP();
      if((numOnLine = isNumberNext()) != NULL) PrintBasLine((*numOnLine) & 0x7F);
      else { int j=0;
      for(int i=0; i<128; i++) 
        { if(PrintBasLine(i)) j++;
          if(j == 15) {j=0; WriteTextln("Press any key..."); GetKeyChar();}  // List long program in 20 rows pages
        }
      }
      break;  // LIST
    // ...........................................................
    case 0xA8: line_buffer[0] = (unsigned char)0xFF;   // CLEAR
      for(uint8_t i=0; i<128; i++) {linePointer=0; I2C_write_line(i); Delay_Ms(10);} 
      RunMode = 0; break;  // end CLEAR
    // ...........................................................
    case 0xAA: RunMode = 1; currentLine = 0; stackPointer=0; break;  // RUN
    // ...........................................................
    case 0xAB: RunMode = 0; currentLine = 0; break; // END
    // ...........................................................
    case 0xA5: // LET
      { uint8_t ArrFlag=0;
      RemoveNextWhtSP();
      if(line_buffer[linePointer] == '@') ArrFlag=1;
      if((numOnLine = isVariableNext()) != NULL) 
        { int* pointVar = numOnLine;      // Here is variable address
          RemoveWhtSP();
          if(line_buffer[linePointer] == '=') {
            RemoveNextWhtSP();
            if((numOnLine = isExpression()) != NULL) {    // First variable is normally filled up
              *pointVar = *numOnLine;
              if(ArrFlag) {   // Array can be filled many times
                RemoveWhtSP();
                do{
                  if(line_buffer[linePointer] == ',') RemoveNextWhtSP();
                  pointVar++;
                  if(((numOnLine = isNumberNext()) != NULL) && (pointVar<(arrayVariable+sizeof(arrayVariable)))) *pointVar = *numOnLine;
                  RemoveWhtSP();
                } while (line_buffer[linePointer] == ',');
              }
            } else WriteErrMsg("Error value!");
          } else WriteErrMsg("Missing =");
        } else WriteErrMsg("Missing variable!");
      }
      break;  // end LET
    // ...........................................................
    case 0xA6: // GOSUB
      { unsigned char retLine = currentLine + 1;
        if(retLine < 128) { 
          if(stackPointer < 8) { 
            stack_buffer[stackPointer] = retLine; 
            stackPointer++;
            RemoveNextWhtSP();
            if( (numOnLine = isExpression()) != NULL )
              { if((*numOnLine >= 0) && (*numOnLine < 128)) currentLine = (unsigned char) *numOnLine; 
                else WriteErrMsg("Line number OVR!");
              }
            else WriteErrMsg("Error value!");
          }
        } else WriteErrMsg("Line number OVR!");
      }
      break;  // end GOSUB
    // ...........................................................
    case 0xA7: // RETURN
      if(stackPointer > 0) {stackPointer--; currentLine = stack_buffer[stackPointer];}
      else WriteErrMsg("No return!");
      break;  // RETURN
    // ...........................................................
    case 0xA3: // GOTO
      RemoveNextWhtSP();
      if( (numOnLine = isExpression()) != NULL )
        { if((*numOnLine >= 0) && (*numOnLine < 128)) currentLine = (unsigned char) *numOnLine; 
          else WriteErrMsg("Line number OVR!");
        }
        else WriteErrMsg("Error value!");
      break;  // end GOTO
    // ...........................................................
    case 0xA0: // PRINT
      RemoveNextWhtSP();
      do{
        if(line_buffer[linePointer] == ',') RemoveNextWhtSP();
        if((numOnLine = isExpression()) != NULL ) WriteNum(*numOnLine);
        else { 
          if(line_buffer[linePointer] == '"') {
            linePointer++;
            while((line_buffer[linePointer] != '"') && (line_buffer[linePointer] != 0xFF) && (linePointer < 32)) {
              if(line_buffer[linePointer] == '\\') {  // Escape sequences
                linePointer++; 
                if(line_buffer[linePointer] == 'n') WriteChar(0x0A);
                if(line_buffer[linePointer] == 't') WriteChar(0x09);
                if(line_buffer[linePointer] == 'f') WriteChar(0x0C); 
              }
              else WriteChar(line_buffer[linePointer]);
              linePointer++;
            }
            if(line_buffer[linePointer] == '"') linePointer++;
          }
        }
        RemoveWhtSP();
      } while (line_buffer[linePointer] == ',');
      break;  // end PRINT
    // ...........................................................
    case 0xA4:    // INPUT
      RemoveNextWhtSP();
      do{
        if(line_buffer[linePointer] == ',') RemoveNextWhtSP();
        uint8_t varName = linePointer;      // save pointer to variable name for prompt
        if( (numOnLine = isVariableNext()) != NULL ) { 
          WriteText("Insert "); 
          WriteChar(line_buffer[varName]);
          WriteTextln(":"); *numOnLine = 0;
          uint8_t recChar = '0'; 
          int minusConst = 1; 
          uint8_t isFirstChar = 1;
          // read integer number
          do{
            recChar = GetKeyChar(); WriteChar(recChar);
            // if char is number, proccess it
            if( isFirstChar && (recChar == '-')) { isFirstChar = 0; minusConst = -1; recChar = '0';}
            else { if((recChar >= '0') && (recChar <= '9')) { *numOnLine *= 10; *numOnLine += (0x0F & recChar); } }
          } while ((recChar >= '0') && (recChar <= '9'));
          *numOnLine *= minusConst;
        }
        RemoveWhtSP();
      } while (line_buffer[linePointer] == ',');
      break;  // end INPUT
    // ...........................................................
    case 0xA1:    // IF - THEN
      RemoveNextWhtSP();
      if( (numOnLine = isExpression()) != NULL ) {   // in the numLine variable is first expression
        int firstExpression = *numOnLine; 
        uint8_t typeRelOp = 0;
        RemoveWhtSP();
        if(line_buffer[linePointer] == '=') { typeRelOp = 1; linePointer++; } else
        if(line_buffer[linePointer] == '<') { typeRelOp = 2; RemoveNextWhtSP();
          if(line_buffer[linePointer] == '=') { typeRelOp = 3; linePointer++;  }
          else if(line_buffer[linePointer] == '>') { typeRelOp = 4; linePointer++; }
        } else
        if(line_buffer[linePointer] == '>') { typeRelOp = 5; RemoveNextWhtSP();
          if(line_buffer[linePointer] == '=') { typeRelOp = 6; linePointer++; }
          else if(line_buffer[linePointer] == '<') { typeRelOp = 4; linePointer++; }
        } else 
        if(line_buffer[linePointer] == '&') { typeRelOp = 7; linePointer++; }
        int* secExpression;
        RemoveWhtSP();
        if( (secExpression = isExpression()) != NULL ) {
          if(typeRelOp) {
            uint8_t relOP = 0;
            switch (typeRelOp) {
              case 1: if(firstExpression == *secExpression) relOP = 1; break;  // =
              case 2: if(firstExpression < *secExpression) relOP = 1; break;  // <
              case 3: if(firstExpression <= *secExpression) relOP = 1; break;  // <=
              case 4: if(firstExpression != *secExpression) relOP = 1; break;  // <>, ><
              case 5: if(firstExpression > *secExpression) relOP = 1; break;  // >
              case 6: if(firstExpression >= *secExpression) relOP = 1; break;  // >=
              case 7: if(firstExpression & *secExpression) relOP = 1; break;  // bit &
              }
            RemoveWhtSP();
            if(line_buffer[linePointer] == 0xA2) {
              RemoveNextWhtSP();
              if(relOP) Statement(); else {if(currentLine<127) currentLine++; else RunMode=0;}
            }  else WriteErrMsg("No THEN");
          } else WriteErrMsg("Operator error");
        } else WriteErrMsg("No 2. expr");
      } else WriteErrMsg("No 1. expr");   
      break;  // end IF - THEN
    // ...........................................................
    // ...........................................................
    // Extension "Basic" commands can be added here:
    case 0xAC:    // TIME
      RemoveNextWhtSP();
      if((numOnLine = isVariableNext()) != NULL) 
        {
	        *numOnLine = basTimer;
        } else WriteErrMsg("Missing variable!");
      break; // TIME
    // ...........................................................
    case 0xAD:     // CURSOR
      RemoveNextWhtSP();
      if((numOnLine = isExpression()) != NULL) { 
        if((*numOnLine >= 0) && (*numOnLine < 800))
        dispPointer = (uint16_t)*numOnLine;
      } else WriteErrMsg("Error value!");
      break;  // end CURSOR
    // ...........................................................
    case 0xAE:       // PUTCH
      RemoveNextWhtSP();
      uint16_t ShiftTo=0;
      // Test special characters for moving cursor
      if((line_buffer[linePointer] == '.') || (line_buffer[linePointer] == '^') || (line_buffer[linePointer] == '<') 
            ||(line_buffer[linePointer] == '>') || (line_buffer[linePointer] == 'v')|| (line_buffer[linePointer] == 'p')) {
        switch(line_buffer[linePointer])
        { // Cursor will be moved only in 4 cases - "p" is only flag
          case '^': ShiftTo=768; break;
          case '<': ShiftTo=799; break;
          case '>': ShiftTo=1; break;
          case 'v': ShiftTo=32; break;
          case 'p': ShiftTo=999; break;
        }
        RemoveNextWhtSP();
        if((numOnLine = isExpression()) != NULL) 
          { 
            if(ShiftTo==999) { // Put char to position directly
              uint32_t charToPut = *numOnLine;    // save character value
              RemoveWhtSP();
              if(line_buffer[linePointer] == ',') {
                RemoveNextWhtSP();
                if((numOnLine = isExpression()) != NULL) {      // *numOnLine holds position
                  if((*numOnLine >= 0) && (*numOnLine < 800) && (((uint8_t)charToPut & 0xFF)>0x1F) && (((uint8_t)charToPut & 0xFF)<0xA0))   // Putchar to allowed range only
                    VideoRAM[(uint16_t)*numOnLine] = ((uint8_t)charToPut & 0xFF)-0x20;
                } else WriteErrMsg("Err posit!");
              } else WriteErrMsg("Missing ,");
            } else  // Put char to cursor position with cursor update
            if((((uint8_t)*numOnLine & 0xFF)>0x1F) && (((uint8_t)*numOnLine & 0xFF)<0xA0)) { 
              VideoRAM[dispPointer] = ((uint8_t)*numOnLine & 0xFF)-0x20; // Only printable
              // And move(over) display pointer
              if((ShiftTo==1) || (ShiftTo==799)) {
                dispPointer = (dispPointer & 0x3E0) | ((dispPointer+ShiftTo) & 0x1F);
              }
              else{
                dispPointer+=ShiftTo;
                if(dispPointer >799) dispPointer-=800;
              }
            }
          } else WriteErrMsg("Missing value!");
      } else  // No special character parameter, so normal print
      if((numOnLine = isExpression()) != NULL) 
        { 
          WriteChar((uint8_t)*numOnLine & 0xFF);
        } else WriteErrMsg("Missing value!");
      break;  // end PUTCH
    // ...........................................................
    case 0xAF:      // GETCH
      RemoveNextWhtSP();
      uint16_t VideoPoint=dispPointer;
      // Test special character for direct get
      if(line_buffer[linePointer] == 'p'){
        RemoveNextWhtSP();
        if((numOnLine = isExpression()) != NULL) {      // *numOnLine holds position
          if((*numOnLine >= 0) && (*numOnLine < 800))   // Get char from allowed range only
            VideoPoint=(uint16_t)*numOnLine; else VideoPoint = 999;
          RemoveWhtSP();
          if(line_buffer[linePointer] == ',') RemoveNextWhtSP(); else WriteErrMsg("Missing ,");
        } else WriteErrMsg("Err posit!");
      }
      if((VideoPoint<800) && ((numOnLine = isVariableNext()) != NULL)) 
        {
	        *numOnLine = 0x20 + VideoRAM[VideoPoint];
        } else WriteErrMsg("Err param!");
      break; // end GETCH
    // ...........................................................
    case 0xB0:      // INKEY
      RemoveNextWhtSP();
      if((numOnLine = isVariableNext()) != NULL) 
	      *numOnLine = kbdRead();
      else WriteErrMsg("Missing variable!");
      break; // end INKEY
    // ...........................................................
    case 0xB1:      // FILE
      RemoveNextWhtSP();
      if((numOnLine = isExpression()) != NULL) noOfMemory = (uint8_t)(*numOnLine & 0x0F);
      else {  // No parameter, so print setting
        WriteChar(0x0A);   // Print number of FILE on new line
        WriteNum(noOfMemory & 0x0F);
        WriteChar(0x0A);
      }
      break; // end FILE
    // ...........................................................
    case 0xB2:      // DISK
      RemoveNextWhtSP();
      if((line_buffer[linePointer] == 'f') && (compFlags & 0x80)) {compFlags |= 0x02;linePointer++;}
      else if(line_buffer[linePointer] == 'h') {compFlags &= 0xF0;linePointer++;} 
      else {
        WriteChar(0x0A);   // Print DISK character on new line
        if(compFlags & 0x02) WriteChar('f'); else WriteChar('h');
        WriteChar(0x0A);
      }
      break; // end DISK
    // ...........................................................
    case 0xB3:      // DELAY
      RemoveNextWhtSP();
      if((numOnLine = isExpression()) != NULL) 
        { if((*numOnLine > 0) && (*numOnLine <= 1000000)) Delay_Ms(*numOnLine); }
      else WriteErrMsg("Error value!");
      break; // end DELAY
    // ...........................................................
    case 0xB4:      // BEEP
      RemoveNextWhtSP();
      if((numOnLine = isExpression()) != NULL) 
        { int32_t Frequency = *numOnLine;      // First parameter is frequency
          RemoveWhtSP();
          if(line_buffer[linePointer] == ',') {
            RemoveNextWhtSP();
            if((numOnLine = isExpression()) != NULL) {      // *numOnLine holds duration (milisec)
              if((Frequency >= 100) && (Frequency <= 4000) && (*numOnLine >= 20) && (*numOnLine <= 100000))
              {
                Frequency = 500000/Frequency;   // Change Frequency to pulse duration HI or LOW [microsec]
                for(int32_t i=0; i<((*numOnLine * 500) / Frequency); i++ ) {
                  GPIOC->BSHR = 0x00000001;
                  Delay_Us(Frequency);
                  GPIOC->BSHR = 0x00010000;
                  Delay_Us(Frequency);
                }
              }
            } else WriteErrMsg("Error Time!");
          } else WriteErrMsg("Missing ,");
        } else WriteErrMsg("Error Freq!");
      break; // end BEEP
    // ...........................................................
    case 0xB5:  break; // REM
    // ...........................................................
    case 0xBB:      // COPY
      RunMode = 0; currentLine = 0;   // COPY stops program every time
      RemoveNextWhtSP();
      if(line_buffer[linePointer] == 's') {     // Copy lines in this file
        RemoveNextWhtSP();
        if((numOnLine = isNumberNext()) != NULL) { 
          int32_t Source = *numOnLine;      // First parameter is Source line number
          RemoveWhtSP();
          if(line_buffer[linePointer] == ',') {
            RemoveNextWhtSP();
            if((numOnLine = isNumberNext()) != NULL) {
              int32_t Target = *numOnLine;      // Second parameter is Target line number
              RemoveWhtSP();
              if(line_buffer[linePointer] == ',') {
                RemoveNextWhtSP();
                if((numOnLine = isNumberNext()) != NULL) {
                  int32_t nCount = *numOnLine;      // Third parameter is Count line number
                  // Check parameters range
                  if((Source<128)&&(Target<128)&&(nCount>0)&&(nCount<128)&&(Source!=Target)) {
                    int32_t UpOrDown;
                    if(Source>Target) {
                      UpOrDown=1;   // Copy must be from low to high
                      if((Source+nCount)>127) nCount=128-Source;    // Trim counter to max possible
                    } else {
                      UpOrDown=-1;   // Copy must be from high to low
                      if((Target+nCount)>127) nCount=128-Target;    // Trim counter to max possible
                      Source += (nCount-1); Target += (nCount-1);
                    }
                    do {    // Ready to copy, so move it
                      I2C_read_line((uint8_t)Source); Delay_Ms(5);
                      linePointer = 0;I2C_write_line((uint8_t)Target); Delay_Ms(10);
                      Source+=UpOrDown;Target+=UpOrDown;nCount--;
                    } while(nCount);
                  } else WriteErrMsg("Over range!");
                } else WriteErrMsg("Error Count!");
              } else WriteErrMsg("Missing ,");
            } else WriteErrMsg("Error Target!");
          } else WriteErrMsg("Missing ,");
        } else WriteErrMsg("Error Source!");
      } else
      if((line_buffer[linePointer] == 'h') || (line_buffer[linePointer] == 'f')) {     // Copy this file to h or f disk
        uint8_t ChDisk = line_buffer[linePointer];
        RemoveNextWhtSP();
        if(!(compFlags & 0x80) && (ChDisk == 'f')) WriteErrMsg("No f DISK!");   // Is Disk f connected
        else {    // We can copy, f disk is connected or h disk must be on board
          if((numOnLine = isNumberNext()) != NULL) {    // Get number of file
            uint8_t Target = (uint8_t)(*numOnLine & 0x0F);
            uint8_t OldnoOfMemory = noOfMemory;   // Save variables for I2C memory communication
            uint8_t OldcompFlags = compFlags;
            for(uint8_t i=0; i<128; i++) {    // Copy 128 lines of program to new destination
              I2C_read_line(i); Delay_Ms(5);
              noOfMemory = Target;
              if(ChDisk == 'h') compFlags = 0x00; else compFlags = 0x02;    // h/f Disk address is different
              linePointer = 0;
              I2C_write_line(i); Delay_Ms(10);
              noOfMemory = OldnoOfMemory;
              compFlags = OldcompFlags;
            }   // end of copy so signal it
            WriteTextln("\nCopy to disk "); WriteChar(ChDisk);
          } else WriteErrMsg("No file number!");
        }
      } else WriteErrMsg("Error parameter");
      break; // end COPY
    // ...........................................................
    // ...........................................................
    // Extension "Input / Output" commands can be added here:
    case 0xB6:      // AINP
      RemoveNextWhtSP();
      if((numOnLine = isVariableNext()) != NULL) 
        { 
	        ADC1->CTLR2 |= ADC_SWSTART;   // Start sw conversion
        	while(!(ADC1->STATR & ADC_EOC)) {};    // Wait for conversion complete
          *numOnLine = ADC1->RDATAR;  // Save value to variable
        } else WriteErrMsg("Missing variable!");
      break; // end AINP
    // ...........................................................
    case 0xB7:     // DINP
      RemoveNextWhtSP();
      if((numOnLine = isVariableNext()) != NULL) 
        { // First parameter is variable
          RemoveWhtSP();
          if(line_buffer[linePointer] == ',') {
            RemoveNextWhtSP();
            GPIO_InitTypeDef GPIO_InitStructure = {0};  // Prepare GPIO for input with pull-up
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            if(line_buffer[linePointer] == '1') {      // Is input 1?
              GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
              GPIO_Init(GPIOD, &GPIO_InitStructure);
              *numOnLine = GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6);
              linePointer++;    // Prepare for second statement
            }  
            else if(line_buffer[linePointer] == '2') {      // Is input 2?
              GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
              GPIO_Init(GPIOD, &GPIO_InitStructure);
              *numOnLine = GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_5);
              linePointer++;    // Prepare for second statement
            }  
#ifndef SYSCLK_EXT_CRYSTAL
            else if(line_buffer[linePointer] == '3') {      // Is input 3?
              GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
              GPIO_Init(GPIOA, &GPIO_InitStructure);
              *numOnLine = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1);
              linePointer++;    // Prepare for second statement
            }  
            else if(line_buffer[linePointer] == '4') {      // Is input 4?
              GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
              GPIO_Init(GPIOA, &GPIO_InitStructure);
              *numOnLine = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2);
              linePointer++;    // Prepare for second statement
            }  
#endif
            else WriteErrMsg("Error Input!");
          } else WriteErrMsg("Missing ,");
        } else WriteErrMsg("Error variable!");
      break; // end DINP
    // ...........................................................
    case 0xB8:     // DOUT
      RemoveNextWhtSP();
      if((numOnLine = isExpression()) != NULL) 
        { // First parameter is value
          RemoveWhtSP();
          if(line_buffer[linePointer] == ',') {
            RemoveNextWhtSP();
            GPIO_InitTypeDef GPIO_InitStructure = {0};  // Prepare GPIO for output
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
            if(line_buffer[linePointer] == '1') {      // Is output 1?
              GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
              GPIO_Init(GPIOD, &GPIO_InitStructure);
              if(*numOnLine) GPIO_SetBits(GPIOD,GPIO_Pin_6); else GPIO_ResetBits(GPIOD,GPIO_Pin_6);
              linePointer++;    // Prepare for second statement
            }  
            else if(line_buffer[linePointer] == '2') {      // Is output 2?
              GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
              GPIO_Init(GPIOD, &GPIO_InitStructure);
              if(*numOnLine) GPIO_SetBits(GPIOD,GPIO_Pin_5); else GPIO_ResetBits(GPIOD,GPIO_Pin_5);
              linePointer++;    // Prepare for second statement
            }  
#ifndef SYSCLK_EXT_CRYSTAL
            else if(line_buffer[linePointer] == '3') {      // Is output 3?
              GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
              GPIO_Init(GPIOA, &GPIO_InitStructure);
              if(*numOnLine) GPIO_SetBits(GPIOA,GPIO_Pin_1); else GPIO_ResetBits(GPIOA,GPIO_Pin_1);
              linePointer++;    // Prepare for second statement
            }  
            else if(line_buffer[linePointer] == '4') {      // Is output 4?
              GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
              GPIO_Init(GPIOA, &GPIO_InitStructure);
              if(*numOnLine) GPIO_SetBits(GPIOA,GPIO_Pin_2); else GPIO_ResetBits(GPIOA,GPIO_Pin_2);
              linePointer++;    // Prepare for second statement
            }
#endif  
            else WriteErrMsg("Error Output!");
          } else WriteErrMsg("Missing ,");
        } else WriteErrMsg("Error variable!"); 
      break; // end DOUT
    // ...........................................................
    case 0xB9:     // I2CW
      RemoveNextWhtSP();
      if((numOnLine = isExpression()) != NULL) {
        if(compFlags & 0x40)
          { 
            // Communication is similar to 24Cxxx - see above
            while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );
            I2C_GenerateSTART( I2C1, ENABLE );
            while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );
            I2C_Send7bitAddress( I2C1, 0x40, I2C_Direction_Transmitter );
            while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );
            I2C_SendData( I2C1, *numOnLine & 0x0FF );
            while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
            I2C_SendData( I2C1, (*numOnLine >> 8) & 0x0FF );
            while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
            I2C_GenerateSTOP( I2C1, ENABLE );
            // And speed-up I2C for memory communication
          } 
      } else WriteErrMsg("Missing value!");
      break; // end I2CW
    // ...........................................................
    case 0xBA:     // I2CR
      RemoveNextWhtSP();
      if((numOnLine = isVariableNext()) != NULL) {
        if(compFlags & 0x40) 
          { 
            // Communication is similar to 24Cxxx - see above
            while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );
            I2C_GenerateSTART( I2C1, ENABLE );
            while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );
            I2C_Send7bitAddress( I2C1, 0x40, I2C_Direction_Receiver );
            while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) );
            while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED ) );
            int recI2Cval = I2C_ReceiveData( I2C1 );
            while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED ) );
            recI2Cval |= (I2C_ReceiveData( I2C1 )) << 8;
            I2C_GenerateSTOP( I2C1, ENABLE );
            *numOnLine = recI2Cval;
            // And speed-up I2C for memory communication
          }
      } else WriteErrMsg("Missing variable!");
      break; // end I2CR
    // ...........................................................
    // ...........................................................

    default:
      WriteErrMsg("no such command!");
    }
}


/*********************************************************************
 * @fn      main
 * @brief   Main program.
 * @return  none
 */

int main(void)
{
  SystemCoreClockUpdate();    // Standard functions...
  Delay_Init();   // ... at start-up
  GPIO_InOut_INIT();    // Initialize GPIO ports
  TIM1_INT_Init();    // Initialize timer SPI and interrupts
  DMA_Tx_Init(DMA1_Channel3, (u32)&SPI1->DATAR, (u32)DMABuff, 47);    // Setup DMA

  // The next part is time-critical, so assembler is used
  // Be careful to change this part!
  TIM1->CTLR1 |= TIM_CEN;   // Timer1 on
  // Piece of assembler code for precise timing between TIMER and DMA start
  asm volatile (
    "   addi sp, sp, -16 \n"  // Allocate stack
    "   sw t0, 12(sp) \n"    // Save my working register
    "   li t0, 56 \n"       // Get Delay time - slight change will move display horizontally left or right
    "shorttim:  addi  t0,t0,-1  \n"   // Decrement value in register
    "   bne  t0, x0, shorttim  \n"     // Test for zero and branch
    "   lw t0, 12(sp) \n"    // Restore working register
    "   addi sp, sp, 16 \n"  // Return allocated stack
  );
  DMA1_Channel3->CFGR |= 0x00001; // DMA channel on
  // End of time-critical part

  kbd_init(); // Start keyboard
  adc_init(); // Start A/D converter
  WriteTextln("TinyBasic 1.0");  // Introductory message
  line_buffer[32] = (unsigned char)0xFF;  // Stop symbol at end of buffer
  IIC_Init(100000, 0x20);  // Start I2C module @ 100 000kHz at first
  // Check devices connected to I2C bus
  if(!CheckI2CDevice(0xA2)) WriteText("No DISK f, "); else compFlags |= 0x80;
  if(!CheckI2CDevice(0x40)) WriteText("No PCF8575, "); else compFlags |= 0x40;
#ifdef SYSCLK_EXT_CRYSTAL 
  WriteTextln("Ext Xtal");
#else
  WriteTextln("Int OSC");
#endif
  I2C1->CKCFGR = 0xC005;      // Speed up I2C to slightly bellow 400000 kHz
  // Completion of the initial report

  // Test for autorun (Is statement RUN - 0xAA in the first byte of file 0?)
  I2C_read_line(0);   // Read the first line of file
  if(line_buffer[0] == (unsigned char)0xAA) {   // Test first byte for RUN statement
    linePointer=1;    // First byte is bytecode of RUN, so move to next byte...
    KbdData = 0x5A;   // ... and pretend that user pressed Enter after RUN
  }
  char recChar;   // Processed character from keyboard
  while(1)
  {
    // Get char to buffer (only to max size of buffer or end of line (Enter))
    recChar = GetKeyChar();
    // and return it to display
    if(recChar == 0x09) {  // Tab needs special cure 
      if(linePointer < 27) WriteText("    "); // Tab inserts only 4 spaces
    } else {  // Other characters
      if(linePointer < 32) WriteChar(recChar);
    }
    // Char must be saved to line_buffer
    // Is it char to start process line (Enter)?
    if (recChar == '\n') {
      // look for the newline. That's the end of your sentence:
      line_buffer[linePointer] = (unsigned char)0xFF;    // End of line is signalled with 0xFF
      currentLine = 0; linePointer=0;  // Prepare pointer to beginning,...
      // ... and process line
      int* numLine;
      RemoveWhtSP();
      if((numLine = isNumberNext()) != NULL) {    // If line starts with number ...
        RemoveWhtSP();  // ... discard number and white spaces ...
        if((*numLine >= 0) && (*numLine < 128)) I2C_write_line((unsigned char) *numLine); // ... and save line
        else WriteErrMsg("Error line number!");
      }
      else {  // No number at begin of line
        do{ // Execute statements while RunMode
            uint8_t auxLine = currentLine;    // Save line number
            // -----------------------------------------------------------
            Statement();
            // Test for the second Statement on the line
            RemoveWhtSP();  // IF, GOTO or GOSUB will discard second Statement
            if((auxLine == currentLine) && (line_buffer[linePointer] == ';')) {
              RemoveNextWhtSP();
              Statement();
            }
            // Test for the third Statement on the line
            RemoveWhtSP();  // IF, GOTO or GOSUB will discard third Statement
            if((auxLine == currentLine) && (line_buffer[linePointer] == ';')) {
              RemoveNextWhtSP();
              Statement();
            }
            if(RunMode && (auxLine == currentLine)) currentLine++; // No jump, so go to next line
            // Prepare next program line
            if(currentLine > 127) { RunMode = 0; currentLine = 0;} // Number of line step over maximum line
            else { I2C_read_line(currentLine); linePointer = 0; }   // Line number is OK, so prepare it
        } while(RunMode);
        WriteTextln("\nOK");
      }
      // ... line was proccessed so...
      linePointer=0;  // ... move pointer to beginning again.
    }
    else {       // No end of line - just process character
      if(recChar==0x08) {    // Is backspace, so return char(s) in the line
        if (linePointer > 0) {  // Edit can be only in current line
          linePointer--;    // Return pointer 1 char
          if(line_buffer[linePointer] < 0xA0) { dispPointer--;VideoRAM[dispPointer] = 0x00;} // Discard normal character
          else {  // It is keyword - that means rather difficult
            if(line_buffer[linePointer] < 0xBC) { // Just within keyword range
              int pKeyWord = ((line_buffer[linePointer] - (uint8_t)0xA0)) << 3;  // Get lenght of keyword
              for(int j=0; j<KeyWordTab[pKeyWord];j++){
                VideoRAM[dispPointer] = 0x00;     // Discard 1 letter of keyword
                // Test pointer and stop roll display off
                if(dispPointer>1) dispPointer--;
              }
              VideoRAM[dispPointer] = 0x00;     // And the discard last letter of keyword
            }
          } 
        }
      } else {
        if (linePointer < 32) {
          // Put normal char to line buffer
          if(recChar == 0x09) {    // Special char is Tab
            if(linePointer < 27) {  // Expand Tab to 4 spaces (for future backspace)
              line_buffer[linePointer++] = ' '; line_buffer[linePointer++] = ' ';
              line_buffer[linePointer++] = ' '; line_buffer[linePointer++] = ' '; 
            }
          } else {
          // Save real "normal" char finally
            line_buffer[linePointer] = recChar;
            linePointer++;  // max 32 chars in buffer
          }
        }
      }
    }
  }
}
