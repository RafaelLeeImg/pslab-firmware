#include <stdint.h>
#include "../registers/converters/adc1.h"
#include "../registers/memory/dma.h"
#include "../registers/system/pin_manager.h"
#include "../registers/timers/tmr5.h"
#include "../bus/spi/spi_driver.h"
#include "../bus/uart/uart1.h"
#include "../helpers/buffer.h"
#include "../commands.h"

/* Static function prototypes */
static void Capture(void);
static void ResetTrigger(void);
static void SetTimeGap(void);
void SetCS(uint8_t channel);
void UnsetCS();

response_t OSCILLOSCOPE_CaptureOne(void) {
    SetCHANNELS(0); // Capture one channel.
    Capture();    
    return SUCCESS;
}

response_t OSCILLOSCOPE_CaptureTwo(void) {
    SetCHANNELS(1); // Capture two channels.
    Capture();
    return SUCCESS;
}

response_t OSCILLOSCOPE_CaptureThree(void) {
    SetCHANNELS(2); // Capture four channels, but ignore the fourth.
    Capture();
    return SUCCESS;
}

response_t OSCILLOSCOPE_CaptureFour(void) {
    SetCHANNELS(3);  // Capture four channels.
    Capture();
    return SUCCESS;
}

static void Capture(void) {
    uint8_t config = UART1_Read();
    SetSAMPLES_REQUESTED(UART1_ReadInt());
    SetDELAY(UART1_ReadInt()); // Wait DELAY / 8 us between samples.

    uint8_t ch0sa = config & 0x0F;
    uint8_t ch123sa = config & 0x10;
    uint8_t trigger = config & 0x80;

    ADC1_SetOperationMode(ADC1_10BIT_SIMULTANEOUS_MODE, ch0sa, ch123sa);
    ADC1_ConversionChannelsSet(GetCHANNELS());

    if (trigger) ResetTrigger();
    else SetTRIGGERED(1);

    int i;
    for (i = 0; i <= GetCHANNELS(); i++) {
        SetBUFFER_IDX(i, &BUFFER[i * GetSAMPLES_REQUESTED()]);
    }
    
    SetCONVERSION_DONE(0);
    SetSAMPLES_CAPTURED(0);
    SetBUFFER_IDX(0, &BUFFER[0]);
    SetTimeGap();
    ADC1_InterruptFlagClear();
    ADC1_InterruptEnable();
    LED_SetLow();
}

response_t OSCILLOSCOPE_CaptureDMA(void) {
    uint8_t config = UART1_Read();
    SetSAMPLES_REQUESTED(UART1_ReadInt());
    SetDELAY(UART1_ReadInt());  // Wait DELAY / 8 us between samples.

    uint8_t ch0sa = config & 0x0F;
    uint8_t mode = config & 0x80 ? ADC1_12BIT_DMA_MODE : ADC1_10BIT_DMA_MODE;

    SetCHANNELS(0);  // Capture one channel.
    ADC1_SetOperationMode(mode, ch0sa, 0);

    DMA_StartAddressASet(DMA_CHANNEL_0, (uint16_t) &BUFFER[0]);
    DMA_PeripheralAddressSet(DMA_CHANNEL_0, (uint16_t) &ADC1BUF0);
    DMA_TransferCountSet(DMA_CHANNEL_0, GetSAMPLES_REQUESTED() - 1);
    DMA_FlagInterruptClear(DMA_CHANNEL_0);
    DMA_InterruptEnable(DMA_CHANNEL_0);
    DMA_ChannelEnable(DMA_CHANNEL_0);

    SetSAMPLES_CAPTURED(GetSAMPLES_REQUESTED());
    SetCONVERSION_DONE(1); // Assume it's all over already.
    SetTimeGap();
    LED_SetLow();
    
    return SUCCESS;
}

static void ResetTrigger(void) {
    SetTRIGGER_WAITING(0);
    SetTRIGGER_READY(0);
    SetTRIGGERED(0);
}

static void SetTimeGap(void) {
    TMR5_Initialize();
    TMR5_StopWhenIdle();
    TMR5_Period16BitSet(GetDELAY() - 1);
    TMR5_SetPrescaler(TMR_PRESCALER_8);
    TMR5_InterruptFlagClear();
    TMR5_InterruptDisable();
    TMR5_Start();
}

response_t OSCILLOSCOPE_GetCaptureStatus(void) {
    UART1_Write(GetCONVERSION_DONE());
    UART1_WriteInt(GetSAMPLES_CAPTURED());
    return SUCCESS;
}

response_t OSCILLOSCOPE_ConfigureTrigger(void) {
    uint8_t config = UART1_Read();
    SetTRIGGER_CHANNEL(config & 0x0F);
    SetTRIGGER_PRESCALER(config >> 4);
    SetTRIGGER_LEVEL(UART1_ReadInt());
    return SUCCESS;
}

void UnsetCS() {
    SetCS(7); // select channel 7 which is not connected to any device
}

response_t OSCILLOSCOPE_SetPGAGain(void) {
    uint8_t channel = UART1_Read();
    uint8_t gain = UART1_Read();
    uint8_t max5400_resistor;
    switch (gain) {
        case 1:
            gain = 2;
            max5400_resistor = 128; // register=127.50
            break;
        case 2:
            gain = 4;
            max5400_resistor = 64; // register=63.75
            break;
        case 3:
            gain = 5;
            max5400_resistor = 51; // register=51.00
            break;
        case 4:
            gain = 8;
            max5400_resistor = 32; // register=31.87
            break;
        case 5:
            gain = 10;
            max5400_resistor = 26; // register=25.50
            break;
        case 6:
            gain = 16;
            max5400_resistor = 16; // register=15.94
            break;
        case 7:
            gain = 32;
            max5400_resistor = 8; // register=7.97
            break;
        default: // including case 0
            gain = 1;
            max5400_resistor = 255;
            break;
    }

    SPI_DRIVER_Close();
    SPI_DRIVER_Open(PGA_CONFIG);
    SetCS(channel);
    SPI_DRIVER_ExchangeByte(max5400_resistor);
    UnsetCS();
    SPI_DRIVER_Close();
    LED_SetHigh();

    return SUCCESS;
}

void SetCS(uint8_t channel) {
    switch(channel) {
        case 1:
            CS_CH1_SetLow();
            CS_CH2_SetLow();
            CS_CH3_SetLow();
            break;
        case 2:
            CS_CH1_SetHigh();
            CS_CH2_SetLow();
            CS_CH3_SetLow();
            break;
        case 3:
            CS_CH1_SetLow();
            CS_CH2_SetHigh();
            CS_CH3_SetLow();
            break;
        case 4:
            CS_CH1_SetHigh();
            CS_CH2_SetHigh();
            CS_CH3_SetLow();
            break;
        case 5:
            CS_CH1_SetLow();
            CS_CH2_SetLow();
            CS_CH3_SetHigh();
            break;
        case 6:
            CS_CH1_SetHigh();
            CS_CH2_SetLow();
            CS_CH3_SetHigh();
            break;
        default: // case 0, 7 and other conditions
            CS_CH1_SetHigh();
            CS_CH2_SetHigh();
            CS_CH3_SetHigh();
            break;
    }
    return;
}
