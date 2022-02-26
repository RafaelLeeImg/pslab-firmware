#include <stdbool.h>
#include "../commands.h"
#include "../bus/uart/uart1.h"
#include "../bus/i2c/i2c.h"
#include "../helpers/delay.h"
#include "../registers/system/pin_manager.h"
#include "powersource.h"

response_t POWER_SOURCE_SetPower(void) {
    SPI_DRIVER_Close();
    SPI_DRIVER_Open(PGA_CONFIG);
    SetCS(channel);
    int ret = SPI_DRIVER_ExchangeByte(max5400_resistor);
    SetCS(7); // select channel 7 which is not connected to any device
    SPI_DRIVER_Close();
    return ret;
}

response_t POWER_SOURCE_SetDAC(void) {

    uint8_t address = UART1_Read();
    uint8_t channel = UART1_Read();
    uint8_t power = UART1_ReadInt();

    I2C_InitializeIfNot(I2C_BAUD_RATE_400KHZ, DISABLE_INTERRUPTS);

    I2C_StartSignal();
    I2C_Transmit(address);
    I2C_Transmit(64 | (channel << 1));
    I2C_Transmit(0xFF & (power >> 8));
    I2C_Transmit(0xFF & power);
    I2C_StopSignal();

    DELAY_us(6);

    return SUCCESS;
}