#include <stdbool.h>
#include "../commands.h"
#include "../bus/uart/uart1.h"
#include "../bus/i2c/i2c.h"
#include "../bus/spi/spi_driver.h"
#include "../helpers/delay.h"
#include "../registers/system/pin_manager.h"
#include "powersource.h"

extern void SetCS(uint8_t channel);
extern void UnsetCS();


#define MCP4822_CHANNEL_A 0
#define MCP4822_CHANNEL_B 0x8000

#define MCP4822_GAIN_1 0
#define MCP4822_GAIN_2 0x2000

#define MCP4822_ACTIVE 0x1000
#define MCP4822_SHUTDOWN 0

#define CS_CHANNEL_3 2 // CS3 linked to A2 in 74HC4051
#define CS_CHANNEL_4 3


// PVS1 PVS3 share the same channel
// PCS and PVS2 share the same channel
static uint16_t channel_mapping_table[4][2] = {
    {CS_CHANNEL_3, MCP4822_CHANNEL_A},
    {CS_CHANNEL_3, MCP4822_CHANNEL_B},
    {CS_CHANNEL_3, MCP4822_CHANNEL_A},
    {CS_CHANNEL_3, MCP4822_CHANNEL_B}
};


response_t POWER_SOURCE_SetPower(void) {
    // the uart read need to be at the top of the function
    // since the data is sending continuously.
    uint8_t channel = UART1_Read() & 0x03;
    uint16_t power = UART1_ReadInt() & 0xFFF;
    // the 2D array is 16 bits
    uint8_t cs_pin = 0xFF & channel_mapping_table[channel][0];
    uint16_t mcp4822_channel = channel_mapping_table[channel][1];
    uint16_t send_data = mcp4822_channel|MCP4822_ACTIVE|MCP4822_GAIN_1|power;

    SPI_DRIVER_Close();
    SPI_DRIVER_Open(DAC_CONFIG);
    SetCS(cs_pin);
    SPI_DRIVER_ExchangeWord(send_data);
    UnsetCS();

    return SUCCESS;
}

response_t POWER_SOURCE_SetDAC(void) {
    uint8_t address = UART1_Read();
    uint8_t channel = UART1_Read();
    uint8_t power = UART1_ReadInt();

    I2C_StartSignal();
    I2C_Transmit(address);
    I2C_Transmit(64 | (channel << 1));
    I2C_Transmit(0xFF & (power >> 8));
    I2C_Transmit(0xFF & power);
    I2C_StopSignal();

    DELAY_us(6);

    return SUCCESS;
}