#include "LPC13xx.h"
#include "clkconfig.h"
#include "gpio.h"
#include "config.h"
#include "uart.h"
#include "ssp.h"
#include "timer32.h"
#include "nRF24L01.h"

#define SSEL_PORT 2
#define SSEL 2
#define SSEL_LOW GPIOSetValue( SSEL_PORT, SSEL, LED_OFF)
#define SSEL_HIGH GPIOSetValue( SSEL_PORT, SSEL, LED_ON)

#define CE_PORT 2
#define CE 0
#define CE_LOW GPIOSetValue(CE_PORT, CE, LED_OFF)
#define CE_HIGH GPIOSetValue(CE_PORT, CE, LED_ON)

#define IRQ_PORT 2
#define IRQ 1

#define RX_MODE 1
#define TX_MODE 0

#define PAYLOAD_SIZE 16

typedef struct nrf_state
{
    uint8_t is_transmitting;
    uint8_t payload_size;
} nrf_state;

nrf_state nrf;

void PIOINT2_IRQHandler(void)
{
    GPIOIntClear(IRQ_PORT, IRQ);
}

uint8_t SPI(uint8_t TX_Data) {
	while ((LPC_SSP->SR & (SSPSR_TNF | SSPSR_BSY)) != SSPSR_TNF);
	LPC_SSP->DR = TX_Data;

	while ((LPC_SSP->SR & (SSPSR_BSY | SSPSR_RNE)) != SSPSR_RNE);
	return LPC_SSP->DR;
}

uint8_t nrf_read_register(uint8_t reg, uint8_t *data, uint8_t len)
{
      SSEL_LOW;
      uint8_t r = SPI(reg);

      for (int i=0; i<len; i++)
          data[i] = SPI(0xff); // send dummy byte
      SSEL_HIGH;

      return r;
}

uint8_t nrf_write_register(uint8_t reg, const uint8_t *data, uint8_t len)
{
    SSEL_LOW;
    uint8_t r = SPI(0x20 | (0x1f & reg));

    for (int i=0; i<len; i++)
        SPI(data[i]);
    SSEL_HIGH;

    return r;
}

void nrf_set_mode(uint8_t is_rx)
{
    uint8_t config_reg = 0;

    nrf_read_register(CONFIG, &config_reg, 1);

    if (is_rx){
        nrf.is_transmitting = 0;
        config_reg |= 1 << PWR_UP | 1 << PRIM_RX;
        CE_HIGH;
    }
    else
    {
        nrf.is_transmitting = 1;
        config_reg &= 0xfe; // clear PRIM_RX
        CE_LOW;
    }
    nrf_write_register(CONFIG, &config_reg, 1);
}

void nrf_setup(uint8_t channel,
               uint8_t payload_size,
               const uint8_t* raddr,
               const uint8_t* taddr)
{

    nrf.payload_size = payload_size;

    // set channel
    nrf_write_register(RF_CH, &channel, 1);

    // set payload size
    nrf_write_register(RX_PW_P0, &payload_size, 1);

    // set taddr
    nrf_write_register(TX_ADDR, taddr, 5);

    // set raddr
    CE_LOW;
    nrf_write_register(RX_ADDR_P0, raddr, 5);
    CE_HIGH;

    // start in rx mode
    nrf.is_transmitting = 0;
    nrf_set_mode(RX_MODE);
};

void interface_setup()
{

    // set SSEL
    GPIOSetDir(SSEL_PORT, SSEL, 1);
    SSEL_HIGH;

    // set interrupt, active low
    GPIOSetDir(IRQ_PORT, IRQ, 0);
    GPIOSetInterrupt(IRQ_PORT, IRQ, 0, 0, 1);
    GPIOIntEnable(IRQ_PORT, IRQ);

    // set CE pin
    GPIOSetDir(CE_PORT, CE, 1);
    CE_HIGH;

    // start SSP
    SSPInit();

    // start uart
    UARTInit(115200);

    // setup timer
    init_timer32(0, TIME_INTERVAL);
    enable_timer32(0);
}

uint8_t nrf_get_status()
{
    SSEL_LOW;
    uint8_t status = SPI(NOP);
    SSEL_HIGH;

    return status;
}

uint8_t nrf_data_ready()
{
    return nrf_get_status() & (1 << RX_DR);
}

void nrf_get_data(uint8_t* buffer)
{
    SSEL_LOW;
    SPI(R_RX_PAYLOAD);

    for (int i=0; i<nrf.payload_size; i++)
    {
        buffer[i] = SPI(NOP);
    }
    SSEL_HIGH;
    uint8_t v = 1 << RX_DR;
    nrf_write_register(STATUS, &v, 1);
}

int main (void)
{
    uint8_t input_buffer[PAYLOAD_SIZE];
    GPIOInit();
    interface_setup();

    nrf_setup(1, PAYLOAD_SIZE, (uint8_t*)"lpcRA", (uint8_t*)"avrTA");
    while (1)
    {
        if (nrf_data_ready())
        {
            nrf_get_data(input_buffer);
            delay32Ms(0,10);
        }
    }
}

