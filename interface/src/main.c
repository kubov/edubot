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
#define CE_LOW GPIOSetValue( CE_PORT, CE, LED_OFF)
#define CE_HIGH GPIOSetValue( CE_PORT, CE, LED_ON)

#define IRQ_PORT 2
#define IRQ 1


void PIOINT2_IRQHandler(void)
{
  GPIOIntClear(IRQ_PORT, IRQ);
  return;
}


uint8_t SPI(uint8_t TX_Data) {
	while ((LPC_SSP->SR & (SSPSR_TNF | SSPSR_BSY)) != SSPSR_TNF);
	LPC_SSP->DR = TX_Data;

	while ((LPC_SSP->SR & (SSPSR_BSY | SSPSR_RNE)) != SSPSR_RNE);
	return LPC_SSP->DR;
}

uint8_t mirf_init()
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
}

uint8_t mirf_read_register(uint8_t reg, uint8_t *data, uint8_t len)
{
      SSEL_LOW;
      SPI(reg);

      for (int i=0; i<len; i++)
          data[i] = SPI(0xff); // send dummy byte
      SSEL_HIGH;

      return data[0];
}

void mirf_write_register(uint8_t reg, uint8_t *data, uint8_t len)
{
    SSEL_LOW;
    SPI(0x20 | (0x1f & reg));

    for (int i=0; i<len; i++)
        SPI(data[i]);
    SSEL_HIGH;
}

int main (void)
{
    char status = 0xff;
    char value = 0xcc;

    init_timer32(0, TIME_INTERVAL);
    enable_timer32(0);
    mirf_init();
    UARTInit(115200);

    mirf_write_register(0x0a, "wlan0", 5);
    uint8_t buf[5];
    while (1)
    {
        mirf_read_register(0x0a, buf, 5);
        delay32Ms(0, 500);
        //__WFI();
    }
}
