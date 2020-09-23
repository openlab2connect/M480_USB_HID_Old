/*
    Microchip CAP driver over serial bus
 */

#include <stdio.h>
#include <stdbool.h>
#include "NuMicro.h"
#include "spi_cap.h"
#include "hid_transfer.h"

/**
 * @brief       GPIO PB IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PB default IRQ, declared in startup_M480.s.
 */
void GPA_IRQHandler(void)
{
	//uint8_t active_low;
    uint8_t cap_delta[6] = {0};
	uint8_t sensor_int[2] = {0x7d, 0x00};
	uint8_t mainctrl[4] = {0x7d, 0x0, 0x0, 0x00};
	uint8_t sensorinput[2] = {0x7d, 0x10};

	if (GPIO_GET_INT_FLAG(PA, BIT12))
    {
        // active_low = CAP_Read(sensor_int, sizeof(sensor_int), &active_low);
        // if (actual_low & 0x01) {
        //     CAP_Write(mainctrl0, sizeof(mainctrl));
        // }

        // GPIO_CLR_INT_FLAG(PA, BIT12);
        //printf("PA.12 INT occurred\n");
        
        //Read register 10h-15h
        CAP_Read(sensorinput, sizeof(sensorinput)+5, &cap_delta[0]);

        CAP_Write(mainctrl, sizeof(mainctrl));
        GPIO_CLR_INT_FLAG(PA, BIT12);

        //Send CAP sensing data back to usb host
        HID_SetInReport(&cap_delta[0], sizeof(cap_delta));
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        PA->INTSRC = PA->INTSRC;
        printf("Un-expected interrupts.\n");
    }
}

void delay_us(int usec)
{
    /*
     *  Configure Timer0, clock source from XTL_12M. Prescale 12
     */
    /* TIMER0 clock from HXT */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR0SEL_Msk)) | CLK_CLKSEL1_TMR0SEL_HXT;
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk;
    TIMER0->CTL = 0;        /* disable timer */
    TIMER0->INTSTS = (TIMER_INTSTS_TIF_Msk | TIMER_INTSTS_TWKF_Msk);   /* write 1 to clear for safety */
    TIMER0->CMP = usec;
    TIMER0->CTL = (11 << TIMER_CTL_PSC_Pos) | TIMER_ONESHOT_MODE | TIMER_CTL_CNTEN_Msk;

    while (!TIMER0->INTSTS);
}

// PA12 low-level trigger drive by CAP
void CAP_Interrupt()
{
    GPIO_SetMode(PA, BIT12, GPIO_MODE_INPUT);
    GPIO_EnableInt(PA, 12, GPIO_INT_LOW);
    NVIC_EnableIRQ(GPA_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PA, BIT12);
}

void CAP_Initialization()
{
	uint8_t ledlink[4] = {0x7d, 0x72, 0x0, 0x3f};
    uint8_t cal[4] = {0x7d, 0x26, 0x00, 0x3f};
    uint8_t mainctrl[4] = {0x7d, 0x0, 0x0, 0x00};

    //Control sensor input is linked to an LED output
    CAP_Write(ledlink, sizeof(ledlink));
    //Forcew sensor intputs to be re-calibrated affecting 
    //both the analog and digital blocks
    CAP_Write(cal, sizeof(cal));
    delay_us(600000);
    //Control the primary power state
    CAP_Write(mainctrl, sizeof(mainctrl));
}

void CAP_Write(uint8_t * byteArray, size_t byteSize)
{
	uint32_t i;
	uint8_t rb;
	uint8_t write = 0x7e;

	for (i=0; i < byteSize; i++) {
		if (i == 2)
		  SPI_WRITE_TX(SPI1, (uint32_t)write);
		else
		  SPI_WRITE_TX(SPI1, (uint32_t)byteArray[i]);
		/* Check SPI1 busy status */
		while(SPI_IS_BUSY(SPI1));
		rb = SPI_READ_RX(SPI1);
		//printf("SPI:write 0x%x MISO:0x%x\n", (i != 2) ?byteArray[i] :write, rb);
	}
}

uint32_t CAP_Read(char * byteArray, size_t byteSize, char * byteRead)
{
	uint32_t i;
    uint32_t readlen = 0; 
	uint8_t rb;
	uint8_t read = 0x7f;
	// bool stop = false;

	//Register actual value read back from the 2nd MOSI 0x7f write
    //and thus the total read bytes = byteSize + 2
	for (i=0; i < (byteSize + 2); i++) {
		if (i >= 2)
		  SPI_WRITE_TX(SPI1, (uint32_t)read);
		else
		  SPI_WRITE_TX(SPI1, (uint32_t)byteArray[i]);

		/* Check SPI1 busy status */
		while(SPI_IS_BUSY(SPI1));
		rb = SPI_READ_RX(SPI1);
		//printf("SPI:read 0x%x MISO:0x%x\n", (i >= 2) ?read :byteArray[i], rb);

		// if (byteStop != 0) {
		// 	if (byteStop == i) {
		// 		stop = true;
		// 		break;
		// 	}
		// }

        if (i >= 3) {
          byteRead[readlen++] = rb;
        }

	}
	return readlen;
}

void CAP_Reset(void)
{
	uint8_t i;
	uint8_t rb;
	uint8_t reset = 0x7a;

	for (i=0; i<2; i++) {
	  SPI_WRITE_TX(SPI1, (uint32_t)reset);
	  /* Check SPI1 busy status */
	  while(SPI_IS_BUSY(SPI1));
	  rb = SPI_READ_RX(SPI1);
	  printf("SPI:reset MISO:0x%x\n", rb);
	}
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2MHz */

    /* 8-bit transaction */
	SPI_Open(SPI1, SPI_MASTER, SPI_MODE_3, 8, 2000000);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    //SPI_EnableAutoSS(SPI0, SPI_SS, SPI_SS_ACTIVE_LOW);

    SPI_EnableAutoSS(SPI1, SPI_SS, SPI_SS_ACTIVE_LOW);
}
