#ifndef __CAP_SPI__
#define __CAP_SPI__

void CAP_Reset(void);
void CAP_Write(uint8_t *, size_t);
uint32_t CAP_Read(char *, size_t, char *);
// void CAP_Report(void);

void CAP_Initialization(void);
void CAP_Interrupt(void);

void SPI_Init(void);

void delay_us(int usec);

#endif