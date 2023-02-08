#ifndef _BOARD_H
#define _BOARD_H

#include <msp430.h>

#define NULL 0

#define SPI_DELAY(x) 1
#define SPI_CHECK_MISO(x) 0
//SET P1.5
#define SPI_MOSI_SET(x) P1OUT |= BIT5
//CLR P1.5
#define SPI_MOSI_CLR(x) P1OUT &= ~BIT5

#define SPI_CHANNEL_AD  0
#define SPI_CHANNEL_MCP 1

void SPI_CS_SET(int channel);
void SPI_CS_CLR(int channel);
void SPI_CLK_SET(int channel);
void SPI_CLK_CLR(int channel);

#endif
