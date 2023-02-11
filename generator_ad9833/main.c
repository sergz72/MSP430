#include "board.h"
#include <generic_dds.h>
#include <ad9833.h>
#include <mcp41_42.h>
#include <spi_soft.h>

#define DDS_COMMAND_QUEUE_LENGTH 5

const dds_config config =
{
    .max_frequency = 12500000,
    .min_frequency = 1,
    .mclk_MHz = 25,
    .max_vout_mV = 4000,
    .max_attenuator_value = 160,
    .channels = 1,
    .accumulator_bits = 28,
    .out_square_divider_bits = 1,
    .supported_modes = (1 << DDS_MODE_SINE) | (1 << DDS_MODE_TRIANGLE) | (1 << DDS_MODE_SQUARE)
};

static dds_i2c_command command[DDS_COMMAND_QUEUE_LENGTH];
static volatile unsigned char *tx_data_p, *rx_data_p;
static volatile unsigned int rx_cnt, tx_cnt, current_command;

static void initGPIO(void)
{
    P1OUT  = BIT0|BIT3; // FSY
    P1DIR  = BIT0|BIT3|BIT4|BIT5;             // P1.0,3,4,5 output
    P1SEL  = BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
    P1SEL2 = BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
    P2OUT  = BIT0; // CS
    P2DIR  = BIT0;
}

static void initI2C(void)
{
    UCB0CTL1 |= UCSWRST;                      // Enable SW reset
    UCB0CTL0 = UCMODE_3 + UCSYNC;             // I2C Slave, synchronous mode
    UCB0I2COA = 1;                            // Own Address is 1
    UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
    IE2 |= UCB0RXIE | UCB0TXIE;               // Enable RX/TX interrupt
    UCB0I2CIE |= UCSTTIE | UCSTPIE;           // Enable STT interrupt
}

int main(void)
{
  unsigned short ad9833_cfg, ad9833_divider;
  dds_i2c_command *cmd;
  unsigned int command_to_process;

  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  initGPIO();
  initI2C();

  command_to_process = current_command = 0;

  ad9833_cfg = AD9833_CFG_SLEEP1|AD9833_CFG_SLEEP12;
  ad9833_set_config(0, ad9833_cfg);
  ad9833_divider = 1;

  while (1)
  {
      while (current_command != command_to_process)
      {
          cmd = &command[command_to_process];
          switch (cmd->c3.command)
          {
          case DDS_COMMAND_SET_FREQUENCY_CODE:
              ad9833_set_freq0_word(0, cmd->c8.freq);
              if (ad9833_cfg & AD9833_CFG_MSBDIV2)
              {
                  ad9833_divider = cmd->c8.div;
                  if (ad9833_divider == 1)
                  {
                      if ((ad9833_cfg & AD9833_CFG_MSB) == AD9833_CFG_MSBDIV2)
                          ad9833_update_mode(0, &ad9833_cfg, AD9833_CFG_MSB);
                  }
                  else if ((ad9833_cfg & AD9833_CFG_MSB) == AD9833_CFG_MSB)
                      ad9833_update_mode(0, &ad9833_cfg, AD9833_CFG_MSBDIV2);
              }
              break;
          case DDS_COMMAND_SET_MODE:
              switch (cmd->c3.parameter)
              {
              case DDS_MODE_SINE:
                ad9833_update_mode(0, &ad9833_cfg, AD9833_CFG_SINUS);
                break;
              case DDS_MODE_TRIANGLE:
                ad9833_update_mode(0, &ad9833_cfg, AD9833_CFG_TRIANGLE);
                break;
              case DDS_MODE_SQUARE:
                if (ad9833_divider == 1)
                    ad9833_update_mode(0, &ad9833_cfg, AD9833_CFG_MSB);
                else
                    ad9833_update_mode(0, &ad9833_cfg, AD9833_CFG_MSBDIV2);
                break;
              }
              break;
          case DDS_COMMAND_SET_ATTENUATOR:
              mcp41_42_set(0, MCP41_42_SET_DATA0, cmd->c4.parameter);
              break;
          case DDS_COMMAND_ENABLE_OUTPUT:
              if (cmd->c3.parameter)
                  ad9833_cfg &= ~(AD9833_CFG_SLEEP1|AD9833_CFG_SLEEP12);
              else
                  ad9833_cfg |= AD9833_CFG_SLEEP1|AD9833_CFG_SLEEP12;
              ad9833_set_config(0, ad9833_cfg);
              break;
          }
          command_to_process++;
          if (command_to_process >= DDS_COMMAND_QUEUE_LENGTH)
            command_to_process = 0;
      }
      __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
  }
}

// USCI_B0 Data ISR
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
    volatile unsigned int dummy;

    if (IFG2 & UCB0TXIFG) // TX
    {
        if (tx_cnt < sizeof(dds_config))
        {
            UCB0TXBUF = *tx_data_p++;
            tx_cnt++;
        }
        else
            UCB0TXBUF = 0;
    }
    else
    {
        if (rx_cnt < sizeof(dds_i2c_command))
        {
          *rx_data_p++ = UCB0RXBUF;
          rx_cnt++;
        }
        else
          dummy = UCB0RXBUF;
    }
}

// USCI_B0 State ISR
#pragma vector = USCIAB0RX_VECTOR
__interrupt void usci_i2c_state_isr(void)
{
    unsigned char stat = UCB0STAT;
    UCB0STAT &= ~(UCSTTIFG | UCSTPIFG);        // Clear start/stop condition int flag
    if (stat & UCSTTIFG) // start
    {
        P1OUT |= BIT0;
        rx_data_p = command[current_command].bytes;
        tx_data_p = (unsigned char*)&config;
        rx_cnt = tx_cnt = 0;
    }
    else // stop
    {
        P1OUT &= ~BIT0;
        if (rx_cnt)
        {
          current_command++;
          if (current_command >= DDS_COMMAND_QUEUE_LENGTH)
            current_command = 0;
        }
    }
    __bic_SR_register_on_exit(CPUOFF);        // Exit LPM0
}

void mcp41_42_set(int channel, unsigned char command, unsigned char data)
{
  spi_command(SPI_CHANNEL_MCP, command, &data, NULL, 1, 1);
}

void ad9833_write(int channel, unsigned short data)
{
  unsigned char cmd, d;
  cmd = data >> 8;
  d = data & 0xFF;
  spi_command(SPI_CHANNEL_AD, cmd, &d, NULL, 1, 1);
}

void SPI_CS_SET(int channel)
{
    if (channel == SPI_CHANNEL_AD)
    {
        // SET P1.3
        P1OUT |= BIT3;
    }
    else
    {
        //SET P2.0
        P2OUT |= BIT0;
    }
}

void SPI_CS_CLR(int channel)
{
  if (channel == SPI_CHANNEL_AD)
  {
      // CLR P1.3
      P1OUT &= ~BIT3;
  }
  else
  {
      //CLR P2.0
      P2OUT &= ~BIT0;
  }
}

void SPI_CLK_SET(int channel)
{
  if (channel == SPI_CHANNEL_AD)
  {
      //CLR P1.4
      P1OUT &= ~BIT4;
  }
  else
  {
      //SET P1.4
      P1OUT |= BIT4;
  }
}

void SPI_CLK_CLR(int channel)
{
  if (channel == SPI_CHANNEL_AD)
  {
      //SET P1.4
      P1OUT |= BIT4;
  }
  else
  {
      //CLR P1.4
      P1OUT &= ~BIT4;
  }
}

