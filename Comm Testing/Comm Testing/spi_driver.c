#include "cc112x_easy_link_reg_config.h"
#include "CC112x_spi.h"
#include "CC112x_spi.c"

void spi_master_init(void){
  DDRB = 0b00000111;
  SPCR = 0b01010000;
  SPCR |= (1<<SPIE);
  PORTB &= ~(1<<PB0);
}

void spi_master_transmit(uint8_t data){
  SPDR = data;
  while(!(SPSR & (1<<SPIF)));
}

uint8_t spi_master_receive(void){
  while(!(SPSR & (1<<SPIF)));
  return  SPDR;
}

  void spi_slave_init(void){
  DDRB |= (1<<DDB3);
  SPCR |= (1<<SPE);
}

void spi_slave_transmit(uint8_t data){
  SPDR = data;
  while(!(SPSR & (1<<SPIF)));
}

uint8_t spi_slave_recieve(void){
  while(!(SPSR & (1<<SPIF)));
  return SPDR;
}

static void registerConfig(void) {

  uint8_t writeByte;

  // Reset radio
  trxSpiCmdStrobe(CC112X_SRES);

  for(uint16_t i = 0;
  i < (sizeof(preferredSettings)/sizeof(registerSetting_t)); i++) {
    writeByte = preferredSettings[i].data;
    cc112xSpiWriteReg(preferredSettings[i].addr, &writeByte, 1);
  }
}

uint8_t trx8BitRegAccess(uint8_t accessType, uint8_t addrByte, uint8_t *pData, uint16_t len)
{
  uint8_t readValue;
  PORTB &= ~(1<<PB0); //master ss low

  readValue = SPI_Master_TXRX(accessType|addrByte);

  trxReadWriteBurstSingle(accessType|addrByte,pData,len);

  SPI_MasterSSHigh(  &PORTD, PIN4_bm);
  PORTB |= (1<<PB0); //master ss high
  /* return the status byte value */
  return(readValue);
}

uint8_t trxSpiCmdStrobe(uint8_t cmd)
{
  spi_master_transmit(cmd);
}


