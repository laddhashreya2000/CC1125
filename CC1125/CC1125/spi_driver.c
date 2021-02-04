/*
 * spi_driver.c
 *
 * Created: 12-05-2018 23:01:56
 *  Author: PRASHANT KURREY
 */ 
/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      XMEGA SPI driver source file.
 *
 *      This file contains the function implementations the XMEGA SPI driver.
 *
 *      The driver is not intended for size and/or speed critical code, since
 *      most functions are just a few lines of code, and the function call
 *      overhead would decrease code performance. The driver is intended for
 *      rapid prototyping and documentation purposes for getting started with
 *      the XMEGA SPI module.
 *
 *      For size and/or speed critical code, it is recommended to copy the
 *      function contents directly into your application instead of making
 *      a function call.
 *
 *      Several functions use the following construct:
 *          "some_register = ... | (some_parameter ? SOME_BIT_bm : 0) | ..."
 *      Although the use of the ternary operator ( if ? then : else ) is discouraged,
 *      in some occasions the operator makes it possible to write pretty clean and
 *      neat code. In this driver, the construct is used to set or not set a
 *      configuration bit based on a boolean input parameter, such as
 *      the "some_parameter" in the example above.
 *
 * \par Application note:
 *      AVR1309: Using the XMEGA SPI
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Revision: 5407 $
 * $Date: 2011-10-12 14:53:14 +0200 (on, 12 okt 2011) $  \n
 *
 * Copyright (c) 2009 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.

 * 4. This software may only be redistributed and used in connection with an
 * Atmel AVR product.

 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include "spi_driver.h"
#include "hal_spi_rf_trxeb.h"

////for cc1125
static void trxReadWriteBurstSingle(uint8_t addr,uint8_t *pData,uint16_t len);
/////closed cc1125


/*! \brief Initialize SPI module as master.
 *
 *  This function initializes a SPI module as master. The CTRL and INTCTRL
 *  registers for the SPI module is set according to the inputs to the function.
 *  In addition, data direction for the MOSI and SCK pins is set to output.
 *
 *  \param spi            The SPI_Master_t struct instance.
 *  \param module         The SPI module.
 *  \param port           The I/O port where the SPI module is connected.
 *  \param lsbFirst       Data order will be LSB first if this is set to a
 *                        non-zero value.
 *  \param mode           SPI mode (Clock polarity and phase).
 *  \param intLevel       SPI interrupt level.
 *  \param clk2x	      SPI double speed mode
 *  \param clockDivision  SPI clock prescaler divison factor.
 */
void SPI_MasterInit(SPI_Master_t *spi,
                    SPI_t *module,
                    PORT_t *port,
                    bool lsbFirst,
                    SPI_MODE_t mode,
                    SPI_INTLVL_t intLevel,
                    bool clk2x,
                    SPI_PRESCALER_t clockDivision)
{
	spi->module         = module;
	spi->port           = port;
	spi->interrupted    = false;

	spi->module->CTRL   = clockDivision |                  /* SPI prescaler. */
	                      (clk2x ? SPI_CLK2X_bm : 0) |     /* SPI Clock double. */
	                      SPI_ENABLE_bm |                  /* Enable SPI module. */
	                      (lsbFirst ? SPI_DORD_bm  : 0) |  /* Data order. */
	                      SPI_MASTER_bm |                  /* SPI master. */
	                      mode;                            /* SPI mode. */

	/* Interrupt level. */
	spi->module->INTCTRL = intLevel;

	/* No assigned data packet. */
	spi->dataPacket = NULL;

 	/* MOSI and SCK as output. */
	spi->port->DIRSET  = SPI_MOSI_bm | SPI_SCK_bm;
}



/*! \brief Initialize SPI module as slave.
 *
 *  This function initializes a SPI module as slave. The CTRL and INTCTRL
 *  registers for the SPI module is set according to the inputs to the function.
 *  In addition, data direction for the MISO pin is set to output.
 *
 *  \param spi                  The SPI_Slave_t instance.
 *  \param module               Pointer to the SPI module.
 *  \param port                 The I/O port where the SPI module is connected.
 *  \param lsbFirst             Data order will be LSB first if this is set to true.
 *  \param mode                 SPI mode (Clock polarity and phase).
 *  \param intLevel             SPI interrupt level.
 */
void SPI_SlaveInit(SPI_Slave_t *spi,
                   SPI_t *module,
                   PORT_t *port,
                   bool lsbFirst,
                   SPI_MODE_t mode,
                   SPI_INTLVL_t intLevel)
{
	/* SPI module. */
	spi->module       = module;
	spi->port         = port;

	spi->module->CTRL = SPI_ENABLE_bm |                /* Enable SPI module. */
	                    (lsbFirst ? SPI_DORD_bm : 0) | /* Data order. */
	                    mode;                          /* SPI mode. */

	/* Interrupt level. */
	spi->module->INTCTRL = intLevel;

	/* MISO as output. */
	spi->port->DIRSET = SPI_MISO_bm;
}





/*! \brief SPI mastertransceive byte
 *
 *  This function clocks data in the DATA register to the slave, while data
 *  from the slave is clocked into the DATA register. The function does not
 *  check for ongoing access from other masters before initiating a transfer.
 *  For multimaster systems, checkers should be added to avoid bus contention.
 *
 *  SS line(s) must be pulled low before calling this function and released
 *  when finished.
 *
 *  \note This function is blocking and will not finish unless a successful
 *        transfer has been completed. It is recommended to use the
 *        interrupt-driven driver for applications where blocking
 *        functionality is not wanted.
 *
 *  \param spi        The SPI_Master_t struct instance.
 *  \param TXdata     Data to transmit to slave.
 *
 *  \return           Data received from slave.
 */
uint8_t SPI_MasterTransceiveByte(SPI_Master_t *spi, uint8_t TXdata)
{
	/* Send pattern. */
	spi->module->DATA = TXdata;
	

	/* Wait for transmission complete. */
	while(!(spi->module->STATUS & SPI_IF_bm)) {

	}
	/* Read received data. */
	uint8_t result = spi->module->DATA;

	return(result);
}

uint8_t SPI_Master_TXRX(uint8_t TXdata)
{
	SPID.DATA=TXdata;
	while(!(SPID.STATUS& SPI_IF_bm));
	uint8_t result=SPID.DATA;
	return(result);
	
}

////////////////////////////FOR CC1125///////////////////////////////////////////////////////////
////////////////////////////FOR CC1125//////////////////////////////////////////////////////////
/*******************************************************************************
 * @fn          trx8BitRegAccess
 *
 * @brief       This function performs a read or write from/to a 8bit register
 *              address space. The function handles burst and single read/write
 *              as specfied in addrByte. Function assumes that chip is ready.
 *
 * input parameters
 *
 * @param       accessType - Specifies if this is a read or write and if it's
 *                           a single or burst access. Bitmask made up of
 *                           RADIO_BURST_ACCESS/RADIO_SINGLE_ACCESS/
 *                           RADIO_WRITE_ACCESS/RADIO_READ_ACCESS.
 * @param       addrByte - address byte of register.
 * @param       pData    - data array
 * @param       len      - Length of array to be read(TX)/written(RX)
 *
 * output parameters
 *
 * @return      chip status
 */
uint8_t trx8BitRegAccess(uint8_t accessType, uint8_t addrByte, uint8_t *pData, uint16_t len)
{
  uint8_t readValue;

  /* Pull CS_N low and wait for SO to go low before communication starts */
  //SPI_MasterInit(&spiMasterD,&SPID,
  //&PORTD,
  //false,
  //SPI_MODE_0_gc,
  //SPI_INTLVL_OFF_gc,
  //false,
  //SPI_PRESCALER_DIV4_gc);
  SPI_MasterSSLow(  &PORTD, PIN4_bm);
  PORTF.OUT=0x07;
  while((PORTD.IN & 0x40)==1);
  /* send register address byte */
 // TRXEM_SPI_TX(accessType|addrByte);
 // TRXEM_SPI_WAIT_DONE();
  /* Storing chip status */
  //readValue = TRXEM_SPI_RX();
  //readValue=SPI_MasterTransceiveByte(&spiMasterD, accessType|addrByte);
  readValue=SPI_Master_TXRX(accessType|addrByte);
  trxReadWriteBurstSingle(accessType|addrByte,pData,len);
  SPI_MasterSSHigh(  &PORTD, PIN4_bm);
  /* return the status byte value */
  return(readValue);
}

/******************************************************************************
 * @fn          trx16BitRegAccess
 *
 * @brief       This function performs a read or write in the extended adress
 *              space of CC112X.
 *
 * input parameters
 *
 * @param       accessType - Specifies if this is a read or write and if it's
 *                           a single or burst access. Bitmask made up of
 *                           RADIO_BURST_ACCESS/RADIO_SINGLE_ACCESS/
 *                           RADIO_WRITE_ACCESS/RADIO_READ_ACCESS.
 * @param       extAddr - Extended register space address = 0x2F.
 * @param       regAddr - Register address in the extended address space.
 * @param       *pData  - Pointer to data array for communication
 * @param       len     - Length of bytes to be read/written from/to radio
 *
 * output parameters
 *
 * @return      rfStatus_t
 */
uint8_t trx16BitRegAccess(uint8_t accessType, uint8_t extAddr, uint8_t regAddr, uint8_t *pData, uint8_t len)
{
  uint8_t readValue;

    SPI_MasterSSLow(  &PORTD, PIN4_bm);
	PORTF.OUT=0x08;
    while((PORTD.IN & 0x40)==1);
  /* send extended address byte with access type bits set */
 // TRXEM_SPI_TX(accessType|extAddr);
  //TRXEM_SPI_WAIT_DONE();
  /* Storing chip status */
  //readValue = TRXEM_SPI_RX();
  //replace with xmega spi
  //readValue=SPI_MasterTransceiveByte(&spiMasterD, accessType|extAddr);
  readValue=SPI_Master_TXRX(accessType|extAddr);
  //TRXEM_SPI_TX(regAddr);
 // TRXEM_SPI_WAIT_DONE();
 //above replaced with spi 
 //SPI_MasterTransceiveByte(&spiMasterD,regAddr);
	SPI_Master_TXRX(regAddr);
  /* Communicate len number of bytes */
  trxReadWriteBurstSingle(accessType|extAddr,pData,len);
SPI_MasterSSHigh(  &PORTD, PIN4_bm);
  /* return the status byte value */
  return(readValue);
}

/*******************************************************************************
 * @fn          trxSpiCmdStrobe
 *
 * @brief       Send command strobe to the radio. Returns status byte read
 *              during transfer of command strobe. Validation of provided
 *              is not done. Function assumes chip is ready.
 *
 * input parameters
 *
 * @param       cmd - command strobe
 *
 * output parameters
 *
 * @return      status byte
 */
uint8_t trxSpiCmdStrobe(uint8_t cmd)
{
    uint8_t rc;
   // TRXEM_SPI_BEGIN();
    //while(TRXEM_PORT_IN & TRXEM_SPI_MISO_PIN);
	//above is replaced with xmegaspi
	  SPI_MasterSSLow( &PORTD, PIN4_bm);
	  //PORTF.OUT=0x09;
	  while((PORTD.IN & 0x40)==1){PORTF.OUT=0xff;_delay_ms(1000); PORTF.OUT=PORTD.IN;_delay_ms(1000);};
	PORTF.OUT=0x55;
	//_delay_ms(1000);
	//TRXEM_SPI_TX(cmd);
	//TRXEM_SPI_WAIT_DONE();
	//rc =SPI_MasterTransceiveByte(&spiMasterD, cmd);
   // rc = TRXEM_SPI_RX();
  rc=SPI_Master_TXRX(cmd);
    //TRXEM_SPI_END();
	SPI_MasterSSHigh(&PORTD, PIN4_bm);
    return(rc);
}

/*******************************************************************************
 * @fn          trxReadWriteBurstSingle
 *
 * @brief       When the address byte is sent to the SPI slave, the next byte
 *              communicated is the data to be written or read. The address
 *              byte that holds information about read/write -and single/
 *              burst-access is provided to this function.
 *
 *              Depending on these two bits this function will write len bytes to
 *              the radio in burst mode or read len bytes from the radio in burst
 *              mode if the burst bit is set. If the burst bit is not set, only
 *              one data byte is communicated.
 *
 *              NOTE: This function is used in the following way:
 *
 *              TRXEM_SPI_BEGIN();
 *              while(TRXEM_PORT_IN & TRXEM_SPI_MISO_PIN);
 *              ...[Depending on type of register access]
 *              trxReadWriteBurstSingle(uint8 addr,uint8 *pData,uint16 len);
 *              TRXEM_SPI_END();
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * @return      void
 */
static void trxReadWriteBurstSingle(uint8_t addr,uint8_t *pData,uint16_t len)
{
	uint16_t i;
	/* Communicate len number of bytes: if RX - the procedure sends 0x00 to push bytes from slave*/
  if(addr&RADIO_READ_ACCESS)
  {
    if(addr&RADIO_BURST_ACCESS)
    {
      for (i = 0; i < len; i++)
      {
          //TRXEM_SPI_TX(0);            /* Possible to combining read and write as one access type */
          //TRXEM_SPI_WAIT_DONE();
          //*pData = TRXEM_SPI_RX();     /* Store pData from last pData RX */
		 // *pData=SPI_MasterTransceiveByte(&spiMasterD,0);
		 *pData= SPI_Master_TXRX(0);
          pData++;
      }
    }
    else
    {
      //TRXEM_SPI_TX(0);
      //TRXEM_SPI_WAIT_DONE();
      //*pData = SPI_MasterTransceiveByte(&spiMasterD,0);
	  *pData= SPI_Master_TXRX(0);
    }
  }
  else
  {
    if(addr&RADIO_BURST_ACCESS)
    {
      /* Communicate len number of bytes: if TX - the procedure doesn't overwrite pData */
      for (i = 0; i < len; i++)
      {
      //  TRXEM_SPI_TX(*pData);
        //TRXEM_SPI_WAIT_DONE();
		//replaced with spi of xmega
		//SPI_MasterTransceiveByte(&spiMasterD,*pData);
		 SPI_Master_TXRX(*pData);
        pData++;
      }
    }
    else
    {
     // TRXEM_SPI_TX(*pData);
      //TRXEM_SPI_WAIT_DONE();
	  //SPI_MasterTransceiveByte(&spiMasterD,*pData);
	   SPI_Master_TXRX(*pData);
    }
  }
  return;
}

