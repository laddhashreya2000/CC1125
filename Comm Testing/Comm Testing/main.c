

#include "avr_compiler.h"
#include "spi_driver.h"
#include "CC112x_spi.h"
#include "cc112x_easy_link_reg_config.h"
/////////////////CC1125////////////
#define PKTLEN                  100  // 1 < PKTLEN < 126
uint16_t packetCounter = 0;
////////////////Closed Cc1125//////////


/*! \brief The number of test data bytes. */
#define NUM_BYTES     4

/* Global variables */

/*! \brief SPI master module on PORT D. */
SPI_Master_t spiMasterD;

/*! \brief SPI slave module on PORT D. */
//SPI_Slave_t spiSlaveD;

/*! \brief SPI Data packet */
SPI_DataPacket_t dataPacket;

/*! \brief Test data to send from master. */
uint8_t masterSendData[NUM_BYTES] = {0x11, 0x22, 0x33, 0x44};

/*! \brief Data received from slave. */
uint8_t masterReceivedData[NUM_BYTES];

/*! \brief Result of the test. */
bool success = true;

//////////cc1125

static void registerConfig(void);
static void manualCalibration(void);
static void runTX(void);
static void createPacket(uint8_t randBuffer[]);
static uint8_t  packetSemaphore;

/*! \brief Test function.
 *
 *  This function tests the SPI master and slave drivers in polled operation,
 *  with a master (on port C) communicating with a slave (on port D).
 *
 *  Hardware setup:
 *
 *    - Connect PC4 to PD4 (SS)
 *    - Connect PC5 to PD5 (MOSI)
 *    - Connect PC6 to PD6 (MISO)
 *    - Connect PC7 to PD7 (SCK)
 *
 *  The drivers are tested in two phases:
 *
 *  1: Data is transmitted on byte at a time from the master to the slave.
 *     The slave  received data and sends it back to computer via UART. 
 *  
 *
 *  The variable, 'success', will be non-zero when the function reaches the
 *  infinite for-loop if the test was successful.
 */
int main(void)
{
	
	/* setting the clock and checkin its status */
	CLK_CTRL=0x03;
	OSC_XOSCCTRL=0xDB;
	OSC_CTRL=0x08;
	while(!(OSC_STATUS & 0x08));
	
	
	
	/* Init SS pin as output with wired AND and pull-up. */
	PORTD.DIRSET = PIN4_bm|PIN5_bm;
	PORTD.PIN4CTRL = PORT_OPC_WIREDANDPULL_gc;
	PORTD.DIRCLR = PIN6_bm;
	PORTD.PIN6CTRL = PORT_OPC_WIREDANDPULL_gc;
	/* Set SS output to high. (No slave addressed). */
	PORTD.OUTSET = PIN4_bm;
	PORTF.DIRSET=0xFF;
	PORTB.DIRSET=0x01;
	PORTB.PIN0CTRL = PORT_OPC_WIREDANDPULL_gc;
	/* Instantiate pointer to ssPort. */
	PORT_t *ssPort = &PORTD;

	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm|PMIC_MEDLVLEN_bm;

	/* Enable global interrupts. */
	sei();
	/* Initialize SPI master on port D. */
	SPI_MasterInit(&spiMasterD,
	               &SPID,
	               &PORTD,
	               false,
	               SPI_MODE_0_gc,
	               SPI_INTLVL_OFF_gc,
	               false,
	               SPI_PRESCALER_DIV4_gc);

//////////////////////////////////spi closed///////////////////////////////////
 
int *data=0;
////////////////////////////////cc1125////////////////////////////

registerConfig(); //resets cc registers and writes the data into it
cc112xSpiReadReg(CC112X_FREQ2, data, 1);
_delay_ms(100);
runTX();
}



/////////////////////////////////////CC1125/////////////////////////////////////

/*******************************************************************************
*   @fn         runTX
*
*   @brief      Continuously sends packets on button push until button is pushed
*               again. After the radio has gone into TX the function waits for
*               interrupt that packet has been sent. Updates packet counter and
*               display for each packet sent.
*
*   @param      none
*
*   @return    none
*/
static void runTX(void) {

	//// Initialize packet buffer of size PKTLEN + 1
	uint8_t txBuffer[PKTLEN+1] = {0};

	packetSemaphore=0x11;
	// Calibrate radio according to errata
	manualCalibration();

	// Infinite loop
	while(1) {
		// Update packet counter
		packetCounter++;

		// Create a random packet with PKTLEN + 2 byte packet
		// counter + n x random bytes
		createPacket(txBuffer);

		// Write packet to TX FIFO
		cc112xSpiWriteTxFifo(txBuffer, sizeof(txBuffer));
		trxSpiCmdStrobe(CC112X_STX);
				
		int rec=0xFF;
		while(rec!=0x0F) {
			rec = trxSpiCmdStrobe(CC112X_SNOP); //reads status byte
		}
	}
}


/*******************************************************************************
*   @fn         registerConfig
*
*   @brief      Write register settings as given by SmartRF Studio found in
*               cc112x_easy_link_reg_config.h
*
*   @param      none
*
*   @return     none
*/
static void registerConfig(void) {

	uint8_t writeByte;

	// Reset radio
	trxSpiCmdStrobe(CC112X_SRES);
	//PORTD.OUT=0xFF;
	// Write registers to radio
	for(uint16_t i = 0;
	i < (sizeof(preferredSettings)/sizeof(registerSetting_t)); i++) {
		writeByte = preferredSettings[i].data;
		cc112xSpiWriteReg(preferredSettings[i].addr, &writeByte, 1);
	}
	//uint8_t FSK=0x56;
	//cc112xSpiWriteReg(CC112X_FREQ2, &FSK, 1);
}


/*******************************************************************************
*   @fn         createPacket
*
*   @brief      This function is called before a packet is transmitted. It fills
*               the txBuffer with a packet consisting of a length byte, two
*               bytes packet counter and n random bytes.
*
*               The packet format is as follows:
*               |--------------------------------------------------------------|
*               |           |           |           |         |       |        |
*               | pktLength | pktCount1 | pktCount0 | rndData |.......| rndData|
*               |           |           |           |         |       |        |
*               |--------------------------------------------------------------|
*                txBuffer[0] txBuffer[1] txBuffer[2]            txBuffer[PKTLEN]
*
*   @param       Pointer to start of txBuffer
*
*   @return      none
*/
static void createPacket(uint8_t txBuffer[]) {

	txBuffer[0] = PKTLEN;                           // Length byte
	txBuffer[1] = (uint8_t) (packetCounter >> 8);     // MSB of packetCounter
	txBuffer[2] = (uint8_t)  packetCounter;           // LSB of packetCounter

	// Fill rest of buffer with random bytes
	for(uint8_t i = 3; i < (PKTLEN + 1); i++) {
		txBuffer[i] = 'A';
	}
}


/*******************************************************************************
*   @fn         manualCalibration
*
*   @brief      Calibrates radio according to CC112x errata
*
*   @param      none
*
*   @return     none
*/
#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2
static void manualCalibration(void) {

	uint8_t original_fs_cal2;
	uint8_t calResults_for_vcdac_start_high[3];
	uint8_t calResults_for_vcdac_start_mid[3];
	uint8_t marcstate;
	uint8_t writeByte;

	// 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
	writeByte = 0x00;
	cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

	// 2) Start with high VCDAC (original VCDAC_START + 2):
	cc112xSpiReadReg(CC112X_FS_CAL2, &original_fs_cal2, 1);
	writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
	cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

	// 3) Calibrate and wait for calibration to be done
	//   (radio back in IDLE state)
	trxSpiCmdStrobe(CC112X_SCAL);

	do {
		cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
		PORTF.OUT=marcstate;
	} while (marcstate != 0x41);

	// 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with
	//    high VCDAC_START value
	cc112xSpiReadReg(CC112X_FS_VCO2,
	&calResults_for_vcdac_start_high[FS_VCO2_INDEX], 1);
	cc112xSpiReadReg(CC112X_FS_VCO4,
	&calResults_for_vcdac_start_high[FS_VCO4_INDEX], 1);
	cc112xSpiReadReg(CC112X_FS_CHP,
	&calResults_for_vcdac_start_high[FS_CHP_INDEX], 1);

	// 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
	writeByte = 0x00;
	cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

	// 6) Continue with mid VCDAC (original VCDAC_START):
	writeByte = original_fs_cal2;
	cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

	// 7) Calibrate and wait for calibration to be done
	//   (radio back in IDLE state)
	trxSpiCmdStrobe(CC112X_SCAL);

	do {
		cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
	} while (marcstate != 0x41);

	// 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained
	//    with mid VCDAC_START value
	cc112xSpiReadReg(CC112X_FS_VCO2,
	&calResults_for_vcdac_start_mid[FS_VCO2_INDEX], 1);
	cc112xSpiReadReg(CC112X_FS_VCO4,
	&calResults_for_vcdac_start_mid[FS_VCO4_INDEX], 1);
	cc112xSpiReadReg(CC112X_FS_CHP,
	&calResults_for_vcdac_start_mid[FS_CHP_INDEX], 1);

	// 9) Write back highest FS_VCO2 and corresponding FS_VCO
	//    and FS_CHP result
	if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] >
	calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
		writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
		cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
		writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
		cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
		writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
		cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
		} else {
		writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
		cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
		writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
		cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
		writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
		cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
	}
}

/////////////////////////////////////////closed CC1125/////////////////////////////////