

#include "avr_compiler.h"
#include "spi_driver.h"
#include "USART.h"
#include "CC112x_spi.h"
#include "cc112x_easy_link_reg_config.h"
#include "port_driver.h"
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

/*! Define that selects the Usart used in example. */
#define USART USARTC0

/*! USART data struct used in example. */
USART_data_t USART_data;


//////////cc1125

static void registerConfig(void);
static void manualCalibration(void);
static void runTX(void);
static void createPacket(uint8_t randBuffer[]);
static uint8_t  packetSemaphore;
///////////////cc1125

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

/* This PORT setting is only valid to USARTC0 if other USARTs is used a
	 * different PORT and/or pins are used. */
  	/* PC3 (TXD0) as output. */
	PORTC.DIRSET   = PIN3_bm;
	/* PC2 (RXD0) as input. */
	PORTC.DIRCLR   = PIN2_bm;

	/* Use USARTC0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USART_data, &USART, USART_DREINTLVL_LO_gc);

	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(USART_data.usart, USART_CHSIZE_8BIT_gc,
                     USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USART_data.usart, USART_RXCINTLVL_LO_gc);

	/* Set Baudrate to 9600 bps:
	 * Use the default I/O clock frequency that is 2 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
	USART_Baudrate_Set(&USART, 12 , 0);

	/* Enable both RX and TX. */
	USART_Rx_Enable(USART_data.usart);
	USART_Tx_Enable(USART_data.usart);

//inter for port c
/*	PORT_ConfigurePins( &PORTC,
	false,
	false,
	PORT_OPC_TOTEM_gc,
	PORT_ISC_FALLING_gc );
	
	PORT_ConfigureInterrupt0( &PORTC, PORT_INT0LVL_MED_gc, 0x01 );
	PORTC_INTFLAGS=0x01;
	PORTC.PIN0CTRL |= PORT_OPC_WIREDANDPULL_gc;
//////////////////////////END


	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm|PMIC_MEDLVLEN_bm;

	/* Enable global interrupts. */
	sei();
	/* Initialize SPI master on port C. */
	SPI_MasterInit(&spiMasterD,
	               &SPID,
	               &PORTD,
	               false,
	               SPI_MODE_0_gc,
	               SPI_INTLVL_OFF_gc,
	               false,
	               SPI_PRESCALER_DIV4_gc);

//////////////////////////////////////SPI for xmega/////////////////////////

	/* for testing */
	PORTF.OUT=0xF0;
	_delay_ms(1000);
	PORTF.OUT=0x0F;
	_delay_ms(1000);
	PORTB.OUTCLR=0x01;
	_delay_ms(10);
	PORTB.OUTSET=0x01;
	_delay_ms(10);
	/* PHASE 1: Transceive individual bytes. */

	/* MASTER: Pull SS line low. This has to be done since
	 *         SPI_MasterTransceiveByte() does not control the SS line(s). */
	//SPI_MasterSSLow(ssPort, PIN4_bm);
//
	//for(uint8_t i = 0; i < NUM_BYTES; i++) {
		///* MASTER: Transmit data from master to slave. */
		//SPI_MasterTransceiveByte(&spiMasterD, masterSendData[i]);
//
	//}
		///* MASTER: Release SS to slave. */
	////	SPI_MasterSSHigh(ssPort, PIN4_bm);           //To stop the transmission
//
	//
//
	//while(true) {
		//for(uint8_t i = 0; i < NUM_BYTES; i++) {
			///* MASTER: Transmit data from master to slave. */
			//SPI_MasterTransceiveByte(&spiMasterD, masterSendData[i]);
			//_delay_ms(100);
//}


//////////////////////////////////spi closed///////////////////////////////////
 
 int *data=0;
////////////////////////////////cc1125////////////////////////////

registerConfig(); //resets cc registers and writes the data into it
cc112xSpiReadReg(CC112X_FREQ2, data, 1);
PORTF.OUT=*data; //outsets the led (for testing)

UART_TXBuffer_PutByte(&USART_data, *data); //testing
_delay_ms(100);
 // Enter runTX, never coming back
runTX();
/*  int *data=0;
 while(1){ 
 //int write =0x56;
 
//cc112xSpiWriteReg(CC112X_FREQ2, &write, 1);
cc112xSpiReadReg(CC112X_FREQ0, data, 1);
PORTF.OUT=*data;
UART_TXBuffer_PutByte(&USART_data, *data);
_delay_ms(100);
 }*/
}

ISR(USARTC0_RXC_vect)
{ int receive=0;
	USART_RXComplete(&USART_data); //stores received data in rx buffer
	if (USART_RXBufferData_Available(&USART_data)) {       // check if data is available
		receive = USART_RXBuffer_GetByte(&USART_data);}    // receive the data
	UART_TXBuffer_PutByte(&USART_data, receive);       // send data 
	
}

ISR(PORTC_INT0_vect)
{	
	packetSemaphore=0x11;
	PORTC_INTFLAGS=0x01;
	PORTF.OUT=0x0F;
	UART_TXBuffer_PutByte(&USART_data, 'B');
	
}

/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTC0_DRE_vect)
{
	USART_DataRegEmpty(&USART_data);
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
//

	//// Connect ISR function to GPIO2
	//ioPinIntRegister(IO_PIN_PORT_1, GPIO2, &radioTxISR);
//
	//// Interrupt on falling edge
	//ioPinIntTypeSet(IO_PIN_PORT_1, GPIO2, IO_PIN_FALLING_EDGE);
//
	//// Clear ISR flag
	//ioPinIntClear(IO_PIN_PORT_1, GPIO2);
//
	//// Enable interrupt
	//ioPinIntEnable(IO_PIN_PORT_1, GPIO2);
//
	//// Update LCD
	//updateLcd();
	packetSemaphore=0x11;
	// Calibrate radio according to errata
	manualCalibration();

	// Infinite loop
	while(1) {

		// Wait for button push
		//if(bspKeyPushed(BSP_KEY_ALL)) {

			// Continiously sent packets until button is pressed
		//	do {

				// Update packet counter
				packetCounter++;

				// Create a random packet with PKTLEN + 2 byte packet
				// counter + n x random bytes
				createPacket(txBuffer);

				// Write packet to TX FIFO
				cc112xSpiWriteTxFifo(txBuffer, sizeof(txBuffer));
				
				
				// Strobe TX to send packet
				/*int *data=0;
				cc112xSpiReadReg(CC112X_NUM_TXBYTES, data, 1);
				UART_TXBuffer_PutByte(&USART_data, *data);
				UART_TXBuffer_PutByte(&USART_data, 'A');*/
				
				
				trxSpiCmdStrobe(CC112X_STX);
			
				
				//uint8_t status =cc112xGetTxStatus();
				//while((status&0x70)==0);
				/*while(status==0x31){
					status=cc112xGetTxStatus();
					PORTF.OUT=status;}*/
				//while(packetSemaphore != 0x11)
				int rec=0xFF;
				
				while(rec!=0x0F){
				rec = trxSpiCmdStrobe(CC112X_SNOP); //reads status byte
					UART_TXBuffer_PutByte(&USART_data, rec); //writes it for testing
				}
			
				/*PORTF.OUT=0x01;
				_delay_ms(100);
				UART_TXBuffer_PutByte(&USART_data, 'A');
				PORTF.OUT=0x80;
				_delay_ms(100);*/
				// Wait for interrupt that packet has been sent.
				// (Assumes the GPIO connected to the radioRxTxISR function is
				// set to GPIOx_CFG = 0x06)
				//while(packetSemaphore != ISR_ACTION_REQUIRED);

				// Clear semaphore flag
				//packetSemaphore = ISR_IDLE;

				// Update LCD
			//	updateLcd();
				
				//Wait for a random amount of time between each packet
				//waitMs(3*(rand()%10+3));
				
			//} while (!bspKeyPushed(BSP_KEY_ALL));
		//}
	}
}


/*******************************************************************************
*   @fn         radioTxISR
*
*   @brief      ISR for packet handling in TX. Sets packet semaphore
*               and clears ISR flag
*
*   @param      none
*
*   @return     none
*/
//static void radioTxISR(void) {
//
	//// Set packet semaphore
	//packetSemaphore = ISR_ACTION_REQUIRED;
//
	//// Clear ISR flag
	//ioPinIntClear(IO_PIN_PORT_1, GPIO2);
//}


/*******************************************************************************
*   @fn         initMCU
*
*   @brief      Initialize MCU and board peripherals
*
*   @param      none
*
*   @return     none
*/
//static void initMCU(void) {
//
	//// Init clocks and I/O
	//bspInit(BSP_SYS_CLK_8MHZ);
//
	//// Init LEDs
	//bspLedInit();
//
	//// Init buttons
	//bspKeyInit(BSP_KEY_MODE_POLL);
//
	//// Initialize SPI interface to LCD (shared with SPI flash)
	//bspIoSpiInit(BSP_FLASH_LCD_SPI, BSP_FLASH_LCD_SPI_SPD);
//
	//// Init LCD
	//lcdInit();
//
	//// Instantiate transceiver RF SPI interface to SCLK ~ 4 MHz
	//// Input parameter is clockDivider
	//// SCLK frequency = SMCLK/clockDivider
	//trxRfSpiInterfaceInit(2);
//
	//// Enable global interrupt
	//_BIS_SR(GIE);
//}


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
*   @fn         updateLcd
*
*   @brief      Updates LCD buffer and sends buffer to LCD module
*
*   @param      none
*
*   @return     none
*/
//static void updateLcd(void) {
//
	//// Update LDC buffer and send to screen.
	//lcdBufferClear(0);
	//lcdBufferPrintString(0, "EasyLink Test", 0, eLcdPage0);
	//lcdBufferSetHLine(0, 0, LCD_COLS-1, 7);
	//lcdBufferPrintString(0, "Sent packets:", 0, eLcdPage3);
	//lcdBufferPrintInt(0, packetCounter, 70, eLcdPage4);
	//lcdBufferPrintString(0, "Packet TX" , 0, eLcdPage7);
	//lcdBufferSetHLine(0, 0, LCD_COLS-1, 55);
	//lcdBufferInvertPage(0, 0, LCD_COLS, eLcdPage7);
	//lcdSendBuffer(0);
//}

/*******************************************************************************
*   @fn         waitMs
*
*   @brief      Busy wait function. Waits the specified number of milliseconds.
*               Use assumptions about number of clock cycles needed for the
*               various instructions.
*
*               NB! This function is highly dependent on architecture and
*               compiler!
*
*   @param      uint16 mSec - number of milliseconds delay
*
*   @return     none
*/
//#pragma optimize=none
//static void waitMs(uint16 mSec) {
	//while(mSec-- > 0) {
		//waitUs(1000);
	//}
//}

/*******************************************************************************
*   @fn         waitUs
*
*   @brief      Busy wait function. Waits the specified number of microseconds.
*               Use assumptions about number of clock cycles needed for the
*               various instructions. The duration of one cycle depends on MCLK.
*               In this HAL it is set to 8 MHz, thus 8 cycles per us.
*
*               NB! This function is highly dependent on architecture
*               and compiler!
*
*   @param      uint16 uSec - number of microseconds delay
*
*   @return     none
*/
//#pragma optimize=none
//static void waitUs(uint16 uSec) { // 5 cycles for calling
//
	//// The least we can wait is 3 usec:
	//// ~1 usec for call, 1 for first compare and 1 for return
	//while(uSec > 3) {  // 2 cycles for compare
		//// 2 cycles for jump
		//NOP();         // 1 cycle for nop
		//NOP();         // 1 cycle for nop
		//NOP();         // 1 cycle for nop
		//NOP();         // 1 cycle for nop
		//NOP();         // 1 cycle for nop
		//NOP();         // 1 cycle for nop
		//NOP();         // 1 cycle for nop
		//NOP();         // 1 cycle for nop
		//uSec -= 2;     // 1 cycle for optimized decrement
	//}
//}                      // 4 cycles for returning

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