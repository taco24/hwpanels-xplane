/* 
 Target device: AVR ATmega8, running at 16MHz.
 */

#include <avr/io.h>  
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <compat/twi.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include "General.h"
#include "Delay.h"

#define TWCR_ACK TWCR = (1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC)|(1<<TWEN)|(1<<TWIE);
#define TWRESET TWCR  = (0<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(0<<TWWC)|(0<<TWEN)|(0<<TWIE);

#define TWI_GEN_BIT   0
#define TWI_ADR_BITS  1

// Rotary Encoder Pins
#define PHASE_A		(PINB & 1<<3)
#define PHASE_B		(PINB & 1<<4)
#define RE_BTN          (PINB & 1<<5)
#define STANDBY_BTN     (PINB & 1<<0)
//#define TEST_BTN        (PINB & 1<<2)  // NAV-PANEL
#define TEST_BTN        (PINC & 1<<0)  // COMM-PANEL
#define DEBUG         0

// The AVR can be waken up by a TWI address match from all sleep modes,
// But it only wakes up from other TWI interrupts when in idle mode.
// If POWER_MANAGEMENT_ENABLED is defined the device will enter power-down 
// mode when waiting for a new command and enter idle mode when waiting
// for TWI receives and transmits to finish.
//#define POWER_MANAGEMENT_ENABLED
#define nop()  __asm__ __volatile__("nop")

// TWI variables: 
static volatile int8_t buffer_data[] = { 0, 0, 0 };
static volatile int8_t buffer_data_new[] = { 0, 0, 0 };
static volatile int8_t buffer_data_master[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
static volatile int8_t ssd_idx[] = { 8, 6, 1, 3, 2, 9, 7, 5, 0, 4 };
static volatile uint8_t twiTransmitSuccess = 0;
static volatile uint8_t twiReceiveSuccess = 0;
static volatile uint8_t loopCount = 0;
// Rotary Encoder
static volatile int8_t enc_delta; // -128 ... 127
static volatile int8_t last;
static int8_t local_change = 0;
static int8_t reBtn_state = 0;
static int8_t reBtn_state_blink = 0;
static uint8_t encoder_change = 0;
static uint32_t reCounter = 0;
static uint16_t standbyBtnCounter = 0;
static uint16_t testBtnCounter = 0;

int8_t encode_read4(void) // read four step encoders
{
	int8_t val;

	cli();
	val = enc_delta;
	enc_delta = val & 3;
	sei();
	return val >> 2;
}

void encode_init(void) {
	int8_t new;

	new = 0;
	if (PHASE_A)
		new = 3;
	if (PHASE_B)
		new ^= 1; // convert gray to binary
	last = new; // power on state
	enc_delta = 0;
	TIMSK |= (1 << TOIE0); // Interrupts aktivieren und damit Timer starten
	TCCR0 |= (1 << CS01) || (1 << CS00); // TEILER 64 für 1ms
	//TCNT1  = 255;			// Vorladen

}

void IO_init(void) //This procedure sets up the IO-pins of the ATmega32.
{
	DDRC = 0b00001110; // 7Segment PC1, PC2, PC3, SDA, SCL, RESET
	PORTC = 0b00000001; // Pull-up for TEST_BTN

	DDRB = 0b11000010; // OC1A, OC1B, XTAL1, XTAL2
	PORTB = 0b00111101; // Pull-up for PHASE A and B, RE_BTN and TRANS

	DDRD = 0b11101111; // 7Segment PD0..3, JCounter PD5..7
	PORTD = 0b00010000;

	// PB1 OC1 Encoder Timer Interrupt
	// PC7 PHASE A Input with pull-up
	// PC6 PHASE B Input with pull-up
	// Connect Rotary Encoder to Ground
	encode_init();
}

void TWI_slave_init(unsigned char twi_slave_addr) {
	TWBR = 0x12;
	TWSR |= 0x01; //baudrate is 16MOSC 100K
	TWAR = (twi_slave_addr);
	TWCR = 0b01000101; // TWINT  TWEA  TWSTA  TWSTO  TWWC  TWEN  -  TWIE
}

void peripheral_init(void)
{
	//Initialize the USART:
	UCSRB |= (1 << RXEN); //Receiver enable
	UCSRB |= (1 << TXEN); //Transmitter enable
	UCSRC |= ((1 << UCSZ0) | (1 << UCSZ1)); //8 Data Bits, 1 Stop Bit, Asynchronous
	UBRRH = 0; //Write UBRRH Register
	UBRRL = 103; //9600 Baud //Write UBRRL Register
	SREG |= (1 << 7); //Enable Interrupts globally
}

void UART_send(char *text, int number, int linefeed) {
	if (DEBUG) {
		char str[10];
		while (*text) //End of string = 0
		{
			while (!(UCSRA & (1 << UDRE))) // Wait until Buffer is empty - ready to send.
			{
			}
			UDR = *text;
			text++;
		}
		itoa(number, str, 10);
		text = &str[0];
		while (*text) //End of string = 0
		{
			while (!(UCSRA & (1 << UDRE))) // Wait until Buffer is empty - ready to send.
			{
			}
			UDR = *text;
			text++;
		}
		if (linefeed) {
			//Next line in the terminal:
			while (!(UCSRA & (1 << UDRE))) // Wait until Buffer is empty - ready to send.
			{
			}
	//		UDR = 0x0D; //Carriage Return
	//		while (!(UCSRA & (1 << UDRE))) // Wait until Buffer is empty - ready to send.
	//		{
	//		}
			UDR = 0x0A; //Line Feed
		}
	}
}

void twi_action(unsigned char rw_status) {
	if (rw_status) {
		UART_send("TWI WRITE ", rw_status, 0);
		uint8_t a = buffer_data_master[0] * 100 + buffer_data_master[1] * 10
				+ buffer_data_master[2];
		uint8_t b = buffer_data_master[3] * 10 + buffer_data_master[4];
		UART_send(" a- ", a, 0);
		UART_send("b- ", b, 0);
		uint8_t c = buffer_data_master[5] * 100 + buffer_data_master[6] * 10
				+ buffer_data_master[7];
		uint8_t d = buffer_data_master[8] * 10 + buffer_data_master[9];
		UART_send("c- ", c, 0);
		UART_send("d- ", d, 1);
	} else {
		if (reBtn_state == 1) {
			buffer_data[2] = encoder_change;
		} else {
			buffer_data[2] = encoder_change * 16;
		}
		buffer_data[0] = buffer_data_new[0];
		buffer_data[1] = buffer_data_new[1];
		encoder_change = 0;
		UART_send("TWI READ ", buffer_data[0], 0);
		UART_send("", buffer_data[1], 0);
		UART_send("", buffer_data[2], 1);
	}
}

void processTestButton(void) {
	testBtnCounter++;
	if (!TEST_BTN) {
		if (testBtnCounter > 100) {
			testBtnCounter = 0;
			buffer_data_new[1] |= 0x01;
		}
	} else {
		if (testBtnCounter > 100) {
			testBtnCounter = 0;
			buffer_data_new[1] &= 0xFE;
		}
	}
}

void processStandbyButton(void) {
	standbyBtnCounter++;
	if (!STANDBY_BTN) {
		if (standbyBtnCounter > 100) {
			standbyBtnCounter = 0;
			buffer_data_new[0] |= 0x01;
		}
	} else {
		if (standbyBtnCounter > 100) {
			standbyBtnCounter = 0;
			buffer_data_new[0] &= 0xFE;
		}
	}
}

void processRotaryEncoder(void) {
	reCounter++;
	if (!RE_BTN) {
		if (reCounter > 100) {
			reCounter = 0;
			buffer_data_new[1] |= 0x02;
		}
	} else {
		if (reCounter > 100) {
			reCounter = 0;
			buffer_data_new[1] &= 0xFD;
		}
	}
}

void Update7SegDisplay(uint8_t segmentIndex) {
// static volatile int8_t ssd_idx[] = { 8, 6, 1, 3, 2, 9, 7, 5, 0, 4 };
	switch (segmentIndex) {
	case 0:
		PORTC &= 0b11111011;
		PORTD &= 0b11110000;
		PORTD |= buffer_data_master[ssd_idx[4]];
		break;
	case 1:
		PORTC &= 0b11111011;
		PORTD &= 0b11110000;
		PORTD |= buffer_data_master[ssd_idx[3]];
		break;
	case 2:
		PORTC &= 0b11111011;
		PORTD &= 0b11110000;
		PORTD |= buffer_data_master[ssd_idx[2]];
		break;
	case 3:
		PORTC &= 0b11111011;
		PORTD &= 0b11110000;
		PORTD |= buffer_data_master[ssd_idx[1]];
		break;
	case 4:
		PORTC &= 0b11111011;
		PORTD &= 0b11110000;
		PORTD |= buffer_data_master[ssd_idx[0]];
		break;
	case 5:
		PORTC &= 0b11111011;
		PORTD &= 0b11110000;
		PORTD |= buffer_data_master[ssd_idx[9]];
		break;
	case 6:
		PORTC &= 0b11111011;
		PORTD &= 0b11110000;
		PORTD |= buffer_data_master[ssd_idx[8]];
		break;
	case 7:
		PORTC &= 0b11111011;
		PORTD &= 0b11110000;
		PORTD |= buffer_data_master[ssd_idx[7]];
		break;
	case 8:
		PORTC &= 0b11111011;
		PORTD &= 0b11110000;
		PORTD |= buffer_data_master[ssd_idx[6]];
		break;
	case 9:
		PORTC &= 0b11111011;
		PORTD &= 0b11110000;
		PORTD |= buffer_data_master[ssd_idx[5]];
		break;
	default:
		//PORTC  &= 0b11111111;
		break;
	}

	if (segmentIndex == 0) {
		SET_BIT(PORTD, 7); // reset johnson counter
		CLEAR_BIT(PORTD, 7); // reset johnson counter
	} else {
		if (segmentIndex < 9) {
			SET_BIT(PORTD, 6); // inc johnson counter
		}
	}
	if (segmentIndex > 1) {
		// before decimal point
		SET_BIT(PORTC, 2); // Enable 7Seg
	}
	SET_BIT(PORTC, 2); // Enable 7Seg
}

ISR( TWI_vect)
{
	static unsigned char i2c_state;
	static unsigned char i2c_state_transmit;

	switch (TW_STATUS) {
	case TW_SR_SLA_ACK:
        //UART_send("SR_SLA_ACK ", TW_STATUS, 1);
		i2c_state = 0;
		TWCR_ACK
		break;
	case TW_SR_DATA_ACK:
        //UART_send("SR_DATA_ACK ", TWDR, 1);
		buffer_data_master[i2c_state++] = TWDR;
		TWCR_ACK
		break;
	case TW_SR_STOP:
        //UART_send("SR_STOP ", TW_STATUS, 1);
		if (i2c_state >= 10) {
			twiReceiveSuccess = 1;
			twi_action(1);
		}
		i2c_state = 0;
		TWCR_ACK
		break;
		// Slave Transmitter Mode
	case TW_ST_SLA_ACK:
        //UART_send("ST_SLA_ACK ", TW_STATUS, 1);
		twi_action(0);
		i2c_state_transmit = 0;
		TWCR_ACK
		break;
	case TW_ST_DATA_ACK:
        //UART_send("ST_DATA_ACK ", buffer_data[i2c_state_transmit], 1);
		TWDR = buffer_data[i2c_state_transmit++];
		TWCR_ACK
		break;
	case TW_ST_DATA_NACK:
        //UART_send("ST_DATA_NACK ", buffer_data[i2c_state_transmit++], 1);
		TWDR = buffer_data[i2c_state_transmit++];
		if (i2c_state_transmit >= 3) {
			twiTransmitSuccess = 1;
		}
		i2c_state_transmit = 0;
		TWCR_ACK
		break;
	case TW_ST_LAST_DATA:
 		//UART_send("ST_LAST_DATA ", 0, 1);
	case TW_BUS_ERROR:
		loopCount++;
 		UART_send("BUS_A ", TWAR, 1);
 		UART_send("BUS_S ", TW_STATUS, 1);
 		UART_send("BUS_D ", TWDR, 1);
 		UART_send("BUS_E ", TWCR, 1);
		PORTB = TWCR;
		TWRESET
		//TWAR=(0x21);
		//TWCR=0b01000101;
		//TWDR = 0x00;
		i2c_state = 0;
		i2c_state_transmit = 0;
		break;
	default:
 //		UART_send("XXXXXXXXXX ", TW_STATUS, 1);
		;
	}
}

ISR( TIMER0_OVF_vect) // 1ms for manual movement
{
	int8_t new1, diff;

	new1 = 0;
	if (PHASE_A)
		new1 = 3;
	if (PHASE_B)
		new1 ^= 1; // convert gray to binary
	diff = last - new1; // difference last - new
	if (diff & 1) { // bit 0 = value (1)
		last = new1; // store new as next last
		enc_delta += (diff & 2) - 1; // bit 1 = direction (+/-)
	}
}

int main(void) {
	uint8_t TWI_slaveAddress = (0x20); // Set TWI slave address
	uint8_t digit = 0; //digit to be displayed on the seven-segment-display
	uint16_t encoderCounter = 0;
	uint16_t segmentCounter = 0;

	cli();
	IO_init();
	TWI_slave_init((unsigned char) ((TWI_slaveAddress)) | (1 << TWI_GEN_BIT));
	if (DEBUG) {
		peripheral_init(); // UART              
	}
	Delay_ms(100);
	sei();
	// Start Transceiver
	TWCR_ACK;

	UART_send("main loop entered: ", 0, 1);

	SET_BIT(PORTD, 5); // reset johnson counter
	CLEAR_BIT(PORTD, 6);
	CLEAR_BIT(PORTD, 7);
	Delay_ms(1);
	CLEAR_BIT(PORTD, 5); // reset johnson counter

	// init bcd to 7 seg
	PORTC &= 0b00001111;
	PORTC |= 0b00001110;

	while (1) {
		if (digit > 9) {
			digit = 0;
		}

		processTestButton();
		processRotaryEncoder();
		processStandbyButton();


		segmentCounter++;
		if (segmentCounter == 5) {
			if (digit == 0) {
				CLEAR_BIT(PORTD, 6); // MR to High after Reset
			}
		} else if (segmentCounter == 10) {
			// prepare johnson counter
			TOOGLE_BIT(PORTD, 6);
		} else if (segmentCounter > 15) {
			segmentCounter = 0;
			Update7SegDisplay(digit);
			digit++;
		}

		encoderCounter++;
		if (encoderCounter > 100) {
			encoder_change += encode_read4(); // read a two step encoder
			encoderCounter = 0;
			local_change = 0;
		}

		/*		if (twiTransmitSuccess || twiReceiveSuccess) {
		 //UART_send("act ", lastAction);
		 if (twiReceiveSuccess == 1) {
		 twiReceiveSuccess = 0;
		 UART_send("<Receive from Master ", 0);
		 UART_send("<[0] ", vorFreq10++);
		 UART_send("<[1] ", vorFreq11);
		 UART_send("<[2] ", vorFreq20);
		 UART_send("<[3] ", vorFreq21);
		 UART_send("             ", 0);
		 } else if (twiTransmitSuccess == 1) {
		 twiTransmitSuccess = 0;
		 UART_send(">Transmit to Master ", 0);
		 UART_send(">[0] ", bufOut[0]);
		 UART_send(">[1] ", bufOut[1]);
		 UART_send(">[2] ", bufOut[2]);
		 UART_send(">[3] ", bufOut[3]);
		 } else {
		 UART_send("act ", lastAction);
		 UART_send("counter ", twiCounter);
		 }
		 }
		 */
	}

}


