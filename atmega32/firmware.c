/* 
 Target device: AVR ATmega32, running at 16MHz.
 
 7-Segment Driver 74 4511N
 IA - PB0
 IB - PB1
 IC - PB2
 ID - PB3
 LT - PB4 (Lamp test
 BI - PB6 (Blanking Input)
 LE - PB5 (Latch enable)
 
 Johnson Counter 74 4017N
 CP0 - PD2 (clock input LOW to HIGH)
 CP1 - PD0 (clock input HIGH to LOW)
 MR1 - PD1 (Master reset)
 
 Rotary Encoder
 A0 - PA6
 B0 - PA7
 A1 - PA4
 B1 - PA5
 SW - PA0
 
 Decimal Point:   PA1
 Switch STANDBY:  PB7
 Switch COMMTEST: PC3
 Switch TEST:     PC2
 
 TWI:
 SDA: PC1
 SCL: PC0
 
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
#define PHASE_A		(PINA & 1<<6)
#define PHASE_B		(PINA & 1<<7)
#define PHASE_A1		(PINA & 1<<4)
#define PHASE_B1		(PINA & 1<<5)
#define RE_BTN          (PINA & 1<<0)
#define STANDBY_BTN     (PINB & 1<<7)
//#define TEST_BTN        (PINC & 1<<2)  // NAV-PANEL
#define TEST_BTN        (PINC & 1<<3)  // COMM-PANEL
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
static volatile uint8_t twiTransmitSuccess = 0;
static volatile uint8_t twiReceiveSuccess = 0;
// Rotary Encoder
static volatile int8_t enc_delta; // -128 ... 127
static volatile int8_t last;
static int8_t reBtn_state_toggle = 0;
static int8_t reBtn_state = 0;
static int8_t encoder_change = 0;
static volatile int8_t reIncrement = 0;
static volatile int8_t reDecrement = 0;
static uint32_t reCounter = 0;
static uint16_t standbyBtnCounter = 0;
static uint16_t testBtnCounter = 0;
static uint8_t TWI_slaveAddress = (0x20); // Set TWI slave address

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
/* OLD
	DDRC = 0b00001110; // 7Segment PC1, PC2, PC3, SDA, SCL, RESET
	PORTC = 0b00000001; // Pull-up for TEST_BTN

	DDRB = 0b11000010; // OC1A, OC1B, XTAL1, XTAL2
	PORTB = 0b00111101; // Pull-up for PHASE A and B, RE_BTN and TRANS

	DDRD = 0b11101111; // 7Segment PD0..3, JCounter PD5..7
	PORTD = 0b00010000;
*/

	DDRA = 0b11110011; // Rotary Encoder
	PORTA = 0b11110001; // Pull-up

	DDRC = 0b00001000; // SCL PC0, SDA PC1, COMMTEST PC3
	PORTC = 0b00001000; // Pull-up for TEST_BTN

	DDRB = 0b11111111; // 7Segment PB0..3, PB4..6, TRANSFER_BTN P7
	PORTB = 0b10010000;

	DDRD = 0b00000111; // JohnsonCounter PD0..3
	PORTD = 0b00000000;

	// PB1 OC1 Encoder Timer Interrupt
	// PC7 PHASE A Input with pull-up
	// PC6 PHASE B Input with pull-up
	// PD5 = Johnson decade counter CP1 (HIGH-to-LOW, edge-triggered)
	// PD6 = Johnson decade counter CP0 clock input (LOW-to-HIGH, edge-triggered)
	// PD7 = Johnson decade counter MR (active high)
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
		if (DEBUG) {
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
		}
	} else {
		buffer_data[0] = buffer_data_new[0];
		buffer_data[1] = 0;
		buffer_data[2] = 0;
		if (reBtn_state == 1) {
			if (reIncrement > 0) {
				buffer_data[1] = reIncrement;
			} else if (reDecrement > 0) {
				buffer_data[1] = reDecrement*16;
			}
		} else {
			if (reIncrement > 0) {
				buffer_data[2] = reIncrement;
			} else if (reDecrement > 0) {
				buffer_data[2] = reDecrement*16;
			}
		}
		reIncrement = 0; // reset
		reDecrement = 0; // reset
//		UART_send("TWI READ ", buffer_data[0], 0);
//		UART_send("", buffer_data[1], 0);
//		UART_send("", buffer_data[2], 1);
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

void processTestButton(void) {
	testBtnCounter++;
	if (!TEST_BTN) {
		if (testBtnCounter > 100) {
			testBtnCounter = 0;
			buffer_data_new[0] |= 0x02;
		}
	} else {
		if (testBtnCounter > 100) {
			testBtnCounter = 0;
			buffer_data_new[0] &= 0xFD;
		}
	}
}

void processRotaryEncoder(void) {
	reCounter++;
	if (!RE_BTN) {
		if (reCounter > 100) {
			reCounter = 0;
			buffer_data_new[0] |= 0x04;
			if (reBtn_state_toggle == 1) {
				if (reBtn_state == 1) {
					reBtn_state = 0;
				} else {
					reBtn_state = 1;
				}
				reBtn_state_toggle = 0;
			}			
		}
	} else {
		if (reCounter > 100) {
			reCounter = 0;
			buffer_data_new[0] &= 0xFB;
			reBtn_state_toggle = 1;
		}
	}
}

/*	 7-Segment Driver 74 4511N
	 IA - PB0
	 IB - PB1
	 IC - PB2
	 ID - PB3
	 LT - PB4 (Lamp test)
	 BI - PB6 (Blanking Input)
	 LE - PB5 (Latch enable)
 */
// PD0 = Johnson decade counter CP1 (HIGH-to-LOW, edge-triggered)
// PD2 = Johnson decade counter CP0 clock input (LOW-to-HIGH, edge-triggered)
// PD1 = Johnson decade counter MR (active high)

void Update7SegDisplay(uint8_t segmentIndex) {
// 120.10 127.50 -> buffer_data_master[9] {1,2,0,1,0,1,2,7,5,0}
// SegmentIndex :  0 1 2 3  4 5 6 7 8  9 (segmentIndex) = PANEL POSITION
// JC Pin       :  3 2 4 7 10 1 5 6 9 11
// Panel        :  2 3 1 6  8 4 0 5 7  9 (buffer[2] = segmentIndex/case)
//	return;

	// Disable 7Seg
	CLEAR_BIT(PORTB, 5);
	CLEAR_BIT(PORTB, 6);

	// inc johnson counter
	SET_BIT(PORTD, 2);

	if (segmentIndex == 2 || segmentIndex == 7) { // Buffer[2] Buffer[7]
		if (!(buffer_data_master[7] == 0x0F)) {
			CLEAR_BIT(PORTA, 1);   // enable decimal point
		}
	} else {
		SET_BIT(PORTA, 1); // disable decimal point
	}
	
	switch (segmentIndex) {
	case 0:
		PORTB &= 0b11110000;
		PORTB |= buffer_data_master[9];
		break;
	case 1:
		PORTB &= 0b11110000;
		PORTB |= buffer_data_master[8];
		break;
	case 2:
		PORTB &= 0b11110000;
		PORTB |= buffer_data_master[7];
		break;
	case 3:
		PORTB &= 0b11110000;
		PORTB |= buffer_data_master[6];
		break;
	case 4:
		PORTB &= 0b11110000;
		PORTB |= buffer_data_master[5];
		break;
	case 5:
		PORTB &= 0b11110000;
		PORTB |= buffer_data_master[4];
		break;
	case 6:
		PORTB &= 0b11110000;
		PORTB |= buffer_data_master[3];
		break;
	case 7:
		PORTB &= 0b11110000;
		PORTB |= buffer_data_master[2];
		break;
	case 8:
		PORTB &= 0b11110000;
		PORTB |= buffer_data_master[1];
		break;
	case 9:
		PORTB &= 0b11110000;
		PORTB |= buffer_data_master[0];
		break;
	default:
		PORTB &= 0b11110000;
		PORTB |= 0b00001111;
		break;
	}
	
	if (!TEST_BTN) {
		// ALL LEDs
		PORTB &= 0b11110000;
		PORTB |= 0b00001000;
		// enable decimal point
		CLEAR_BIT(PORTA, 1);
	} else {
	}

	// Enable 7Seg
	SET_BIT(PORTB, 6);
//	CLEAR_BIT(PORTD, 2);
}

ISR( TWI_vect)
{
	static unsigned char i2c_state;
	static unsigned char i2c_state_transmit;

	switch (TW_STATUS) {
	case TW_SR_SLA_ACK:
        UART_send("SR_SLA_ACK ", TW_STATUS, 1);
		i2c_state = 0;
		TWCR_ACK
		break;
	case TW_SR_DATA_ACK:
        UART_send("SR_DATA_ACK ", TWDR, 1);
		buffer_data_master[i2c_state++] = TWDR;
		TWCR_ACK
		break;
	case TW_SR_STOP:
        UART_send("SR_STOP ", TW_STATUS, 1);
		if (i2c_state >= 10) {
			twiReceiveSuccess = 1;
			twi_action(1);
		}
		i2c_state = 0;
		TWCR_ACK
		break;
		// Slave Transmitter Mode
	case TW_ST_SLA_ACK:
        UART_send("ST_SLA_ACK ", TW_STATUS, 1);
		twi_action(0);
		i2c_state_transmit = 0;
		TWCR_ACK
		break;
	case TW_ST_DATA_ACK:
        UART_send("ST_DATA_ACK ", buffer_data[i2c_state_transmit], 1);
		TWDR = buffer_data[i2c_state_transmit++];
		TWCR_ACK
		break;
	case TW_ST_DATA_NACK:
        UART_send("ST_DATA_NACK ", buffer_data[i2c_state_transmit++], 1);
		TWDR = buffer_data[i2c_state_transmit++];
		if (i2c_state_transmit >= 3) {
			twiTransmitSuccess = 1;
		}
		i2c_state_transmit = 0;
		TWCR_ACK
		break;
	case TW_ST_LAST_DATA:
 		UART_send("ST_LAST_DATA ", 0, 1);
	case TW_BUS_ERROR:
// 		UART_send("BUS_A ", TWAR, 1);
// 		UART_send("BUS_S ", TW_STATUS, 1);
// 		UART_send("BUS_D ", TWDR, 1);
// 		UART_send("BUS_E ", TWCR, 1);
		TWRESET
		nop();
		nop();
		nop();
		TWAR=(((TWI_slaveAddress)) | (1 << TWI_GEN_BIT));
		TWCR=0b01000101;
		TWDR = 0x00;
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
	uint8_t digit = 0; //digit to be displayed on the seven-segment-display
	uint16_t encoderCounter = 0;
	uint16_t multiplexCounter = 0;

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

	// PD5 = Johnson decade counter CP1 (HIGH-to-LOW, edge-triggered)
	// PD6 = Johnson decade counter CP0 clock input (LOW-to-HIGH, edge-triggered)
	// PD7 = Johnson decade counter MR (active high) CarryT = 0
	CLEAR_BIT(PORTD, 0); 
	CLEAR_BIT(PORTD, 2);
	SET_BIT(PORTD, 1);
	Delay_ms(1);
	CLEAR_BIT(PORTD, 1); // JC: Master Reset
	digit = 0;

	// init bcd to 7 seg
	PORTB &= 0b00000000;
	PORTB |= 0b10010000;

	while (1) {
		processTestButton();
		processRotaryEncoder();
		processStandbyButton();

		multiplexCounter++;
		if (multiplexCounter == 10) {
			// prepare johnson counter
			CLEAR_BIT(PORTD, 2);
		} else if (multiplexCounter > 15) {
			multiplexCounter = 0;
			if (digit == 9) {
				digit = 0;
			} else {
				if (digit == 0) {
					SET_BIT(PORTD, 1);
					CLEAR_BIT(PORTD, 1); // JC: Master Reset
				}
				digit++;
			}
			Update7SegDisplay(digit);
		}

		encoderCounter++;
		if (encoderCounter > 100) {
			encoder_change = encode_read4(); // read a two step encoder
			if (encoder_change > 0) {
				reIncrement += encoder_change;
				if (reIncrement > 15) {
					reIncrement = 15;
				}
				reDecrement = 0;
			} else if (encoder_change < 0) {
				reDecrement += abs(encoder_change);
				if (reDecrement > 15) {
					reDecrement = 15;
				}
				reIncrement = 0;
			}
			encoderCounter = 0;
		}
	}

}


