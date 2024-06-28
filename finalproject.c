#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "ds18b20.h"
#include <avr/eeprom.h>

#include "lcd.h"

void play_note(uint16_t);
void variable_delay_us(int16_t);
void servo_temp_control();
void timer1_init();
void timer2_init();


// Frequencies for natural notes from middle C (C4)
// up one octave to C5.
uint16_t frequency[8] =
    { 262, 294, 330, 349, 392, 440, 494, 523 };

volatile uint8_t new_state, old_state;
volatile uint8_t changed = 1;  // Flag for state change
volatile int16_t count = 0; // Count to display
volatile uint8_t lowcount;
volatile uint8_t highcount;
volatile uint8_t a, b;
volatile uint8_t x;
char islow = 1;
char alerted = 0;
char playingsound = 0;
char showingtempchange = 0;
volatile char indicatetemp = 1;
volatile uint32_t freq_cycle = 0;
volatile uint32_t servo_cycle = 0;
volatile uint16_t curr_temp = 70;

int main(void) {

    // Initialize DDR and PORT registers and LCD
	PORTC |= (1 << PC1) | (1 << PC2); // for the encoder
	DDRB |= (1 << PB4); // for the temperature sensor
	DDRC |= (1 << PC3) | (1 << PC4) | (1 << PC5); // for the LEDs
	DDRD &= (~(1 << PD2) & ~(1 << PD3)); // for the buttons
	PORTD |= (1 << PD2) | (1 << PD3); // for the LCD
	DDRB |= (1 << PB5); // for the buzzer
	DDRB |= (1 << PB3); // added for the OC1B for servo moter

	// all initialization
	lcd_init();
	timer1_init();
	timer2_init();

    // Write a spash screen to the LCD
	lcd_writecommand(1);
	lcd_moveto(0,2);
    lcd_stringout("Shane Yokota");
    lcd_moveto(1,1);
    lcd_stringout("EE 109 Project");
    _delay_ms(1000);
	lcd_writecommand(1);


	// PCIER register edit for interrupt
	PCICR |= (1 << PCIE1); // Turn on for Port C
	PCMSK1 |= (1 << PCINT9) | (1 << PCINT10); // Turn on for PC1 & 2
	PCMSK1 |= (1 << PCINT13); // TUrn on for PC5 (for the buzzer)

	// enable global interrupt
	sei();

    // Read the A and B inputs to determine the intial state.
    // In the state number, B is the MSB and A is the LSB.
    // Warning: Do NOT read A and B separately.  You should read BOTH inputs
    // at the same time, then determine the A and B values from that value. 
	x = PINC;   
	a = (x & (1 << PC1));
	b = (x & (1 << PC2));

    if (!b && !a)
	old_state = 0;
    else if (!b && a)
	old_state = 1;
    else if (b && !a)
	old_state = 2;
    else
	old_state = 3;
    new_state = old_state;

	unsigned char t[2];

    if (ds_init() == 0) {    // Initialize the DS18B20
        // Sensor not responding
    }

    ds_convert();    // Start first temperature conversion

	// read lowcount and highcount from eeprom
	lowcount = eeprom_read_byte((void *) 100);
	if(lowcount == 0xFF || (lowcount < 50 && lowcount > 90)){
		lowcount = 60;
	}
	highcount = eeprom_read_byte((void *) 101);
	if(highcount == 0xFF || (highcount < 50 && highcount > 90)){
		highcount = 70;
	}


    while (1) {                 // Loop forever
		if (ds_temp(t)) {    // True if conversion complete
            /*
              Process the values returned in t[0]
              and t[1] to find the temperature.
            */

			// conversion algorithm
			uint16_t temp = (t[1] << 8) | t[0];
			temp = (temp*9 + 2560)/8;
			curr_temp = temp;
			int upper = temp/10;
			int decimal = temp%10;

			// print out temp to LCD
			lcd_moveto(0,0);
			char temperature[20];
			snprintf(temperature, 20, "Temp: %d.%d", upper, decimal);
			lcd_stringout(temperature);

            ds_convert();   // Start next conversion
        }

		// OCR2A edit for the servo motor
		// indicate temp or temp editing
		if(indicatetemp == 1){
			OCR2A = ((((1000-curr_temp)*10)/25) + 120)/10;
		} else if(indicatetemp == 0){
			if(islow == 0){
				OCR2A = ((((1000-highcount*10)*10)/25) + 120)/10;
			} else {
				OCR2A = ((((1000-lowcount*10)*10)/25) + 120)/10;
			}
		}

		// update LEDs
		if(curr_temp > highcount*10){
			PORTC |= (1 << PC4) | (1 << PC3);
			PORTC &= (~(1 << PC5));
		} else if(curr_temp < lowcount*10){
			PORTC |= (1 << PC5) | (1 << PC4);
			PORTC &= (~(1 << PC3));
		} else {
			PORTC |= (1 << PC5) | (1 << PC3);
			PORTC &= (~(1 << PC4));
		}

		// Buzzer check
		if((curr_temp > (highcount*10+30) || curr_temp < (lowcount*10-30)) && alerted != 1){
			// keeps acivating if above the threshold
			// only actiate once
			alerted = 1;
			play_note(262);
		} else if((curr_temp < (highcount*10) && curr_temp > (lowcount*10)) && alerted == 1){
			alerted = 0;
		}
	
		// Read the low/high buttons
		char lowbutton = PIND & (1 << PD2);
		char highbutton = PIND & (1 << PD3);
		if(lowbutton == 0){
			islow = 0;
			changed = 1;
			indicatetemp = 0;
			servo_temp_control();
		} else if(highbutton == 0){
			islow = 1;
			changed = 1;
			indicatetemp = 0;
			servo_temp_control();
		}

        if (changed) { // Did state change?
			changed = 0;        // Reset changed flag

			// Output count to LCD
			lcd_moveto(1,0);
			char buf[22];
			if(islow != 0){ // control low
				snprintf(buf, 22, "Low? %2d High= %2d", lowcount, highcount);
			} else { // control high
				snprintf(buf, 22, "Low= %2d High? %2d", lowcount, highcount);
			}
			lcd_stringout(buf);

        }
    }
}

/*
  Play a tone at the frequency specified for one second
*/
void play_note(uint16_t freq)
{
	OCR1A = 16000000 / (2 * freq);
	freq_cycle = 15000/(10000/(2*freq));
	TCCR1B |= (1 << CS10); // set prescaler to 1
	playingsound = 1;
}

void servo_temp_control()
{
	OCR1A = 62500/2;
	servo_cycle = 1;
	TCCR1B |= (1 << CS12) | (1 << CS10); // set prescaler to 1024 101
	showingtempchange = 1;
}

/*
    variable_delay_us - Delay a variable number of microseconds
*/
void variable_delay_us(int delay)
{
    int i = (delay + 5) / 10;

    while (i--)
        _delay_us(10);
}

ISR(PCINT1_vect)
{
    // Read the encoder inputs and determine the new
    // count value

	// take in the input
	x = PINC;
	a = ((x & (1 << PC1)) >> 1);
	b = ((x & (1 << PC2)) >> 2);

	if (old_state == 0) {
		// Handle A and B inputs for state 0
		if (a == 1){
			if(islow == 0){
				if(highcount < 90){
					highcount++;
				}
			} else {
				if(lowcount < highcount){
					lowcount++;
				}
			}
			new_state = 1;
		} else if(b == 1){
			if(islow == 0){
				if(highcount > lowcount){
					highcount--;
				}
			} else {
				if(lowcount > 50){
					lowcount--;
				}
			}
			new_state = 2;
		}
	}
	else if (old_state == 1) {
		// Handle A and B inputs for state 1
		if (a == 0){
			if(islow == 0){
				if(highcount > lowcount){
					highcount--;
				}
			} else {
				if(lowcount > 50){
					lowcount--;
				}
			}
			new_state = 0;
		} else if(b == 1){
			if(islow == 0){
				if(highcount < 90){
					highcount++;
				}
			} else {
				if(lowcount < highcount){
					lowcount++;
				}
			}
			new_state = 3;
		}
	}
	else if (old_state == 2) {
		// Handle A and B inputs for state 2
		if (a == 1){
			if(islow == 0){
				if(highcount > lowcount){
					highcount--;
				}
			} else {
				if(lowcount > 50){
					lowcount--;
				}
			}
			new_state = 3;
		} else if(b == 0){
			if(islow == 0){
				if(highcount < 90){
					highcount++;
				}
			} else {
				if(lowcount < highcount){
					lowcount++;
				}
			}
			new_state = 0;
		}
	}
	else {   // old_state = 3
		// Handle A and B inputs for state 3
		if (a == 0){
			if(islow == 0){
				if(highcount < 90){
					highcount++;
				}
			} else {
				if(lowcount < highcount){
					lowcount++;
				}
			}
			new_state = 2;
		} else if(b == 0){
			if(islow == 0){
				if(highcount > lowcount){
					highcount--;
				}
			} else {
				if(lowcount > 50){
					lowcount--;
				}
			}
			new_state = 1;
		}
	}	

	// If state changed, update the value of old_state,
	// and set a flag that the state has changed.
	if (new_state != old_state) {
		changed = 1;
		old_state = new_state;
		eeprom_update_byte((void *) 100, lowcount);
		eeprom_update_byte((void *) 101, highcount);
	}
}


void timer1_init()
{
    // Inititialize TIMER1, but don't start it counting
	TCCR1B |= (1 << WGM12); // Set CTC mode  
    TIMSK1 = (1 << OCIE1A); // Enable the compare interrupt (local intereupt)
}

void timer2_init(void)
{
	TCCR2A |= (0b11 << WGM20);  // Fast PWM mode, modulus = 256
	TCCR2A |= (0b10 << COM2A0); // Turn D11 on at 0x00 and off at OCR2A
	TCCR2B |= (0b111 << CS20);  // Prescaler = 1024 for 16ms period
}

ISR(TIMER1_COMPA_vect)
{
    // Change the output bit to the buzzer, and to turn
    // off the timer after enough periods of the signal
	if(playingsound == 1){
		if(freq_cycle == 0){ // if the freq is 0, turn off the timer
			TCCR1B &= ~(1 << CS10);
			PORTB &= ~(1 << PB5);
			playingsound = 0;
		} else if(freq_cycle != 0){ // else, toggle the buzzer
			PORTB ^= (1 << PB5);
			freq_cycle--;
		}
	}
	
	// timer counting(decrement) for the servo motor (4 sec)
	if(showingtempchange == 1 && indicatetemp == 0){
		if(servo_cycle == 0){
			indicatetemp = 1;
			TCCR1B &= ~(1 << CS10);
			TCCR1B &= ~(1 << CS12);
			showingtempchange = 0;
		} else if(servo_cycle != 0){
			servo_cycle--;
		}
	}
}
