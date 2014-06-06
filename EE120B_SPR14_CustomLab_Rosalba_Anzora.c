/*	Partner 1 Name & E-mail: Rosalba Anzora   ranzo001@ucr.edu
*	Lab Section: 023
*	Assignment: Custom Lab
*	Lab Description: Security System
*
*	I acknowledge all content contained herein, excluding template or example
*	code, is my own original work.
*/

#include <avr/io.h>
#include "io.c"
#include <avr/interrupt.h>
#include <bit.h>
#include <timer.h>
#include <stdio.h>

//--------PWM functions-------------------------------------------------------

// Sets up D7 as a PWM pin for the red function of the RGB LED
void set_PWM(double frequency) {
	static double current_frequency; // Keeps track of the currently set frequency
	// Will only update the registers when the frequency changes, otherwise allows
	// music to play uninterrupted.
	if (frequency != current_frequency) {
		if (!frequency) { TCCR2B &= 0x08; } //stops timer/counter
		else { TCCR2B |= 0x03; } // resumes/continues timer/counter
		
		// prevents OCR2A from overflowing, using prescaler 64
		// 0.954 is smallest frequency that will not result in overflow
		if (frequency < 0.954) { OCR2A = 0xFFFF; }
		
		// prevents OCR2A from underflowing, using prescaler 64					// 31250 is largest frequency that will not result in underflow
		else if (frequency > 31250) { OCR2A = 0x0000; }
		
		// set OCR2A based on desired frequency
		else { OCR2A = (short)(8000000 / (128 * frequency)) - 1; }

		TCNT2 = 0; // resets counter
		current_frequency = frequency; // Updates the current frequency
	}
}

void PWM_on() {
	TCCR2A = (1 << COM2A0);
	// COM2A0: Toggle PD7 on compare match between counter and OCR3A
	TCCR2B = (1 << WGM22) | (1 << CS21) | (1 << CS20);
	// WGM22: When counter (TCNT3) matches OCR2A, reset counter
	// CS21 & CS20: Set a prescaler of 64
	set_PWM(0);
}

void PWM_off() {
	TCCR2A = 0x00;
	TCCR2B = 0x00;
}

//--------End PWM functions---------------------------------------------------

//--------Find GCD function --------------------------------------------------
unsigned long int findGCD(unsigned long int a, unsigned long int b)
{
	unsigned long int c;
	while(1){
		c = a%b;
		if(c==0){return b;}
		a = b;
		b = c;
	}
	return 0;
}
//--------End find GCD function ----------------------------------------------

//--------Task scheduler data structure---------------------------------------
// Struct for Tasks represent a running process in our simple real-time operating system.
typedef struct _task {
	/*Tasks should have members that include: state, period,
		a measurement of elapsed time, and a function pointer.*/
	signed char state; //Task's current state
	unsigned long int period; //Task period
	unsigned long int elapsedTime; //Time elapsed since last task tick
	int (*TickFct)(int); //Task tick function
} task;

//--------End Task scheduler data structure-----------------------------------

//--------Shared Functions----------------------------------------------------
//GetKeypadKey interprets a keypad push into a character value
unsigned char GetKeypadKey() {

	PORTC = 0xEF; // Enable col 4 with 0, disable others with 1’s
	asm("nop"); // add a delay to allow PORTC to stabilize before checking
	if (GetBit(PINC,0)==0) { return('1'); }
	if (GetBit(PINC,1)==0) { return('4'); }
	if (GetBit(PINC,2)==0) { return('7'); }
	if (GetBit(PINC,3)==0) { return('*'); }

	// Check keys in col 2
	PORTC = 0xDF; // Enable col 5 with 0, disable others with 1’s
	asm("nop"); // add a delay to allow PORTC to stabilize before checking
	if (GetBit(PINC,0)==0) { return('2'); }
	if (GetBit(PINC,1)==0) { return('5'); }
	if (GetBit(PINC,2)==0) { return('8'); }
	if (GetBit(PINC,3)==0) { return('0'); }

	// Check keys in col 3
	PORTC = 0xBF; // Enable col 6 with 0, disable others with 1’s
	asm("nop"); // add a delay to allow PORTC to stabilize before checking
	if (GetBit(PINC,0)==0) { return('3'); }
	if (GetBit(PINC,1)==0) { return('6'); }
	if (GetBit(PINC,2)==0) { return('9'); }
	if (GetBit(PINC,3)==0) { return('#'); }

	// Check keys in col 4
	PORTC = 0x7F; //Enable col 7 with 0, disable others with 1's
	asm("nop"); // add a delay to allow PORTC to stabilize before checking
	if (GetBit(PINC,0)==0) { return('A'); }
	if (GetBit(PINC,1)==0) { return('B'); }
	if (GetBit(PINC,2)==0) { return('C'); }
	if (GetBit(PINC,3)==0) { return('D'); }

	return('\0'); // default value

}

//LCD_CreateCustomCharacter creates a custom character from a bit pattern array
// Sites used to find out how to create custom characters:
//      http://www.8051projects.net/lcd-interfacing/lcd-custom-character.php
//      http://saeedsolutions.blogspot.com/2012/12/how-to-display-custom-characters-on-lcd.html

void LCD_CreateCustomCharacter(unsigned char pattern_location, unsigned char *pattern_ptr){
	unsigned char i;
	if(pattern_location<8){
		LCD_WriteCommand(0x40 + (pattern_location*8));
		for(i=0;i<8;i++){
			LCD_WriteData(pattern_ptr[i]);
		}
	}
	LCD_WriteCommand(0x80);
	
}

//--------End Shared Functions------------------------------------------------

//--------Shared Variables----------------------------------------------------
unsigned char sensor_reading; // used in Detection()
unsigned char movement; // is found in Detection() and used in SoundAlarm()
unsigned char arm; // flag of the system that notifies whether the system is 
                   // armed or disarmed
unsigned char input_keypad_data;  // holds the value inputed from keypad 
                                  // translated by GetKeypadKey()
// bit patterns for a closed pad lock character
unsigned char lck_char_pattern[8] = {0x0E, 0x11, 0x11, 0x1F, 0x1B, 0x1B, 0x1F, 0x00};
// bit patterns for a opened pad lock character
unsigned char unlck_char_pattern[8] = {0x0E, 0x01, 0x01, 0x1F, 0x1B, 0x1B, 0x1F, 0x00};

//--------End Shared Variables------------------------------------------------

//--------User defined FSMs---------------------------------------------------
// Keypad Input SM
// Receives a keypad input and calls GetKeypadKey() to interpret the input
// TESTED AND WORKS
enum input_state{init};
int InputsKey(int state) {
	switch(state) {  //Transitions
		case init:
			state = init;
			break;
		default:
			state = init;
			break;
	}
	
	switch(state) {   //State Actions
		
		case init:
			input_keypad_data = GetKeypadKey();
			break;
		default:
			break;
	}
	return state;
}

// Alarm_Code is the main control of the system. It is what tells all
// the other SMs whether the system is ARMED or DISARMED. To ARM you must
// press 'A' on the keypad. To DISARM you must enter '1' '2' '3' '4' 'D'.
// TESTED AND WORKS
enum alarmCode_States {alarm_off, alarm_on, disarm_1, disarm_2, disarm_3, disarm_4};
int Alarm_Code(int state) {
	switch (state){
		case alarm_off:
			if (input_keypad_data =='A'){
				state = alarm_on;
				}else{
				state = alarm_off;
			}
			break;
		case alarm_on:
			if (input_keypad_data =='1'){
				state = disarm_1;
				}else{
				state = alarm_on;
			}
			break;
		case disarm_1:
			if (input_keypad_data =='2'){
				state = disarm_2;
				}else if (!(input_keypad_data == '2')){
				state = disarm_1;
			}
			break;
		case disarm_2:
			if (input_keypad_data == '3'){
				state = disarm_3;
				}else if (!(input_keypad_data == '3')){
				state = disarm_2;
			}
			break;
		case disarm_3:
			if (input_keypad_data == '4'){
				state = disarm_4;
				}else if (!(input_keypad_data == '4')){
				state = disarm_3;
			}
			break;
		case disarm_4:
			if (input_keypad_data == 'D'){
				state = alarm_off;
				}else if (!(input_keypad_data == 'D')){
				state = disarm_4;
			}
			break;
		default:
			state = alarm_off;
			break;
	}
	
	switch (state){
		case alarm_off:
			//arm flag remains low
			arm = 0;
			//Create unlock character
			LCD_CreateCustomCharacter(1, unlck_char_pattern);
			//display the unlock character on LCD
			LCD_WriteData(0x01);
			break;
		case alarm_on:
			//arm flag remains high
			arm = 1;
			//Create lock character
			LCD_CreateCustomCharacter(0, lck_char_pattern);
			//display the lock character on LCD
			LCD_WriteData(0x00);
			break;
		case disarm_1:
			break;
		case disarm_2:
			break;
		case disarm_3:
			break;
		case disarm_4:
			break;
		default:
			break;
	}
	return state;
}

// BlinkLED SM
// RGB decides whether the LED should blink red or not based on the state
// of the security system. I use PWM to blink the LED.
// IS NOT WORKING THE WAY ITS SUPPOSE TO
enum LEDStates {disarmed, armed_blink_off, armed_blink_on};
int RGB_Freq(int state){
	switch(state){    // State Transitions
		case disarmed:
			if(arm){
				state = armed_blink_off;
				}else if(!arm){
				state = disarmed;
			}
			break;
		case armed_blink_off:
			if(movement){
				state = armed_blink_on;
			}else if(!arm){
				state = disarmed;
			}else if(!movement && arm){
				state = armed_blink_off;
			}
			break;
		case armed_blink_on:
			if(arm){
				state = armed_blink_on;
			}else if(!arm){
				state = disarmed;
			}
			break;
		default:
			state = disarmed;
			break;
	}
	
	switch(state){    // State Actions
		case disarmed:
			set_PWM(0.0);
			break;
		case armed_blink_off:
			set_PWM(0.0);
			break;
		case armed_blink_on:
			set_PWM(255.0);
			break;
		default:
			break;
	}
	return state;
}

// Detection SM
// Detects motion when the the security system is ARMED
// TESTED AND WORKS
enum detectStates {detect_init, check_movement, movement_detected};
int Detection(int state){
	//sensor reading is the input coming from the PIR Sensor
	sensor_reading = PIND & 0x01;
	
	switch(state){    // State Transitions
		case detect_init:
			movement = 0;
			state = check_movement;
			break;
		case check_movement:
			if(sensor_reading){
				state = movement_detected;
				}else if(!sensor_reading){
				state = check_movement;
			}
			break;
		case movement_detected:
			if(sensor_reading){
				state = movement_detected;
				}else if(!sensor_reading){
				state = check_movement;
			}
			break;
		default:
			state = detect_init;
			break;
	}
	
	switch(state){    // State Actions
		case check_movement:
			movement = 0;
			//RGB_pins = 0x01;    // used to test detection
			break;
		case movement_detected:
			movement = 1;
			//RGB_pins = 0x04;    // used to test detection
			break;
		default:
			break;
	}
	return state;
}

// Bluetooth Module SM
//
// TO BE COMPLETED
//

// --------END User defined FSMs-----------------------------------------------

// Implement scheduler code from PES.
int main()
{
	//----------------------------------
	// Set Data Direction Registers
	//----------------------------------
	// LCD Pins 7-14 (on all 9 A pins)
	DDRA = 0xFF; PORTA = 0x00;
	// LCD Pins 4 and 6 (on B0 and B1, respectively)
	DDRB = 0xFF; PORTB = 0x00;
	// Keypad Pins (C0-C3 input, C4-C7 output)
	DDRC = 0xF0; PORTC = 0x0F;
	// Pin D0 input from sensor, Pin D2(RX, input) gets TX from BT Module, 
	// Pin D3(TX, output)s TX from BT Module, Pin D7 outputs to RGB LED
	DDRD = 0x88; PORTD = 0x77;

	//----------------------------------
	// Period for the tasks
	//----------------------------------
	unsigned long int InputsKeySM_calc = 50;
	unsigned long int AlarmCodeSM_calc = 500;
	unsigned long int RGBFreqSM_calc = 1000;
	unsigned long int DetectionSM_calc = 100;

	//----------------------------------
	//Calculating GCD
	//----------------------------------
	unsigned long int tmpGCD = 1;
	tmpGCD = findGCD(InputsKeySM_calc, AlarmCodeSM_calc);
	tmpGCD = findGCD(tmpGCD, RGBFreqSM_calc);
	tmpGCD = findGCD(tmpGCD, DetectionSM_calc);

	//-----------------------------------------------------------------------
	//Greatest common divisor for all tasks or smallest time unit for tasks.
	//-----------------------------------------------------------------------
	unsigned long int GCD = tmpGCD;

	//--------------------------------------
	//Recalculate GCD periods for scheduler
	//--------------------------------------
	unsigned long int InputsKeySM_period = InputsKeySM_calc/GCD;
	unsigned long int AlarmCodeSM_period = AlarmCodeSM_calc/GCD;
	unsigned long int RGBFreqSM_period = RGBFreqSM_calc/GCD;
	unsigned long int DetectionSM_period = DetectionSM_calc/GCD;

	//----------------------------------
	//Declare an array of tasks 
	//----------------------------------
	static task task1, task2, task3, task4;
	task *tasks[] = { &task1, &task2, &task3, &task4 };
	const unsigned short numTasks = sizeof(tasks)/sizeof(task*);

	// Task 1
	task1.state = init;//Task initial state.
	task1.period = InputsKeySM_period;//Task Period.
	task1.elapsedTime = InputsKeySM_period;//Task current elapsed time.
	task1.TickFct = &InputsKey;//Function pointer for the tick.

	// Task 2
	task2.state = alarm_off;//Task initial state.
	task2.period = AlarmCodeSM_period;//Task Period.
	task2.elapsedTime = AlarmCodeSM_period;//Task current elapsed time.
	task2.TickFct = &Alarm_Code;//Function pointer for the tick.

	// Task 3
	task3.state = disarmed;//Task initial state.
	task3.period = RGBFreqSM_period;//Task Period.
	task3.elapsedTime = RGBFreqSM_period; // Task current elapsed time.
	task3.TickFct = &RGB_Freq; // Function pointer for the tick.

	// Task 4
	task4.state = detect_init;//Task initial state.
	task4.period = DetectionSM_period;//Task Period.
	task4.elapsedTime = DetectionSM_period; // Task current elapsed time.
	task4.TickFct = &Detection; // Function pointer for the tick.

	//----------------------------------
	// Set the timer and turn it on
	//----------------------------------
	TimerSet(GCD);
	TimerOn();

	//----------------------------------
	// Turn on PWM
	//----------------------------------
	PWM_on();
	
	//----------------------------------
	// Initialize LCD Screen
	//----------------------------------
	LCD_init();

	unsigned short i; // Scheduler for-loop iterator
	while(1) {
		// Scheduler code
		for ( i = 0; i < numTasks; i++ ) {
			// Task is ready to tick
			if ( tasks[i]->elapsedTime == tasks[i]->period ) {
				// Setting next state for task
				tasks[i]->state = tasks[i]->TickFct(tasks[i]->state);
				// Reset the elapsed time for next tick.
				tasks[i]->elapsedTime = 0;
			}
			tasks[i]->elapsedTime += 1;
		}
		while(!TimerFlag);
		TimerFlag = 0;
	}

	// Error: Program should not exit!
	return 0;
}