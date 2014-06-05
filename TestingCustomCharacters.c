/*
 * TestingCustomCharacters.c
 *
 * Created: 6/3/2014 3:06:48 PM
 *  Author: Rosalba Anzora
 */ 


#include <avr/io.h>
#include "io.c"



//LCD_CreateCustomCharacter creates a custom character from a bit pattern array
void LCD_CreateCustomCharacter(unsigned char pattern_location, unsigned char *pattern_ptr){
	unsigned char i;
	if(pattern_location<8){
		// Tell the LCD where to save the data for the custom characters
		// Commands, like 0x40 and 0x80, are futher described in the LCD datasheet
		LCD_WriteCommand(0x40 + (pattern_location*8));
		for(i=0;i<8;i++){
			// Now that the LCD knows where to send the data,
			//    write each bit pattern in one at a time
			// Bit patterns where created through an outside source
			LCD_WriteData(pattern_ptr[i]);
		}
	}
	// Tell the LCD it is done writing to its memory and get it ready to display
	LCD_WriteCommand(0x80);
	
}

int main(void)
{
	// LCD Pins 7-14 (on all 9 A pins)
	DDRA = 0xFF; PORTA = 0x00;
	// LCD Pins 4 and 6 (on B0 and B1, respectively)
	DDRB = 0xFF; PORTB = 0x00;
	
	// Always initiate the LCD
	LCD_init();
	
	// bit patterns where acquired through a website
	unsigned char lck_char_pattern[8] = {0x0E, 0x11, 0x11, 0x1F, 0x1B, 0x1B, 0x1F, 0x00};
	unsigned char unlck_char_pattern[8] = {0x0E, 0x01, 0x01, 0x1F, 0x1B, 0x1B, 0x1F, 0x00};
	
	//Create lock character
	//     *** 
	//    *   *
	//    *   *
	//    *****
	//    ** **
	//    ** **
	//    *****
	//	       
	LCD_CreateCustomCharacter(0, lck_char_pattern);
	//Create unlock character
	//     ***
	//        *
	//        *
	//    *****
	//    ** **
	//    ** **
	//    *****
	//
	LCD_CreateCustomCharacter(1, unlck_char_pattern);
	
	//Display Character 1
	LCD_WriteData(0x00);
	
	//Display Character 2
	LCD_WriteData(0x01);
	
	//The characters will appear side by side on the first 2 spaces of the LCD screen
    while(1)
    {
		
    }
}