#ifndef __lcd_h__
#define __lcd_h__

/* defines needed:
	
	LCD_DB4
	LCD_DB5
	LCD_DB6
	LCD_DB7
	LCD_RS
	LCD_E

	SET_OUTP
	SETUP_OUT_PIN

	*/

void init_lcd();
void lcd_print(char* c, short line);

#endif

