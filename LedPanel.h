/*******************************************************************
* LibSpider16x128. Library for Arduino Control of a Led Panel made by
* Spider LTDA. A brasilian proprietary electronics company.
*
* Copyright (C) 2012 Pedro Belasco pbelasco@gmail.com
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published by
* the Free Software Foundation; either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Affero General Public License for more details.
*
* You should have received a copy of the GNU Affero General Public License
* along with this program; if not, see http://www.gnu.org/licenses
* or write to the Free Software Foundation,Inc., 51 Franklin Street,
* Fifth Floor, Boston, MA 02110-1301  USA
******************************************************************************/
/*
  Pin Mapping
  Inside the product, there is a 10 pin slot numbered as:

	CN6
	+----+----+
	| 05 | 06 |
	+---------+
	| 04 | 07 |
	+---------+
	| 03 | 08 |
	+---------+
	| 02 | 09 |
	+---------+
	| 01 | 10 |
	+---------+

Where, the function of each port is defined by:

	01. GND	: Ground
	02. SECK: Serial clock pin.
	03. SERI: Flag to determine bit status before Clock issued.
	04. L0	: Line Selector. The first bit of 3 bit binary line selector
	07. L1	: Line Selector. The second bit of 3 bit binary line selector
	05. L2	: Line Selector. The third bit of 3 bit binary line selector
	06. SEY	: Select Y. Flag to select the four lower lines
	10. SEX - Select X. Flag to select the four higher lines
	08. RECK - Serial Latch pin. To write an entire line.
	09. N/C - Not Connected

*/

#include <inttypes.h>
#include "Arduino.h"

#ifndef LedPanel_h
#define LedPanel_h

// Fonts
#define FONT            System5x7
#define CHAR_WIDTH_5x7  2
#define CHAR_HEIGHT_5x7 3
#define FIRST_CHAR_5x7  4
#define FONT_COUNT_5x7  5
#define START_CHARS_5x7 6

// Buffer
#define TYPEOF         uint8_t
#define BITSIZE        8
#define BITSIZE_MINONE 7
#define NUMVARS        16 //Buffer size/Bits per var

#define WINDOW_W	128
#define WINDOW_H	16

#define SWITCH_TO_Y(M_SEX, M_SEY) {PORTD &= ~M_SEX; PORTB |=  M_SEY; } 
#define SWITCH_TO_X(M_SEX, M_SEY) {PORTD |=  M_SEX; PORTB &= ~M_SEY; } 
#define SWITCH_L_ON()   {PORTD |= 0b10110000;} // Liga 4, 5, 7

#define LATCH(M_RECK)   {PORTB |= M_RECK; PORTB &= ~M_RECK;}
#define CLOCK(M_SECK)   {PORTD |= M_SECK; PORTD &= ~M_SECK;}

class LedPanel {
public:
    
    LedPanel(
        uint8_t seck  = 2,
        uint8_t seri  = 3,
        uint8_t line0 = 4,
        uint8_t line1 = 7,
        uint8_t line2 = 5,
        uint8_t sey   = 9,
        uint8_t sex   = 6,
        uint8_t reck  = 8);
    
    /**
     * @brief Specifies the dimensions (width and height) of the display.
     */
    void begin();
    
    /**
     * Scan buffer to serial for debug
     */
    void scan2serial();
    void scan(int xini = 0, int yini = 0);
    void clear();
    void enqueueChar(byte c, int xPos = 0, int yPos = 0);
    
    // Graphic functions
    void line(int x0, int y0, int x1, int y1, boolean mode = true);
    void rect(int x0, int y0, int x1, int y1, boolean mode = true);
    void circle(int xm, int ym, int r, boolean mode = true);
    void ellipse(int x0, int y0, int x1, int y1, boolean mode = true);
    void bezier(int x0, int y0, int x1, int y1, int x2, int y2, boolean mode = true);
    
private:
    
    uint8_t _seck_mask;
    uint8_t _seri_mask;
    uint8_t _line0_mask;
    uint8_t _line1_mask;
    uint8_t _line2_mask;
    uint8_t _sey_mask;
    uint8_t _sex_mask;
    uint8_t _reck_mask;
    uint8_t _buffer[WINDOW_H][NUMVARS];  //Buffer
    
    void fastSelectLine(uint8_t n);
    void drawline(int x1, int y1, int x2, int y2, boolean clearing = false);
    void spitbit(boolean b);
    void setbit(TYPEOF buffer[], byte pos, boolean value);
    boolean readbit(TYPEOF buffer[], byte pos);
    
    void initPin(uint8_t *pro, uint8_t pin);
    void setuptimer(int divider=630);
    
    /* fast integer (1 byte) modulus */
    // http://code.google.com/p/ht1632c/wiki/Optimizations
    byte _mod(byte n, byte d);
    byte _div(byte n, byte d);
    byte _rnd(byte min, byte max);
    uint8_t _pow(uint8_t a, uint8_t b);
};

#endif