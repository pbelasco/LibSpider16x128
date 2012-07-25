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

#include "SystemFont5x7.h"
#include "messages.h"
//Definição de constantes de fontes
#define FONT            System5x7
#define CHAR_WIDTH_5x7  2
#define CHAR_HEIGHT_5x7 3
#define FIRST_CHAR_5x7  4
#define FONT_COUNT_5x7  5
#define START_CHARS_5x7 6
//Constantes de Buffer
#define TYPEOF          uint8_t
#define BITSIZE         8
#define BITSIZE_MINONE  7
#define NUMVARS         16 //Buffer size/Bits per var

#define WINDOW_W	128
#define WINDOW_H	16
#define SWITCH_TO_Y()   {PORTD &= ~MASK_SEX; PORTB |=  MASK_SEY; } 
#define SWITCH_TO_X()   {PORTD |=  MASK_SEX; PORTB &= ~MASK_SEY; } 
#define SWITCH_L_ON()   {PORTD |= 0b10110000;} // Liga 4, 5, 7

#define LATCH()         {PORTB |= MASK_RECK; PORTB &= ~MASK_RECK;}
#define CLOCK()         {PORTD |= MASK_SECK; PORTD &= ~MASK_SECK;}
#define MASK_SECK  	0b00000100 //2 PORTD
#define MASK_SERI  	0b00001000 //3 PORTD
#define MASK_L0    	0b00010000 //4 PORTD
#define MASK_L2    	0b00100000 //5 PORTD
#define MASK_SEX   	0b01000000 //6 PORTD
#define MASK_L1    	0b10000000 //7 PORTD
#define MASK_RECK  	0b00000001 //8 PORTB
#define MASK_SEY   	0b00000010 //9 PORTB

#define L0             	4
#define L1             	7
#define L2             	5
#define SEX            	6
#define SEY            	9
#define SERI           	3
#define SECK           	2
#define RECK           	8

/*Fasts Rand*/
byte rnd(byte min, byte max)
{
  static byte seed;
  seed = (21 * seed + 21);
  return min + _mod(seed, --max);
}

/* fast integer (1 byte) modulus */
//http://code.google.com/p/ht1632c/wiki/Optimizations
byte _mod(byte n, byte d) {
  while(n >= d) n -= d;
  return n;
}

/* fast integer (1 byte) division */
byte _div(byte n, byte d) {
  byte q = 0;
  while(n >= d) {
    n -= d;
    q++;
  }
  return q;
}

//Variveis Globais 
TYPEOF buffer[WINDOW_H][NUMVARS];  //Buffer

//Desenha o buffer na saida, tomando 
//como [0,0] do display, os pontos iniciais e finais 
void scan(int xini=0, int yini=0) {
  int i, j = 0;
  for(i=0; i<WINDOW_H; i++) {
    //fast_selectline(i);    
    fast_selectline(i);
    for(j=0; j<WINDOW_W; j++){
      spitbit(readbit(buffer[i], j));
    } 
    LATCH();
  }
}


void selectline(byte n) {
//n=n<<1;
  digitalWrite(L0, (n & 0x01) ? HIGH : LOW);
  digitalWrite(L1, (n & 0x02) ? HIGH : LOW);
  digitalWrite(L2, (n & 0x04) ? HIGH : LOW);
  digitalWrite(SEX, (n & 0x08) ? HIGH : LOW);
  digitalWrite(SEY, !(n & 0x08) ? HIGH : LOW);
  
}

//Seleciona uma linha com base 
//nos pinos SEX, SEY, L0, L1, L2
void fast_selectline(uint8_t n) {
  (n<7)?n+=7:n-=9; //altera ordem de impressao
  SWITCH_L_ON();
  SWITCH_TO_Y()
  if (!(n & 0x08)) SWITCH_TO_X();
  if (!(n & 0x01)) PORTD &= ~MASK_L0; //4
  if (!(n & 0x02)) PORTD &= ~MASK_L1; //7
  if (!(n & 0x04)) PORTD &= ~MASK_L2; //5
}
void spitbit(boolean b){
  //escreve status na porta 3

  PORTD |= MASK_SERI; //porta goes high
  if(!b)
    PORTD &= ~MASK_SERI; //porta para low
        delayMicroseconds(2);
  CLOCK();
}

void clear() {
  for (int j=0; j<WINDOW_H; j++)
    for (int i=0; i<NUMVARS; i++)
      buffer[j][i]=0;
}

void drawline(int x1,int y1,int x2,int y2, boolean clearing=false) {
  int x,y;
  for(x=x1; x<=x2; x++) {
      y=y1+(((y2-y1)/(x2-x1))*(x-x1));
      setbit(buffer[y], x, clearing ? true : false );
    }
} 
void enqueueChar5x7 (byte c, int xPos=0, int yPos=0)
{
  byte j, i;			                    //bitmask and iterators
  uint8_t off = pgm_read_byte (FONT + FIRST_CHAR_5x7); //offset of charset
  uint8_t w = pgm_read_byte (FONT + CHAR_WIDTH_5x7);   //char width
  uint8_t h = pgm_read_byte (FONT + CHAR_HEIGHT_5x7);  //char height
  int zc = int(FONT) + START_CHARS_5x7;	                //index of 0 char
  for (i = 0; i < h; i++) { 
    for (j = 0; j < w; j++) {
        setbit(buffer[i+yPos],j+xPos,(pgm_read_byte(zc+((c-off)*w)+j) & (1<<i)));
    }
  }
}

boolean readbit (TYPEOF buffer[], byte pos) {
   //return (buffer[pos/BITSIZE] & (0x80 >> (pos%BITSIZE)));  
   //return (buffer[pos/BITSIZE] & (1<< BITSIZE-1-(pos%BITSIZE)));  
   return (buffer[_div(pos,BITSIZE)] & (0x80 >> (_mod(pos,BITSIZE))));
}

void setbit(TYPEOF buffer[], byte pos, boolean value) {
  if (value)
    buffer[pos/BITSIZE] |=  (0x80 >> (pos%BITSIZE));
  else
    buffer[pos/BITSIZE] &= ~(0x80 >> (pos%BITSIZE));
}


/* graphic primitives based on Bresenham's algorithms */

void line(int x0, int y0, int x1, int y1, boolean mode=true)
{
  int dx =  abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1; 
  int err = dx + dy, e2; /* error value e_xy */

  for(;;) {
    setbit(buffer[y0], x0, mode);
    if (x0 == x1 && y0 == y1) break;
    e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; } /* e_xy+e_x > 0 */
    if (e2 <= dx) { err += dx; y0 += sy; } /* e_xy+e_y < 0 */
  }
}

void rect(int x0, int y0, int x1, int y1, boolean mode=true)
{
  line(x0, y0, x0, y1, mode); /* left line   */
  line(x1, y0, x1, y1, mode); /* right line  */
  line(x0, y0, x1, y0, mode); /* top line    */
  line(x0, y1, x1, y1, mode); /* bottom line */
}

void circle(int xm, int ym, int r, boolean mode=true)
{
  int x = -r, y = 0, err = 2 - 2 * r; /* II. Quadrant */ 
  do {
    setbit(buffer[ym + y], xm - x, mode); /*   I. Quadrant */
    setbit(buffer[ym - x], xm - y, mode); /*  II. Quadrant */
    setbit(buffer[ym - y], xm + x, mode); /* III. Quadrant */
    setbit(buffer[ym + x], xm + y, mode); /*  IV. Quadrant */
    r = err;
    if (r >  x) err += ++x * 2 + 1; /* e_xy+e_x > 0 */
    if (r <= y) err += ++y * 2 + 1; /* e_xy+e_y < 0 */
  } while (x < 0);
}

void ellipse(int x0, int y0, int x1, int y1, boolean mode=true)
{
  int a = abs(x1 - x0), b = abs(y1 - y0), b1 = b & 1; /* values of diameter */
  long dx = 4 * (1 - a) * b * b, dy = 4 * (b1 + 1) * a * a; /* error increment */
  long err = dx + dy + b1 * a * a, e2; /* error of 1.step */

  if (x0 > x1) { x0 = x1; x1 += a; } /* if called with swapped points */
  if (y0 > y1) y0 = y1; /* .. exchange them */
  y0 += (b + 1) / 2; /* starting pixel */
  y1 = y0 - b1;
  a *= 8 * a; 
  b1 = 8 * b * b;

  do {
    setbit(buffer[y0], x1, mode); /*   I. Quadrant */
    setbit(buffer[y0], x0, mode); /*  II. Quadrant */
    setbit(buffer[y1], x0, mode); /* III. Quadrant */
    setbit(buffer[y1], x1, mode); /*  IV. Quadrant */
    e2 = 2 * err;
    if (e2 >= dx) { x0++; x1--; err += dx += b1; } /* x step */
    if (e2 <= dy) { y0++; y1--; err += dy += a; }  /* y step */ 
  } while (x0 <= x1);

  while (y0 - y1 < b) {  /* too early stop of flat ellipses a=1 */
    setbit(buffer[++y0], x0 - 1, mode); /* -> complete tip of ellipse */
    setbit(buffer[--y1], x0 - 1, mode); 
  }
}

void bezier(int x0, int y0, int x1, int y1, int x2, int y2, boolean mode=true)
{
  int sx = x0 < x2 ? 1 : -1, sy = y0 < y2 ? 1 : -1; /* step direction */
  int cur = sx * sy * ((x0 - x1) * (y2 - y1) - (x2 - x1) * (y0 - y1)); /* curvature */
  int x = x0 - 2 * x1 + x2, y = y0 - 2 * y1 + y2, xy = 2 * x * y * sx * sy;
                                /* compute error increments of P0 */
  long dx = (1 - 2 * abs(x0 - x1)) * y * y + abs(y0 - y1) * xy - 2 * cur * abs(y0 - y2);
  long dy = (1 - 2 * abs(y0 - y1)) * x * x + abs(x0 - x1) * xy + 2 * cur * abs(x0 - x2);
                                /* compute error increments of P2 */
  long ex = (1 - 2 * abs(x2 - x1)) * y * y + abs(y2 - y1) * xy + 2 * cur * abs(y0 - y2);
  long ey = (1 - 2 * abs(y2 - y1)) * x * x + abs(x2 - x1) * xy - 2 * cur * abs(x0 - x2);

  if (cur == 0) { line(x0, y0, x2, y2, mode); return; } /* straight line */

  x *= 2 * x; y *= 2 * y;
  if (cur < 0) {                             /* negated curvature */
    x = -x; dx = -dx; ex = -ex; xy = -xy;
    y = -y; dy = -dy; ey = -ey;
  }
  /* algorithm fails for almost straight line, check error values */
  if (dx >= -y || dy <= -x || ex <= -y || ey >= -x) {        
    line(x0, y0, x1, y1, mode);                /* simple approximation */
    line(x1, y1, x2, y2, mode);
    return;
  }
  dx -= xy; ex = dx+dy; dy -= xy;              /* error of 1.step */

  for(;;) {                                         /* plot curve */
    setbit(buffer[y0], x0, mode);
    ey = 2 * ex - dy;                /* save value for test of y step */
    if (2 * ex >= dx) {                                   /* x step */
      if (x0 == x2) break;
      x0 += sx; dy -= xy; ex += dx += y; 
    }
    if (ey <= 0) {                                      /* y step */
      if (y0 == y2) break;
      y0 += sy; dx -= xy; ex += dy += x; 
    }
  }
}
void setuptimer(int divider=630) {
       // initialize Timer1
    cli();          // disable global interrupts
    TCCR1A = 0;     // set entire TCCR1A register to 0
    TCCR1B = 0;     // same for TCCR1B
    // set compare match register to desired timer count:
    OCR1A = divider;
    // turn on CTC mode:
    TCCR1B |= (1 << WGM12);
    // Set CS10 and CS12 bits for 1024 prescaler:
    TCCR1B |= (1 << CS10);
        // Set ONLY CS12 bits for 256 prescaler:
    TCCR1B |= (1 << CS12);
    // enable timer compare interrupt:
    TIMSK1 |= (1 << OCIE1A);
    // enable global interrupts:
    sei();
}

void setup () {
  for(byte p = 2; p <=9; p++ ) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  Serial.begin(9600);
  setuptimer(250);
  clear();
}

char ch;
int chCnt, xpos, ypos, i, j, k = 0;
void loop () {
  for(i=0;i<16;i++){
    enqueueChar5x7((i%8)+48, xpos, ypos);
    line(10, i,  WINDOW_W-10, 16-i);
    delay(50);
    clear();   
  }
  byte r=14;
  for(i=0; i<WINDOW_W-1-r;i++) {
    rect(i,0,r+i,r);
    delay(5);
    clear();
  }
    for(i=WINDOW_W-1-r; i>0;i--) {
    rect(i,0,r+i,r);
    delay(5);
    clear();
  }
  
   r=7;
  for(i=0; i<WINDOW_W-1-r;i++) {
    circle(r+i, r,r);
    delay(5);
    clear();
  }
    for(i=WINDOW_W-1-r; i>0;i--) {
    circle(r+i,r,r);
    delay(5);
    clear();
  }
  
      delay(50);
    clear();
  
  for(i=0;i<15; i++) {
    bezier(0,0, 64+i,i, 127,0);
    delay(50);
    clear();
  } 
    for(i=15;i>0; i--) {
    bezier(0,0, 64-i,i, 127,0);
    delay(50);
    clear();
  } 
  while  (Serial.available()) {
    // get the new byte:
    enqueueChar5x7((char)Serial.read(), xpos, ypos);
    xpos += 6;
    ++chCnt;
    if ((chCnt%20) == 0) { ypos += 8; xpos = 0; }
  }
  
   /*
  xpos=0;
  ypos=0;
  for (j=0; j<36; j++) {
    for (i=j+20;i<j+60;i++) {
      enqueueChar5x7(i, xpos, ypos);
      xpos += 6;
      ++chCnt;
      if ((chCnt%20) == 0) { ypos += 8; xpos = 0; }
  clear();
    }
  }
  //scan2serial();
  delay(1000);
  */

}

//Executes on ach TIMER1 overflow.
//Set divider in setuptimer()
ISR(TIMER1_COMPA_vect) { scan(); }


void scan2serial(){
  int i, j = 0;
  for(i=0; i<WINDOW_H; i++) {   
    for(j=0; j<WINDOW_W; j++){
      Serial.print(readbit(buffer[i], j) ? '#' : '-');
    } 
   Serial.print('\n');
  }
  Serial.print('\n');
}



