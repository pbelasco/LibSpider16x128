#include <SystemFont5x7.h>
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
#define NUMVARS         32 //Buffer size/Bits per var

#define WINDOW_W	128
#define WINDOW_H	16


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

//Variveis Globais 
TYPEOF buffer[WINDOW_H][NUMVARS];  //Buffer

void printbufferSerial(TYPEOF m[][NUMVARS], int xini=0, int yini=0) {
  int i, j = 0;
  for(i=0; i<WINDOW_H; i++) {
    for(j=0; j<BITSIZE*NUMVARS; j++) { 
      Serial.print(readbit(m[i+xini], j+yini) ? '#' : '.' );
    }
   Serial.println(' ');
  }
  Serial.println(' ');
}

void printbuffer(TYPEOF m[][NUMVARS], int xini=0, int yini=0) {
  int i, j = 0;
  for(i=0; i<16; i++) {
    fast_selectline(i);
    for(j=0; j<128; j++){
      spitbit(readbit(m[i], j));
    } 
    latch();
  }
}

//Seleciona uma linha com base 
//nos pinos SEX, SEY, L0, L1, L2
void fast_selectline(byte n) {
  --n;
  if (n & 0x08) {
    PORTD |=  MASK_SEX; //6
    PORTB &= ~MASK_SEY; //9
  } else {
    PORTD &= ~MASK_SEX; //6
    PORTB |=  MASK_SEY; //9
  }
  if (n & 0x01) //4
    PORTD |= MASK_L0;
  else
    PORTD &= ~MASK_L0;
    
  if (n & 0x02) //7
    PORTD |= MASK_L1;
  else
    PORTD &= ~MASK_L1;

  if (n & 0x04) //5
    PORTD |= MASK_L2;
  else 
    PORTD &= ~MASK_L2;
}
void spitbit(volatile boolean b){
  //escreve status na porta 3
  if(b) {
    PORTD |= MASK_SERI;
    clock();
    PORTD &= ~MASK_SERI; //Retorna a porta para low
  } else
    clock();
}
void clock(){
  PORTD ^= MASK_SECK;
  PORTD ^= MASK_SECK;
}

void latch(){
  PORTB  ^= MASK_RECK; 
  PORTB  ^= MASK_RECK;
}

void clearbuffer () {
  for (int j=0; j<WINDOW_H; j++)
    for (int i=0; i<NUMVARS; i++)
      buffer[j][i]=0;
}

void drawline(TYPEOF m[][NUMVARS],int x1,int y1,int x2,int y2) {
  clearbuffer();
  int x,y;
  for(x=x1; x<=x2; x++) {
      y=y1+(((y2-y1)/(x2-x1))*(x-x1));
      setbit(m[y], x, true);
    }
} 
void enqueueChar5x7 (byte c, int xPos=0, int yPos=0)
{
  uint8_t m, j, i;			                    //bitmask and iterators
  uint8_t off = pgm_read_byte (FONT + FIRST_CHAR_5x7); //offset of charset
  uint8_t w = pgm_read_byte (FONT + CHAR_WIDTH_5x7);   //char width
  uint8_t h = pgm_read_byte (FONT + CHAR_HEIGHT_5x7);  //char height
  int zc = int(FONT) + START_CHARS_5x7;	                //index of 0 char
  for (i = 0; i < h; i++) { 
    for (j = 0; j < w; j++) {
        setbit(buffer[i+yPos], j+xPos, (pgm_read_byte(zc+((c-off)*w)+j) & (1<<i)));
    }
  }
}

boolean readbit (TYPEOF buffer[], int pos) {
  return (buffer[pos/BITSIZE] & (1 << (BITSIZE_MINONE-(pos%BITSIZE))));
}

void setbit(TYPEOF buffer[], int pos, boolean value) {
  TYPEOF bitmask = (1 << BITSIZE_MINONE-(pos%BITSIZE));
  if (value) {
      buffer[pos/BITSIZE] |= bitmask;
  } else {
    buffer[pos/BITSIZE] &= ~bitmask;
  }
}

void setuptimer(int divider=670) {
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
  setuptimer();
  clearbuffer();
}

char ch;
int chCnt, xpos, ypos, i, j, k = 0;
void loop () {
  while (Serial.available()) {
    // get the new byte:
    enqueueChar5x7((char)Serial.read(), xpos, ypos);
    xpos += 6;
    chCnt++;
    if ((chCnt%19) == 0) { ypos += 8; xpos = 0; }
  }
}

ISR(TIMER1_COMPA_vect)
{
  printbuffer(buffer);
}
