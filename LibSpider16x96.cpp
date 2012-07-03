#include <SystemFont5x7.h>
//Definição de constantes de fontes
#define FONT            System5x7
#define CHAR_WIDTH_5x7  2
#define CHAR_HEIGHT_5x7 3
#define FIRST_CHAR_5x7  4
#define FONT_COUNT_5x7  5
#define START_CHARS_5x7 6
//Constantes de Buffer
#define TYPEOF         uint8_t
#define BITSIZE        8
#define NUMVARS        16
#define NUMROWS        16

#define MASK_PIN_3     B00001000
#define MASK_PIN_2     B00000100
#define MASK_PIN_8     B00000001
#define L0             4
#define L1             7
#define L2             5
#define SEX            6
#define SEY            9
#define SERI           3
#define SECK           2
#define RECK           8

void printbufferSerial(TYPEOF m[][NUMVARS], int xini=0, int yini=0) {
  int i, j = 0;
  for(i=0; i<NUMROWS; i++) {
    for(j=0; j<BITSIZE*NUMVARS; j++) { 
      Serial.print(readbit(m[i+xini], j+yini) ? '#' : '.' );
    }
   Serial.println(' ');
  }
  Serial.println(' ');
}

void printbuffer(TYPEOF m[][NUMVARS], int xini=0, int yini=0) {
  int i, j = 0;
  for(i=yini; i<NUMROWS; i++) {
    selectline(i);
    for(j=xini; j<NUMVARS*BITSIZE; j++){
      spitbit(readbit(m[i], j));
    } 
    latch();
  }
}

void selectline(byte n) {
  digitalWrite(SEX, (n & 0x08) ? HIGH : LOW);
  digitalWrite(SEY, (n & 0x08) ? LOW : HIGH);
  digitalWrite(L0,  (n & 0x01) ? HIGH : LOW);
  digitalWrite(L1,  (n & 0x02) ? HIGH : LOW);
  digitalWrite(L2,  (n & 0x04) ? HIGH : LOW); 
}
void spitbit(boolean b){
  //escreve status na porta 3
  if(b == true) PORTD ^= MASK_PIN_3;
  clock();
  PORTD &= ~MASK_PIN_3; //Retorna a porta para low
}
void clock(){
  PORTD &=  ~MASK_PIN_2;
  PORTD ^=   MASK_PIN_2;
}

void latch(){
  PORTB ^= MASK_PIN_8; 
  PORTB &= ~MASK_PIN_8;
}

void clearbuffer (TYPEOF buffer[][NUMVARS]) {
  for (int j=0; j<NUMROWS; j++)
    for (int i=0; i<NUMVARS; i++)
      buffer[j][i]=0;
}

void invertbuffer (TYPEOF buffer[][NUMVARS]) {
  for (int j=0; j<NUMROWS; j++)
    for (int i=0; i<NUMVARS*BITSIZE; i++)
      setbit(buffer[j], readbit(buffer[j], i) ? false : true, i);
}

void drawline(TYPEOF m[][NUMVARS],int x1,int y1,int x2,int y2) {
  clearbuffer(m);
  int x,y;
  for(x=x1; x<=x2; x++) //we follow x coord. one-by-one
    {
      y=y1+(((y2-y1)/(x2-x1))*(x-x1)); //y is the same precentage of trip x made on its trip
      setbit(m[y], x, true);
    }
} 

void enqueueChar5x7 (byte c, TYPEOF buffer[][NUMVARS], int xPos=0, int yPos=0)
{
  uint8_t m, j, i;			                    //bitmask and iterators
  uint8_t off = pgm_read_byte (FONT + FIRST_CHAR_5x7); //offset of charset
  uint8_t w = pgm_read_byte (FONT + CHAR_WIDTH_5x7);   //char width
  uint8_t h = pgm_read_byte (FONT + CHAR_HEIGHT_5x7);  //char height
  int zc = int(FONT) + START_CHARS_5x7;	                //index of 0 char
  for (i = 0; i < h; i++) { 
    for (j = 0; j < w; j++) {
        setbit(buffer[i+yPos],(pgm_read_byte(zc+((c-off)*w)+j) & (1<<i)) ? true : false, j+xPos );
    }
  }
}

boolean readbit (TYPEOF buffer[], int pos) {
  return (buffer[pos/BITSIZE] & (1 << (BITSIZE-(pos%BITSIZE))-1)) ? true : false;
}

void setbit(TYPEOF buffer[], boolean value, int pos) {
  TYPEOF bitmask = (1 << BITSIZE-(pos%BITSIZE)-1);
  if (value == true) {
      buffer[pos/BITSIZE] |= bitmask;
  } else if (value == false) {
    buffer[pos/BITSIZE] &= ~bitmask;
  }
}

//Variveis Globais 
TYPEOF buffer[NUMROWS][NUMVARS];  //Buffer

void setup () {
  for(byte p = 2; p <=9; p++ ) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  clearbuffer(buffer);
}

char ch;
int xpos, ypos = 0;
int i, j;
void loop () {
  clearbuffer(buffer);
  for (i=0;i<NUMROWS; i++) {
    for (j=0;j<NUMVARS*BITSIZE; j++) {
      //drawline(buffer, j- NUMVARS*BITSIZE, i, NUMVARS*BITSIZE - j, i);
      setbit(buffer[i],j,true);
      printbuffer(buffer);
    }
  }
}

