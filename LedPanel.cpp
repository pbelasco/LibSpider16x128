// #include <math.h>

#include "SystemFont5x7.h"
#include "LedPanel.h"

LedPanel::LedPanel(uint8_t seck, uint8_t seri,
        uint8_t line0, uint8_t line1, uint8_t line2,
        uint8_t sey, uint8_t sex, uint8_t reck)
{
    initPin(&_seck_mask , seck);
    initPin(&_seri_mask , seri);
    initPin(&_line0_mask, line0);
    initPin(&_line1_mask, line1);
    initPin(&_line2_mask, line2);
    initPin(&_sey_mask  , sey);
    initPin(&_sex_mask  , sex);
    initPin(&_reck_mask , reck);
}

void LedPanel::initPin(uint8_t *mask_pro, uint8_t pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    *mask_pro = _pow(2, (pin > 7 ? (pin - (_pow(8, pin / 8))) : pin));
}

/**
 * @todo: Transfer setuptimer for this
 */
void LedPanel::begin() {
    setuptimer(250);
    clear();
}

//Desenha o buffer na saida, tomando 
//como [0,0] do display, os pontos iniciais e finais 
void LedPanel::scan(int xini, int yini) {
    Serial.write("scan");
    int i, j = 0;
    for(i = 0; i < WINDOW_H; i++) {
        fastSelectLine(i);
        for(j = 0; j < WINDOW_W; j++){
            spitbit(readbit(_buffer[i], j));
        }
        LATCH(_reck_mask);
    }
}

// Seleciona uma linha com base 
// nos pinos SEX, SEY, L0, L1, L2
void LedPanel::fastSelectLine(uint8_t n) {
    (n < 7) ? n += 7 : n -= 9; //altera ordem de impressao
    SWITCH_L_ON();
    SWITCH_TO_Y(_sex_mask, _sey_mask);
    if (!(n & 0x08)) SWITCH_TO_X(_sex_mask, _sey_mask);
    if (!(n & 0x01)) PORTD &= ~_line0_mask; //4
    if (!(n & 0x02)) PORTD &= ~_line1_mask; //7
    if (!(n & 0x04)) PORTD &= ~_line2_mask; //5
}

// escreve status na porta 3
void LedPanel::spitbit(boolean b) {
    PORTD |= _seri_mask; // porta goes high
    if(!b)
        PORTD &= ~_seri_mask; //porta para low
            delayMicroseconds(2);
    CLOCK(_seck_mask);
}

void LedPanel::clear() {
    for (int j = 0; j < WINDOW_H; j++)
        for (int i = 0; i < NUMVARS; i++)
            _buffer[j][i] = 0;
}

void LedPanel::drawline(int x1, int y1, int x2, int y2, boolean clearing) {
    int x,y;
    for(x = x1; x <= x2; x++) {
        y = y1+(((y2-y1)/(x2-x1))*(x-x1));
        setbit(_buffer[y], x, clearing ? true : false );
    }
}

void LedPanel::enqueueChar(byte c, int xPos, int yPos) {
    byte j, i;                                           //bitmask and iterators
    uint8_t off = pgm_read_byte (FONT + FIRST_CHAR_5x7); //offset of charset
    uint8_t w = pgm_read_byte (FONT + CHAR_WIDTH_5x7);   //char width
    uint8_t h = pgm_read_byte (FONT + CHAR_HEIGHT_5x7);  //char height
    int zc = int(FONT) + START_CHARS_5x7;	             //index of 0 char
    for (i = 0; i < h; i++) { 
        for (j = 0; j < w; j++) {
            setbit(_buffer[i+yPos],j+xPos,(pgm_read_byte(zc+((c-off)*w)+j) & (1<<i)));
        }
    }
}

boolean LedPanel::readbit(TYPEOF buffer[], byte pos) {
    return (buffer[_div(pos, BITSIZE)] & (0x80 >> (_mod(pos, BITSIZE))));
}

void LedPanel::setbit(TYPEOF buffer[], byte pos, boolean value) {
    if (value)
        buffer[pos/BITSIZE] |=  (0x80 >> (pos%BITSIZE));
    else
        buffer[pos/BITSIZE] &= ~(0x80 >> (pos%BITSIZE));
}

/**
 * Math fast functions
 */
byte LedPanel::_mod(byte n, byte d) {
    while(n >= d) n -= d;
    return n;
}

/* fast integer (1 byte) division */
byte LedPanel::_div(byte n, byte d) {
    byte q = 0;
    while(n >= d) {
        n -= d;
        q++;
    }
    return q;
}

byte LedPanel::_rnd(byte min, byte max) {
    static byte seed;
    seed = (21 * seed + 21);
    return min + _mod(seed, --max);
}

uint8_t LedPanel::_pow(uint8_t a, uint8_t b) {
    if (b==0) return 1;
    if (a==0) return 0;
    if (b%2==0) { return _pow(a * a, b/2); }
    else if (b%2==1) { return a * _pow(a * a, b/2); } 
    return 0;
}

/* graphic primitives based on Bresenham's algorithms */
void LedPanel::line(int x0, int y0, int x1, int y1, boolean mode) {
    int dx =  abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1; 
    int err = dx + dy, e2; /* error value e_xy */
    
    for(;;) {
        setbit(_buffer[y0], x0, mode);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; } /* e_xy+e_x > 0 */
        if (e2 <= dx) { err += dx; y0 += sy; } /* e_xy+e_y < 0 */
    }
}

void LedPanel::rect(int x0, int y0, int x1, int y1, boolean mode) {
    line(x0, y0, x0, y1, mode); /* left line   */
    line(x1, y0, x1, y1, mode); /* right line  */
    line(x0, y0, x1, y0, mode); /* top line    */
    line(x0, y1, x1, y1, mode); /* bottom line */
}

void LedPanel::circle(int xm, int ym, int r, boolean mode) {
    int x = -r, y = 0, err = 2 - 2 * r; /* II. Quadrant */ 
    do {
        setbit(_buffer[ym + y], xm - x, mode); /*   I. Quadrant */
        setbit(_buffer[ym - x], xm - y, mode); /*  II. Quadrant */
        setbit(_buffer[ym - y], xm + x, mode); /* III. Quadrant */
        setbit(_buffer[ym + x], xm + y, mode); /*  IV. Quadrant */
        r = err;
        if (r >  x) err += ++x * 2 + 1; /* e_xy+e_x > 0 */
        if (r <= y) err += ++y * 2 + 1; /* e_xy+e_y < 0 */
    } while (x < 0);
}

void LedPanel::ellipse(int x0, int y0, int x1, int y1, boolean mode) {
    int a    = abs(x1 - x0), b = abs(y1 - y0), b1 = b & 1; /* values of diameter */
    long dx  = 4 * (1 - a) * b * b, dy = 4 * (b1 + 1) * a * a; /* error increment */
    long err = dx + dy + b1 * a * a, e2; /* error of 1.step */
    
    if (x0 > x1) { x0 = x1; x1 += a; } /* if called with swapped points */
    if (y0 > y1) y0 = y1; /* .. exchange them */
    y0 += (b + 1) / 2; /* starting pixel */
    y1 = y0 - b1;
    a *= 8 * a; 
    b1 = 8 * b * b;
    
    do {
        setbit(_buffer[y0], x1, mode); /*   I. Quadrant */
        setbit(_buffer[y0], x0, mode); /*  II. Quadrant */
        setbit(_buffer[y1], x0, mode); /* III. Quadrant */
        setbit(_buffer[y1], x1, mode); /*  IV. Quadrant */
        e2 = 2 * err;
        if (e2 >= dx) { x0++; x1--; err += dx += b1; } /* x step */
        if (e2 <= dy) { y0++; y1--; err += dy += a; }  /* y step */ 
    } while (x0 <= x1);
    
    while (y0 - y1 < b) {  /* too early stop of flat ellipses a=1 */
        setbit(_buffer[++y0], x0 - 1, mode); /* -> complete tip of ellipse */
        setbit(_buffer[--y1], x0 - 1, mode); 
    }
}

void LedPanel::bezier(int x0, int y0, int x1, int y1, int x2, int y2, boolean mode) {
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
        setbit(_buffer[y0], x0, mode);
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

// initialize Timer1
void LedPanel::setuptimer(int divider) {
    cli();           // disable global interrupts
    
    TCCR1A = 0;      // set entire TCCR1A register to 0
    TCCR1B = 0;      // same for TCCR1B
    OCR1A = divider; // set compare match register to desired timer count:
    
    TCCR1B |= (1 << WGM12);  // turn on CTC mode:
    TCCR1B |= (1 << CS10);   // Set CS10 and CS12 bits for 1024 prescaler:
    TCCR1B |= (1 << CS12);   // Set ONLY CS12 bits for 256 prescaler:
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt:
    
    sei(); // enable global interrupts:
}

// Scan buffer to serial for debug
void LedPanel::scan2serial() {
    int i, j = 0;
    for(i = 0; i < WINDOW_H; i++) {   
        for(j=0; j < WINDOW_W; j++){
            Serial.print(readbit(_buffer[i], j) ? '#' : '-');
        }
        Serial.print('\n');
    }
    Serial.print('\n');
}