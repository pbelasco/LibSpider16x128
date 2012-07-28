#include "LedPanel.h"

// LedPanel init
LedPanel lp;

void setup () {
    Serial.begin(9600);
    Serial.write("Power on\n");
    lp.begin();
}

char ch;
int chCnt, xpos, ypos, i, j, k = 0;
void loop () {
    for(i=0;i<16;i++){
        lp.enqueueChar((i%8)+48, xpos, ypos);
        lp.line(10, i,  WINDOW_W-10, 16-i);
        delay(50);
        lp.clear();   
    }
    
    byte r=14;
    for(i=0; i<WINDOW_W-1-r;i++) {
        lp.rect(i,0,r+i,r);
        delay(5);
        lp.clear();
    }
    for(i=WINDOW_W-1-r; i>0;i--) {
        lp.rect(i,0,r+i,r);
        delay(5);
        lp.clear();
    }
    
    r=7;
    for(i=0; i<WINDOW_W-1-r;i++) {
        lp.circle(r+i, r,r);
        delay(5);
        lp.clear();
    }
    for(i=WINDOW_W-1-r; i>0;i--) {
        lp.circle(r+i,r,r);
        delay(5);
        lp.clear();
    }
    
    delay(50);
    lp.clear();
    
    for(i=0;i<15; i++) {
        lp.bezier(0,0, 64+i,i, 127,0);
        delay(50);
        lp.clear();
    } 
    for(i=15;i>0; i--) {
      lp.bezier(0,0, 64-i,i, 127,0);
      delay(50);
      lp.clear();
    }
        
    // while(Serial.available()) {
    //     // Serial.read();
    //     // get the new byte:
    //     int c = Serial.read();
    //     if (c == 127) {
    //         c = ' ';
    //         if (chCnt > 0) --chCnt;
    //         if (xpos > 0)  xpos -= 6;
    //         lp.enqueueChar(c, xpos, ypos);
    //     } else {
    //         lp.enqueueChar(c, xpos, ypos);
    //         xpos += 6;
    //         ++chCnt;
    //         if ((chCnt%20) == 0) { ypos += 8; xpos = 0; }
    //     }
    // }
    // delay(100);
    
    // lp.scan2serial();
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
ISR(TIMER1_COMPA_vect) { lp.scan(); }
