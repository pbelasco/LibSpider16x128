#ifndef LedPanelText_h
#define LedPanelText_h

#include "LedPanel.h"
#include "Print.h"
#include "SystemFont5x7.h"

// #define TEXT_PER_LINE 18
// #define TEXT_LINES 2
// char textBuffer[TEXT_PER_LINE * TEXT_LINES];  //Buffer

class LedPanelText : public Print, public LedPanel {
public:
    LedPanelText();
    
    /**
     * Clears the LedPanel screen and positions the cursor in the upper-left corner.
     * 
     */
    void clear();
    
    /**
     * Positions the cursor in the upper-left of the LedPanel.
     * That is, use that location in outputting subsequent text to the display.
     * To also clear the display, use the clear() function instead.
     */
    void home();
    
    /**
     * Turns off the LCD display, without losing the text currently shown on it.
     */
    void noDisplay();
    
    /**
     * Turns on the LCD display, after it's been turned off with noDisplay().
     * This will restore the text (and cursor) that was on the display.
     */
    void display();
    
    /**
     * Turns off the blinking LCD cursor.
     */
    void noBlink();
    
    /**
     * Display the blinking LCD cursor.
     * If used in combination with the cursor() function, the result will depend on the particular display.
     */
    void blink();
    
    /**
     * Display the LCD cursor: an underscore (line) at the position to which the next character will be written.
     */
    void noCursor();
    
    /**
     * Hides the LCD cursor.
     */
    void cursor();
    
    /**
     * Scrolls the contents of the display (text and cursor) one space to the left.
     */
    void scrollDisplayLeft();
    
    /**
     * Scrolls the contents of the display (text and cursor) one space to the right.
     */
    void scrollDisplayRight();
    
    /**
     * Set the direction for text written to the LCD to left-to-right, the default. 
     * This means that subsequent characters written to the display will go from left to right,
     * but does not affect previously-output text.
     */
    void leftToRight();
    
    /**
     * Set the direction for text written to the LCD to right-to-left (the default is left-to-right).
     * This means that subsequent characters written to the display will go from right to left,
     * but does not affect previously-output text.
     */
    void rightToLeft();
    
    /**
     * Turns on automatic scrolling of the LCD.
     * This causes each character output to the display to push previous characters over by one space.
     * If the current text direction is left-to-right (the default), the display scrolls to the left;
     * if the current direction is right-to-left, the display scrolls to the right.
     * This has the effect of outputting each new character to the same location on the LCD.
     */
    void autoscroll();
    
    /**
     * Turns off automatic scrolling of the LCD.
     */
    void noAutoscroll();

    /**
     * Create a custom character (gylph) for use on the LCD.
     * Up to eight characters of 5x8 pixels are supported (numbered 0 to 7).
     * The appearance of each custom character is specified by an array of seven bytes, one for each row.
     * The five least significant bits of each byte determine the pixels in that row.
     * To display a custom character on the screen, write() its number.
     * @param num which character to create (0 to 7)
     * @param the character's pixel data
     */
    void createChar(uint8_t num, uint8_t data[]);
    
    /**
     * Position the LCD cursor; that is, set the location at which subsequent text written to the LCD will be displayed.
     */
    void setCursor(uint8_t, uint8_t);
    
    virtual size_t write(uint8_t);    
    void command(uint8_t);
};

#endif