#ifndef DISPLAY_H
#define DISPLAY_H

#include <U8g2lib.h>

extern U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2;
extern int fontHeight;
extern int fontAscent;
extern int cursorY;

void setupDisplay(const char[]);
void moveCursorTop();
void moveCursor(int, int);
void newLine();
void underline(int);
void updateFontParameters();
void printDot();

#endif
