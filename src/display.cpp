#include "display.h"
#include <U8g2lib.h>


U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2(U8G2_R0);
int fontHeight;
int fontAscent;
int cursorY;

void setupDisplay(const char firmwareVersion[])
{
  u8g2.begin();
  u8g2.enableUTF8Print(); // enable UTF8 support for the Arduino print() function
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_tom_thumb_4x6_tf); // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
  updateFontParameters();
  cursorY = fontAscent;
  u8g2.setCursor(0, cursorY);
  u8g2.print(F("setup     "));
  u8g2.print(firmwareVersion);
  underline(64);
  u8g2.sendBuffer();
}

void moveCursorTop()
{
  cursorY = fontAscent;
  u8g2.setCursor(0, cursorY);
}

void moveCursor(int x, int y)
{
  cursorY = y;
  u8g2.setCursor(x, y);
}

void newLine()
{
  cursorY += fontHeight + 1;
  u8g2.setCursor(0, cursorY);
}

void underline(int width)
{
  moveCursor(0, cursorY + 1);
  u8g2.drawHLine(0, cursorY, width);
}

void updateFontParameters()
{
  fontHeight = u8g2.getMaxCharHeight();
  fontAscent = u8g2.getAscent();
}

void printDot()
{
  u8g2.print(F("."));
  u8g2.sendBuffer();
}
