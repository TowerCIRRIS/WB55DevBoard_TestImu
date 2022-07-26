/*!
 * @file teamAt_SSD1306.h
 *
 * This is an adaptation og the Adafruit's SSD1306 library for monochrome
 * OLED displaysto work with DMA spi.  For the standard library check teir website: http://www.adafruit.com/category/63_98
 *
 * This library was debelopped with the Arduino MKRzero, not tested on any other board so far.
 *
 */

#ifndef _TEAMAT_DISPLAY_H_
#define _TEAMAT_DISPLAY_H_

// ONE of the following three lines must be #defined:
//#define SSD1306_128_64 ///< DEPRECTAED: old way to specify 128x64 screen
//#define SSD1306_128_32 ///< DEPRECATED: old way to specify 128x32 screen
//#define SSD1306_96_16  ///< DEPRECATED: old way to specify 96x16 screen
// This establishes the screen dimensions in old Adafruit_SSD1306 sketches
// (NEW CODE SHOULD IGNORE THIS, USE THE CONSTRUCTORS THAT ACCEPT WIDTH
// AND HEIGHT ARGUMENTS).

// #if defined(ARDUINO_STM32_FEATHER)
// typedef class HardwareSPI SPIClass;
// #endif

#include "../GFX_Library/TeamAT_GFX.h"
//#include <SPI.h>
//#include <Wire.h>

#include "at_plaformAbstraction_V1_1.h"


#define PIXEL_ON  1
#define PIXEL_OFF 0
////Data send state machine
//#define SEND_COMMAND_STATE 0
//#define SEND_DATA_STATE    1

/*! \class teamAt_display
    \brief  Class for interacting with generic displays..
*/
class teamAt_display : public TeamAT_GFX {
public:

  teamAt_display( uint16_t w, uint16_t h); 
  //teamAt_display(dmaSpiManagerClass *spiCommPtr, atPin_t dc_pin, atPin_t rst_pin, atPin_t cs_pin, uint16_t w = SSD1306_LCDWIDTH, uint16_t h=SSD1306_LCDHEIGHT);

  ~teamAt_display(void){};

  virtual bool begin()=0;
  
  
  virtual void display(void){};
  virtual void displayOn(void){};
  virtual void displayOff(void){};
  virtual void clearDisplay(uint16_t color){};
  virtual void clearDisplay(void){};
  virtual void invertDisplay(bool i){};
  virtual void dim(bool dim){};
  virtual void drawPixel(int16_t x, int16_t y, uint16_t color){};
  virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color){};
  virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color){};
  virtual void startscrollright(uint8_t start, uint8_t stop){};
  virtual void startscrollleft(uint8_t start, uint8_t stop){};
  virtual void startscrolldiagright(uint8_t start, uint8_t stop){};
  virtual void startscrolldiagleft(uint8_t start, uint8_t stop){};
  virtual void stopscroll(void){};
  //virtual uint8_t ssd1306_command(uint8_t c) = 0;
  virtual bool getPixel(int16_t x, int16_t y) = 0;
  
  uint16_t m_BgColor = 0;
  uint16_t m_FgColor = 0;

  virtual void setBackgroundColor(uint16_t bgColor) {m_BgColor = bgColor;};
  virtual void setForegroundColor(uint16_t fgColor) {m_FgColor = fgColor;};

  virtual uint16_t getBackgroundColor() {return m_BgColor;};
  virtual uint16_t getForegroundColor() {return m_FgColor;};


  uint8_t *getBuffer(void);         // Get data buffer pointer
  uint8_t *getCommandBuffer(void);  // Get command buffer pointer.

  virtual void handle(void){};
  virtual void waitForTxComplete(){};

//  virtual bool mDataToSend = false;
//  virtual bool mCommandToSend = false;

private:
  // inline void SPIwrite(uint8_t d) __attribute__((always_inline));
  virtual void drawFastHLineInternal(int16_t x, int16_t y, int16_t w, uint16_t color){};
  virtual void drawFastVLineInternal(int16_t x, int16_t y, int16_t h, uint16_t color){};
  //uint8_t  ssd1306_command1(uint8_t c);
  //uint8_t ssd1306_commandList(const uint8_t *c, uint8_t n);

 // dmaSpiManagerClass *spiComm; 
  

 uint8_t *buffer;

  // int8_t vccstate;
  // int8_t page_end;
  // atPin_t  dcPin;
  // atPin_t  csPin;
  // atPin_t rstPin;

  // uint8_t contrast; // normal contrast setting for this device

  // uint8_t mDataState = 0;

  // dmaSpiStruct commandSpiStruct;
  // int commandSpiIndex;
  // dmaSpiStruct dataSpiStruct;
  // int dataSpiIndex;

  // #define commandBufferSize 100
  // uint8_t commandBuffer[commandBufferSize]; //TODO optimize buffer size
  // int commandBufferWritePos = 0;
 
};

#endif // _teamAt_SSD1306_H_
