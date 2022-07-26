/*!
 * @file teamAt_SSD1306.h
 *
 * This is an adaptation og the Adafruit's SSD1306 library for monochrome
 * OLED displaysto work with DMA spi.  For the standard library check teir website: http://www.adafruit.com/category/63_98
 *
 * This library was debelopped with the Arduino MKRzero, not tested on any other board so far.
 *
 */

#ifndef _teamAt_GC9A01_DMA_H_
#define _teamAt_GC9A01_DMA_H_


#include "../GFX_Library/TeamAT_GFX.h"
#include <SPI.h>
//#include <Wire.h>

//#include "at_plaformAbstraction.h"
#include "teamAt_display_V1_0.h"
#include "dma_comm_stm32_V1.0.0.h"
#include <dmaspi_comm_stm32_V1.0.0.h> //#include <dmaspi_comm_mkrzero_V1.0.0.h>

#define GC9A01_TFTWIDTH 240
#define GC9A01_TFTHEIGHT 240

// Color definitions
#define BLACK 0x0000       ///<   0,   0,   0
#define NAVY 0x000F        ///<   0,   0, 123
#define DARKGREEN 0x03E0   ///<   0, 125,   0
#define DARKCYAN 0x03EF    ///<   0, 125, 123
#define MAROON 0x7800      ///< 123,   0,   0
#define PURPLE 0x780F      ///< 123,   0, 123
#define OLIVE 0x7BE0       ///< 123, 125,   0
#define LIGHTGREY 0xC618   ///< 198, 195, 198
#define DARKGREY 0x7BEF    ///< 123, 125, 123
#define BLUE 0x001F        ///<   0,   0, 255
#define GREEN 0x07E0       ///<   0, 255,   0
#define CYAN 0x07FF        ///<   0, 255, 255
#define RED 0xF800         ///< 255,   0,   0
#define MAGENTA 0xF81F     ///< 255,   0, 255
#define YELLOW 0xFFE0      ///< 255, 255,   0
#define WHITE 0xFFFF       ///< 255, 255, 255
#define ORANGE 0xFD20      ///< 255, 165,   0
#define GREENYELLOW 0xAFE5 ///< 173, 255,  41
#define PINK 0xFC18        ///< 255, 130, 198


/// The following "raw" color names are kept for backwards client compatability
/// They can be disabled by predefining this macro before including the Adafruit
/// header client code will then need to be modified to use the scoped enum
/// values directly
//#ifndef NO_ADAFRUIT_SSD1306_COLOR_COMPATIBILITY
//#define BLACK SSD1306_BLACK     ///< Draw 'off' pixels
//#define WHITE SSD1306_WHITE     ///< Draw 'on' pixels
//#define INVERSE SSD1306_INVERSE ///< Invert pixels
//#endif
/// fit into the SSD1306_ naming scheme
#define SSD1306_BLACK PIXEL_OFF  ///< Draw 'off' pixels
#define SSD1306_WHITE PIXEL_ON   ///< Draw 'on' pixels
#define SSD1306_INVERSE 2 ///< Invert pixels

#define GC9A01_MEMORYMODE 	0x2c//0x20          ///////< See datasheet
#define GC9A01_COLUMNADDR 	0x2a//0x21          /////< See datasheet
#define GC9A01_PAGEADDR 	0x2b//0x22            /////< See datasheet
#define SSD1306_SETCONTRAST 0x81         ///< See datasheet
#define SSD1306_CHARGEPUMP 0x8D          ///< See datasheet
#define SSD1306_SEGREMAP 0xA0            ///< See datasheet
#define SSD1306_DISPLAYALLON_RESUME 0xA4 ///< See datasheet
#define SSD1306_DISPLAYALLON 0xA5        ///< Not currently used
#define GC9A01_NORMALDISPLAY 0x20//0xA6       ///< See datasheet
#define GC9A01_INVERTDISPLAY 0x21//0xA7       ///< See datasheet
#define SSD1306_SETMULTIPLEX 0xA8        ///< See datasheet
#define GC9A01_DISPLAYOFF 0x28//0xAE          ///< See datasheet
#define GC9A01_DISPLAYON 0x29//0xAF           ///< See datasheet
#define SSD1306_COMSCANINC 0xC0          ///< Not currently used
#define SSD1306_COMSCANDEC 0xC8          ///< See datasheet
#define SSD1306_SETDISPLAYOFFSET 0xD3    ///< See datasheet
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5  ///< See datasheet
#define SSD1306_SETPRECHARGE 0xD9        ///< See datasheet
#define SSD1306_SETCOMPINS 0xDA          ///< See datasheet
#define SSD1306_SETVCOMDETECT 0xDB       ///< See datasheet

#define SSD1306_SETLOWCOLUMN 0x00  ///< Not currently used
#define SSD1306_SETHIGHCOLUMN 0x10 ///< Not currently used
#define SSD1306_SETSTARTLINE 0x40  ///< See datasheet

#define SSD1306_EXTERNALVCC 0x01  ///< External display voltage source
#define SSD1306_SWITCHCAPVCC 0x02 ///< Gen. display voltage from 3.3V

#define SSD1306_RIGHT_HORIZONTAL_SCROLL 0x26              ///< Init rt scroll
#define SSD1306_LEFT_HORIZONTAL_SCROLL 0x27               ///< Init left scroll
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29 ///< Init diag scroll
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A  ///< Init diag scroll
#define SSD1306_DEACTIVATE_SCROLL 0x2E                    ///< Stop scroll
#define SSD1306_ACTIVATE_SCROLL 0x2F                      ///< Start scroll
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3             ///< Set scroll range

#define GC9A01_TFTWIDTH 240
#define GC9A01_TFTHEIGHT 240

#define GC9A01_RST_DELAY 120    ///< delay ms wait for reset finish
#define GC9A01_SLPIN_DELAY 120  ///< delay ms wait for sleep in finish
#define GC9A01_SLPOUT_DELAY 120 ///< delay ms wait for sleep out finish

#define GC9A01_NOP 0x00
#define GC9A01_SWRESET 0x01
#define GC9A01_RDDID 0x04
#define GC9A01_RDDST 0x09

#define GC9A01_SLPIN 0x10
#define GC9A01_SLPOUT 0x11
#define GC9A01_PTLON 0x12
#define GC9A01_NORON 0x13

#define GC9A01_INVOFF 0x20
#define GC9A01_INVON 0x21
#define GC9A01_DISPOFF 0x28
#define GC9A01_DISPON 0x29

#define GC9A01_CASET 0x2A
#define GC9A01_RASET 0x2B
#define GC9A01_RAMWR 0x2C
#define GC9A01_RAMRD 0x2E

#define GC9A01_PTLAR 0x30
#define GC9A01_COLMOD 0x3A
#define GC9A01_MADCTL 0x36

#define GC9A01_MADCTL_MY 0x80
#define GC9A01_MADCTL_MX 0x40
#define GC9A01_MADCTL_MV 0x20
#define GC9A01_MADCTL_ML 0x10
#define GC9A01_MADCTL_RGB 0x00

#define GC9A01_RDID1 0xDA
#define GC9A01_RDID2 0xDB
#define GC9A01_RDID3 0xDC
#define GC9A01_RDID4 0xDD



//Data send state machine
#define SEND_COMMAND_STATE 0
#define SEND_DATA_STATE    1 

#define DATA_BUFFER_WIDTH 16

/*! \class teamAt_SSD1306
    \brief  Class for interacting with SSD1306 OLED displays over SPI using DMA.

    This clas is baste on the adafruit_SSD1306 class and modified to work with DMA SPI.
*/
class teamAt_GC9A01 : public teamAt_display {
public:

  // SPI init
  teamAt_GC9A01(dmaCommManagerClass *spiCommPtr, atPin_t dc_pin, atPin_t rst_pin, atPin_t cs_pin, uint16_t w = GC9A01_TFTWIDTH, uint16_t h=GC9A01_TFTHEIGHT);

  ~teamAt_GC9A01(void);

  bool begin();
  void display(void);
  void displayOn(void);
  void displayOff(void);
  void clearDisplay(uint16_t color);
  void clearDisplay(void);
  void invertDisplay(bool i);
  void dim(bool dim);
  void drawPixel(int16_t x, int16_t y, uint16_t color);
  virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
  virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  void startscrollright(uint8_t start, uint8_t stop);
  void startscrollleft(uint8_t start, uint8_t stop);
  void startscrolldiagright(uint8_t start, uint8_t stop);
  void startscrolldiagleft(uint8_t start, uint8_t stop);
  void stopscroll(void);
  uint8_t ssd1306_command(uint8_t c);
  bool getPixel(int16_t x, int16_t y);
  
  uint16_t *getBuffer(void);         // Get data buffer pointer
  uint8_t *getCommandBuffer(void);  // Get command buffer pointer.

  void handle(void);
  void waitForTxComplete();

//  uint16_t m_BgColor = BLACK;
//   uint16_t m_FgColor = WHITE;

   void setBackgroundColor(uint16_t bgColor)
 		 {
 			m_BgColor = bgColor;
 		 }

 		 void setForegroundColor(uint16_t fgColor)
 		  {
 			m_FgColor = fgColor;
 		  }


  bool mDataToSend = false;
  bool mCommandToSend = false; 

private:
  inline void SPIwrite(uint8_t d) __attribute__((always_inline));
  void drawFastHLineInternal(int16_t x, int16_t y, int16_t w, uint16_t color);
  void drawFastVLineInternal(int16_t x, int16_t y, int16_t h, uint16_t color);
  uint8_t  ssd1306_command1(uint8_t c);
  uint8_t ssd1306_commandList(const uint8_t *c, uint8_t n);

  int m_regionSelect = 0;



  void modeCommand();
  void modeData();
  uint16_t swapColorBytes(uint16_t inputColor);


  //new

  void sendData8Blocking(uint8_t c);
  void sendData16Blocking(uint16_t c1);
  void sendCommandBlocking(uint8_t c);

  void writeAddrWindow(int16_t x, int16_t y, uint16_t w, uint16_t h);


  uint16_t _currentX = 0;
  uint16_t _currentY = 0;
  uint16_t _currentW = 0;
  uint16_t _currentH = 0;

  uint16_t _xStart = 0;
  uint16_t _yStart = 0;

  //
  dmaCommManagerClass *commManager;
  
  uint8_t spiMode;
  uint8_t I2CMode;

  uint8_t i2cAddress;

  //uint16_t *buffer;
  uint16_t buffer[GC9A01_TFTWIDTH*GC9A01_TFTHEIGHT];

  int8_t vccstate;
  int8_t page_end;

  atPin_t  dcPin;
  atPin_t  csPin;
  atPin_t rstPin;

  uint8_t contrast; // normal contrast setting for this device

  uint8_t mDataState = 0;

  dmaStruct commandSpiStruct;
  int commandSpiIndex;
  dmaStruct dataSpiStruct;
  int dataSpiIndex;

  #define commandBufferSize 100
  uint8_t commandBufferFull[commandBufferSize]; //TODO optimize buffer size
  uint8_t* commandBuffer;
  int commandBufferWritePos = 0;
 
};

#endif // _teamAt_SSD1306_H_
