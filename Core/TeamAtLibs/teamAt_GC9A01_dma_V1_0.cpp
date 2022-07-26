


#ifdef __AVR__
#include <avr/pgmspace.h>
#elif defined(ESP8266) || defined(ESP32)
#include <pgmspace.h>
#else
#define pgm_read_byte(addr)                                                    \
  (*(const unsigned char *)(addr)) ///< PROGMEM workaround for non-AVR
#endif

#if !defined(__ARM_ARCH) && !defined(ENERGIA) && !defined(ESP8266) &&          \
    !defined(ESP32) && !defined(__arc__)
#include <util/delay.h>
#endif

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <teamAt_display_V1_0.h>
#include <teamAt_GC9A01_dma_V1_0.h>
#include "splash.h"
#include "TeamAT_GFX.h"

// SOME DEFINES AND STATIC VARIABLES USED INTERNALLY -----------------------

#define display_swap(a, b)                                                     \
  (((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b))) ///< No-temp-var swap operation


// CONSTRUCTORS, DESTRUCTOR ------------------------------------------------

/*!
    @brief  Constructor for SPI SSD1306 displays, using software (bitbang)
            SPI.
    @param  w
            Display width in pixels
    @param  h
            Display height in pixels
    @param  mosi_pin
            MOSI (master out, slave in) pin (using Arduino pin numbering).
            This transfers serial data from microcontroller to display.
    @param  sclk_pin
            SCLK (serial clock) pin (using Arduino pin numbering).
            This clocks each bit from MOSI.
    @param  dc_pin
            Data/command pin (using Arduino pin numbering), selects whether
            display is receiving commands (low) or data (high).
    @param  rst_pin
            Reset pin (using Arduino pin numbering), or -1 if not used
            (some displays might be wired to share the microcontroller's
            reset pin).
    @param  cs_pin
            Chip-select pin (using Arduino pin numbering) for sharing the
            bus with other devices. Active low.
    @return Adafruit_SSD1306 object.
    @note   Call the object's begin() function before use -- buffer
            allocation is performed there!
*/
teamAt_GC9A01::teamAt_GC9A01(dmaCommManagerClass *spiCommPtr, atPin_t dc_pin, atPin_t rst_pin, atPin_t cs_pin, uint16_t w, uint16_t h):
	teamAt_display(w, h)//TODO,  remettre buffer(NULL)//, dcPin(dc_pin), rstPin(rst_pin), csPin(cs_pin)
        {
          commManager = spiCommPtr;
          spiMode = 1;
          I2CMode = 0;

          dcPin = dc_pin;
          rstPin = rst_pin;
          csPin = cs_pin;
        }
//teamAt_SSD1306::teamAt_SSD1306(dmaCommManagerClass *i2cCommPtr, uint16_t deviceAddress, uint16_t w, uint16_t h):
//	teamAt_display(w, h), buffer(NULL)
//        {
//          commManager = i2cCommPtr;
//          spiMode = 0;
//          I2CMode = 1;
//          dcPin.port = NULL;
//		  rstPin.port = NULL;
//		  csPin.port = NULL;
//		  i2cAddress = deviceAddress;
//        }


/*!
    @brief  Destructor for teamAt_SSD1306 object.
*/
teamAt_GC9A01::~teamAt_GC9A01(void) {
//  if (buffer) {
//    free(buffer);
//    buffer = NULL;
//  }
}

// LOW-LEVEL UTILS ---------------------------------------------------------

/**
 * @brief Append a single command in the command buffer for the SSD1306
 * 
 * @param c the command byte
 * 
 * @return 1: success
 * @return 0: Error, buffer full
 */
uint8_t teamAt_GC9A01::ssd1306_command1(uint8_t c) {

  if(commandBufferWritePos < commandBufferSize)
  {
    commandBuffer[commandBufferWritePos] = c;
    commandBufferWritePos++; 

    return 1; // success
  }

  return 0; //Buffer Full
  
}

/**
 * @brief Append a list of command bytes to the command buffer
 * 
 * @param c Pointer to the source buffer
 * @param n Number of bytes to copy
 * 
 * @return 1: success
 * @return 0: Error, buffer full
 * 
 */
uint8_t teamAt_GC9A01::ssd1306_commandList(const uint8_t *c, uint8_t n) {

	if((commandBufferWritePos + (n-1))  < commandBufferSize)
	{
		while (n--){
			commandBuffer[commandBufferWritePos] = *c;
			c++;
			commandBufferWritePos++;
  
		}
      return 1; // success
	}
  
   return 0; // No enough space 
}

/**
 * @brief A public version of ssd1306_command1()
 * 
 * @param c the command byte
 * 
 * @return 1: success
 * @return 0: Error, buffer full
 */
uint8_t teamAt_GC9A01::ssd1306_command(uint8_t c)
{
  uint8_t returnValue;

  returnValue = ssd1306_command1(c);
  if(returnValue)
  {
      mCommandToSend = true; 
  }
  
  return returnValue;
}

// ALLOCATE & INIT DISPLAY -------------------------------------------------

/*!
    @brief  Allocate RAM for image buffer, initialize peripherals and pins.
    @param  vcs
            VCC selection. Pass SSD1306_SWITCHCAPVCC to generate the display
            voltage (step up) from the 3.3V source, or SSD1306_EXTERNALVCC
            otherwise. Most situations with Adafruit SSD1306 breakouts will
            want SSD1306_SWITCHCAPVCC.
    
    @param  reset
            If true, and if the reset pin passed to the constructor is
            valid, a hard reset will be performed before initializing the
            display. If using multiple SSD1306 displays on the same bus, and
            if they all share the same reset pin, you should only pass true
            on the first display being initialized, false on all others,
            else the already-initialized displays would be reset. Default if
            unspecified is true.
    @return true on successful allocation/init, false otherwise.
            Well-behaved code should check the return value before
            proceeding.
    @note   MUST call this function before any drawing or updates!
*/
bool teamAt_GC9A01::begin() {

	bool reset = true;
	uint8_t vcs = SSD1306_SWITCHCAPVCC;

	commandBuffer = commandBufferFull;

//	if ((!buffer) && !(buffer = (uint16_t *)malloc((WIDTH *HEIGHT)*2))) // *2 because it is 16 bits
//		     return false;

  // Connect Display buffers to the DMA spi communication manager to spped up transfer and recuse memory footprint.
  // We trean spi commands and spi data as 2 distinctive devices because they use different buffer but they are 
  // physically teh same device.
  // Command
  commandSpiStruct.txBuffer = commandBuffer; //display.getBuffer();
  commandSpiStruct.chipSelectPin = csPin;         // Chip select pin for the device
  commandSpiStruct.csPolarity = DMA_CSPOL_ACTIVE_LOW;
  commandSpiIndex = commManager->addDevice(&commandSpiStruct); 
  
  //Data
  dataSpiStruct.txBuffer = (uint8_t*)buffer;
  dataSpiStruct.chipSelectPin = csPin; // 7;          // Chip select pin for the device
  dataSpiStruct.csPolarity = DMA_CSPOL_ACTIVE_LOW;
  dataSpiIndex = commManager->addDevice(&dataSpiStruct);

  clearDisplay();

  vccstate = vcs;

  // Reset if requested and reset pin specified in constructor
  if (reset && (rstPin.pin >= 0)) {

	//TODO pinMode(rstPin, OUTPUT);
	  atPinWrite(rstPin,AT_PIN_HIGH);
	  delay(100);                   	// VDD goes high at start, pause for 1 ms
	  atPinWrite(rstPin,AT_PIN_LOW);
	  delay(100);                  // Wait 10 ms
	  atPinWrite(rstPin,AT_PIN_HIGH);
	  delay(100);
  }

  	sendCommandBlocking(0x36);//ok
  	sendData8Blocking(0X68);//ok

  	sendCommandBlocking(0xef);//ok
  	sendCommandBlocking(0xeb);//ok
  	sendData8Blocking(0x14);//ok

  	sendCommandBlocking(0xfe);//ok
  	sendCommandBlocking(0xef);//ok

  	sendCommandBlocking(0xeb);//ok
  	sendData8Blocking(0x14);//ok

  	sendCommandBlocking(0x84);//ok
  	sendData8Blocking(0x40);//ok

  	sendCommandBlocking(0x85);//ok
  	sendData8Blocking(0xFF);//ok

	sendCommandBlocking(0x86);//ok
  	sendData8Blocking(0xff);//ok

	sendCommandBlocking(0x87);//ok
  	sendData8Blocking(0xFF);//ok

	sendCommandBlocking(0x88);//ok
  	sendData8Blocking(0xA0);//ok

	sendCommandBlocking(0x89);//ok
  	sendData8Blocking(0x21);//ok

	sendCommandBlocking(0x8a);//ok
  	sendData8Blocking(0x00);//ok

	sendCommandBlocking(0x8b);//ok
  	sendData8Blocking(0x80);//ok

	sendCommandBlocking(0x8c);//ok
  	sendData8Blocking(0x01);//ok

	sendCommandBlocking(0x8d);//ok
  	sendData8Blocking(0x01);//ok

	sendCommandBlocking(0x8e);//ok
  	sendData8Blocking(0xFF);//ok

	sendCommandBlocking(0x8f);//ok
  	sendData8Blocking(0xFF);//ok

	sendCommandBlocking(0xb6);//ok
	sendData8Blocking(0x00);//ok
	sendData8Blocking(0x20);//ok

 	sendCommandBlocking(0x36); // ajout
  	sendData8Blocking(0x08); // ajout

  	sendCommandBlocking(0x3a);//ok
  	sendData8Blocking(0x05);//ok

  	sendCommandBlocking(0x90);//ok
  	sendData8Blocking(0x08);//ok
  	sendData8Blocking(0x08);//ok
  	sendData8Blocking(0x08);//ok
  	sendData8Blocking(0x08);//ok

  	sendCommandBlocking(0xbd);//ok
  	sendData8Blocking(0x06);//ok

  	sendCommandBlocking(0xbc);//ok
  	sendData8Blocking(0x00);//ok

  	sendCommandBlocking(0xff);//ok
  	sendData8Blocking(0x60);//ok
  	sendData8Blocking(0x01);//ok
  	sendData8Blocking(0x04);//ok

  	sendCommandBlocking(0xc3);//ok
  	sendData8Blocking(0x13);//ok

  	sendCommandBlocking(0xc4);//ok
  	sendData8Blocking(0x13);//ok

  	sendCommandBlocking(0xc9);//ok
  	sendData8Blocking(0x22);//ok

  	sendCommandBlocking(0xbe);//ok
  	sendData8Blocking(0x11);//ok

  	sendCommandBlocking(0xe1);//ok
  	sendData8Blocking(0x10);//ok
  	sendData8Blocking(0x0e);//ok


 	sendCommandBlocking(0xdf);//ok
  	sendData8Blocking(0x21);//ok
  	sendData8Blocking(0x0c);//ok
  	sendData8Blocking(0x02);//ok

  	sendCommandBlocking(0xf0);//ok
	sendData8Blocking(0x45);//ok
	sendData8Blocking(0x09);//ok
	sendData8Blocking(0x08);//ok
	sendData8Blocking(0x08);//ok
	sendData8Blocking(0x26);//ok
	sendData8Blocking(0x2a);//ok

  	sendCommandBlocking(0xf1);//ok
	sendData8Blocking(0x43);//ok
	sendData8Blocking(0x70);//ok
	sendData8Blocking(0x72);//ok
	sendData8Blocking(0x36);//ok
	sendData8Blocking(0x37);//ok
	sendData8Blocking(0x6f);//ok

  	sendCommandBlocking(0xf2);//ok
	sendData8Blocking(0x45); //mod (0x44);
	sendData8Blocking(0x09);//ok
	sendData8Blocking(0x08);//ok
	sendData8Blocking(0x08);//ok
	sendData8Blocking(0x26);//ok
	sendData8Blocking(0x2a);//ok

  	sendCommandBlocking(0xf3);//ok
	sendData8Blocking(0x43);//ok
	sendData8Blocking(0x70);//ok
	sendData8Blocking(0x72);//ok
	sendData8Blocking(0x36);//ok
	sendData8Blocking(0x37);//ok
	sendData8Blocking(0x6f);//ok

  	sendCommandBlocking(0xed);//ok
  	sendData8Blocking(0x1b);//ok
  	sendData8Blocking(0x0b);//ok

  	sendCommandBlocking(0xae);//ok
  	sendData8Blocking(0x77);//ok

  	sendCommandBlocking(0xcd);//ok
  	sendData8Blocking(0x63);//ok

  	sendCommandBlocking(0x70);//ok
	sendData8Blocking(0x07);//ok
	sendData8Blocking(0x07);//ok
	sendData8Blocking(0x04);//ok
	sendData8Blocking(0x0e);//ok
	sendData8Blocking(0x0f);//ok
	sendData8Blocking(0x09);//ok
	sendData8Blocking(0x07);//ok
	sendData8Blocking(0x08);//ok
	sendData8Blocking(0x03);//ok

  	sendCommandBlocking(0xe8);//ok
	sendData8Blocking(0x34);//ok

  	sendCommandBlocking(0x62);//ok
	sendData8Blocking(0x18);//ok
	sendData8Blocking(0x0d);//ok
	sendData8Blocking(0x71);//ok
	sendData8Blocking(0xed);//ok
	sendData8Blocking(0x70);//ok
	sendData8Blocking(0x70);//ok
	sendData8Blocking(0x18);//ok
	sendData8Blocking(0x0f);//ok
	sendData8Blocking(0x71);//ok
	sendData8Blocking(0xef);//ok
	sendData8Blocking(0x70);//ok
	sendData8Blocking(0x70);//ok


	sendCommandBlocking(0x63);//ok
	sendData8Blocking(0x18);//ok
	sendData8Blocking(0x11);//ok
	sendData8Blocking(0x71);//ok
	sendData8Blocking(0xf1);//ok
	sendData8Blocking(0x70);//ok
	sendData8Blocking(0x70);//ok
	sendData8Blocking(0x18);//ok
	sendData8Blocking(0x13);//ok
	sendData8Blocking(0x71);//ok
	sendData8Blocking(0xf3);//ok
	sendData8Blocking(0x70);//ok
	sendData8Blocking(0x70);//ok

	sendCommandBlocking(0x64);//ok
	sendData8Blocking(0x28);//ok
	sendData8Blocking(0x29);//ok
	sendData8Blocking(0xf1);//ok
	sendData8Blocking(0x01);//ok
	sendData8Blocking(0xf1);//ok
	sendData8Blocking(0x00);//ok
	sendData8Blocking(0x07);//ok

	sendCommandBlocking(0x66);//ok
	sendData8Blocking(0x3c);//ok
	sendData8Blocking(0x00);//ok
	sendData8Blocking(0xcd);//ok
	sendData8Blocking(0x67);//ok
	sendData8Blocking(0x45);//ok
	sendData8Blocking(0x45);//ok
	sendData8Blocking(0x10);//ok
	sendData8Blocking(0x00);//ok
	sendData8Blocking(0x00);//ok
	sendData8Blocking(0x00);//ok

	sendCommandBlocking(0x67);//ok
	sendData8Blocking(0x00);//ok
	sendData8Blocking(0x3c);//ok
	sendData8Blocking(0x00);//ok
	sendData8Blocking(0x00);//ok
	sendData8Blocking(0x00);//ok
	sendData8Blocking(0x01);//ok
	sendData8Blocking(0x54);//ok
	sendData8Blocking(0x10);//ok
	sendData8Blocking(0x32);//ok
	sendData8Blocking(0x98);//ok

	sendCommandBlocking(0x74);//ok
	sendData8Blocking(0x10);//ok
	sendData8Blocking(0x85);//ok
	sendData8Blocking(0x80);//ok
	sendData8Blocking(0x00);//ok
	sendData8Blocking(0x00);//ok
	sendData8Blocking(0x4e);//ok
	sendData8Blocking(0x00);//ok

  	sendCommandBlocking(0x98);//ok
  	sendData8Blocking(0x3e);//ok
  	sendData8Blocking(0x07);//ok


	sendCommandBlocking(0x35);//ok

	sendCommandBlocking(0x21);//ok

	sendCommandBlocking(0x11);//ok

	delay(120);//ok

//	sendCommandBlocking(0x29);//ok
//
//	delay(20);//ok

	// Scan direction
	sendCommandBlocking(0x36); //MX, MY, RGB mode
	sendData8Blocking(0XC8);	//0XC8: horiz 0x68: vert

  return true; // Success
}


// DRAWING FUNCTIONS -------------------------------------------------------

/*!
    @brief  Set/clear/invert a single pixel. This is also invoked by the
            Adafruit_GFX library in generating many higher-level graphics
            primitives.
    @param  x
            Column of display -- 0 at left to (screen width - 1) at right.
    @param  y
            Row of display -- 0 at top to (screen height -1) at bottom.
    @param  color
            Pixel color, one of: SSD1306_BLACK, SSD1306_WHITE or SSD1306_INVERT.
    @return None (void).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
void teamAt_GC9A01::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if ((x >= 0) && (x < width()) && (y >= 0) && (y < height())) {
    // Pixel is in-bounds. Rotate coordinates if needed.
    switch (getRotation()) {
    case 1:
      display_swap(x, y);
      x = WIDTH - x - 1;
      break;
    case 2:
      x = WIDTH - x - 1;
      y = HEIGHT - y - 1;
      break;
    case 3:
      display_swap(x, y);
      y = HEIGHT - y - 1;
      break;
    }

	buffer[x + (y*WIDTH)] = swapColorBytes(color);

  }
}

uint16_t teamAt_GC9A01::swapColorBytes(uint16_t inputColor)
{
	uint16_t color =  ((inputColor & 0x00FF) << 8) | ((inputColor & 0xFF00) >> 8);

	return color;
}

/*!
    @brief  Clear contents of display buffer (set all pixels to off).
    @return None (void).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
void teamAt_GC9A01::clearDisplay(void) {

	clearDisplay(m_BgColor);
}

void teamAt_GC9A01::clearDisplay(uint16_t color) {

	for(int i = 0 ; i < (WIDTH * HEIGHT); i++)
	{
		buffer[i] = swapColorBytes(color); //swapColorBytes(RED);
	}

}

/*!
    @brief  Draw a horizontal line. This is also invoked by the Adafruit_GFX
            library in generating many higher-level graphics primitives.
    @param  x
            Leftmost column -- 0 at left to (screen width - 1) at right.
    @param  y
            Row of display -- 0 at top to (screen height -1) at bottom.
    @param  w
            Width of line, in pixels.
    @param  color
            Line color, one of: SSD1306_BLACK, SSD1306_WHITE or SSD1306_INVERT.
    @return None (void).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
void teamAt_GC9A01::drawFastHLine(int16_t x, int16_t y, int16_t w,
                                     uint16_t color) {
  bool bSwap = false;
  switch (rotation) {
  case 1:
    // 90 degree rotation, swap x & y for rotation, then invert x
    bSwap = true;
    display_swap(x, y);
    x = WIDTH - x - 1;
    break;
  case 2:
    // 180 degree rotation, invert x and y, then shift y around for height.
    x = WIDTH - x - 1;
    y = HEIGHT - y - 1;
    x -= (w - 1);
    break;
  case 3:
    // 270 degree rotation, swap x & y for rotation,
    // then invert y and adjust y for w (not to become h)
    bSwap = true;
    display_swap(x, y);
    y = HEIGHT - y - 1;
    y -= (w - 1);
    break;
  }

  if (bSwap)
    drawFastVLineInternal(x, y, w, color);
  else
    drawFastHLineInternal(x, y, w, color);
}

void teamAt_GC9A01::drawFastHLineInternal(int16_t x, int16_t y, int16_t w,
                                             uint16_t color) {

  if ((y >= 0) && (y < HEIGHT)) { // Y coord in bounds?
    if (x < 0) {                  // Clip left
      w += x;
      x = 0;
    }
    if ((x + w) > WIDTH) { // Clip right
      w = (WIDTH - x);
    }
    if (w > 0) { // Proceed only if width is positive
      //uint16_t *pBuf = &buffer[(y / DATA_BUFFER_WIDTH) * WIDTH + x];
    	uint16_t *pBuf = &buffer[y*WIDTH + x];
      while (w--) {
      *pBuf++ = swapColorBytes(color);
      }

    }
  }
}

/*!
    @brief  Draw a vertical line. This is also invoked by the Adafruit_GFX
            library in generating many higher-level graphics primitives.
    @param  x
            Column of display -- 0 at left to (screen width -1) at right.
    @param  y
            Topmost row -- 0 at top to (screen height - 1) at bottom.
    @param  h
            Height of line, in pixels.
    @param  color
            Line color, one of: SSD1306_BLACK, SSD1306_WHITE or SSD1306_INVERT.
    @return None (void).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
void teamAt_GC9A01::drawFastVLine(int16_t x, int16_t y, int16_t h,
                                     uint16_t color) {
  bool bSwap = false;
  switch (rotation) {
  case 1:
    // 90 degree rotation, swap x & y for rotation,
    // then invert x and adjust x for h (now to become w)
    bSwap = true;
    display_swap(x, y);
    x = WIDTH - x - 1;
    x -= (h - 1);
    break;
  case 2:
    // 180 degree rotation, invert x and y, then shift y around for height.
    x = WIDTH - x - 1;
    y = HEIGHT - y - 1;
    y -= (h - 1);
    break;
  case 3:
    // 270 degree rotation, swap x & y for rotation, then invert y
    bSwap = true;
    display_swap(x, y);
    y = HEIGHT - y - 1;
    break;
  }

  if (bSwap)
    drawFastHLineInternal(x, y, h, color);
  else
    drawFastVLineInternal(x, y, h, color);
}

void teamAt_GC9A01::drawFastVLineInternal(int16_t x, int16_t __y,
                                             int16_t __h, uint16_t color) {

  if ((x >= 0) && (x < WIDTH)) { // X coord in bounds?
    if (__y < 0) {               // Clip top
      __h += __y;
      __y = 0;
    }
    if ((__y + __h) > HEIGHT) { // Clip bottom
      __h = (HEIGHT - __y);
    }
    if (__h > 0) { // Proceed only if height is now positive
      // this display doesn't need ints for coordinates,
      // use local byte registers for faster juggling
      uint8_t y = __y, h = __h;

      for(int i = 0 ; i < h; i++)
      {
    	  buffer[(y+i)* WIDTH + x] = swapColorBytes(color);
      }

      // do the first partial byte, if necessary - this requires some masking
//      uint8_t mod = (y & 7);
//      if (mod) {
//        // mask off the high n bits we want to set
//        mod = 8 - mod;
//        // note - lookup table results in a nearly 10% performance
//        // improvement in fill* functions
//        // uint8_t mask = ~(0xFF >> mod);
//        static const uint8_t premask[8] = {0x00, 0x80, 0xC0, 0xE0,
//                                                   0xF0, 0xF8, 0xFC, 0xFE};
//        uint8_t mask = pgm_read_byte(&premask[mod]);
//        // adjust the mask if we're not going to reach the end of this byte
//        if (h < mod)
//          mask &= (0XFF >> (mod - h));
//
//        switch (color) {
//        case SSD1306_WHITE:
//          *pBuf |= mask;
//          break;
//        case SSD1306_BLACK:
//          *pBuf &= ~mask;
//          break;
//        case SSD1306_INVERSE:
//          *pBuf ^= mask;
//          break;
//        }
//        pBuf += WIDTH;
//      }

//      if (h >= mod) { // More to go?
//        h -= mod;
//        // Write solid bytes while we can - effectively 8 rows at a time
//        if (h >= 8) {
//          if (color == SSD1306_INVERSE) {
//            // separate copy of the code so we don't impact performance of
//            // black/white write version with an extra comparison per loop
//            do {
//              *pBuf ^= 0xFF; // Invert byte
//              pBuf += WIDTH; // Advance pointer 8 rows
//              h -= 8;        // Subtract 8 rows from height
//            } while (h >= 8);
//          } else {
//            // store a local value to work with
//            uint8_t val = (color != SSD1306_BLACK) ? 255 : 0;
//            do {
//              *pBuf = val;   // Set byte
//              pBuf += WIDTH; // Advance pointer 8 rows
//              h -= 8;        // Subtract 8 rows from height
//            } while (h >= 8);
//          }
//        }
//
//        if (h) { // Do the final partial byte, if necessary
//          mod = h & 7;
//          // this time we want to mask the low bits of the byte,
//          // vs the high bits we did above
//          // uint8_t mask = (1 << mod) - 1;
//          // note - lookup table results in a nearly 10% performance
//          // improvement in fill* functions
//          static const uint8_t  postmask[8] = {0x00, 0x01, 0x03, 0x07,
//                                                      0x0F, 0x1F, 0x3F, 0x7F};
//          //uint8_t mask = pgm_read_byte(&postmask[mod]);
//          switch (color) {
//          case SSD1306_WHITE:
//            *pBuf |= mask;
//            break;
//          case SSD1306_BLACK:
//            *pBuf &= ~mask;
//            break;
//          case SSD1306_INVERSE:
//            *pBuf ^= mask;
//            break;
//          }
//          *pBuf = pgm_read_byte(&postmask[mod]);
//        }
//      }
    } // endif positive height
  }   // endif x in bounds
}

/*!
    @brief  Return color of a single pixel in display buffer.
    @param  x
            Column of display -- 0 at left to (screen width - 1) at right.
    @param  y
            Row of display -- 0 at top to (screen height -1) at bottom.
    @return true if pixel is set (usually SSD1306_WHITE, unless display invert
   mode is enabled), false if clear (SSD1306_BLACK).
    @note   Reads from buffer contents; may not reflect current contents of
            screen if display() has not been called.
*/
bool teamAt_GC9A01::getPixel(int16_t x, int16_t y) {
  if ((x >= 0) && (x < width()) && (y >= 0) && (y < height())) {
    // Pixel is in-bounds. Rotate coordinates if needed.
    switch (getRotation()) {
    case 1:
      display_swap(x, y);
      x = WIDTH - x - 1;
      break;
    case 2:
      x = WIDTH - x - 1;
      y = HEIGHT - y - 1;
      break;
    case 3:
      display_swap(x, y);
      y = HEIGHT - y - 1;
      break;
    }
    return (buffer[x + y * WIDTH] > 0);
  }
  return false; // Pixel out of bounds
}

/*!
    @brief  Get base address of display data buffer for direct reading or writing.
    @return Pointer to an unsigned 8-bit array, column-major, columns padded
            to full byte boundary if needed.
*/
uint16_t *teamAt_GC9A01::getBuffer(void) { return buffer; }

/**
 * @brief Get base address of display command buffer for direct reading or writing.
 * 
 * @return uint8_t* Pointer to an unsigned 8-bit array
 */
uint8_t *teamAt_GC9A01::getCommandBuffer(void) { return commandBuffer; }

// REFRESH DISPLAY ---------------------------------------------------------

/*!
    @brief  Activate flag to push data currently in RAM to SSD1306 display.
    @return None (void).
    @note   Drawing operations are not visible until this function is
            called. Call after each graphics command, or after a whole set
            of graphics commands, as best needed by one's own application.
    @note   In DMA spi, this does not send the data. It raises a flag sao that the
            communication manager is aware that the buffer is fil;led as needed
            and ready to be sent.
*/
void teamAt_GC9A01::display(void) {
 
  mDataToSend = true;


}


void teamAt_GC9A01::displayOn(void){

	sendCommandBlocking(GC9A01_DISPLAYON);
	delay(120);
}

void teamAt_GC9A01::displayOff(void){
	sendCommandBlocking(GC9A01_DISPLAYOFF);
	delay(120);
}

// SCROLLING FUNCTIONS -----------------------------------------------------

/*!
    @brief  Activate a right-handed scroll for all or part of the display.
    @param  start
            First row.
    @param  stop
            Last row.
    @return None (void).
*/
// To scroll the whole display, run: display.startscrollright(0x00, 0x0F)
void teamAt_GC9A01::startscrollright(uint8_t start, uint8_t stop) {
  
  //COMMAND_START
  static const uint8_t  scrollList1a[] = {
      SSD1306_RIGHT_HORIZONTAL_SCROLL, 0X00};
  ssd1306_commandList(scrollList1a, sizeof(scrollList1a));
  ssd1306_command1(start);
  ssd1306_command1(0X00);
  ssd1306_command1(stop);
  static const uint8_t  scrollList1b[] = {0X00, 0XFF,
                                                 SSD1306_ACTIVATE_SCROLL};
  ssd1306_commandList(scrollList1b, sizeof(scrollList1b));
 
  mCommandToSend = true;
}

/*!
    @brief  Activate a left-handed scroll for all or part of the display.
    @param  start
            First row.
    @param  stop
            Last row.
    @return None (void).
*/
// To scroll the whole display, run: display.startscrollleft(0x00, 0x0F)
void teamAt_GC9A01::startscrollleft(uint8_t start, uint8_t stop) {
  
  //COMMAND_START
  static const uint8_t  scrollList2a[] = {SSD1306_LEFT_HORIZONTAL_SCROLL,
                                                 0X00};
  ssd1306_commandList(scrollList2a, sizeof(scrollList2a));
  ssd1306_command1(start);
  ssd1306_command1(0X00);
  ssd1306_command1(stop);
  static const uint8_t  scrollList2b[] = {0X00, 0XFF,
                                                 SSD1306_ACTIVATE_SCROLL};
  ssd1306_commandList(scrollList2b, sizeof(scrollList2b));
  mCommandToSend = true;  
}

/*!
    @brief  Activate a diagonal scroll for all or part of the display.
    @param  start
            First row.
    @param  stop
            Last row.
    @return None (void).
*/
// display.startscrolldiagright(0x00, 0x0F)
void teamAt_GC9A01::startscrolldiagright(uint8_t start, uint8_t stop) {

  static const uint8_t  scrollList3a[] = {
      SSD1306_SET_VERTICAL_SCROLL_AREA, 0X00};
  ssd1306_commandList(scrollList3a, sizeof(scrollList3a));
  ssd1306_command1(HEIGHT);
  static const uint8_t  scrollList3b[] = {
      SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL, 0X00};
  ssd1306_commandList(scrollList3b, sizeof(scrollList3b));
  ssd1306_command1(start);
  ssd1306_command1(0X00);
  ssd1306_command1(stop);
  static const uint8_t  scrollList3c[] = {0X01, SSD1306_ACTIVATE_SCROLL};
  ssd1306_commandList(scrollList3c, sizeof(scrollList3c));

  mCommandToSend = true;

}

/*!
    @brief  Activate alternate diagonal scroll for all or part of the display.
    @param  start
            First row.
    @param  stop
            Last row.
    @return None (void).
*/
// To scroll the whole display, run: display.startscrolldiagleft(0x00, 0x0F)
void teamAt_GC9A01::startscrolldiagleft(uint8_t start, uint8_t stop) {
 
  static const uint8_t  scrollList4a[] = {
      SSD1306_SET_VERTICAL_SCROLL_AREA, 0X00};
  ssd1306_commandList(scrollList4a, sizeof(scrollList4a));
  ssd1306_command1(HEIGHT);
  static const uint8_t  scrollList4b[] = {
      SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL, 0X00};
  ssd1306_commandList(scrollList4b, sizeof(scrollList4b));
  ssd1306_command1(start);
  ssd1306_command1(0X00);
  ssd1306_command1(stop);
  static const uint8_t  scrollList4c[] = {0X01, SSD1306_ACTIVATE_SCROLL};
  ssd1306_commandList(scrollList4c, sizeof(scrollList4c));

  mCommandToSend = true;
}

/*!
    @brief  Cease a previously-begun scrolling action.
    @return None (void).
*/
void teamAt_GC9A01::stopscroll(void) {

  ssd1306_command1(SSD1306_DEACTIVATE_SCROLL);
  
  mCommandToSend = true;

}

// OTHER HARDWARE SETTINGS -------------------------------------------------

/*!
    @brief  Enable or disable display invert mode (white-on-black vs
            black-on-white).
    @param  i
            If true, switch to invert mode (black-on-white), else normal
            mode (white-on-black).
    @return None (void).
    @note   This has an immediate effect on the display, no need to call the
            display() function -- buffer contents are not changed, rather a
            different pixel mode of the display hardware is used. When
            enabled, drawing SSD1306_BLACK (value 0) pixels will actually draw
   white, SSD1306_WHITE (value 1) will draw black.
*/
void teamAt_GC9A01::invertDisplay(bool i) {

	sendCommandBlocking(i ? GC9A01_INVERTDISPLAY : GC9A01_NORMALDISPLAY);
  
  //mCommandToSend = true;
}

/*!
    @brief  Dim the display.
    @param  dim
            true to enable lower brightness mode, false for full brightness.
    @return None (void).
    @note   This has an immediate effect on the display, no need to call the
            display() function -- buffer contents are not changed.
*/
void teamAt_GC9A01::dim(bool dim) {
  // the range of contrast to too small to be really useful
  // it is useful to dim the display
  ssd1306_command1(SSD1306_SETCONTRAST);
  ssd1306_command1(dim ? 0 : contrast);

  mCommandToSend = true;

}

void teamAt_GC9A01::handle(void)
{
  // When device is available
  if (!commManager->getTxStatus())
  {
       // If we need to send data
      if(mDataToSend)
      {
    	  // Affichage séparé en 4 régions car sinon on dépassait le uint16 du transfert dma.
    	  // Optimistion possible: Implémenter une logique avec la fênetre. Au lieu d'afficher la fenètre au complet,
    	  // réduire la fenêtre à la grosseur du data.
    	  uint32_t count = WIDTH * (HEIGHT/4);// WIDTH * ((HEIGHT + (DATA_BUFFER_WIDTH-1)) / DATA_BUFFER_WIDTH);;
    	  uint32_t startPos = count*2* m_regionSelect;
    	  writeAddrWindow(0, (this->HEIGHT/4)*m_regionSelect, this->WIDTH, this->HEIGHT/4);

    	  modeData();
//    	      	  for(uint32_t i = 0 ; i < count; i++ )
//    	      	  {
//    	      		  sendData16Blocking(buffer[i]);
//    	      	  }
    	  commManager->sendPartialDataSharedBuffer(startPos,count*2,dataSpiIndex); //device index. // coubnt * 12 because the buffer is sent as bytes
    	  m_regionSelect++;
    	  if(m_regionSelect > 3)
    	  {
    		  m_regionSelect = 0;
    		  mDataToSend = false;
    	  }


    	  // send display content
//    	  startWrite();
//    	  writeAddrWindow(0, 0, this->WIDTH, this->HEIGHT);
//    	  uint32_t count = WIDTH * HEIGHT;// WIDTH * ((HEIGHT + (DATA_BUFFER_WIDTH-1)) / DATA_BUFFER_WIDTH);
//    	  delay(50);
//    	  modeData();
////    	  for(int i = 0 ; i < count; i++ )
////    	  {
////    		  sendData16Blocking(buffer[i]);
////    	  }
//
////    	  writeAddrWindow(0, 0, this->WIDTH, this->HEIGHT);
////    	  delay(50);
//    	  commManager->sendDataSharedBuffer(count*2,dataSpiIndex); //device index. // coubnt * 12 because the buffer is sent as bytes
//    	  while(commManager->getTxStatus())
//		  {
//			  commManager->handle();
//		  }
//    	  endWrite();
//    	  mDataToSend = false;

//        //Data is sent in two steps. Firts the command, next the data iteself
//        switch(mDataState)
//        {
//          case SEND_COMMAND_STATE:
//
//        	  //set the X coordinates
////
////			  sendCommandBlocking(0x2A);
////        	      sendData8Blocking(0x00);
////        	      sendData8Blocking(0); // x start
////        	      sendData8Blocking(0x00);
////        	      sendData8Blocking((uint8_t)(this->WIDTH)); //x stop
////
////        	      //set the Y coordinates
////        	      sendCommandBlocking(0x2B);
////        	      sendData8Blocking(0x00);
////        	      sendData8Blocking(0);//Ystart
////        	      sendData8Blocking(0x00);
////        	      sendData8Blocking((uint8_t)(this->HEIGHT));//Yend
////
////        	      sendCommandBlocking(0X2C);
//        	  startWrite();
//        	  writeAddrWindow(0, 0, this->WIDTH, this->HEIGHT);
//
////            commandBufferWritePos = 0; //Reset buffer
////            static const uint8_t dlist1[] = {
////                  SSD1306_PAGEADDR,
////                  0,                      // Page start address
////                  0xFF,                   // Page end (not really, but works here)
////                  SSD1306_COLUMNADDR, 0}; // Column start address
////            ssd1306_commandList(dlist1, sizeof(dlist1));
////            ssd1306_command1(WIDTH - 1); // Column end address
////
////            modeCommand(); //atPinWrite(dcPin,AT_PIN_LOW);//digitalWrite(dcPin, LOW); // Mode Command
////            if(I2CMode)
////			{
////            	commandBufferWritePos++; // Increase count by 1 to include the Start data byte in i2c
////			}
////            commManager->sendDataSharedBuffer(commandBufferWritePos,commandSpiIndex); //device index.
////            commandBufferWritePos = 0;    //Reset buffer
////            mCommandToSend = false;
////
//            mDataState = SEND_DATA_STATE ;
//
//            break;
//
//          case SEND_DATA_STATE:
//
//            mDataToSend = false;
//            modeData(); //atPinWrite(dcPin,AT_PIN_HIGH);	//digitalWrite(6, HIGH); //Mode Data
//
//
//            uint16_t count = WIDTH * ((HEIGHT + 7) / 8);
//
//            commManager->sendDataSharedBuffer(count,dataSpiIndex); //device index.
//            mDataState = SEND_COMMAND_STATE;
//
//          break;
//        }
        
      }
//      else if(mCommandToSend)
//      {
//        mCommandToSend = false;
//        modeCommand();//atPinWrite(dcPin,AT_PIN_LOW);//digitalWrite(6, LOW); // Mode Command
//        if(I2CMode)
//		{
//			commandBufferWritePos++; // Increase count by 1 to include the Start data byte in i2c
//		}
//        commManager->sendDataSharedBuffer(commandBufferWritePos,commandSpiIndex); //device index.
//        commandBufferWritePos = 0;
//      }
  }
  
}

void teamAt_GC9A01::waitForTxComplete()
{
    while (mCommandToSend || mDataToSend || commManager->getTxStatus())
    {
        handle();
        commManager->handle();
    } 
}

void teamAt_GC9A01::modeCommand()
{
	if(spiMode)
	{
		atPinWrite(dcPin,AT_PIN_LOW);//digitalWrite(dcPin, LOW); // Mode Command
	}
}

void teamAt_GC9A01::modeData()
{
	if(spiMode)
	{
		atPinWrite(dcPin,AT_PIN_HIGH);	//digitalWrite(6, HIGH); //Mode Data
	}

}

void teamAt_GC9A01::writeAddrWindow(int16_t x, int16_t y, uint16_t w, uint16_t h)
{
  if ((x != _currentX) || (w != _currentW) || (y != _currentY) || (h != _currentH))
  {
	  // _bus->writeC8D16D16(GC9A01_CASET, x + _xStart, x + w - 1 + _xStart);
	  sendCommandBlocking(GC9A01_CASET);
	  sendData16Blocking(x + _xStart);
	  sendData16Blocking(x + w - 1 + _xStart);

   // _bus->writeC8D16D16(GC9A01_RASET, y + _yStart, y + h - 1 + _yStart);
	  sendCommandBlocking(GC9A01_RASET);
	  sendData16Blocking(y + _yStart);
	  sendData16Blocking(y + h - 1 + _yStart);

    _currentX = x;
    _currentY = y;
    _currentW = w;
    _currentH = h;
  }
//
//  _bus->writeCommand(GC9A01_RAMWR); // write to RAM
  sendCommandBlocking(GC9A01_RAMWR);
}

void teamAt_GC9A01::sendCommandBlocking(uint8_t c)
{
//	while(commManager->getTxStatus())
//	  {
//		  commManager->handle();
//	  }

	modeCommand();

	 commandBufferWritePos = 0;
	 commandBuffer[commandBufferWritePos] = c;
	 commandBufferWritePos++;
	 commManager->sendDataSharedBuffer(commandBufferWritePos,commandSpiIndex); //device index.

	while(commManager->getTxStatus())
	  {
		  commManager->handle();
	  }
}

void teamAt_GC9A01::sendData8Blocking(uint8_t c)
{
	while(commManager->getTxStatus())
		  {
			  commManager->handle();
		  }

	modeData();
	commandBufferWritePos = 0;
	commandBuffer[commandBufferWritePos] = c;
	commandBufferWritePos++;
	commManager->sendDataSharedBuffer(commandBufferWritePos,commandSpiIndex); //device index.

	while(commManager->getTxStatus())
		  {
			  commManager->handle();
		  }
}

void teamAt_GC9A01::sendData16Blocking(uint16_t c1)
{
	while(commManager->getTxStatus())
	  {
		  commManager->handle();
	  }

	modeData();
	commandBufferWritePos = 0;
	commandBuffer[commandBufferWritePos] = ((c1 >> 8) &  0x00ff);
	commandBufferWritePos++;
	commandBuffer[commandBufferWritePos] = (c1 & 0x00ff);
	commandBufferWritePos++;
	commManager->sendDataSharedBuffer(commandBufferWritePos,commandSpiIndex); //device index.

	while(commManager->getTxStatus())
		  {
			  commManager->handle();
		  }
}
