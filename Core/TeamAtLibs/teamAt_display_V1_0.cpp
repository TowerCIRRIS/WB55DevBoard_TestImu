

#include <stdlib.h>
#include <string.h>
#include "teamAt_display_V1_0.h"

#include "../GFX_Library/TeamAT_GFX.h"

// SOME DEFINES AND STATIC VARIABLES USED INTERNALLY -----------------------




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
teamAt_display::teamAt_display( uint16_t w, uint16_t h):
		TeamAT_GFX(w, h), buffer(NULL)
{

}

///*!
//    @brief  Destructor for teamAt_SSD1306 object.
//*/
//teamAt_display::~teamAt_display(void) {
//
//}


//void teamAt_display::displayOn(void){
//
//}
//
//void teamAt_display::displayOff(void){
//
//}


// LOW-LEVEL UTILS ---------------------------------------------------------




// ALLOCATE & INIT DISPLAY -------------------------------------------------

// /*!
//     @brief  Allocate RAM for image buffer, initialize peripherals and pins.
//     @param  vcs
//             VCC selection. Pass SSD1306_SWITCHCAPVCC to generate the display
//             voltage (step up) from the 3.3V source, or SSD1306_EXTERNALVCC
//             otherwise. Most situations with Adafruit SSD1306 breakouts will
//             want SSD1306_SWITCHCAPVCC.
    
//     @param  reset
//             If true, and if the reset pin passed to the constructor is
//             valid, a hard reset will be performed before initializing the
//             display. If using multiple SSD1306 displays on the same bus, and
//             if they all share the same reset pin, you should only pass true
//             on the first display being initialized, false on all others,
//             else the already-initialized displays would be reset. Default if
//             unspecified is true.
//     @return true on successful allocation/init, false otherwise.
//             Well-behaved code should check the return value before
//             proceeding.
//     @note   MUST call this function before any drawing or updates!
// */
//bool teamAt_display::begin() {
//
//   return true; // Success
//}

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
//void teamAt_display::drawPixel(int16_t x, int16_t y, uint16_t color) {
//
//}

/*!
    @brief  Clear contents of display buffer (set all pixels to off).
    @return None (void).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
//void teamAt_display::clearDisplay(void) {
//
//}

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
//void teamAt_display::drawFastHLine(int16_t x, int16_t y, int16_t w,
//                                     uint16_t color) {
//
//
//}

//void teamAt_display::drawFastHLineInternal(int16_t x, int16_t y, int16_t w,
//                                             uint16_t color) {
//
//}

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
//void teamAt_display::drawFastVLine(int16_t x, int16_t y, int16_t h,
//                                     uint16_t color) {
//}

//void teamAt_display::drawFastVLineInternal(int16_t x, int16_t __y,
//                                             int16_t __h, uint16_t color) {
//
//}

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
//bool teamAt_display::getPixel(int16_t x, int16_t y) {
//  return false; // Pixel out of bounds
//}

// /*!
//     @brief  Get base address of display data buffer for direct reading or writing.
//     @return Pointer to an unsigned 8-bit array, column-major, columns padded
//             to full byte boundary if needed.
// */
// uint8_t *teamAt_display::getBuffer(void) { return buffer; }

// /**
//  * @brief Get base address of display command buffer for direct reading or writing.
//  * 
//  * @return uint8_t* Pointer to an unsigned 8-bit array
//  */
// uint8_t *teamAt_display::getCommandBuffer(void) { return commandBuffer; } 

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
//void teamAt_display::display(void) {
//
//}

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
//void teamAt_display::startscrollright(uint8_t start, uint8_t stop) {
//
//}

/*!
    @brief  Activate a left-handed scroll for all or part of the display.
    @param  start
            First row.
    @param  stop
            Last row.
    @return None (void).
*/
// To scroll the whole display, run: display.startscrollleft(0x00, 0x0F)
//void teamAt_display::startscrollleft(uint8_t start, uint8_t stop) {
//
//
//}

/*!
    @brief  Activate a diagonal scroll for all or part of the display.
    @param  start
            First row.
    @param  stop
            Last row.
    @return None (void).
*/
// display.startscrolldiagright(0x00, 0x0F)
//void teamAt_display::startscrolldiagright(uint8_t start, uint8_t stop) {
//
//}

/*!
    @brief  Activate alternate diagonal scroll for all or part of the display.
    @param  start
            First row.
    @param  stop
            Last row.
    @return None (void).
*/
// To scroll the whole display, run: display.startscrolldiagleft(0x00, 0x0F)
//void teamAt_display::startscrolldiagleft(uint8_t start, uint8_t stop) {
//
//}

/*!
    @brief  Cease a previously-begun scrolling action.
    @return None (void).
*/
//void teamAt_display::stopscroll(void) {
//
//}

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
//void teamAt_display::invertDisplay(bool i) {
//
//}

/*!
    @brief  Dim the display.
    @param  dim
            true to enable lower brightness mode, false for full brightness.
    @return None (void).
    @note   This has an immediate effect on the display, no need to call the
            display() function -- buffer contents are not changed.
*/
//void teamAt_display::dim(bool dim) {
//
//}

//void teamAt_display::handle(void)
//{
//
//}

//void teamAt_display::waitForTxComplete()
//{
//
//}


