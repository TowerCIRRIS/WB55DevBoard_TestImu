/**
 * @file oledDisplayManager_V1_1.h
 * @author Simon Latour
 * @brief Used to facilitate the use display on the oled
 * @version 1.1.1
 * @date 2021-09-24
 * 
 * @copyright Copyright (c) 2020
 * 
 *  Version 1.1.1:
 *  - Separated horizontal padding and vertical pdding for some functions
 * Version 1.1.0:
 *  - Changes in widgets, created types
 * Version 1.0.2:
 *  - fix error in widget_valueSelectUpDown(..) that was not centering bottom arrow properly
 * 
 */
#ifndef OLED_DISPLAY_MANAGER_H
    #define OLED_DISPLAY_MANAGER_H


// Uncomment the following line to use the manager for DMA SPI 
#define DISPLAY_MANAGER_SPIDMA


    #include <vector>
    #include <string>
	//#include <String>
    #include <map>
#ifndef ARDUIO
using namespace std;
typedef string String;
#endif

    //#include "Adafruit_SSD1306.h"
    #include "readaptLogo.h"

    #ifdef DISPLAY_MANAGER_SPIDMA
        //#include "teamAt_SSD1306_dmaspi_V1_0.h"
		#include "teamAt_GC9A01_dma_V1_0.h"
    #endif

    
#include "teamAt_display_V1_0.h"

    #define SCREEN_WIDTH_128 128 // OLED display width, in pixels
    #define SCREEN_HEIGHT_32 32 // OLED display height, in pixels   
    #define SCREEN_HEIGHT_64 64 // OLED display height, in pixels 

    #define OLED_RESET_SHARED     -1 //4 // Reset pin # (Pin number or -1 if sharing Arduino reset pin)
    
    #define DISPLAY_MANAGER_ERROR_NO_ERROR  0
    #define DISPLAY_MANAGER_ERROR_BEGIN     1 

typedef struct{
	const uint8_t* address;
	uint16_t width;
	uint16_t height;
}image_t;

/**
 * @brief Used to defin the behavior of the list widget
 */
typedef enum{
        highlightMode_off,                 // Item not highlighted in any way
        highlightMode_centered,            // Focused item is centered in the view 
        highlightMode_Highlighted,         // Item highlighted with a box.  Menu only moves when item is out of the view area 
        highlightMode_Invisible,           // Same behavioir as  highlightMode_Highlighted but not box
        highlightMode_HighlightedCentered  // Both centered and box

    }highlightMode_t;

/**
 * @brief Used to enable borders of widget
 */
//  enum class borderStyle
//    {

// BORDER STYLE 0XAABBCCCC
// AA: unused / future use
// BB: uint8 Borders on / oo and types
// CCCC: uint16 color

#define GET_BORDERSTYLE_COLOR(x)   ((uint16_t)(x & 0x0000FFFF))
#define BORDERSTYLE_MASK 0x00FF0000
#define BORDERSTYLE_NONE   				(0x00 << 16)
#define	BORDERSTYLE_BOT    				(0x01 << 16)
#define BORDERSTYLE_LEFT   				(0x02 << 16)
#define BORDERSTYLE_RIGHT  				(0x04 << 16)
#define BORDERSTYLE_TOP    				(0x08 << 16)
#define BORDERSTYLE_ALL  				(0x0F << 16)
#define BORDERSTYLE_CIRCLE_CONTAINED 	(0x10 << 16)
#define BORDERSTYLE_CIRCLE  			(0x20 << 16)
//    };


// Text  STYLE 0XAAABCCCC
// AA: unused / future use
// BB: uint8 Text syze
// CCCC: uint16 color

#define GET_TEXTYLE_COLOR(x)   	((uint16_t)(x & 0x0000FFFF))
#define GET_TEXTYLE_SIZE(x)		((uint8_t)((x >>16) & 0x00FF))


#define TEXTSTYLE_SIZE_1	(1 << 16)
#define TEXTSTYLE_SIZE_2	(2 << 16)
#define TEXTSTYLE_SIZE_3	(3 << 16)
#define TEXTSTYLE_SIZE_4	(4 << 16)
#define TEXTSTYLE_SIZE_5	(5 << 16)
#define TEXTSTYLE_SIZE_6	(6 << 16)
/**
 * @brief Alignement for text and images
 */
enum class widgetAlignment
{
    center,
    centerLeft,
    centerRight,
    topCenter,
    topLeft,
    topRight,
    bottomCenter,
    bottomLeft,
    bottomRight
};

     
    // Widget Defines for backward compatibility
    #define W_LEFT_ALIGN    widgetAlignment::centerLeft
    #define W_CENTER_ALIGN  widgetAlignment::center
    #define W_RIGHT_ALIGN   widgetAlignment::centerRight
    #define W_TOP_ALIGN     widgetAlignment::topCenter  
    #define W_BOT_ALIGN     widgetAlignment::bottomCenter 

    #define W_NO_PADDING 0


class oledDisplayManager
{
    private:




    std::map<std::string,image_t> imageMap;

    // Widgets internalfunctions
    int16_t startTextVCenter(uint8_t textSize, int16_t zoneHeight);
    int16_t startTextHCenter(String str,uint8_t textSize, int16_t width);

    bool mForceSleepCommand = false;
    uint8_t m_WakeupCommand = 0;
    uint8_t m_sleepStatus = 1;
    uint32_t sleepTimer = 0;

    uint8_t m_sleepModeEnable = 0;
    uint8_t m_AutoSleepModeEnable = 0;
 

    uint32_t m_refreshRateTime = 0;
    uint32_t m_refreshRateTimer = 0;
    uint32_t m_LastExectutionTime = 0;

    int m_screenWidth = 128;
    int m_screenHeight = 32;


    
    public:




    oledDisplayManager();
	#if defined(DISPLAY_MANAGER_SPIDMA)
			oledDisplayManager(teamAt_display *dmaSpidisplay, uint32_t refreshPeriod = 0);
			teamAt_display *display;
	#else
	   Adafruit_SSD1306 display; // Display class

	   void init_I2C(int screenWidth, int screenHeight, TwoWire *twi = &Wire,int8_t rst_pin = -1, uint32_t refreshPeriode = 0);
	   void init_SPI(int screenWidth, int screenHeight, SPIClass *spi,int dc_pin, int rst_pin, int cs_pin, uint32_t bitrate = 8000000, uint32_t refreshPeriode=0);
	#endif

    /**
     * @brief Start the display manager and display
     *
     * @return uint8_t
     */
    uint8_t start();

    uint32_t m_SleepTimer = 0;
    uint32_t m_msSleepTime = 0;
    uint8_t m_newData = 0;




//    #if defined(DISPLAY_MANAGER_SPIDMA)
//        //teamAt_SSD1306 display;
//    teamAt_display *display;
//    #else
//        Adafruit_SSD1306 display; // Display class
//    #endif




  

    #define SMALL_FONT 1
    #define MED_FONT 2 
    #define LARGE_FONT 3 

    #define SMALL_FONT_ROW_HEIGH 9
    #define MED_FONT_ROW_HEIGHT (SMALL_FONT_ROW_HEIGH * 2)
    #define LARGE_FONT_ROW_HEIGHT (SMALL_FONT_ROW_HEIGH * 3)

    #define MENU_SCROLLBAR_WIDTH  12
    #define MENU_TITLE_BAR_HEIGHT 12
   
    #define SCROLL_UP_MASK 0b00000001
    #define SCROLL_DOWN_MASK 0b00000010

    

    /**
     * @brief Initialization of the display manager
     * 
     * @param screenWidth       width of the screen in pixel
     * @param screenHeight      height of the screen in pixel
     * @param twi               Pointer to the Two Wire interface 
     * @param rst_pin           Reset pin ( -1 if sharde with board) 
     * @param refreshPeriode    Screen refresh periode in ms
    */
   // void init_I2C(int screenWidth, int screenHeight, TwoWire *twi = &Wire,int8_t rst_pin = -1, uint32_t refreshPeriode = 0);
   	
    /**
     * @brief Initialization of the display manager
     * 
     * @param screenWidth       width of the screen in pixel
     * @param screenHeight      height of the screen in pixel
     * @param spi               SPI port
     * @param dc_pin            DC pin
     * @param rst_pin           Reset pin
     * @param cs_pin            Chipselect
     * @param bitrate           data bitrate
     * @param refreshPeriode    refresh periode in milliseconds
     */
   // void init_SPI(int screenWidth, int screenHeight, SPIClass *spi,int dc_pin, int rst_pin, int cs_pin, uint32_t bitrate = 8000000, uint32_t refreshPeriode=0);
    #ifdef DISPLAY_MANAGER_SPIDMA
    	void initDmaSpi(int screenWidth, int screenHeight,dmaSpiManagerClass *spiCommPtr,int8_t rst_pin = -1,int8_t dc_pin = -1,int8_t cs_pin = -1, uint32_t refreshPeriode = 0);
	#endif

    /**
     * @brief Enable the sleep mode
     * 
     * @param msTime The time before in ms before the unit falls into sleep
     */
    void enableSleepMode(uint32_t msTime);
    
    /**
     * @brief Disable the sleep mode of the display
     */
    void disableAutoSleepMode();

    /**
     * @brief Get the Sleep Status object
     * 
     * @return true if display is sleeping
     * @return false if awake
    */
    bool getSleepStatus();

    void sleep();
    void wakeUp();

    /**
     * @brief Main task fo the display manager. Shoudl be call in the main loop of the program 
     * 
     * @param timeMillis is the system time in millisecond. This will be used to manage the refreash rate, 
     * sleep and other time based functionalities
     */
    void run(uint32_t timeMillis = 0);


    void needRefresh();

    void displaySlpashScreen();

    /**
     * @brief Quickly write a message with basic formatting.
     * 
     * For more control, use display functions  
     * Ex: displayManager.display.setTextSize();
     */
    void writeMessage(String stringsent);

    // return screen info
    /**
     * @brief Return the configured screen width
     * @return uint16_t width in pixels
     */
    uint16_t screenWidth();

    /**
     * @brief Return the configured screen height
     * @return height width in pixels
     */
    uint16_t screenHeight();

    //Drink reminder standard menu layout
    void displaySubMenuLayout(const char* title);   
    void displayScrollDownArrow();
    void displayScrollUpArrow();


    /**
    * Was previously called displayScrollbox()
    * @brief Displays a scrollable menu with a list of items
    * 
    * @param itemSelect        The highlited / on focus item
    * @param itemCount         The number of item in the list
    * @param itemLabelList     The list with labels for all items
    * @param multiLine         Display more thant one line?
    * @param textSize          Size of text
    * @param nextMenuPointer   If this is used to go to another sub menu level, display an arrow next to the text
     */
    void displayScrollMenu(unsigned char itemSelect, unsigned char itemCount, const char*  itemLabelList[], bool multiLine, uint8_t textSize, bool nextMenuPointer);
    
    /**
     * @brief Display an item list
     * 
     * @param xPos          starting z position
     * @param yPos          starting y position
     * @param height        Height of the area
     * @param width         width of the area
     * @param itemSelect    Focused item / list element
     * @param itemCount     number of elements in the list
     * @param itemLabelList List of labels for the elements
     * @param highlightmode Highlight mode, /see highlightMode_t for description of behaviors
     * @param textSize      size of text 1,2,3
     * @param hAlignment    horizontal alignement, see 
     * @param padding       add paddign if needed to move a bit the menu
     
     * @return scrollable up or down status ( usefull to display a scroll bar based on the list position)
     *          0b00 or 0 = no scroll possible
     *          0b01 or 1 = scroll up possible
     *          0b10 or 2 =  scroll down possiblew
     *          0b11 or 4 = scroll up and down possible
     */
    uint8_t displayList(int xPos, int yPos, int height, int width, unsigned char itemSelect, unsigned char itemCount, std::vector<std::string>  &itemLabelList,highlightMode_t highlightmode = highlightMode_Highlighted, uint32_t textStyle = TEXTSTYLE_SIZE_1,widgetAlignment hAlignment=widgetAlignment::centerLeft, int16_t hPadding = W_NO_PADDING, int16_t vPadding= W_NO_PADDING);
    
    /**
     * @brief Display a drinking flass
     * 
     * @param xPos  x position of top left corner    
     * @param yPos  y position of top left corner 
     * @param height  height of the glass
     * @param width   width of the glass
     * @param fillPercent Will fill the glass according to this % 
     */
    void displayGlass(int xPos, int yPos, int height, int width,int fillPercent);

    /**
     * @brief Display an horizontal battery indicator
     * 
     * @param xPos  x position of top left corner
     * @param yPos  y position of top left corner 
     * @param height heigh of the drawing
     * @param width  width of the drawing
     * @param fillPercent Will fill the battery according to this %
     * @param color of the drawing for color LCDs
     */
    void displayBattery(int xPos, int yPos, int height, int width,int fillPercent,uint16_t color = 0xFFFF);

    // Widgets that display 1 or multipel rows of text with configurable style
    void widget_3Row(int16_t posX,int16_t posY, int16_t width, int16_t height,String text1,uint32_t text1Style, String text2,uint32_t text2Style, String text3,uint32_t text3Style, widgetAlignment hAlignment= widgetAlignment::center, int16_t padding= 0, uint32_t bStylebStyle = BORDERSTYLE_NONE);
    void widget_2Row(int16_t posX,int16_t posY, int16_t width, int16_t height,String text1,uint32_t text1Style, String text2,uint32_t text2Style, widgetAlignment hAlignment = widgetAlignment::center, int16_t padding= 0, uint32_t bStylebStyle = BORDERSTYLE_NONE);
    //void widget_1row(int16_t posX,int16_t posY, int16_t width, int16_t height,String text1,uint8_t text1Scale = 1, widgetAlignment hAlignment = widgetAlignment::center, int16_t padding = 0,borderStyle bStyle = borderStyle::none);
    void widget_1row(int16_t posX,int16_t posY, int16_t width, int16_t height,std::string text1,uint32_t text1Style = TEXTSTYLE_SIZE_1, widgetAlignment hAlignment = widgetAlignment::center, int16_t hPadding = W_NO_PADDING, int16_t vPadding= W_NO_PADDING,uint32_t bStylebStyle = BORDERSTYLE_NONE);
    
    // Widget to show a division one text over another
    void widget_divisor(int16_t posX,int16_t posY, int16_t width, int16_t height,String textTop,String textbot,uint8_t textScale,int16_t hPadding = W_NO_PADDING, int16_t vPadding= W_NO_PADDING);

    // Widget for value selection with an arrow above and below. 
    void widget_valueSelectUpDown(int16_t posX,int16_t posY, int16_t width, int16_t height,String valueText, bool displayTopArow = false, bool DisplayBotArrow = false ,uint8_t textScale = 1, widgetAlignment hAlignment = widgetAlignment::center);
    
    // Widget to draw a vertical scroll bar
    void widget_scrollBarVertical(int16_t posX,int16_t posY, int16_t width, int16_t height,bool displayTopArow, bool DisplayBotArrow, uint32_t bStyle = BORDERSTYLE_NONE);
    // Widget to draw an horizontal scroll bar
    void widget_scrollBarHorizontal(int16_t posX,int16_t posY, int16_t width, int16_t height,bool displayTopArow, bool DisplayBotArrow, uint32_t bStyle = BORDERSTYLE_NONE );
    
    // alignment functiosn used to get the correct X and Y values to used to match desired alignement
    //int16_t HAlignText(String str,uint8_t textSize, int16_t width, int16_t padding,widgetAlignment hAlignment = widgetAlignment::center);
    int16_t HAlignText(std::string str,uint8_t textSize, int16_t width, int16_t padding, widgetAlignment alignment);
    int16_t VAlignText(String str,uint8_t textSize, int16_t areaHeight, int16_t padding,widgetAlignment alignment = widgetAlignment::center);
    int16_t HAlignImage(int16_t imageWidth, int16_t areaWidth, int16_t padding, widgetAlignment alignment = widgetAlignment::center);
    int16_t VAlignImage(int16_t imageHeight, int16_t areaHeight, int16_t padding, widgetAlignment alignment = widgetAlignment::center);

    //Draw some edges of a rectangle base on the desired borderStyle \see borderStyle enum
    void drawBorder(int16_t posX,int16_t posY, int16_t width, int16_t height,uint32_t bStyle);

};


#endif

