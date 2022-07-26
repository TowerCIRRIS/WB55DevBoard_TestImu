/**
 * @file oledDisplayManager_V1_0.cpp
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

#include "oledDisplayManager_V1_1.h"
#include "readaptLogo.h"
#include "math.h"

  
#ifdef DISPLAY_MANAGER_SPIDMA
	oledDisplayManager::oledDisplayManager(teamAt_display *dmaSpidisplay, uint32_t refreshPeriod )
	{
		display = dmaSpidisplay;

		m_refreshRateTime = refreshPeriod;
		//
		m_screenWidth = display->width();
		m_screenHeight = display->height();


		 image_t image;

		image.height =  arrowRightSmall_height;
		image.width = arrowRightSmall_width;
		image.address = arrowRightSmall;

		imageMap["arrowRightSmall"]=image;

		image.height =  arrowLeftSmall_height;
		image.width = arrowLeftSmall_width;
		image.address = arrowLeftSmall;

		imageMap["arrowLeftSmall"]=image;

	}
#endif

oledDisplayManager::oledDisplayManager(){

   image_t image;   

    image.height =  arrowRightSmall_height;
    image.width = arrowRightSmall_width;
    image.address = arrowRightSmall;

    imageMap["arrowRightSmall"]=image;

    image.height =  arrowLeftSmall_height;
    image.width = arrowLeftSmall_width;
    image.address = arrowLeftSmall;

    imageMap["arrowLeftSmall"]=image;

//   imageMap["arrowDown"]="another element";
//   imageMap["arrowLeft"]=mymap['b'];
//   imageMap["arrowRight"]  
//   imageMap["arrowRightSmall"]
//   imageMap["arrowLeftSmall"] 

}

/*void oledDisplayManager::init_I2C(int screenWidth, int screenHeight, TwoWire *twi,int8_t rst_pin, uint32_t refreshPeriode)
{
    #if !defined(DISPLAY_MANAGER_SPIDMA)
        display = Adafruit_SSD1306(screenWidth, screenHeight, twi, rst_pin);
    #endif

    m_refreshRateTime = refreshPeriode;
    m_screenWidth = screenWidth;
    m_screenHeight = screenHeight;

    start();
} */

//void oledDisplayManager::init_SPI(int screenWidth, int screenHeight, SPIClass *spi,int dc_pin, int rst_pin, int cs_pin, uint32_t bitrate , uint32_t refreshPeriode)
//{
//
//
//    //display =Adafruit_SSD1306(screenWidth, screenHeight, spi, dc_pin, rst_pin, cs_pin);
//     #if !defined(DISPLAY_MANAGER_SPIDMA)
//        display = Adafruit_SSD1306(screenWidth, screenHeight, spi, dc_pin, rst_pin, cs_pin, bitrate);
//    #endif
//    m_refreshRateTime = refreshPeriode;
//
//     m_screenWidth = screenWidth;
//    m_screenHeight = screenHeight;
//
//    start();
//}

#ifdef DISPLAY_MANAGER_SPIDMA
void oledDisplayManager::initDmaSpi(int screenWidth, int screenHeight,dmaSpiManagerClass *spiCommPtr,int8_t rst_pin,int8_t dc_pin,int8_t cs_pin, uint32_t refreshPeriode)
{
//    #if defined(DISPLAY_MANAGER_SPIDMA)
//        //display =  teamAt_SSD1306(spiCommPtr,screenWidth, screenHeight, dc_pin,rst_pin,cs_pin);
//		display =  teamAt_GC9A01(spiCommPtr,screenWidth, screenHeight, dc_pin,rst_pin,cs_pin);
//    #endif

    m_refreshRateTime = refreshPeriode;

     m_screenWidth = screenWidth;
    m_screenHeight = screenHeight;

    start();
}
#endif


uint8_t oledDisplayManager::start()
{
    //uint8_t error = 1;

    #if defined(DISPLAY_MANAGER_SPIDMA)

        // Start display using 3.3V power (Address 0x3C for 128x32)
        //if(!display->begin(SSD1306_SWITCHCAPVCC))
    	if(!display->begin())
        {  
            #ifdef DEBUG_INFO_SERIAL 
                Serial.println(F("initialisation SSD1306 non réussie"));
            #endif

            return DISPLAY_MANAGER_ERROR_BEGIN;
        }
        else 
        {  
          #ifdef DEBUG_INFO_SERIAL
            Serial.println(F("SSD1306 allocation success"));
          #endif
        }

        display->waitForTxComplete(); //Block code to send data
        
    #else
    {
         // Start display using 3.3V power (Address 0x3C for 128x32)
        if(!display->begin(SSD1306_SWITCHCAPVCC, 0x3C))
        {  
            #ifdef DEBUG_INFO_SERIAL 
                Serial.println(F("initialisation SSD1306 non réussie"));
            #endif

            return DISPLAY_MANAGER_ERROR_BEGIN;
        }
        else {  
          // Vide le tampon de l'afficheur.  
          //display->clearDisplay();
          //display->display();
          #ifdef DEBUG_INFO_SERIAL
            Serial.println(F("SSD1306 allocation success"));
          #endif

        }

    }
    #endif
     
   
        

    return DISPLAY_MANAGER_ERROR_NO_ERROR;
}

void oledDisplayManager::enableSleepMode(uint32_t msTime)
{
    m_AutoSleepModeEnable = 1;
    m_msSleepTime = msTime;
}

void oledDisplayManager::disableAutoSleepMode()
{
    m_AutoSleepModeEnable = 0;
}


bool oledDisplayManager::getSleepStatus()
{
    return mForceSleepCommand;
}

void oledDisplayManager::run(uint32_t timeMillis)
{
    uint32_t deltaTime,deltaSleep;

    deltaTime = timeMillis - m_LastExectutionTime;
    m_LastExectutionTime = timeMillis;

    m_SleepTimer += deltaTime;
    m_refreshRateTimer += deltaTime;

    if(m_WakeupCommand || (m_newData && !mForceSleepCommand) ) // New Data wakes-up from auto-sleep display
    {
        m_sleepStatus = 0;
        m_WakeupCommand = 0;
        m_SleepTimer = 0;
        display->displayOn(); //  display->ssd1306_command(SSD1306_DISPLAYON);
        #if defined(DISPLAY_MANAGER_SPIDMA)
            display->handle(); // Run Comm loop task
            display->waitForTxComplete();
        #endif
        
    }

    if(m_newData && (m_refreshRateTimer >= m_refreshRateTime ))
    {
        m_refreshRateTimer = 0;
        m_newData = 0;
        m_SleepTimer = 0;
        display->display();
    }
    
    // If auto sleep or force sleep, turn device off
    if( ((m_AutoSleepModeEnable && (m_SleepTimer >= m_msSleepTime)) || mForceSleepCommand) && !m_sleepStatus)
    {
        //mForceSleepCommand = 1;
        m_sleepStatus = 1;
        display->displayOff(); //display->ssd1306_command(SSD1306_DISPLAYOFF);
        #if defined(DISPLAY_MANAGER_SPIDMA)
            display->handle(); // Run Comm loop task
            display->waitForTxComplete();
        #endif
     }

    #if defined(DISPLAY_MANAGER_SPIDMA)
       display->handle(); // Run Comm loop task
    #endif
}

void oledDisplayManager::writeMessage(String stringsent)
{

    display->clearDisplay();
    display->setTextSize(1);             // Normal 1:1 pixel scale
    display->setTextColor(WHITE);        // Draw white text
    display->setCursor(0,4);             // Start at top-left corner
    //TODO display->println(stringsent);
    m_newData = 1;
}


uint16_t oledDisplayManager::screenWidth()
{
    return m_screenWidth;
}

uint16_t oledDisplayManager::screenHeight()
{
    return m_screenHeight;
}

/**
 * @brief This functions indicated to the display manager that the display needs to be refreshed.
 *          Use this function after direct call to the display.
*/
void oledDisplayManager::needRefresh()
{
     m_newData = 1;
}

/**
 * @brief Wake up the display
 */
void oledDisplayManager::wakeUp()
{
    // turn on display
    // reset time
    m_WakeupCommand = 1;
    mForceSleepCommand = 0;
}

/**
 * @brief Forces the display to sleep
 */
void oledDisplayManager::sleep()
{
    mForceSleepCommand = 1;
}

 /**
 * @brief Shows the splash screen and pause for 3 seconds.
 */
void oledDisplayManager::displaySlpashScreen()
{

    display->clearDisplay();
    display->setTextSize(1);             // Normal 1:1 pixel scale
    display->setTextColor(WHITE);        // Draw white text
    display->setCursor(12,15);             // Start at top-left corner
    //display->println((String)"IngReadaptUL");
    display->drawBitmap(90,1,irul_logo_map,31,30,BLACK,WHITE);
    display->display();
}

#define TOP_LINE (MENU_TITLE_BAR_HEIGHT-1)
#define TEXT_SPACING    9 
#define SCROLL_BAR_WIDTH 12
/**
 * @brief Creates a standard sub menu layout
 * 
 * @param title The title of the menu
 */
void oledDisplayManager::displaySubMenuLayout(const char* title)
{
    int16_t yPos = VAlignImage(arrowLeftSmall_height, MENU_TITLE_BAR_HEIGHT, 0,widgetAlignment::center);
    display->drawBitmap(0,yPos,arrowLeftSmall,arrowLeftSmall_width,arrowLeftSmall_height,BLACK,WHITE);
    //  display->setTextSize(1);
    //  display->setCursor(6,0);
    //  display->setTextColor(WHITE);
    //  display->write(title);
    widget_1row(0,0, m_screenWidth, MENU_TITLE_BAR_HEIGHT,String(title).c_str(),TEXTSTYLE_SIZE_1,widgetAlignment::center, 0,0,BORDERSTYLE_BOT);
    
     //display->drawLine(0,9,m_screenWidth,TOP_LINE,WHITE);
     //display->drawLine(m_screenWidth-SCROLL_BAR_WIDTH,TOP_LINE,m_screenWidth-SCROLL_BAR_WIDTH,m_screenHeight,WHITE);
}


/**
 * @brief To display if page can be scrolled down
 * 
 */
void oledDisplayManager::displayScrollDownArrow()
{
    display->drawBitmap(m_screenWidth - 9,(m_screenHeight-8),arrowDown,9,6,BLACK,WHITE);
}

/**
 * @brief To display if page can be scrolled up
 * 
 */
void oledDisplayManager::displayScrollUpArrow()
{
    display->drawBitmap(m_screenWidth - 9,TOP_LINE+4,arrowUp,9,6,BLACK,WHITE);
}

void oledDisplayManager::displayScrollMenu(unsigned char itemSelect, unsigned char itemCount, const char*  itemLabelList[],bool multiLine, uint8_t textSize, bool nextMenuPointer)
{
    int cursorPos;

    display->setTextSize(textSize);
    
    display->setTextColor(WHITE);
    cursorPos = TOP_LINE+6;
    display->setCursor(0,cursorPos);
    display->write(itemLabelList[itemSelect]);
    if(nextMenuPointer)
    {
        display->drawBitmap(display->getCursorX()+4, 15 ,arrowRightSmall,3,5,BLACK,WHITE);
    } 

    if(itemSelect > 0)
    {
        displayScrollUpArrow();
    }
    if(itemSelect < (itemCount-1))
    {
        displayScrollDownArrow();
        if(multiLine)
        {

            cursorPos += TEXT_SPACING; 

            for(int i = 1; (itemSelect+i) < itemCount;i++)
            {
                display->setCursor(0,cursorPos);
                display->write(itemLabelList[itemSelect+i]);
                cursorPos += TEXT_SPACING;

                if(cursorPos > m_screenHeight)
                {
                        i = itemCount; // exit 
                }
            }

            // display->setCursor(0,25);
            // display->write(itemLabelList[itemSelect+1]);
        }
        
    }
}

        
uint8_t oledDisplayManager::displayList(int xPos, int yPos, int height, int width, unsigned char itemSelect, unsigned char itemCount, std::vector<std::string>  &itemLabelList, highlightMode_t highlightmode,  uint32_t textStyle, widgetAlignment alignement, int16_t hPadding, int16_t vPadding)
{
 int cursorPosY = yPos;
 //int cursorPosX = xPos;
 int firstItemIndex;
 int rowHeight = GET_TEXTYLE_SIZE(textStyle) * SMALL_FONT_ROW_HEIGH;
 uint8_t scrollUp = 0;
 uint8_t scrollDown = 0;
 
 int nbElementsMax = height / rowHeight;
    

    if(highlightmode == highlightMode_centered || highlightmode == highlightMode_HighlightedCentered){
        firstItemIndex = itemSelect;
    }
    else
    {
        if((itemSelect+1) >= nbElementsMax)
        {
            firstItemIndex = (itemSelect+1)-nbElementsMax;
        }
        else{
            firstItemIndex = 0;
        }

        
    }
    if((highlightmode == highlightMode_centered) || (highlightmode == highlightMode_HighlightedCentered))
    {
        if(firstItemIndex > 0)
        {
            scrollUp = SCROLL_UP_MASK;
        }
    }
    else
    {
        if(itemSelect > 0)
        {
            scrollUp = SCROLL_UP_MASK;
        }
    }
    
       


    display->setTextSize(GET_TEXTYLE_SIZE(textStyle));
   

    for(int i = firstItemIndex; i < (itemCount); i++){

        if(i == itemSelect && (highlightmode == highlightMode_HighlightedCentered || highlightmode == highlightMode_Highlighted))
        {
            display->drawRect(xPos,cursorPosY,width,rowHeight,GET_TEXTYLE_COLOR(textStyle));
        }
        else
        {
            display->setTextColor(GET_TEXTYLE_COLOR(textStyle));
        }

        // std::size_t pos = itemLabelList[i].find("/img/");
        // if(pos != std::string::npos)
        // {
        //    std::string imageName = itemLabelList[i].substr(pos+5);
        //    std::map<std::string,image_t>::iterator it;
           
        //    it = imageMap.find(imageName);

        //    if(it != imageMap.end())
        //    {
        //         int16_t xOffset = HAlignImage(imageMap[imageName].width, width, 2, alignement);
        //         int16_t yOffset = VAlignImage(imageMap[imageName].height,rowHeight,0,widgetAlignment::center);

        //      display->drawBitmap(xPos+xOffset,cursorPosY+yOffset,imageMap[imageName].address,imageMap[imageName].width,
        //                          imageMap[imageName].height,BLACK,WHITE);
        //     //widget_1row(xPos,cursorPosY,width, rowHeight,imageName.c_str(),textSize, alignement,padding);
        //    }

            
        // }
        // else
        // {
            widget_1row(xPos,cursorPosY,width, rowHeight,itemLabelList[i].c_str(),textStyle, alignement,hPadding,vPadding);
        //}

        

        cursorPosY += (SMALL_FONT_ROW_HEIGH *GET_TEXTYLE_SIZE(textStyle)); // TEXT_SPACING;

        if(cursorPosY + rowHeight > (yPos+height))
        {
                i = itemCount; // exit 
        }
    }

    if((highlightmode == highlightMode_centered) || (highlightmode == highlightMode_HighlightedCentered))
    {
        if( firstItemIndex != (itemCount-1))
        {
            scrollDown = SCROLL_DOWN_MASK;
        }
    }
    else{
        if( itemSelect != (itemCount-1))
        {
            scrollDown = SCROLL_DOWN_MASK;
        }
    }

    return scrollUp + scrollDown;
}


void oledDisplayManager::displayGlass(int xPos, int yPos, int height, int width,int fillPercent)
{
   int glassSideAngle = 2;
   int endX = (xPos - 1 + width );

   // Glass
   display->drawLine(xPos,yPos,xPos+glassSideAngle,yPos+height,WHITE);
   display->drawLine(endX,yPos,endX-glassSideAngle,yPos+height,WHITE);
   display->drawLine(xPos+glassSideAngle,yPos+height,endX - glassSideAngle,yPos+height,WHITE);

   //FILL
   
    int rectHeigh =  (height-4) * fillPercent / 100;
    int rectPositionY = yPos + ((height-2) - rectHeigh);
    int rectPositionX = xPos + glassSideAngle + 2; 
    int rectWidth = (width - (2*(glassSideAngle + 2) ));
    display->fillRect(rectPositionX,rectPositionY,rectWidth,rectHeigh,WHITE);
            
}


void oledDisplayManager::displayBattery(int xPos, int yPos, int height, int width,int fillPercent,uint16_t color)
{
   int glassSideAngle = 2;
   
   int buttonHeight = height*3/5;
   int buttonWidth = 1+width/10;
   int coreWidth = width-(buttonWidth);
   int endX = xPos + coreWidth-1;

   // Battery
        display->drawRoundRect(xPos,yPos,coreWidth,height,2,color);                       //Core
        display->drawRoundRect(endX,yPos+(height/2)-(buttonHeight/2),buttonWidth,buttonHeight,2,color);   // positive button

   //FILL
    int rectWidth =  (coreWidth-4) * fillPercent / 100;
    int rectPositionY = yPos + 2; // ((height-2) - rectHeigh);
    int rectPositionX = xPos + 2; // glassSideAngle + 2; 
    int rectHeight = height - 4; // (width - (2*(glassSideAngle + 2) ));
    display->fillRect(rectPositionX,rectPositionY,rectWidth,rectHeight,color);
            
}


void oledDisplayManager::widget_3Row(int16_t posX,int16_t posY, int16_t width, int16_t height,String text1,uint32_t text1Style, String text2,uint32_t text2Style, String text3,uint32_t text3Style, widgetAlignment alignement, int16_t padding, uint32_t bStyle)
{
    uint16_t textStartX;
    uint16_t textStartY;
    int16_t sectionHeight1,sectionHeight2,sectionHeight3;
    
    int16_t text1Scale = GET_TEXTYLE_SIZE(text1Style);
    int16_t text2Scale = GET_TEXTYLE_SIZE(text2Style);
    int16_t text3Scale = GET_TEXTYLE_SIZE(text3Style);

    //Section 1
    textStartX = posX + HAlignText(text1.c_str(), text1Scale, width,padding, alignement);//startTextHCenter(text1, text1Scale, width);
    sectionHeight1 = (float)height *  (float)text1Scale/ (float)(text1Scale+text2Scale+text3Scale);
    textStartY = posY + startTextVCenter(text1Scale, sectionHeight1) ;
    display->setCursor(textStartX,textStartY);
    display->setTextSize(text1Scale);
    display->setTextColor(GET_TEXTYLE_COLOR(text1Style));
    display->write(text1.c_str());

    //Section 2
    textStartX = posX + HAlignText(text2.c_str(), text2Scale, width,padding, alignement);//startTextHCenter(text2, text2Scale, width);
    sectionHeight2 = (float)height *  (float)text2Scale/ (float)(text1Scale+text2Scale+text3Scale);
    textStartY = posY + sectionHeight1+startTextVCenter(text2Scale, sectionHeight2) ;
    display->setCursor(textStartX,textStartY);
    display->setTextSize(text2Scale);
    display->setTextColor(GET_TEXTYLE_COLOR(text2Style));
    display->write(text2.c_str());

    //Section 3
    textStartX = posX + HAlignText(text3.c_str(), text3Scale, width,padding, alignement);//startTextHCenter(text3, text3Scale, width);
    sectionHeight3 = (float)height *  (float)text3Scale/ (float)(text1Scale+text2Scale+text3Scale);
    textStartY = posY + sectionHeight1 + sectionHeight2 + startTextVCenter(text3Scale, sectionHeight3) ;
    display->setCursor(textStartX,textStartY);
    display->setTextSize(text3Scale);
    display->setTextColor(GET_TEXTYLE_COLOR(text3Style));
    display->write(text3.c_str());

    drawBorder(posX,posY,width,height,(uint32_t)bStyle);

}


void oledDisplayManager::widget_2Row(int16_t posX,int16_t posY, int16_t width, int16_t height,String text1,uint32_t textStyle, String text2,uint32_t text2Style, widgetAlignment alignement, int16_t padding, uint32_t bStyle)
{
    uint16_t textStartX;
    uint16_t textStartY;
    int16_t sectionHeight1,sectionHeight2;
    
    int16_t text1Scale = GET_TEXTYLE_SIZE(textStyle);
    int16_t text2Scale = GET_TEXTYLE_SIZE(text2Style);


    //Section 1
    
    textStartX = posX + HAlignText(text1.c_str(), text1Scale, width,padding, alignement);
    sectionHeight1 = (float)height *  (float)text1Scale/ (float)(text1Scale+text2Scale);
    textStartY = posY + startTextVCenter(text1Scale, sectionHeight1) ;
    display->setCursor(textStartX,textStartY);
    display->setTextSize(text1Scale);
    display->setTextColor(GET_TEXTYLE_COLOR(textStyle));
    display->write(text1.c_str());

    //Section 2
    textStartX = posX + HAlignText(text2.c_str(), text2Scale, width,padding, alignement);
    sectionHeight2 = (float)height *  (float)text2Scale/ (float)(text1Scale+text2Scale);
    textStartY = posY + sectionHeight1+startTextVCenter(text2Scale, sectionHeight2) ;
    display->setCursor(textStartX,textStartY);
    display->setTextSize(text2Scale);
    display->setTextColor(GET_TEXTYLE_COLOR(text2Style));
    display->write(text2.c_str());

    drawBorder(posX,posY,width,height,(uint32_t)bStyle);
}

// void oledDisplayManager::widget_1row(int16_t posX,int16_t posY, int16_t width, int16_t height,String text1,uint8_t text1Scale, widgetAlignment alignement, int16_t padding, borderStyle bStyle)
// {
    
//     widget_1row(posX,posY, width, height,text1.c_str(),text1Scale, alignement, padding, bStyle);
// }

void oledDisplayManager::widget_1row(int16_t posX,int16_t posY, int16_t width, int16_t height,std::string text1,uint32_t text1Style, widgetAlignment alignement, int16_t hPadding , int16_t vPadding, uint32_t bStyle)
{
    uint16_t textStartX;
    uint16_t textStartY;
    

    int16_t text1Scale = GET_TEXTYLE_SIZE(text1Style);

    std::size_t imagePos = text1.find("/img/");
    if(imagePos != std::string::npos)
    {
        std::string imageName = text1.substr(imagePos+5);
        std::map<std::string,image_t>::iterator it;
        
        it = imageMap.find(imageName);

        if(it != imageMap.end())
        {
            int16_t xOffset = HAlignImage(imageMap[imageName].width, width, 2, alignement);
            int16_t yOffset = VAlignImage(imageMap[imageName].height,height,0, alignement);

            display->drawBitmap(posX+xOffset,posY+yOffset,imageMap[imageName].address,imageMap[imageName].width,
                                imageMap[imageName].height,GET_TEXTYLE_COLOR(text1Style),display->getBackgroundColor());
        
        }
    }
    else
    {
    	//int16_t sectionHeight1;

        //Section 1
        textStartX = posX + HAlignText(text1.c_str(), text1Scale, width, hPadding, alignement);
        //sectionHeight1 = (float)height *  (float)text1Scale/ (float)(text1Scale);
        textStartY = posY + VAlignText(text1.c_str(),text1Scale,height,vPadding,alignement);//startTextVCenter(text1Scale, sectionHeight1) ;
        display->setCursor(textStartX,textStartY);
        display->setTextSize(text1Scale);
        display->setTextColor(GET_TEXTYLE_COLOR(text1Style));
        display->write(text1.c_str());
    }
   
    drawBorder(posX,posY,width,height,(uint32_t)bStyle);
}

void oledDisplayManager::widget_valueSelectUpDown(int16_t posX,int16_t posY, int16_t width, int16_t height,String valueText, bool displayBotArrow, bool displayTopArrow,uint8_t textScale, widgetAlignment alignement)
{

    int16_t imagePositionX, imagePositionY, textHeight, arrowSectionHeight;

    switch(textScale){
        
        case SMALL_FONT:
            textHeight = SMALL_FONT_ROW_HEIGH;
        break;

        case MED_FONT:
            textHeight = MED_FONT_ROW_HEIGHT;
        break;

        case LARGE_FONT:
            textHeight = LARGE_FONT_ROW_HEIGHT;
        break;

    }

    arrowSectionHeight = (height - textHeight)/2;

    if(displayBotArrow)
    {      
        imagePositionX = posX + HAlignImage(arrowUp_width, width, 0, widgetAlignment::center); // Arrow Always aligned
        imagePositionY = posY + VAlignImage(arrowUp_height,arrowSectionHeight,0, alignement);
        display->drawBitmap(imagePositionX,imagePositionY, arrowUp,arrowUp_width,arrowUp_height,BLACK,WHITE);
    }
    
    widget_1row(posX, posY, width,height, valueText.c_str(),textScale, alignement);
    
    if(displayTopArrow)
    {
        imagePositionX = posX + HAlignImage(arrowDown_width, width, 0, widgetAlignment::center); // Arrow Always aligned
        imagePositionY = posY + arrowSectionHeight + textHeight + VAlignImage(arrowDown_height,arrowSectionHeight,0, alignement);
        display->drawBitmap(imagePositionX,imagePositionY,arrowDown, arrowDown_width, arrowDown_height,BLACK,WHITE);
    }    

}


void oledDisplayManager::widget_divisor(int16_t posX,int16_t posY, int16_t width, int16_t height,String textTop,String textbot,uint8_t textScale, int16_t hPadding, int16_t vPadding)
{
    //uint16_t textStartX;
    //uint16_t textStartY;
    //int16_t sectionHeight1;

    int16_t carSize = 5 * textScale;
    int16_t lineWidth = 4 * carSize;
    if(lineWidth > width)
    {
        lineWidth = width;
    }

    int16_t lineStartx = HAlignImage(lineWidth,width,W_NO_PADDING,widgetAlignment::center);
    
    //Top Number
    widget_1row(posX,posY,width*.6,height/2,textTop.c_str(),textScale, widgetAlignment::centerRight,hPadding,vPadding);
    widget_1row(posX+width*.4,posY+height/2,width*.6,height/2,textbot.c_str(),textScale, widgetAlignment::centerLeft,hPadding,vPadding);
    //display->drawFastHLine(posX + width*0.1, posY+height/2, width * .8, WHITE);
    //display->drawLine(posX + int16_t(width*.2)+padding, posY + int16_t(height*.6), posX + int16_t(width*.8)-padding, posY + int16_t(height*.4),WHITE);
    display->drawLine(posX + lineStartx, posY + int16_t(height*.6), posX + lineStartx + lineWidth, posY + int16_t(height*.4),WHITE);
}


void oledDisplayManager::drawBorder(int16_t posX,int16_t posY, int16_t width, int16_t height,uint32_t bStyle)
{
	int radius;
	int16_t halfHeight = height>>1;
	int16_t halfWidth = width>>1;
switch((bStyle & 0x00FF0000))
    {
        case (uint32_t)BORDERSTYLE_ALL :
            display->drawRect(posX,posY,width,height,(uint16_t)(bStyle& 0x0000FFFF));
            break;

        case (uint32_t)BORDERSTYLE_LEFT:
            display->drawFastVLine(posX,posY,height,(uint16_t)(bStyle& 0x0000FFFF));
            break;   

        case (uint32_t)BORDERSTYLE_RIGHT:
            display->drawFastVLine(posX+width-1,posY,height,(uint16_t)(bStyle& 0x0000FFFF));
            break;

        case (uint32_t)BORDERSTYLE_TOP:
            display->drawFastHLine(posX,posY,width,(uint16_t)(bStyle& 0x0000FFFF));
            break;
        
        case (uint32_t)BORDERSTYLE_BOT:
            display->drawFastHLine(posX,posY+height-1,width,(uint16_t)(bStyle& 0x0000FFFF));
            break;

        case (uint32_t)BORDERSTYLE_CIRCLE_CONTAINED :

        	if(width <= height )
        	{
        		radius = halfWidth;
        	}
        	else
        	{
        		radius = halfHeight;
        	}
			display->drawCircle(posX+halfWidth-1,posY+halfHeight-1,radius,(uint16_t)(bStyle& 0x0000FFFF));
			break;

        case (uint32_t)BORDERSTYLE_CIRCLE :
			radius = sqrt(halfWidth*halfWidth+halfHeight*halfHeight);

			display->drawCircle(posX+halfWidth-1,posY+halfHeight-1,radius,(uint16_t)(bStyle& 0x0000FFFF));
			break;

        default:
            break;

    }
}


void oledDisplayManager::widget_scrollBarVertical(int16_t posX,int16_t posY, int16_t width, int16_t height,bool displayBotArrow, bool displayTopArrow, uint32_t bStyle)
{
    int16_t imagePositionX;
    int16_t imagePositionY;

    drawBorder(posX,posY,width,height,(uint32_t)bStyle);
    
    if(displayBotArrow)
    {      
        imagePositionX = posX + HAlignImage(arrowUp_width, width, 0, widgetAlignment::center); // Arrow Always aligned
        imagePositionY = posY + VAlignImage(arrowUp_height,height/2,2, widgetAlignment::topCenter);
        display->drawBitmap(imagePositionX,imagePositionY, arrowUp,arrowUp_width,arrowUp_height,BLACK,WHITE);
    }
    
    if(displayTopArrow)
    {
        imagePositionX = posX + HAlignImage(arrowDown_width, width, 0, widgetAlignment::center); // Arrow Always aligned
        imagePositionY = posY + height/2 + VAlignImage(arrowDown_height,height/2,2,widgetAlignment::bottomCenter);
        display->drawBitmap(imagePositionX,imagePositionY,arrowDown, arrowDown_width, arrowDown_height,BLACK,WHITE);
    }    
}

void oledDisplayManager::widget_scrollBarHorizontal(int16_t posX,int16_t posY, int16_t width, int16_t height,bool displayBotArrow, bool displayTopArrow, uint32_t bStyle)
{
    //TODO later
}



int16_t oledDisplayManager::startTextHCenter(String str,uint8_t textSize, int16_t width)
{
    int16_t nbCar = str.length();
    int16_t textWidth;

    int16_t carSize = 5 * textSize;
    int16_t spaceSize = 1 * textSize;

    textWidth = nbCar * carSize + (nbCar-1)*spaceSize;

    return width / 2 - textWidth/2;
       
}

//int16_t oledDisplayManager::HAlignText(String str,uint8_t textSize, int16_t width, int16_t padding, widgetAlignment alignement)
int16_t oledDisplayManager::HAlignText(std::string str,uint8_t textSize, int16_t width, int16_t padding, widgetAlignment alignement)
{
    int16_t nbCar = str.length();
    int16_t textWidth;

    int16_t carSize = 5 * textSize;
    int16_t spaceSize = 1 * textSize;

    textWidth = (nbCar * carSize) + (nbCar/*-1*/)*spaceSize;

    int16_t startPos=0;
    switch (alignement)
    {

        case widgetAlignment::centerLeft:
        case widgetAlignment::topLeft:
        case widgetAlignment::bottomLeft:
            startPos = padding;//spaceSize;
            break;

        case widgetAlignment::center:
        case widgetAlignment::bottomCenter:
        case widgetAlignment::topCenter:

            startPos = width / 2 - textWidth/2 + padding;
            break;

        case widgetAlignment::centerRight:
        case widgetAlignment::bottomRight:
        case widgetAlignment::topRight:

            startPos = width - textWidth - padding; 
            break;
    
    default:
        startPos = 0;
        break;
    }

    return startPos;
       
}

int16_t oledDisplayManager::VAlignText(String str,uint8_t textSize, int16_t areaHeight,int16_t padding,widgetAlignment alignement)
{

 int16_t carSize = 7 * textSize;

 int16_t startPos;
    switch (alignement)
    {
        case widgetAlignment::topCenter:
        case widgetAlignment::topLeft:
        case widgetAlignment::topRight:
            startPos = padding;
            break;

        case widgetAlignment::center:
        case widgetAlignment::centerLeft:
        case widgetAlignment::centerRight:
            startPos = areaHeight / 2 - carSize/2 + padding;
            break;

        case widgetAlignment::bottomCenter:
        case widgetAlignment::bottomLeft:
        case widgetAlignment::bottomRight:
            startPos = areaHeight - carSize - padding;
            break;
    
    default:
        startPos = 0;
        break;
    }

    return startPos;   
}


int16_t oledDisplayManager::HAlignImage(int16_t imageWidth, int16_t areaWidth, int16_t padding, widgetAlignment alignement)
{
    int16_t startPos;
    switch (alignement)
    {
        case widgetAlignment::centerLeft:
        case widgetAlignment::topLeft:
        case widgetAlignment::bottomLeft:
            startPos = padding;
            break;

        case widgetAlignment::center:
        case widgetAlignment::topCenter:
        case widgetAlignment::bottomCenter:
            startPos = areaWidth / 2 - imageWidth/2 + padding;
            break;

        case widgetAlignment::centerRight:
        case widgetAlignment::bottomRight:
        case widgetAlignment::topRight:
            startPos = areaWidth - imageWidth - padding;
            break;
    
    default:
        startPos = 0;
        break;
    }

    return startPos;   
}

int16_t oledDisplayManager::VAlignImage(int16_t imageHeight, int16_t areaWidth, int16_t padding, widgetAlignment alignement)
{
    int16_t startPos;
    switch (alignement)
    {
        case widgetAlignment::topCenter:
        case widgetAlignment::topLeft:
        case widgetAlignment::topRight:
        startPos = padding;
        break;

        case widgetAlignment::center:
        case widgetAlignment::centerLeft:
        case widgetAlignment::centerRight:
        startPos = areaWidth / 2 - imageHeight/2 + padding;
        break;

        case widgetAlignment::bottomCenter:
        case widgetAlignment::bottomLeft:
        case widgetAlignment::bottomRight:
         startPos = areaWidth - imageHeight - padding;
        break;
    
    default:
        startPos = 0;
        break;
    }

    return startPos;   
}


int16_t oledDisplayManager::startTextVCenter(uint8_t textSize, int16_t zoneHeight)
{
    int16_t carSize = 7 * textSize;

    return zoneHeight / 2 - carSize /2;
       
}
