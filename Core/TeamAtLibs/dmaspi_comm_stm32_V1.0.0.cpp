#include "dmaspi_comm_stm32_V1.0.0.h"

#include <string.h>
//#include "utility/dma.h"
//#include "wiring_private.h" // pinPeripheral() function


dmaSpiManagerClass::dmaSpiManagerClass(SPI_HandleTypeDef* spiPort)
{
	spiHandle = spiPort;
}


void dmaSpiManagerClass::init(void (*txDonecallback)()){


}


void dmaSpiManagerClass::handle(void)
{

  if(transfer_is_done)
  {
    // If a transfer was in progress and is now done, we need to end the trensaction 
    if(transferInProgress)
    {
      //for debug Serial.println("Ending transaction");
      transferInProgress = 0;
      //TODO DMA PLATFORM ABSTRACTION:
      //HAL_SPI_DMAPause(&hspi1);
      //mySPI->endTransaction();
    }

    for(int i = 0 ; i < m_nbDevice ; i++ )
    {
      // To sitch device every time for priority
      deviceSelect ++;
      if(deviceSelect >= m_nbDevice)
      {
        deviceSelect = 0;
      }

      if(m_dataToSend[deviceSelect])
      {
        //for debug Serial.println("Data to send detected");
        m_dataToSend[deviceSelect] = false;
        i = m_nbDevice; // for quit to send only 1 message

        //TODO DMA PLATFORM ABSTRACTION:
     /* myDMA.changeDescriptor(desc,               // DMA descriptor address
        m_peripheralList[deviceSelect]->txBuffer,  // New src; 
        (void *)(&SERCOM1->SPI.DATA.reg),          //dst don't change
        m_peripheralList[deviceSelect]->txLen);    // update length
  */
        //Toggle chip select pin
        //digitalWrite(m_peripheralList[deviceSelect]->chipSelectPin, m_peripheralList[deviceSelect]->csPolarity);
        atPinWrite(m_peripheralList[deviceSelect]->chipSelectPin, m_peripheralList[deviceSelect]->csPolarity);
        //mySPI->beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0));
        //TODO DMA PLATFORM ABSTRACTION:replace: mySPI->beginTransaction(SPISettings(m_spiFreq, MSBFIRST, SPI_MODE0));
        //HAL_SPI_DMAResume(&hspi1);


        transfer_is_done = false;   // Reset 'done' flag
        transferInProgress = true;
        //TODO DMA PLATFORM ABSTRACTION: stat = myDMA.startJob(); // Go!
        HAL_SPI_Transmit_DMA(spiHandle, &(m_peripheralList[deviceSelect]->txBuffer[m_peripheralList[deviceSelect]->startingPos]) , m_peripheralList[deviceSelect]->txLen);
    
      }
    }
  }
}


int dmaSpiManagerClass::addDevice(dmaStruct *deviceStruct){

  if(m_nbDevice < DMASPI_NB_PERIPHERAL_MAX)
  {
     m_peripheralList[m_nbDevice] = deviceStruct;
     m_nbDevice ++;

    return (m_nbDevice - 1);
  }
   
  return -1;
}


dmaStatus dmaSpiManagerClass::sendData(unsigned char* data, int dataLen, int deviceSelect){


  if(!(deviceSelect < DMASPI_NB_PERIPHERAL_MAX) )
  {
    //Serial.println("Error");
    return DMA_ID_ERROR;
  }

  if(!transferInProgress)
  {
      memcpy(m_peripheralList[deviceSelect]->txBuffer, data, dataLen);
      m_peripheralList[deviceSelect]->txLen = dataLen;
      m_dataToSend[deviceSelect] = true;

      handle(); // Run handle to speed up transferinitiation

      return DMA_OK;
  }

  return DMA_BUSY;
    
}

dmaStatus dmaSpiManagerClass::sendDataSharedBuffer(uint16_t dataLen, int deviceSelect){


  if(!(deviceSelect < DMASPI_NB_PERIPHERAL_MAX) )
  {
    //Serial.println("Error");
    return DMA_ID_ERROR;
  }

  if(!transferInProgress)
  {
      //memcpy(m_peripheralList[deviceSelect]->txBuffer, data, dataLen);
      m_peripheralList[deviceSelect]->txLen = dataLen;
      m_peripheralList[deviceSelect]->startingPos = 0;
      m_dataToSend[deviceSelect] = true;

      handle(); // Run handle to speed up transferinitiation

      return DMA_OK;
  }

  return DMA_BUSY;
    
}

dmaStatus dmaSpiManagerClass::sendPartialDataSharedBuffer(uint32_t startingPos, uint16_t dataLen, int deviceSelect){


  if(!(deviceSelect < DMASPI_NB_PERIPHERAL_MAX) )
  {
    //Serial.println("Error");
    return DMA_ID_ERROR;
  }

  if(!transferInProgress)
  {
      //memcpy(m_peripheralList[deviceSelect]->txBuffer, data, dataLen);
      m_peripheralList[deviceSelect]->txLen = dataLen;
      m_peripheralList[deviceSelect]->startingPos = startingPos;
      m_dataToSend[deviceSelect] = true;

      handle(); // Run handle to speed up transferinitiation

      return DMA_OK;
  }

  return DMA_BUSY;

}

// void dmaSpiManagerClass::getDataReceived(int deviceSelect){
// }

// void dmaSpiManagerClass::getData(int deviceSelect,unsigned char* data){

// }


bool dmaSpiManagerClass::getTxStatus(){

  return transferInProgress;
}


/**
 * @brief Call this function when DMA TX done interrupt
 * 
 */
void dmaSpiManagerClass::txDoneCallback()
{
  transfer_is_done = true;
  //TdigitalWrite(m_peripheralList[deviceSelect]->chipSelectPin, !m_peripheralList[deviceSelect]->csPolarity); //Toggle Pin
  atPinWrite(m_peripheralList[deviceSelect]->chipSelectPin,!m_peripheralList[deviceSelect]->csPolarity);
}
