#ifndef DMASPI_COMM_MKRZERO
    #define DMASPI_COMM_MKRZERO

    #include <at_plaformAbstraction_V1_1.h>
#include <SPI.h>
    //#include <Adafruit_ZeroDMA.h>
	#include "dma_comm_stm32_V1.0.0.h"

    #define DMASPI_NB_PERIPHERAL_MAX 2

//    typedef enum{
//        SPIDAM_CSPOL_ACTIVE_LOW,
//        SPIDAM_CSPOL_ACTIVE_HIGH
//
//    }spidmaCSPol;

//    typedef struct {
//        unsigned char* txBuffer;
//        unsigned char* rxBuffer;
//        int txLen;
//        int rxLen;
//        atPin_t chipSelectPin;
//        spidmaCSPol csPolarity;
//    } dmaSpiStruct;

//    enum dmaspiStatus
//    {
//        DMASPI_OK,
//        DMASPI_BUSY,
//        DMASPI_ID_ERROR
//    };

    class dmaSpiManagerClass: public dmaCommManagerClass {

        public:

    	dmaSpiManagerClass(SPI_HandleTypeDef* spiPort);

            /**
             * @brief Initialize the port manager before using
             * 
             * @param SPIref reference to the SPI port to use
             * @param txDonecallback Pointer to the function to call whe DMA transfer complete
             */
            void init(void (*txDonecallback)() = NULL);

            /**
             * @brief Main function to call every loop 
             * 
             */
            void handle(void);
            
            /**
             * @brief This function is used to add a device on the spi port
             * 
             * @param deviceStruct struct that has in required info on the device
             * @return int > 0:  the id of the device on the bus.
             * @return int < 0:  device not added, maximum device reached.  
             */
            int addDevice(dmaStruct *deviceStruct);

            /**
             * @brief This function is used to send data on the spi port 
             * 
             * @param data Pointer to the data to send
             * @param dataLen data lenght
             * @param deviceSelect To which which device to send the data
             * 
             * @note: this function copies the content of the data buffer into an other buffer. To increase spead
             *        see sendDataSharedBuffer(int dataLen, int deviceSelect);
             * 
             * @return dmaspiStatus status / error 
             */
            dmaStatus sendData(unsigned char* data, int dataLen, int deviceSelect);

            /**
             * @brief This functio is used to send data on the SPI, but unlike the "sendData(..)" fucntction,
             * this function is to be used when the buffer to send is already conencted ( same buffer that was passsed in the dmaSpiStruct)
             * This has the advantage of saving buffer copy tiem for long data.
             *
             * @param dataLen is the number of bytes to send
             * @param deviceSelect is the device to use ( use the index that was returned by addDevice(..) function)
             * @return DMASPI_OK if succesful. \see  enum dmaspiStatus for possibilities.
             */
            dmaStatus sendDataSharedBuffer(uint16_t dataLen, int deviceSelect);
            
            dmaStatus sendPartialDataSharedBuffer(uint32_t startingPos, uint16_t dataLen, int deviceSelect);
            // Futrure implementation
            //void getDataReceived(int deviceSelect);
            //void getData(int deviceSelect,unsigned char* data);

            /**
             * @brief Used to know if a transfer is in progress
             * 
             * @return true if a transfer is in progress
             * @return false no transfer in progress
             */
            bool getTxStatus();

            /**
             * @brief This function must be called when DMA transmit is done ( inside DMA don callback function)
             * 
             */
            void txDoneCallback();

            

        private:
            //TODO DMA PLATFORM ABSTRACTION:
            //Adafruit_ZeroDMA myDMA;
            //DmacDescriptor *desc; // DMA descriptor address (so we can change contents)
            //ZeroDMAstatus    stat; // DMA status codes returned by some functions

        dmaStruct *m_peripheralList[DMASPI_NB_PERIPHERAL_MAX];   // List of connected devices
        bool m_dataToSend[DMASPI_NB_PERIPHERAL_MAX];                // Used to keep track if there is data to send for each devices
        
        int m_nbDevice = 0; // Number of connected devices

        //SPIClass* mySPI;    //Spi class
        //uint32_t m_spiFreq = 8000000;

        SPI_HandleTypeDef* spiHandle;

        bool transfer_is_done = true;   // Done yet?

        int deviceSelect = -1;          //The selected device for communication.
        bool transferInProgress = 0;    //Used to get the status of transfers
  
    };





#endif
