#ifndef DMA_COMM
    #define DMA_COMM


	#include <at_plaformAbstraction_V1_1.h>



    typedef enum{
      DMA_CSPOL_ACTIVE_LOW,
      DMA_CSPOL_ACTIVE_HIGH

    }dmaCSPol;

    typedef struct {
        unsigned char* txBuffer;
        unsigned char* rxBuffer;
        uint16_t txLen;
        uint16_t rxLen;
        uint32_t startingPos;
        atPin_t chipSelectPin;
        dmaCSPol csPolarity;
        uint16_t address;
    } dmaStruct;

    enum dmaStatus
    {
        DMA_OK,
        DMA_BUSY,
        DMA_ID_ERROR,
		DMA_ERROR
    };

    class dmaCommManagerClass {

        public:

            /**
             * @brief Initialize the port manager before using
             * 
             * @param SPIref reference to the SPI port to use
             * @param txDonecallback Pointer to the function to call whe DMA transfer complete
             */
            virtual void init(void (*txDonecallback)() = NULL){};

            /**
             * @brief Main function to call every loop 
             * 
             */
            virtual void handle(void){};
            
            /**
             * @brief This function is used to add a device on the spi port
             * 
             * @param deviceStruct struct that has in required info on the device
             * @return int > 0:  the id of the device on the bus.
             * @return int < 0:  device not added, maximum device reached.  
             */
            virtual int addDevice(dmaStruct *deviceStruct){return 0;};

            /**
             * @brief This function is used to send data on the spi port 
             * 
             * @param data Pointer to the data to send
             * @param dataLen data lenght
             * @param deviceSelect To which which device to send the data
             * 
             * @note: this fucntiosn copies the contect of the data buffer into an other buffer. To increase spead
             *        see sendDataSharedBuffer(int dataLen, int deviceSelect);
             * 
             * @return dmaspiStatus status / error 
             */
            virtual dmaStatus sendData(unsigned char* data, int dataLen, int deviceSelect){return DMA_ERROR;};

            /**
             * @brief This functio is used to send data on the SPI, but unlike the "sendData(..)" fucntction,
             * this function is to be used when the buffer to send is already conencted ( same buffer that was passsed in the dmaSpiStruct)
             * This has the advantage of saving buffer copy tiem for long data.
             *
             * @param dataLen is the number of bytes to send
             * @param deviceSelect is the device to use ( use the index that was returned by addDevice(..) function)
             * @return DMASPI_OK if succesful. \see  enum dmaspiStatus for possibilities.
             */
            virtual dmaStatus sendDataSharedBuffer(uint16_t dataLen, int deviceSelect){return DMA_ERROR;};
            virtual dmaStatus sendPartialDataSharedBuffer(uint32_t startingPos, uint16_t dataLen, int deviceSelect){return DMA_ERROR;};
            
            // Futrure implementation
            //void getDataReceived(int deviceSelect);
            //void getData(int deviceSelect,unsigned char* data);

            /**
             * @brief Used to know if a transfer is in progress
             * 
             * @return true if a transfer is in progress
             * @return false no transfer in progress
             */
            virtual bool getTxStatus(){return false;};

            /**
             * @brief This function must be called when DMA transmit is done ( inside DMA don callback function)
             * 
             */
            virtual void txDoneCallback(){};

            

        private:
            //TODO DMA PLATFORM ABSTRACTION:
            //Adafruit_ZeroDMA myDMA;
            //DmacDescriptor *desc; // DMA descriptor address (so we can change contents)
            //ZeroDMAstatus    stat; // DMA status codes returned by some functions

//        dmaStruct *m_peripheralList[DMA_NB_PERIPHERAL_MAX];   // List of connected devices
//        bool m_dataToSend[DMAI2C_NB_PERIPHERAL_MAX];                // Used to keep track if there is data to send for each devices
//        int m_nbDevice = 0; // Number of connected devices
//
//
//        bool transfer_is_done = true;   // Done yet?
//
//        int deviceSelect = -1;          //The selected device for communication.
//        bool transferInProgress = 0;    //Used to get the status of transfers
  
    };





#endif
