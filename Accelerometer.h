#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H


#include <Arduino.h>
#include <Wire.h>

#define ACCELEROMETER_ADDRESS                   0x53 // SDO LOW
//#define ACCELEROMETER_ADDRESS                     0x1D // SDO HIGH

//////////////////////////////////////////////////////////////////////////////////
/////                            Register map definition
//////////////////////////////////////////////////////////////////////////////////

#define DEVICE_ID_ADDRESS                       0x00       // R

#define TAP_THRESHOLD_ADDRESS                   0x1D       // R/W  unsigned 8 bit - magnitude to count as a tap

#define OFFSET_X_ADDRESS                        0x1E       // R/W  signed   8 bit
#define OFFSET_Y_ADDRESS                        0x1F       // R/W  signed   8 bit
#define OFFSET_Z_ADDRESS                        0x20       // R/W  signed   8 bit

#define TAP_DURATION_ADDRESS                    0x21       // R/W  unsigned 8 bit - how long above threshold to count as tap?
#define TAP_LATENCY_ADDRESS                     0x22       // R/W  unsigned 8 bit - how long after 1st tap to start listening for 2nd tap
#define TAP_WINDOW_ADDRESS                      0x23       // R/W  unsigned 8 bit - how long to wait for 2nd tap?
#define ACTIVITY_THRESHOSHOLD_ADDRESS           0x24       // R/W  unsigned 8 bit - threshold value for detecting activity
#define INACTIVITY_THRESHOLD_ADDRESS            0x25       // R/W  unsigned 8 bit - threshold value for detecting inactivity
#define INACTIVITY_TIME_ADDRESS                 0x26       // R/W  unsigned 8 bit - time that must be below threshold to count as inactive
#define ACTIVITY_AND_INACTIVITY_CONTROL_ADDRESS 0x27       // R/W
#define FREE_FALL_THRESHOLD_ADDRESS             0x28       // R/W  unsigned 8 bit - threshold for free fall
#define FREE_FALL_TIME_ADDRESS                  0x29       // R/W  unsigned 8 bit - min time to count as freefall
#define TAP_AXIS_CONTROL_ADDRESS                0x2A       // R/W  8 bit code     - bits 7-4 N/A - bit 3 = suppress double tap if greater than thresh tap - bits 2-0 = axis tap enable
#define ACTIVITY_TAP_STATUS_ADDRESS             0x2B       // R    8 bit code     - indicates first axis involved in tap OR actvity
#define BW_RATE_ADDRESS                         0x2C       // R/W  8 bit code     - set low power mode & bandwidth
#define POWER_CONTROL_ADDRESS                   0x2D       // R/W  8 bit code     - various options - see data sheet
#define INTERRUPT_ENABLE_ADDRESS                0x2E       // R/W  8 bit code     - enable various interrupts - see data sheet
#define INTERRUPT_MAP_ADDRESS                   0x2F       // R/W  8 bit code     - assigns each interrupt to pin 1 or 2
#define INTERRUPT_SOURCE_ADDRESS                0x30       // R    8 bit code     - signifies which type of interrupt has been triggered
#define DATA_FORMAT_ADDRESS                     0x31       // R/W  8 bit code     - choose the format of output data - Range and resolution set here
#define OUTPUT_X1_ADDRESS                       0x32       // R    signed   8 bit - output data for each axis
#define OUTPUT_X2_ADDRESS                       0x33       // R        //
#define OUTPUT_Y1_ADDRESS                       0x34       // R        //
#define OUTPUT_Y2_ADDRESS                       0x35       // R        //
#define OUTPUT_Z1_ADDRESS                       0x36       // R        //
#define OUTPUT_Z2_ADDRESS                       0x37       // R        //
#define FIFO_CONTROL_ADDRESS                    0x38       // R/W  8 bit code     - various options - see data sheet
#define FIFO_STATUS_ADDRESS                     0x39       // R    8 bit code

// INTERRUPT TYPE BIT MAPS
#define DATA_READY_BIT				0x80	   // bit in interrupt registers representing data ready
#define SINGLE_TAP				0x40	   // bit in interrupt registers representing single tap
#define DOUBLE_TAP				0x20	   // bit in interrupt registers representing double tap
#define ACTIVITY 				0x10	   // bit in interrupt registers representing activity
#define INACTIVITY 				0x08	   // bit in interrupt registers representing inactivity
#define FREE_FALL 				0x04	   // bit in interrupt registers representing free-fall
#define WATERMARK 				0x02	   // bit in interrupt registers representing watermark
#define OVERRUN 				0x01	   // bit in interrupt registers representing overrun

// INTERRUPT TYPE PIN MAPS
#define INT1_PIN  				0x00
#define INT2_PIN  				0x01

//////////////////////////////////////////////////////////////////////////////////
/////                            define settings
//////////////////////////////////////////////////////////////////////////////////

#define DEVICE_ID                               0xE5
#define ACTIVITY_AND_INACTIVITY_CONTROL         0x00
//#define BW_RATE                                 0x0A        // 200Hz High Power (set 0x1A for low power)
#define BW_RATE                                 0x0F
#define POWER_CONTROL_ON                        0x08        // power on with 8Hz sleep check
#define POWER_CONTROL_OFF                       0x00
#define DATA_FORMAT                             0x00        // self test disables, 4wire spi, high interrupt, right justified with sign bit +/- 2g

//////////////////////////////////////////////////////////////////////////////////
/////                            other defines
//////////////////////////////////////////////////////////////////////////////////

#define INITIALIZATION_TIMEOUT   1000000                    //1 second timeout
#define INITIAL_RAW_ERROR_VALUE  999
#define NUM_OF_OUTPUT_BYTES      6

//////////////////////////////////////////////////////////////////////////////////
/////                            Message definitions
//////////////////////////////////////////////////////////////////////////////////

#define INITIALIZING_MESSAGE            "\nInitializing Accelerometer"
#define ACCELEROMETER_TURNED_ON_MESSAGE  "\nAccelerometer turned on"
#define ACCELEROMETER_TURNED_OFF_MESSAGE "\nAccelerometer turned off"
#define DEVICE_ID_READ_SUCCESSFUL       "\nAccerometer device ID read Succesfully"
#define DEVICE_ID_READ_UNSUCCESSFUL     "\nERROR\nAccerometer device ID not read Succesfully"
#define CONNECTION_SUCCESFUL_MESSAGE    "\nConnected to Accelerometer succesfully \nrecieving data"
#define CANNOT_CONNECT_ERROR_MESSAGE    "\nCannot connect to Accelerometer \nno data available \n1) check wiring \n2) check I2C Address"

struct Raw_Accelerometer{
    int16_t raw_x = INITIAL_RAW_ERROR_VALUE,
             raw_y = INITIAL_RAW_ERROR_VALUE,
             raw_z = INITIAL_RAW_ERROR_VALUE;
};

struct G_Values{
    float   g_x,
            g_y,
            g_z;
};

class Accelerometer
{
    public:
        Accelerometer();
        void initialize();
        void turnAccelerometerOn();
        void turnAccelerometerOff();
        Raw_Accelerometer     readRawData();
        G_Values              readGValues();
        void                  printRawData();
        void                  printRawX();
        void                  printRawY();
        void                  printRawZ();
        void                  printGValues();
        void                  printGValuesX();
        void                  printGValuesY();
        void                  printGValuesZ();

	bool		      getIsInterruptTriggered();
	bool		      getIsDataReady();			      
	bool		      getIsSingleTapTriggered();			      
	bool		      getIsDoubleTapTriggered();			      
	bool		      getIsActivityTriggered();			      
	bool		      getIsInactivityTriggered();			      
	bool		      getIsFreeFallTriggered();
	bool		      getIsWatermarkTriggered();			      
	bool		      getIsOverrunTriggered();			      
	
	void		      setIsDataReadyInterruptOnINT1Pin(bool iDRIOI1P);			
	void		      setIsSingleTapInterruptOnINT1Pin(bool iSTIOI1P);
	void		      setIsDoubleTapInterruptOnINT1Pin(bool iDTIOI1P);
	void		      setIsActivityInterruptOnINT1Pin(bool iAIOI1P);
	void		      setIsInactivityInterruptOnINT1Pin(bool iIOI1P);
	void		      setIsFreeFallInterruptOnINT1Pin(bool iFFIOI1P);
	void		      setIsWatermarkInterruptOnINT1Pin(bool iOIOI1P);
	void		      setIsOverrunInterruptOnINT1Pin(bool iOIOI1P);

	void 		      setDataReadyInterrupt(bool isDataReadyEnabled);
	void 		      setSingleTapInterrupt(bool isSingleTapEnabled);
	void 		      setDoubleTapInterrupt(bool isDoubleTapEnabled);
	void 		      setActivityInterrupt(bool isActivityEnabled);
	void 		      setInactivityInterrupt(bool isInactivityEnabled);
	void 		      setFreeFallInterrupt(bool isFreeFallEnabled);
	void 		      setWatermarkInterrupt(bool isWatermarkEnabled);
	void 		      setOverrunInterrupt(bool isOverrunEnabled);







        uint8_t               range;
        float                 resolution,
                              tap_threshold      = 1.5f;
        uint64_t              turnOnMicros       = 0,
                              microsSinceTurnOn  = 0,
                              tap_duration       = 500000; // 0.5s
        Raw_Accelerometer     raw_data           = Raw_Accelerometer();
        G_Values              g_values           = G_Values();
    protected:
        void     write(int address, int data);
        uint8_t* read (int address, int number_of_bytes);
	uint8_t  readByte(int address);

    private:
        void defineActivityControlSettings();
        void defineTapThreshold();
        void defineTapDuration();
        void defineDataFormat();
        void defineBandwidthAndOutputRate();
        void defineRange();
        void defineResolution();
        void connectToAccelerometer();
        void checkDeviceID();
        uint8_t rangesArray[4] = {2,4,8,16};
	uint8_t buffer	       = 0x00;
        long  currentMicros  = 0,
              previousMicros = 0;
};

#endif // ACCELEROMETER_H
