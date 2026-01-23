/******************************************************************************
 *                                                                            *
 *                                Copyright by                                *
 *                                                                            *
 *                              Azoteq (Pty) Ltd                              *
 *                          Republic of South Africa                          *
 *                                                                            *
 *                           Tel: +27(0)21 863 0033                           *
 *                           E-mail: info@azoteq.com                          *
 *                                                                            *
 * ========================================================================== *
 * @file        IQS323.cpp                                                    *
 * @brief       This file contains the constructors and methods which allow   *
 *              ease of use of an IQS323 Integrated Circuit (IC). The IQS323  *
 *              is a 3 Channel Self-Capacitive / 3 Channel Mutual-Capacitive  *
 *              / 2 Channel Inductive sensing controller with Touch and       *
 *              Proximity user interfaces. This class provides an easy means  *
 *              of initializing and interacting with the IQS323 device from   *
 *              an Arduino-based device.                                      *
 * @author      JN. Lochner - Azoteq PTY Ltd                                  *
 * @version     v1.5.2                                                        *
 * @date        2023                                                          *
 * ========================================================================== *
 * @attention  Makes use of the following standard Arduino libraries:         *
 * - Arduino.h -> Included in IQS323.h, comes standard with Arduino           *
 * - Wire.h    -> Included in IQS323.h, comes standard with Arduino           *
 *****************************************************************************/

/* Include Files */
#include "IQS323.h"

#include "IQS323_config_LP.h"
#include "rtc_wake_stub_trmnl_x.h"

/* Private Global Variables */
bool iqs323_deviceRDY = false;
time_t last_rdy_time = 0;
time_t prev_rdy_time = 0;
uint8_t iqs323_ready_pin;

/* Private Functions */
void iqs323_ready_interrupt(void);

/*****************************************************************************/
/*                             CONSTRUCTORS                                  */
/*****************************************************************************/
IQS323::IQS323(){
}

/*****************************************************************************/
/*                            PUBLIC METHODS                                 */
/*****************************************************************************/

/**
  * @name   begin
  * @brief  A method to initialize the IQS323 device with the device address
  *         and ready pin specified by the user.
  * @param  deviceAddress ->  The address of the IQS323 device.
  * @param  readyPin      ->  The Arduino pin which is connected to the ready
  *                           pin of the IQS323 device.
  * @retval None.
  * @note   - Receiving a true return value does not mean that initialization 
  *           was successful.
  *         - Receiving a true return value only means that the IQS device
  *           responded to the request for communication.
  *         - Receiving a false return value means that initialization did not
  *           take place at all.
  *         - If communication is successfully established then it is unlikely
  *           that initialization will fail.
  */
void IQS323::begin(uint8_t deviceAddressIn, int sdaPinIn, int sclPinIn, uint8_t readyPinIn, bool to_init)
{
  /* Initialize I2C communication here, since this library can't function
  without it. */
  // Wire.begin(sdaPinIn, sclPinIn); // ADDED to use custom pins
  // Wire.setClock(100000);
  
  // device_reset(); // Reset the device to ensure it is in a known state

  // printf("IQS323 reseted!\n");
  
  _deviceAddress = deviceAddressIn;
  iqs323_ready_pin = readyPinIn;

  attachInterrupt(iqs323_ready_pin, iqs323_ready_interrupt, CHANGE);

  /* Initialize "running" and "init" state machine variables. */
  if (to_init) {
    printf("IQS323 to be initialized!\n");
    iqs323_state.state = IQS323_STATE_START;
    iqs323_state.init_state = IQS323_INIT_VERIFY_PRODUCT;
  }
  else {
    printf("IQS323 skipped initialization!\n");
    iqs323_state.state = IQS323_STATE_RUN;
    iqs323_state.init_state = IQS323_INIT_DONE;
  }
}

/**
  * @name   init
  * @brief  A method that runs through a normal start-up routine to set up the
  *         IQS323 with the desired settings from the IQS323_init.h file.
  * @retval Returns true if the full start-up routine has been completed,
  *         returns false if not.
  * @note   - No false return will be given, the program will thus be stuck
  *           when one of the cases is not able to finish.
  *         - See serial communication to find the ERROR case
  */
bool IQS323::init(void)
{
  uint16_t prod_num;
  uint8_t ver_maj, ver_min;

  printf("IQS323 INIT State: %d\n", iqs323_state.init_state);

  bool comm_requested = false;

  switch (iqs323_state.init_state)
  {
    /* Verifies product number to determine if the correct device is connected
    for this example */
    case IQS323_INIT_VERIFY_PRODUCT:
      if(iqs323_deviceRDY)
      {
        Serial.println("\tIQS323_INIT_VERIFY_PRODUCT");
        prod_num = getProductNum(RESTART);
        ver_maj = getmajorVersion(RESTART);
        ver_min = getminorVersion(STOP);
        Serial.print("\t\tProduct number is: ");
        Serial.print(prod_num);
        Serial.print(" v");
        Serial.print(ver_maj);
        Serial.print(".");
        Serial.println(ver_min);
        if(prod_num == IQS323_PRODUCT_NUM_001)
        {
          Serial.println("\t\tIQS323 Release UI Confirmed!");
          iqs323_state.init_state = IQS323_INIT_READ_RESET;
        }
        else if(prod_num == IQS323_PRODUCT_NUM_A01)
        {
          Serial.println("\t\tIQS323 Movement UI Confirmed!");
          iqs323_state.init_state = IQS323_INIT_READ_RESET;
        }
        else
        {
          Serial.println("\t\tDevice is not a IQS323!");
          iqs323_state.init_state = IQS323_INIT_NONE;
        }
      }
    break;

    /* Verify if a reset has occurred */
    case IQS323_INIT_READ_RESET:
      if(iqs323_deviceRDY)
      {
        Serial.println("\tIQS323_INIT_READ_RESET");
        updateInfoFlags(STOP);
        if (checkReset())
        {
          Serial.println("\t\tReset event occurred.");
          iqs323_state.init_state = IQS323_INIT_UPDATE_SETTINGS;
        }
        else
        {
          Serial.println("\t\t No Reset Event Detected - Request SW Reset");
          iqs323_state.init_state = IQS323_INIT_CHIP_RESET;
        }
      }
    break;

    /* Perform SW Reset */
    case IQS323_INIT_CHIP_RESET:
      if(iqs323_deviceRDY)
      {
        Serial.println("\tIQS323_INIT_CHIP_RESET");

        //Perform SW Reset
        SW_Reset(STOP);
        Serial.println("\t\tSoftware Reset Bit Set.");
        delay(100);
        iqs323_state.init_state = IQS323_INIT_READ_RESET;
      }
    break;

    /* Write all settings to IQS323 from .h file */
    case IQS323_INIT_UPDATE_SETTINGS:
      if(iqs323_deviceRDY)
      {
        Serial.println("\tIQS323_INIT_UPDATE_SETTINGS");
        writeMM(STOP);
        iqs323_state.init_state = IQS323_INIT_ACK_RESET;
      }
    break;

    /* Acknowledge that the device went through a reset */
    case IQS323_INIT_ACK_RESET:
      if(iqs323_deviceRDY)
      {
        Serial.println("\tIQS323_INIT_ACK_RESET");
        acknowledgeReset(STOP);
        iqs323_state.init_state = IQS323_INIT_ATI;
      }
      break;

    /* Run the ATI algorithm to recalibrate the device with newly added settings */
    case IQS323_INIT_ATI:
      // if (!comm_requested) {
      //   force_I2C_communication();
      //   comm_requested = true;
      // }
      if(iqs323_deviceRDY)
      {
        Serial.println("\tIQS323_INIT_ATI");
        ReATI(STOP);
        iqs323_state.init_state = IQS323_INIT_WAIT_FOR_ATI;
        Serial.println("\tIQS323_INIT_WAIT_FOR_ATI");
        // comm_requested = false;
      }
    break;

    /* Read the ATI Active bit to see if the rest of the program can continue */
    case IQS323_INIT_WAIT_FOR_ATI:
      // if (!comm_requested) {
      //   force_I2C_communication();
      //   comm_requested = true;
      // }
      if(iqs323_deviceRDY)
      {
        if(!readATIactive())
        {
          Serial.println("\t\tDONE");
          iqs323_state.init_state = IQS323_INIT_READ_DATA;
          // comm_requested = false;
        }
      }
    break;

    /* Read the latest data from the iqs323 */
    case IQS323_INIT_READ_DATA:
      // if (!comm_requested) {
      //   force_I2C_communication();
      //   comm_requested = true;
      // }
      if (iqs323_deviceRDY)
      {
        Serial.println("\tIQS323_INIT_READ_DATA");
        queueValueUpdates();
        iqs323_state.init_state = IQS323_INIT_ACTIVATE_EVENT_MODE;
        // comm_requested = false;
      }
    break;

    /* Turn on I2C event mode */
    case IQS323_INIT_ACTIVATE_EVENT_MODE:
      if(iqs323_deviceRDY)
      {
        Serial.println("\tIQS323_INIT_ACTIVATE_EVENT_MODE");
        setEventMode(STOP);
        iqs323_state.init_state = IQS323_INIT_DONE;
      }
    break;

    /* If all operations have been completed correctly, the RDY pin can be set
     * up as an interrupt to indicate when new data is available */
    case IQS323_INIT_DONE:
      Serial.println("\tIQS323_INIT_DONE");
      new_data_available = true;
      return true;
    break;

    default:
      break;
  }
  return false;
}

/**
  * @name   run
  * @brief  This method is called continuously during runtime, which serves as
  *         the main state machine.
  * @param  None.
  * @retval None.
  * @note   The state machine continuously checks for specific events and
  *         updates the state machine accordingly. A reset event will cause
  *         the state machine to re-initialize the device.
  *
  *         queueValueUpdates can be edited by the user if other data should be
  *         read every time a RDY window is received.
  */
void IQS323::run(void)
{
  // printf("IQS323 State: %d\n", iqs323_state.state);
  switch (iqs323_state.state)
  {
    /* After a hardware reset, this is the starting position of the main
    state machine */
    case IQS323_STATE_NONE:
      Serial.println("IQS323 None state, something seems wrong.");
    break;

    case IQS323_STATE_START:
      Serial.println("IQS323 Initialization:");
      iqs323_state.state = IQS323_STATE_INIT;
    break;

    /* Perform the initialization routine on the IQS323 */
    case IQS323_STATE_INIT:
      if(init())
      {
        Serial.println("IQS323 Initialization complete!\n");
        iqs323_state.state = IQS323_STATE_RUN;
      }
    break;

    /* Send an I2C software reset in the next RDY window */
    case IQS323_STATE_SW_RESET:
      if(iqs323_deviceRDY)
      {
        SW_Reset(STOP);
        iqs323_state.state = IQS323_STATE_RUN;
      }
    break;

    /* Continuous reset monitoring state, ensure no reset event has occurred
    for data to be valid */
    case IQS323_STATE_CHECK_RESET:
      if(checkReset())
      {
        Serial.println("Reset Occurred!\n");
        new_data_available = false;
        iqs323_state.state = IQS323_STATE_START;
        iqs323_state.init_state = IQS323_INIT_VERIFY_PRODUCT;
      }

      /* A reset did not occur, move to the run state and wait for a new ready
      window */
      else
      {
        new_data_available = true; /* No reset, thus data is valid */
        iqs323_state.state = IQS323_STATE_RUN;
      }
    break;

    /* If a RDY Window is open, read the latest values from the IQS323 */
    case IQS323_STATE_RUN:
      if (iqs323_deviceRDY)
      {
        Serial.println("RDY time: " + String(last_rdy_time) + " us, " + "time delta: " + String(micros() - last_rdy_time) + " us, " + "time between RDYs: " + String(last_rdy_time - prev_rdy_time) + " us");
        prev_rdy_time = last_rdy_time;
        queueValueUpdates();
        iqs323_deviceRDY = false;
        new_data_available = false;
        iqs323_state.state = IQS323_STATE_CHECK_RESET;
      }
    break;
  }
}

/**
  * @name   iqs323_ready_interrupt
  * @brief  A method used as an interrupt function. Activated when a High-Low and
  *         Low-High interrupt is seen on the correct Arduino interrupt pin.
  * @param  None.
  * @retval None.
  * @note   Keep this function as simple as possible to prevent stuck states
  *         and slow operations.
  */
void iqs323_ready_interrupt(void)
{
  if(gpio_get_level((gpio_num_t)iqs323_ready_pin))
  {
    iqs323_deviceRDY = false;
  }
  else
  {
    iqs323_deviceRDY = true;
    last_rdy_time = micros();
  }
}

/**
  * @name   clearRDY
  * @brief  A method used to clear the ready interrupt bit.
  * @param  None.
  * @retval None.
  */
void IQS323::clearRDY(void)
{
  iqs323_deviceRDY = false;
}

/**
  * @name   getRDYStatus
  * @brief  A method used to retrieve the device RDY status.
  * @param  None.
  * @retval Returns the boolean IQS323 RDY state.
  *         - True when RDY line is LOW
  *         - False when RDY line is HIGH
  */
bool IQS323::getRDYStatus(void)
{
  return iqs323_deviceRDY;
}

/**
  * @name   queueValueUpdates
  * @brief  All I2C read operations in the queueValueUpdates method will be
  *         performed each time the IQS323 opens a RDY window.
  * @param  None.
  * @retval None.
  * @note   Any Address in the IQS323 memory map can be read from here.
  */
void IQS323::queueValueUpdates(void)
{
  printf("Queue value updates...\n");
  uint8_t transferBytes[18];	// The array which will hold the bytes to be transferred.

	/* Read the info flags. */
	readRandomBytes(IQS323_MM_SYSTEM_STATUS, 18, transferBytes, STOP);

	/* Assign the System Status */
  IQSMemoryMap.SYSTEM_STATUS[0] =  transferBytes[0];
  IQSMemoryMap.SYSTEM_STATUS[1] =  transferBytes[1];

  /* Assign the Gestures */
  IQSMemoryMap.GESTURES[0]  = transferBytes[2];
  IQSMemoryMap.GESTURES[1]  = transferBytes[3];

  /* Assign the Slider Coordinates */
  IQSMemoryMap.SLIDER_COORDINATES[0]  = transferBytes[4];
  IQSMemoryMap.SLIDER_COORDINATES[1] = transferBytes[5];

  /* Assign Channel 0 Counts */
  IQSMemoryMap.CH0_COUNTS_LTA[0]  = transferBytes[6];
  IQSMemoryMap.CH0_COUNTS_LTA[1]  = transferBytes[7];

  /* Assign Channel 0 LTA */
  IQSMemoryMap.CH0_COUNTS_LTA[2]  = transferBytes[8];
  IQSMemoryMap.CH0_COUNTS_LTA[3]  = transferBytes[9];

  /* Assign Channel 1 Counts */
  IQSMemoryMap.CH1_COUNTS_LTA[0]  = transferBytes[10];
  IQSMemoryMap.CH1_COUNTS_LTA[1]  = transferBytes[11];

  /* Assign Channel 1 LTA */
  IQSMemoryMap.CH1_COUNTS_LTA[2]  = transferBytes[12];
  IQSMemoryMap.CH1_COUNTS_LTA[3]  = transferBytes[13];

  /* Assign Channel 2 Counts */
  IQSMemoryMap.CH2_COUNTS_LTA[0]  = transferBytes[14];
  IQSMemoryMap.CH2_COUNTS_LTA[1]  = transferBytes[15];

  /* Assign Channel 2 LTA */
  IQSMemoryMap.CH2_COUNTS_LTA[2]  = transferBytes[16];
  IQSMemoryMap.CH2_COUNTS_LTA[3]  = transferBytes[17];
}

/**
  * @name	  readATIactive
  * @brief  A method that checks if the ATI routine is still active
  * @param  None.
  * @retval Returns true if the ATI_ACTIVE_BIT is cleared, false if the
  *         ATI_ACTIVE_BIT is set.
  * @note   If the ATI routine is active the channel states (NONE, PROX, TOUCH)
  *         might exhibit unwanted behaviour. Thus it is advised to wait for
  *         the routine to complete before continuing.
  */
bool IQS323::readATIactive(void)
{
  /* Read the Info flags */
  updateInfoFlags(STOP);

  /* Return the ATI Active status */
  return getBit(IQSMemoryMap.SYSTEM_STATUS[0], IQS323_ATI_ACTIVE_BIT);
}

/**
  * @name	  checkReset
  * @brief  A method that checks if the device has reset and returns the reset
  *         status.
  * @param  None.
  * @retval Returns true if a reset has occurred, false if no reset has occurred.
  * @note   If a reset has occurred the device settings should be reloaded using
  *         the begin function. After new device settings have been reloaded
  *         the acknowledge reset function can be used to clear the reset flag.
  */
bool IQS323::checkReset(void)
{
	/* Perform a bitwise AND operation with the SHOW_RESET_BIT to return the
  reset status */
  return getBit(IQSMemoryMap.SYSTEM_STATUS[0], IQS323_SHOW_RESET_BIT);
}

/**
  * @name	  checkProductNum
  * @brief  A method that checks the device product number and compares the
  *         result to the defined value to return a boolean result.
  * @param  stopOrRestart ->  Specifies whether the communications window must
  *                           be kept open or must be closed after this action.
  *                           Use the STOP and RESTART definitions.
  * @retval Returns the product number as a unit16_t value.
  * @note   If the product is not correctly identified an appropriate messages
  *         should be displayed.
  */
uint16_t IQS323::getProductNum(bool stopOrRestart)
{
	uint8_t transferBytes[2];	      // A temporary array to hold the byte to be transferred.
  uint8_t prodNumLow = 0;         // Temporary storage for the Counts low byte.
  uint8_t prodNumHigh = 0;        // Temporary storage for the Counts high byte.
  uint16_t prodNumReturn = 0;     // The 16bit return value.

  printf("Reading Product Number from register 0x%02X...\n", IQS323_MM_PROD_NUM);

	/* Read the Device info from the IQS323. */
	readRandomBytes(IQS323_MM_PROD_NUM, 2, transferBytes, stopOrRestart);

  printf("\tProduct Number: %02X %02X\n", transferBytes[0], transferBytes[1]);

  /* Construct the 16bit return value. */
  prodNumLow = transferBytes[0];
  prodNumHigh = transferBytes[1];
  prodNumReturn = (uint16_t)(prodNumLow);
  prodNumReturn |= (uint16_t)(prodNumHigh<<8);
  /* Return the Counts value. */
  return prodNumReturn;
}

/**
  * @name	getmajorVersion
  * @brief  A method that checks the device firmware version's major value.
  * @param  stopOrRestart -> Specifies whether the communications window must
  *                           be kept open or must be closed after this action.
  *                          Use the STOP and RESTART definitions.
  * @retval Returns major version number as a uint8_t value.
  */
uint8_t IQS323::getmajorVersion(bool stopOrRestart)
{
	uint8_t transferBytes[2];	// A temporary array to hold the byte to be transferred.
  uint8_t ver_maj = 0;      // Temporary storage for the firmware version major number.

	/* Read the Device info from the IQS323. */
	readRandomBytes(IQS323_MM_MAJOR_VERSION_NUM, 2, transferBytes, stopOrRestart);

  /* Get major value from correct byte */
  ver_maj = transferBytes[0];
  /* Return the major firmware version number value. */
  return ver_maj;
}

/**
  * @name	getminorVersion
  * @brief  A method that checks the device firmware version's minor value.
  * @param  stopOrRestart ->  Specifies whether the communications window must
  *                           be kept open or must be closed after this action.
  *                           Use the STOP and RESTART definitions.
  * @retval Returns minor version number as a unit8_t value.
  */
uint8_t IQS323::getminorVersion(bool stopOrRestart)
{
	uint8_t transferBytes[2];	// A temporary array to hold the byte to be transferred.
  uint8_t ver_min = 0;      // Temporary storage for the firmware version minor number.


	/* Read the Device info from the IQS323. */
	readRandomBytes(IQS323_MM_MINOR_VERSION_NUM, 2, transferBytes, stopOrRestart);
  /* get major value from correct byte */
  ver_min = transferBytes[0];
  /* Return the minor firmware version number value. */
  return ver_min;
}

/**
  * @name	acknowledgeReset
  * @brief  A method that clears the Reset Event bit by writing it to a 0.
  * @param  stopOrRestart ->  Specifies whether the communications window must
  *                           be kept open or must be closed after this action.
  *                           Use the STOP and RESTART definitions.
  * @retval None.
  * @note   If a reset has occurred the device settings should be reloaded using
  *         the begin function. After new device settings have been reloaded
  *         this method should be used to clear the reset bit.
  */
void IQS323::acknowledgeReset(bool stopOrRestart)
{
	uint8_t transferByte[2];	// A temporary array to hold the bytes to be transferred.
	/* Read the System Flags from the IQS323, these must be read first in order
  not to change any settings. */
	readRandomBytes(IQS323_MM_SYSTEM_CONTROL, 2, transferByte, RESTART);
	/* Set the the ACK_RESET_BIT in the SYSTEM_CONTROL register */
	transferByte[0] = setBit(transferByte[0], IQS323_ACK_RESET_BIT);
	/* Write the new byte to the System Flags address. */
	writeRandomBytes(IQS323_MM_SYSTEM_CONTROL, 2, transferByte, stopOrRestart);
}

/**
  * @name   ReATI
  * @brief  A method which sets the REDO_ATI_BIT in order to force the IQS323
  *         device to run the Automatic Tuning Implementation (ATI) routine.
  * @param  stopOrRestart ->  Specifies whether the communications window must
  *                           be kept open or must be closed after this action.
  *                           Use the STOP and RESTART definitions.
  * @retval None.
  * @note   I2C communications are disabled for the duration of the ATI process.
  */
void IQS323::ReATI(bool stopOrRestart)
{
  uint8_t transferByte[2]; // Array to store the bytes transferred.

  readRandomBytes(IQS323_MM_SYSTEM_CONTROL, 2, transferByte, RESTART);
  /* Set the the RE_ATI_BIT in the SYSTEM_CONTROL register */
  transferByte[0] = setBit(transferByte[0], IQS323_RE_ATI_BIT);
  /* Write the new byte to the required device. */
  writeRandomBytes(IQS323_MM_SYSTEM_CONTROL, 2, transferByte, stopOrRestart);
}

/**
  * @name   SW_Reset
  * @brief  A method that sets the SW RESET bit to force the IQS323 device to
  *         do a software reset.
  * @param  stopOrRestart ->  Specifies whether the communications window must
  *                           be kept open or must be closed after this action.
  *                           Use the STOP and RESTART definitions.
  * @retval None.
  * @note   To perform SW Reset, bit IQS323_SW_RESET_BIT in SYSTEM_CONTROL is set.
  */
void IQS323::SW_Reset(bool stopOrRestart)
{
  uint8_t transferByte[2]; // Array to store the bytes transferred.

  printf("Reading random bytes...\n");
  readRandomBytes(IQS323_MM_SYSTEM_CONTROL, 2, transferByte, STOP);
  /* Set the the SW_RESET_BIT in the SYSTEM_CONTROL register */
  transferByte[0] = setBit(transferByte[0], IQS323_SW_RESET_BIT);
  /* Write the new byte to the required device. */

  printf("Writing random bytes...\n");
  writeRandomBytes(IQS323_MM_SYSTEM_CONTROL, 2, transferByte, stopOrRestart);
}

/**
  * @name   setEventMode
  * @brief  A method to set the IQS323 device into event mode.
  * @param  stopOrRestart ->  Specifies whether the communications window must
  *                           be kept open or must be closed after this action.
  *                           Use the STOP and RESTART definitions.
  * @retval None.
  * @note   All other bits at the IQS323_MM_SYSTEM_CONTROL register address 
  *         are preserved.
  */
void IQS323::setEventMode(bool stopOrRestart)
{
  uint8_t transferByte[2]; // The array which will hold the bytes which are transferred.

  /* First read the bytes at the memory address so that they can be preserved */
  readRandomBytes(IQS323_MM_SYSTEM_CONTROL, 2, transferByte, RESTART);
  /* Set the EVENT_MODE_BIT in SYSTEM_CONTROL */
  transferByte[0] = setBit(transferByte[0], IQS323_EVENT_MODE_BIT);
  /* Write the bytes back to the device */
  writeRandomBytes(IQS323_MM_SYSTEM_CONTROL, 2, transferByte, stopOrRestart);
}

/**
  * @name   updateInfoFlags
  * @brief  A method that reads the info flags from the IQS323 and assigns
  *         it to the local SYSTEM_STATUS register.
  * @param  stopOrRestart ->  Specifies whether the communications window must
  *                           be kept open or must be closed after this action.
  *              			        Use the STOP and RESTART definitions.
  * @retval None.
  * @note   The local SYSTEM_STATUS register is altered with the new value of
  *         the info flags register, retrieved from the IQS323.
  */
void IQS323::updateInfoFlags(bool stopOrRestart)
{
	uint8_t transferBytes[2];	// The array which will hold the bytes to be transferred.

	/* Read the info flags. */
  force_I2C_communication();
	readRandomBytes(IQS323_MM_SYSTEM_STATUS, 2, transferBytes, stopOrRestart);
	/* Assign the info flags to the local SYSTEM_STATUS register */
  IQSMemoryMap.SYSTEM_STATUS[0] =  transferBytes[0];
  IQSMemoryMap.SYSTEM_STATUS[1] =  transferBytes[1];
}

void IQS323::setIQSMemoryMap(iqs323_system_status_t iqs_status)
{
  IQSMemoryMap.SYSTEM_STATUS[0] =  iqs_status.status[0];
  IQSMemoryMap.SYSTEM_STATUS[1] =  iqs_status.status[1];

  /* Assign the Gestures */
  IQSMemoryMap.GESTURES[0]  = iqs_status.gestures[0];
  IQSMemoryMap.GESTURES[1]  = iqs_status.gestures[1];

  /* Assign the Slider Coordinates */
  IQSMemoryMap.SLIDER_COORDINATES[0]  = iqs_status.slider_cords[0];
  IQSMemoryMap.SLIDER_COORDINATES[1] = iqs_status.slider_cords[1];

  /* Assign Channel 0 Counts */
  IQSMemoryMap.CH0_COUNTS_LTA[0]  = iqs_status.ch0_cnts[0];
  IQSMemoryMap.CH0_COUNTS_LTA[1]  = iqs_status.ch0_cnts[1];

  /* Assign Channel 0 LTA */
  IQSMemoryMap.CH0_COUNTS_LTA[2]  = iqs_status.ch0_cnts[2];
  IQSMemoryMap.CH0_COUNTS_LTA[3]  = iqs_status.ch0_cnts[3];

  /* Assign Channel 1 Counts */
  IQSMemoryMap.CH1_COUNTS_LTA[0]  = iqs_status.ch1_cnts[0];
  IQSMemoryMap.CH1_COUNTS_LTA[1]  = iqs_status.ch1_cnts[1];

  /* Assign Channel 1 LTA */
  IQSMemoryMap.CH1_COUNTS_LTA[2]  = iqs_status.ch1_cnts[2];
  IQSMemoryMap.CH1_COUNTS_LTA[3]  = iqs_status.ch1_cnts[3];

  /* Assign Channel 2 Counts */
  IQSMemoryMap.CH2_COUNTS_LTA[0]  = iqs_status.ch2_cnts[0];
  IQSMemoryMap.CH2_COUNTS_LTA[1]  = iqs_status.ch2_cnts[1];

  /* Assign Channel 2 LTA */
  IQSMemoryMap.CH2_COUNTS_LTA[2]  = iqs_status.ch2_cnts[2];
  IQSMemoryMap.CH2_COUNTS_LTA[3]  = iqs_status.ch2_cnts[3];
}

/**
  * @name   get_PowerMode
  * @brief  A method that reads the local SYSTEM_STATUS register and returns
  *         the current power mode.
  * @param  None.
  * @retval Returns the current iqs323_power_modes state the device is in.
  * @note   See Datasheet on power mode options and timeouts.
  *         Normal Power, Low Power and Ultra Low Power (ULP).
  */
iqs323_power_modes IQS323::get_PowerMode(void)
{
  uint8_t buffer = getBit(IQSMemoryMap.SYSTEM_STATUS[1], IQS323_POWER_EVENT_BIT_0);
  buffer += getBit(IQSMemoryMap.SYSTEM_STATUS[1], IQS323_POWER_EVENT_BIT_1) << 1;

  if(buffer == IQS323_NORMAL_POWER_BIT)
  {
    return IQS323_NORMAL_POWER;
  }
  else if(buffer == IQS323_LOW_POWER_BIT)
  {
    return IQS323_LOW_POWER;
  }
  else if(buffer == IQS323_ULP_BIT)
  {
    return IQS323_ULP;
  }
    else if(buffer == IQS323_HALT_BIT)
  {
    return IQS323_HALT;
  }
  return IQS323_POWER_UNKNOWN;
}

/**
  * @name   channel_touchState
  * @brief  A method which reads the local SYSTEM_STATUS register for a given
  *         channel and determines if the channel is in a touch state.
  * @param  channel ->  The channel name on the IQS323 (CH0-CH2) for which a
  *                     touch state needs to be determined.
  * @retval Returns true if a touch is active and false if there is no touch.
  * @note   See the iqs323_channel_e typedef for all possible channel names.
  */
bool IQS323::channel_touchState(iqs323_channel_e channel)
{
  switch(channel)
	{
		case IQS323_CH0:
			return getBit(IQSMemoryMap.SYSTEM_STATUS[1], IQS323_CH0_TOUCH_BIT);
		break;

    case IQS323_CH1:
			return getBit(IQSMemoryMap.SYSTEM_STATUS[1], IQS323_CH1_TOUCH_BIT);
		break;

    case IQS323_CH2:
			return getBit(IQSMemoryMap.SYSTEM_STATUS[1], IQS323_CH2_TOUCH_BIT);
		break;

		default:
			return false;
		break;
	}
}

/**
  * @name   channel_proxState
  * @brief  A method which reads the local SYSTEM_STATUS register for a given
  *         channel and determines if the channel is in a proximity state.
  * @param  channel ->  The channel name on the IQS323 (CH0-CH2) for which a
  *                     proximity state needs to be determined.
  * @retval Returns true if proximity is active and false if there is no proximity.
  * @note   See the IQS323_Channel_e typedef for all possible channel names.
  */
bool IQS323::channel_proxState(iqs323_channel_e channel)
{
  switch(channel)
	{
		case IQS323_CH0:
		 return getBit(IQSMemoryMap.SYSTEM_STATUS[1], IQS323_CH0_PROX_BIT);
		break;

    case IQS323_CH1:
			return getBit(IQSMemoryMap.SYSTEM_STATUS[1], IQS323_CH1_PROX_BIT);
		break;

    case IQS323_CH2:
			return getBit(IQSMemoryMap.SYSTEM_STATUS[1], IQS323_CH2_PROX_BIT);
		break;

		default:
			return false;
		break;
	}
}

/**
  * @name   sliderCoordinate
  * @brief  A method that reads the local SLIDER_COORDINATES register and
  *         calculates the current slider position.
  * @param  None.
  * @retval Returns a 16-bit unsigned integer value that contains the slider 
  *         coordinates from 0 to the resolution maximum.
  */
uint16_t IQS323::sliderCoordinate(void)
{
  uint16_t buffer;
  buffer = IQSMemoryMap.SLIDER_COORDINATES[0];
  buffer += IQSMemoryMap.SLIDER_COORDINATES[1]<<8;
  return buffer;
}

/**
  * @name   getSliderEvent
  * @brief  A method that reads the local SYSTEM_STATUS register and determines if
  *         if a slider event occurred.
  * @param  None
  * @retval Returns true if a slider event occurred and false if there was no 
  *         slider event.
  */
bool IQS323::getSliderEvent(void)
{
  return getBit(IQSMemoryMap.SYSTEM_STATUS[0],IQS323_SLIDER_EVENT_BIT);
}

/**
  * @name   getGestureEvent
  * @brief  A method that reads the local GESTURES register and determines if
  *         if a gesture event occurred.
  * @param  None.
  * @retval Returns true if a gesture occurred and false if no gesture occurred.
  */
bool IQS323::getGestureEvent(void)
{
  return getBit(IQSMemoryMap.GESTURES[0],IQS323_GESTURE_EVENT_BIT);
}

/**
  * @name   getGestureType
  * @brief  A method that returns the gesture event type on the iqs323 slider.
  * @param  None.
  * @retval Returns iqs323_gesture_events representing the current active gesture.
  */
iqs323_gesture_events IQS323::getGestureType(void)
{
  /* Find the slider event that occurred */
  if(getBit(IQSMemoryMap.GESTURES[0], IQS323_GESTURE_TAP_BIT))
  {
    return IQS323_GESTURE_TAP;
  }
  else if(getBit(IQSMemoryMap.GESTURES[0], IQS323_GESTURE_FLICK_POS_BIT))
  {
    return IQS323_GESTURE_FLICK_POSITIVE;
  }
  else if(getBit(IQSMemoryMap.GESTURES[0], IQS323_GESTURE_FLICK_NEG_BIT))
  {
    return IQS323_GESTURE_FLICK_NEGATIVE;
  }
  else if(getBit(IQSMemoryMap.GESTURES[0], IQS323_GESTURE_SWIPE_POS_BIT))
  {
    return IQS323_GESTURE_SWIPE_POSITIVE;
  }
  else if(getBit(IQSMemoryMap.GESTURES[0], IQS323_GESTURE_SWIPE_NEG_BIT))
  {
    return IQS323_GESTURE_SWIPE_NEGATIVE;
  }
  else if(getBit(IQSMemoryMap.GESTURES[0], IQS323_GESTURE_HOLD_BIT))
  {
    return IQS323_GESTURE_HOLD;
  }
  else
  {
    return IQS323_GESTURE_NONE;
  }
}

/**
  * @name   readChannelCounts
  * @brief  A method that reads and returns the Counts value of a specific
  *         channel
  * @param  channel ->  The enumerator which specifies which channel's
  *                     Counts to return.
  * @retval Returns a 16-bit unsigned integer value containing the Counts value 
  *         of the channel.
  */
uint16_t IQS323::readChannelCounts(iqs323_channel_e channel)
{
  uint8_t countsLow = 0;         // Temporary storage for the Counts low byte.
  uint8_t countsHigh = 0;        // Temporary storage for the Counts high byte.
  uint16_t countsReturn = 0;     // The 16bit return value.

  switch(channel)
	{
		case IQS323_CH0:
        countsLow = IQSMemoryMap.CH0_COUNTS_LTA[0];
        countsHigh = IQSMemoryMap.CH0_COUNTS_LTA[1];
		break;

    case IQS323_CH1:
        countsLow = IQSMemoryMap.CH1_COUNTS_LTA[0];
        countsHigh = IQSMemoryMap.CH1_COUNTS_LTA[1];
		break;

    case IQS323_CH2:
        countsLow = IQSMemoryMap.CH2_COUNTS_LTA[0];
        countsHigh = IQSMemoryMap.CH2_COUNTS_LTA[1];
		break;

		default:
			return false;
		break;
	}

  // Construct the 16bit return value.
  countsReturn = (uint16_t)(countsLow);
  countsReturn |= (uint16_t)(countsHigh<<8);

  // Return the Counts value.
  return countsReturn;
}

/**
  * @name   readChannelLTA
  * @brief  A method that reads and returns the LTA value of a specific channel
  * @param  channel       ->  The enumerator which specifies which channel's
  *                           LTA value to return.
  * @retval Returns a 16-bit unsigned integer value containing the LTA value of 
  *         the channel.
  */
uint16_t IQS323::readChannelLTA(iqs323_channel_e channel)
{
  uint8_t ltaLow = 0;             // Temporary storage for the LTA low byte.
  uint8_t ltaHigh = 0;            // Temporary storage for the LTA high byte.
  uint16_t ltaReturn = 0;         // The 16bit return value.

  switch(channel)
	{
		case IQS323_CH0:
        ltaLow = IQSMemoryMap.CH0_COUNTS_LTA[2];
        ltaHigh = IQSMemoryMap.CH0_COUNTS_LTA[3];
		break;

    case IQS323_CH1:
        ltaLow = IQSMemoryMap.CH1_COUNTS_LTA[2];
        ltaHigh = IQSMemoryMap.CH1_COUNTS_LTA[3];
		break;

    case IQS323_CH2:
        ltaLow = IQSMemoryMap.CH2_COUNTS_LTA[2];
        ltaHigh = IQSMemoryMap.CH2_COUNTS_LTA[3];
		break;

		default:
			return false;
		break;
	}

  // Construct the 16bit return value.
  ltaReturn = (uint16_t)(ltaLow);
  ltaReturn |= (uint16_t)(ltaHigh<<8);

  // Return the LTA value.
  return ltaReturn;
}

/*****************************************************************************/
/*									     		ADVANCED PUBLIC METHODS							    	 		   */
/*****************************************************************************/

/**
  * @name   writeMM
  * @brief  Function to write the whole memory map to the device (writable)
  *         registers
  * @param  stopOrRestart ->  Specifies whether the communications window must
  *                           be kept open or must be closed after this action.
  *                           Use the STOP and RESTART definitions.
  * @retval None.
  * @note   - Values are obtained from the init.h file exported from the GUI.
  *         - The project will load the .h file for the corresponding EV-Kit
  *           selected with the IQS323_EV_KIT define in IQS323.h 
  */
void IQS323::writeMM(bool stopOrRestart)
{
	uint8_t transferBytes[30];	// Temporary array which holds the bytes to be transferred.

  /* Change the Sensor 0 Settings */
  /* Memory Map Position 0x30 - 0x39 */
  transferBytes[0]  = S0_SETUP;
  transferBytes[1]  = S0_TX_SELECT;
  transferBytes[2]  = S0_CONV_FREQ_FRAC;
  transferBytes[3]  = S0_CONV_FREQ_PERIOD;
  transferBytes[4]  = S0_PRX_CTRL_0;
  transferBytes[5]  = S0_PRX_CTRL_1;
  transferBytes[6]  = S0_TG_CTRL;
  transferBytes[7]  = S0_RX_SELECT;
  transferBytes[8]  = S0_CALCAP_INACTIVE_RX;
  transferBytes[9]  = S0_PATTERN_SETUP;
  transferBytes[10] = S0_PATTERN_SELECT;
  transferBytes[11] = S0_BIAS_CURRENT;
  transferBytes[12] = S0_ATI_SETUP_0;
  transferBytes[13] = S0_ATI_SETUP_1;
  transferBytes[14] = S0_ATI_BASE_0;
  transferBytes[15] = S0_ATI_BASE_1;
  transferBytes[16] = S0_ATI_COARSE;
  transferBytes[17] = S0_ATI_FINE;
  transferBytes[18] = S0_COMPENSATION_0;
  transferBytes[19] = S0_COMPENSATION_1;

  writeRandomBytes(IQS323_MM_S0_SETUP_0, 20, transferBytes, STOP);
  Serial.println("\t\t1. Write Sensor 0 Settings");

  /* Change the Sensor 1 Settings */
  /* Memory Map Position 0x40 - 0x49 */
  transferBytes[0]  = S1_SETUP;
  transferBytes[1]  = S1_TX_SELECT;
  transferBytes[2]  = S1_CONV_FREQ_FRAC;
  transferBytes[3]  = S1_CONV_FREQ_PERIOD;
  transferBytes[4]  = S1_PRX_CTRL_0;
  transferBytes[5]  = S1_PRX_CTRL_1;
  transferBytes[6]  = S1_TG_CTRL;
  transferBytes[7]  = S1_RX_SELECT;
  transferBytes[8]  = S1_CALCAP_INACTIVE_RX;
  transferBytes[9]  = S1_PATTERN_SETUP;
  transferBytes[10] = S1_PATTERN_SELECT;
  transferBytes[11] = S1_BIAS_CURRENT;
  transferBytes[12] = S1_ATI_SETUP_0;
  transferBytes[13] = S1_ATI_SETUP_1;
  transferBytes[14] = S1_ATI_BASE_0;
  transferBytes[15] = S1_ATI_BASE_1;
  transferBytes[16] = S1_ATI_COARSE;
  transferBytes[17] = S1_ATI_FINE;
  transferBytes[18] = S1_COMPENSATION_0;
  transferBytes[19] = S1_COMPENSATION_1;

  writeRandomBytes(IQS323_MM_S1_SETUP_0, 20, transferBytes, STOP);
  Serial.println("\t\t2. Write Sensor 1 Settings");

  /* Change the Sensor 2 Settings */
  /* Memory Map Position 0x50 - 0x59 */
  transferBytes[0]  = S2_SETUP;
  transferBytes[1]  = S2_TX_SELECT;
  transferBytes[2]  = S2_CONV_FREQ_FRAC;
  transferBytes[3]  = S2_CONV_FREQ_PERIOD;
  transferBytes[4]  = S2_PRX_CTRL_0;
  transferBytes[5]  = S2_PRX_CTRL_1;
  transferBytes[6]  = S2_TG_CTRL;
  transferBytes[7]  = S2_RX_SELECT;
  transferBytes[8]  = S2_CALCAP_INACTIVE_RX;
  transferBytes[9]  = S2_PATTERN_SETUP;
  transferBytes[10] = S2_PATTERN_SELECT;
  transferBytes[11] = S2_BIAS_CURRENT;
  transferBytes[12] = S2_ATI_SETUP_0;
  transferBytes[13] = S2_ATI_SETUP_1;
  transferBytes[14] = S2_ATI_BASE_0;
  transferBytes[15] = S2_ATI_BASE_1;
  transferBytes[16] = S2_ATI_COARSE;
  transferBytes[17] = S2_ATI_FINE;
  transferBytes[18] = S2_COMPENSATION_0;
  transferBytes[19] = S2_COMPENSATION_1;

  writeRandomBytes(IQS323_MM_S2_SETUP_0, 20, transferBytes, STOP);
  Serial.println("\t\t3. Write Sensor 2 Settings");

  /* Change the Channel 0 settings */
  /* Memory Map Position 0x60 - 0x63 */
  transferBytes[0] = CH0_REF_UI_SETUP;
  transferBytes[1] = CH0_FOLLOWER_MASK;
  transferBytes[2] = CH0_PROX_THRESHOLD;
  transferBytes[3] = CH0_PROX_DEBOUNCE;
  transferBytes[4] = CH0_TOUCH_THRESHOLD;
  transferBytes[5] = CH0_TOUCH_HYSTERESIS;
  transferBytes[6] = CH0_FOLLOWER_WEIGHT_0;
  transferBytes[7] = CH0_FOLLOWER_WEIGHT_1;

  

  writeRandomBytes(IQS323_MM_CH0_SETUP_0, 8, transferBytes, STOP);
  Serial.println("\t\t4. Write Channel 0 Settings");

  /* Change the Channel 1 settings */
  /* Memory Map Position 0x70 - 0x75 */
  transferBytes[0] = CH1_REF_UI_SETUP;
  transferBytes[1] = CH1_FOLLOWER_MASK;
  transferBytes[2] = CH1_PROX_THRESHOLD;
  transferBytes[3] = CH1_PROX_DEBOUNCE;
  transferBytes[4] = CH1_TOUCH_THRESHOLD;
  transferBytes[5] = CH1_TOUCH_HYSTERESIS;
  transferBytes[6] = CH1_FOLLOWER_WEIGHT_0;
  transferBytes[7] = CH1_FOLLOWER_WEIGHT_1;

  

  writeRandomBytes(IQS323_MM_CH1_SETUP_0, 8, transferBytes, STOP);
  Serial.println("\t\t5. Write Channel 1 Settings");

  /* Change the Channel 2 settings */
  /* Memory Map Position 0x80 - 0x85 */
  transferBytes[0] = CH2_REF_UI_SETUP;
  transferBytes[1] = CH2_FOLLOWER_MASK;
  transferBytes[2] = CH2_PROX_THRESHOLD;
  transferBytes[3] = CH2_PROX_DEBOUNCE;
  transferBytes[4] = CH2_TOUCH_THRESHOLD;
  transferBytes[5] = CH2_TOUCH_HYSTERESIS;
  transferBytes[6] = CH2_FOLLOWER_WEIGHT_0;
  transferBytes[7] = CH2_FOLLOWER_WEIGHT_1;

  

  writeRandomBytes(IQS323_MM_CH2_SETUP_0, 8, transferBytes, STOP);
  Serial.println("\t\t6. Write Channel 2 Settings");

  /* Change the Slider Configuration */
  /* Memory Map Position 0x90 - 0x98 */
  transferBytes[0]  = SLIDER_SETUP;
  transferBytes[1]  = LOWER_CALIBRATION;
  transferBytes[2]  = UPPER_CALIBRATION;
  transferBytes[3]  = BOTTOM_SPEED;
  transferBytes[4]  = TOP_SPEED_0;
  transferBytes[5]  = TOP_SPEED_1;
  transferBytes[6]  = SLIDER_RESOLUTION_0;
  transferBytes[7]  = SLIDER_RESOLUTION_1;
  transferBytes[8]  = ENABLE_MASK_0;
  transferBytes[9]  = ENABLE_MASK_1;
  transferBytes[10] = ENABLE_STATUS_POINTER_0;
  transferBytes[11] = ENABLE_STATUS_POINTER_1;
  transferBytes[12] = DELTA_LINK0_0;
  transferBytes[13] = DELTA_LINK0_1;
  transferBytes[14] = DELTA_LINK1_0;
  transferBytes[15] = DELTA_LINK1_1;
  transferBytes[16] = DELTA_LINK2_0;
  transferBytes[17] = DELTA_LINK2_1;

  

  writeRandomBytes(IQS323_MM_SLIDER_SETUP_CALIBRATION, 18, transferBytes, STOP);
  Serial.println("\t\t7. Write Slider Configuration");

  /* Change the Gesture Setup */
  /* Memory Map Position 0xA0 - 0xA6 */
  transferBytes[0]  = GESTURE_SELECT;
  transferBytes[1]  = RESERVED_BYTE;
  transferBytes[2]  = MINIMUM_TIME_0;
  transferBytes[3]  = MINIMUM_TIME_1;
  transferBytes[4]  = MAXIMUM_TAP_TIME_0;
  transferBytes[5]  = MAXIMUM_TAP_TIME_1;
  transferBytes[6]  = MAXIMUM_SWIPE_TIME_0;
  transferBytes[7]  = MAXIMUM_SWIPE_TIME_1;
  transferBytes[8]  = MINIMUM_HOLD_TIME_0;
  transferBytes[9]  = MINIMUM_HOLD_TIME_1;
  transferBytes[10] = MAXIMUM_TAP_DISTANCE_0;
  transferBytes[11] = MAXIMUM_TAP_DISTANCE_1;
  transferBytes[12] = MINIMUM_SWIPE_DISTANCE_0;
  transferBytes[13] = MINIMUM_SWIPE_DISTANCE_1;

  

  writeRandomBytes(IQS323_MM_GESTURE_ENABLE, 14, transferBytes, STOP);
  Serial.println("\t\t8. Write Slider Gestures Setup");

  /* Change the Filter Betas */
  /* Memory Map Position 0xB0 - 0xB4 */
  transferBytes[0] = NP_COUNTS_FILTER;
  transferBytes[1] = LP_COUNTS_FILTER;
  transferBytes[2] = NP_LTA_FILTER;
  transferBytes[3] = LP_LTA_FILTER;
  transferBytes[4] = NP_LTA_FAST_FILTER;
  transferBytes[5] = LP_LTA_FAST_FILTER;
  transferBytes[6] = NP_ACTIVATION_LTA_FILTER;
  transferBytes[7] = LP_ACTIVATION_LTA_FILTER;
  transferBytes[8] = FAST_FILTER_BAND_0;
  transferBytes[9] = FAST_FILTER_BAND_1;

  

  writeRandomBytes(IQS323_MM_COUNTS_FILTER_BETA, 10, transferBytes, STOP);
  Serial.println("\t\t9. Write Filter Betas");

  /* Change the I2C Settings and Events Mask */
  /* Memory Map Position 0xD0 - 0xD4 */
  transferBytes[0] = OUTA_MASK_0;
  transferBytes[1] = OUTA_MASK_1;
  transferBytes[2] = I2C_TIMEOUT_0;
  transferBytes[3] = I2C_TIMEOUT_1;
  transferBytes[4] = PROX_EVENT_TIMEOUT;
  transferBytes[5] = TOUCH_EVENT_TIMEOUT;
  transferBytes[6] = EVENTS_ENABLE;
  transferBytes[7] = ACTIVATION_THRESHOLD;
  transferBytes[8] = RELEASE_DELTA_PERCENTAGE;
  transferBytes[9] = DELTA_SNAP_SAMPLE_DELAY;

  

  writeRandomBytes(IQS323_MM_OUTA_MASK, 10, transferBytes, STOP);
  Serial.println("\t\t10. Write General Settings");

  /* Change the Power mode & System Settings */
  /* Memory Map Position 0xC0 - 0xC5 */
  transferBytes[0]  = SYSTEM_CONTROL;
  transferBytes[1]  = RESERVED_BYTE;
  transferBytes[2]  = NP_REPORT_RATE_0;
  transferBytes[3]  = NP_REPORT_RATE_1;
  transferBytes[4]  = LP_REPORT_RATE_0;
  transferBytes[5]  = LP_REPORT_RATE_1;
  transferBytes[6]  = ULP_REPORT_RATE_0;
  transferBytes[7]  = ULP_REPORT_RATE_1;
  transferBytes[8]  = HALT_REPORT_RATE_0;
  transferBytes[9]  = HALT_REPORT_RATE_1;
  transferBytes[10] = POWER_MODE_TIMEOUT_0;
  transferBytes[11] = POWER_MODE_TIMEOUT_1;

  

  writeRandomBytes(IQS323_MM_SYSTEM_CONTROL, 12, transferBytes, STOP);
  Serial.println("\t\t11. Write Power mode & System Settings");


  /* Change the I2C settings */
  /* Memory Map Position 0xE0 - 0xDF */
  transferBytes[0] = I2C_SETUP;

  

  writeRandomBytes(IQS323_MM_I2C_SETUP, 1, transferBytes, STOP);
  Serial.println("\t\t12. Write I2C Settings");
}

/*****************************************************************************/
/*                            PRIVATE METHODS                                */
/*****************************************************************************/

/**
 * @name    readRandomBytes
 * @brief   A method that reads a specified number of bytes from a specified
 *          address and saves it into a user-supplied array. This method is
 *          used by all other methods in this class which read data from the
 *          IQS323 device.
 * @param   memoryAddress ->  The memory address at which to start reading bytes
 *                            from.  See the "iqs323_addresses.h" file.
 * @param   numBytes      ->  The number of bytes that must be read.
 * @param   bytesArray    ->  The array which will store the bytes to be read.
 *                            This array will be overwritten.
 * @param   stopOrRestart ->  A boolean that specifies whether the communication
 *                            window should remain open or be closed after transfer.
 *                            False keeps it open, true closes it. Use the STOP
 *                            and RESTART definitions.
 * @retval  No value is returned, however, the user-supplied array is overwritten.
 * @note    Uses standard Arduino "Wire" library which is for I2C communication.
 *          Take note that C++ cannot return an array, therefore, the array which
 *          is passed as an argument is overwritten with the required values.
 *          Pass an array to the method by using only its name, e.g. "bytesArray",
 *          without the brackets, this passes a pointer to the array.
 */
void IQS323::readRandomBytes(uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[], bool stopOrRestart)
{
  // A simple counter to assist with loading bytes into the user-supplied array.
  uint8_t i = 0;

  force_I2C_communication();

  /* Select the device with the address "_deviceAddress" and start communication. */
  Wire.beginTransmission(_deviceAddress);
  /* Send a bit asking for the "memoryAddress" register. */
  Wire.write(memoryAddress);
  /* Complete the selection and communication initialization. */
  Wire.endTransmission(RESTART);  // Restart transmission for reading that follows.
  /* The required device has now been selected and it has been told which
  register to send information from. */

  /* Request "numBytes" bytes from the device which has address "_deviceAddress"*/
//  do
//  {
    Wire.requestFrom((int)_deviceAddress, (int)numBytes); //, (int)stopOrRestart);
//  }while(Wire.available() == 0);  // Wait for response, this sometimes takes a few attempts

  /* Load the received bytes into the array until there are no more */
  while(Wire.available() && i < numBytes)
  {
    /* Load the received bytes into the user-supplied array */
    bytesArray[i] = Wire.read();
    i++;
  }

  /* Always manually close the RDY window after a STOP is sent to prevent
  writing while ready is closing */
  // if(stopOrRestart == STOP)
  // {
  iqs323_deviceRDY = false;
  // }
}

/**
  * @name   writeRandomBytes
  * @brief  A method that writes a specified number of bytes to a specified
  *         address, the bytes to write are supplied through an array pointer.
  *         This method is used by all other methods of this class which
  *         write data to the IQS323 device.
  * @param  memoryAddress ->  The memory address at which to start writing the
  *                           bytes to. See the "iqs323_addresses.h" file.
  * @param  numBytes      ->  The number of bytes that must be written.
  * @param  bytesArray    ->  The array which stores the bytes which will be
  *                           written to the memory location.
  * @param  stopOrRestart ->  A boolean that specifies whether the communication
  *                           window should remain open or be closed after a
  *                           transfer. False keeps it open, true closes it.
  *                           Use the STOP and RESTART definitions.
  * @retval No value is returned, only the IQS device registers are altered.
  * @note   Uses standard Arduino "Wire" library which is for I2C communication.
  *         Take note that a full array cannot be passed to a function in C++.
  *         Pass an array to the function by using only its name, e.g. "bytesArray",
  *         without the square brackets, this passes a pointer to the
  *         array. The values to be written must be loaded into the array prior
  *         to passing it to the function.
  */
void IQS323::writeRandomBytes(uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[], bool stopOrRestart)
{
  /* Select the device with the address "_deviceAddress" and start
  communication. */

  force_I2C_communication();

  Wire.beginTransmission(_deviceAddress);
  /* Specify the memory address where the IQS323 must start saving the data,
  as designated by the "memoryAddress" variable. */
  Wire.write(memoryAddress);
  /* Write the bytes as specified in the array that the "arrayAddress" pointer
  points to. */
  for(int i=0; i<numBytes; i++)
  {
    Wire.write(bytesArray[i]);
  }
  /* End the transmission and the user decides to STOP or RESTART. */
  Wire.endTransmission(STOP);

  /* Always manually close the RDY window after a STOP is sent to prevent
  writing while the ready is closing */
  iqs323_deviceRDY = false;
}

/**
  * @name   getBit
  * @brief  A method that returns the chosen bit value of the provided byte.
  * @param  data       -> byte of which a given bit value needs to be calculated.
  * @param  bit_number -> a number between 0 and 7 representing the bit in question.
  * @retval The boolean value of the specific bit requested. 
  */
bool IQS323::getBit(uint8_t data, uint8_t bit_number)
{
  return (data & ( 1 << bit_number )) >> bit_number;
}

/**
  * @name   setBit
  * @brief  A method that returns the chosen bit value of the provided byte.
  * @param  data       -> byte of which a given bit value needs to be calculated.
  * @param  bit_number -> a number between 0 and 7 representing the bit in question.
  * @retval Returns an 8-bit unsigned integer value of the given data byte with 
  *         the requested bit set.
  */
uint8_t IQS323::setBit(uint8_t data, uint8_t bit_number)
{
	return (data |= 1UL << bit_number);
}

/**
  * @name   clearBit
  * @brief  A method that returns the chosen bit value of the provided byte.
  * @param  data       -> byte of which a given bit value needs to be calculated.
  * @param  bit_number -> a number between 0 and 7 representing the bit in question.
  * @retval Returns an 8-bit unsigned integer value of the given data byte with 
  *         the requested bit cleared.
  */
uint8_t IQS323::clearBit(uint8_t data, uint8_t bit_number)
{
	return (data &= ~(1UL << bit_number));
}

/**
  * @name   force_I2C_communication
  * @brief  A method which writes data 0x00 to memory address 0xFF to open a
  *         communication window on the IQS323.
  * @param  None.
  * @retval None.
  */
void IQS323::force_I2C_communication(void)
{
  /* Ensure RDY is HIGH at the moment */
  if (!iqs323_deviceRDY)
  {
    /* Select the device with the address "DEMO_IQS323_ADDR" and start
    communication. */
    Wire.beginTransmission(_deviceAddress);

    /* Write to memory address 0xFF that will prompt the IQS323 to open a
    communication window.*/
    Wire.write(0xFF);

    /* End the transmission and the user decides to STOP or RESTART. */
    Wire.endTransmission(STOP);
    iqs323_deviceRDY = false;
    delay(45);
  }
}

void IQS323::read_gesture(void)
{
  if(!gpio_get_level((gpio_num_t)iqs323_ready_pin))
  {
    iqs323_deviceRDY = true;
    queueValueUpdates();
    iqs323_deviceRDY = false;
    new_data_available = false;
    iqs323_state.state = IQS323_STATE_CHECK_RESET;
  }
}
