/*
I2C.h   - I2C library
Copyright (c) 2011-2012 Wayne Truchsess.  All right reserved.
Rev 5.0 - January 24th, 2012
- Removed the use of interrupts completely from the library
so TWI state changes are now polled.
- Added calls to lockup() function in most functions
to combat arbitration problems
- Fixed scan() procedure which left timeouts enabled
and set to 80msec after exiting procedure
- Changed scan() address range back to 0 - 0x7F
- Removed all Wire legacy functions from library
- A big thanks to Richard Baldwin for all the testing
and feedback with debugging bus lockups!
Rev 4.0 - January 14th, 2012
- Updated to make compatible with 8MHz clock frequency
Rev 3.0 - January 9th, 2012
- Modified library to be compatible with Arduino 1.0
- Changed argument type from boolean to uint8_t in pullUp(),
setSpeed() and receiveByte() functions for 1.0 compatability
- Modified return values for timeout feature to report
back where in the transmission the timeout occured.
- added function scan() to perform a bus scan to find devices
attached to the I2C bus.  Similar to work done by Todbot
and Nick Gammon
Rev 2.0 - September 19th, 2011
- Added support for timeout function to prevent
and recover from bus lockup (thanks to PaulS
and CrossRoads on the Arduino forum)
- Changed return type for stop() from void to
uint8_t to handle timeOut function
Rev 1.0 - August 8th, 2011

This is a modified version of the Arduino Wire/TWI
library.  Functions were rewritten to provide more functionality
and also the use of Repeated Start.  Some I2C devices will not
function correctly without the use of a Repeated Start.  The
initial version of this library only supports the Master.


This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

Modified by Soon Kiat Lau (2019) for:
- ping function
- error codes
- allow user to enable/disable internal pull-up resistors in begin()
*/

#ifndef _I2C_h
#define _I2C_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <inttypes.h>

#ifndef I2C_h
#define I2C_h


#define START           0x08
#define REPEATED_START  0x10
#define MT_SLA_ACK	    0x18  // The "T" in MT stands for transmit
#define MT_SLA_NACK	    0x20  // SLA indicates we are sending/receiving the address byte. So MT_SLA_*** are statuses returned when sending out addresses to find a slave device.
#define MT_DATA_ACK     0x28  // DATA indicates we are sending/receiving the data byte(s). So MT_DATA_*** are statuses returned when sending out data to a slave device.
#define MT_DATA_NACK    0x30
#define MR_SLA_ACK	    0x40  // The "R" in MR stands for receive. The SLA and DATA means the same things as explained above.
#define MR_SLA_NACK	    0x48
#define MR_DATA_ACK     0x50
#define MR_DATA_NACK    0x58
#define LOST_ARBTRTN    0x38
#define TWI_STATUS      (TWSR & 0xF8)
#define TWI_STATUS_ACK      0x00  // I2C comm completed successfully (received ACK bit)
#define TWI_STATUS_TIMEOUT  0x01  // I2C comm failed because it took too long to receive a complete response
#define SLA_W(address)  (address << 1)
#define SLA_R(address)  ((address << 1) + 0x01)
#define cbi(sfr, bit)   (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit)   (_SFR_BYTE(sfr) |= _BV(bit))

#define MAX_BUFFER_SIZE 32

// Error codes returned by read/write functions
typedef enum
{
  // Errors categorized according to the functions that return them. Commented-out errors are still returned
  // by that function, but is already defined in the list.

  // Errors returned from the custom I2C library
  I2C_STATUS_OK               = 0,   // No problemo
  I2C_STATUS_START            = 1,   // I2C timeout while waiting for successful completion of a Start bit
  I2C_STATUS_ACKNACK_TRS_ADD  = 2,   // I2C timeout while waiting for ACK/NACK while addressing slave in transmit mode (MT)
  I2C_STATUS_ACKNACK_TRS_DAT  = 3,   // I2C timeout while waiting for ACK/NACK while sending data to the slave
  I2C_STATUS_REPSTART         = 4,   // I2C timeout while waiting for successful completion of a Repeated Start
  I2C_STATUS_ACKNACK_REC_ADD  = 5,   // I2C timeout while waiting for ACK/NACK while addressing slave in receiver mode (MR)
  I2C_STATUS_ACKNACK_REC_DAT  = 6,   // I2C timeout while waiting for ACK/NACK while receiving data from the slave
  I2C_STATUS_STOP             = 7,   // I2C timeout while waiting for successful completion of the Stop bit
  I2C_STATUS_UNKNOWN          = 8,   // Unknown or yet to be defined error

  // Internal use by this library only; do not use this outside of the library
} I2C_STATUS;

class I2C
{
public:
  I2C();
  void begin(bool enableInternalPullUps);
  void end();
  void setTimeOut(uint16_t timeOut);
  void setSpeed(bool useFastMode);
  void pullup(bool activate);
  void scan();
  uint8_t available();
  uint8_t receive();

  // This function simply sends a write request to the addressed device without any additional bytes. This is sometimes used to trigger a device.
  I2C_STATUS ping(uint8_t address);

  // The following are the write and read functions. The "registerAddress" is there to follow the expected I2C standard, whereby immediately after a write/read
  // command byte, the master is expected to specify which register should be written on/read from. However, there are many devices that do not follow this
  // standard strictly, and sometimes a "command" is sent instead of the address of the register. Luckily, the "registerAddress" variable is simply a byte
  // that is sent out right after the write/read byte, therefore it can be replaced with any byte such as a command byte or whatever your device needs to send
  // immediately after the write/read byte. The "data" is simply the third byte in the whole command train. Any additional bytes can be stuffed into an array
  // and sent out with the function using *data.
  //
  // This is how the entire command train looks like:
  // [address+read/write byte] -> ["registerAddress" byte] -> [data (if available) OR *data (sends out one byte for each array entry)]
  //
  //
  I2C_STATUS write(uint8_t address, uint8_t registerAddress);
  I2C_STATUS write(int address, int registerAddress);
  I2C_STATUS write(uint8_t address, uint8_t registerAddress, uint8_t data);
  I2C_STATUS write(int address, int registerAddress, int data);
  I2C_STATUS write(uint8_t address, uint8_t registerAddress, char *data);
  I2C_STATUS write(uint8_t address, uint8_t registerAddress, uint8_t *data, uint8_t numberBytes);
  I2C_STATUS read(uint8_t address, uint8_t numberBytes);
  I2C_STATUS read(int address, int numberBytes);
  I2C_STATUS read(uint8_t address, uint8_t registerAddress, uint8_t numberBytes);
  I2C_STATUS read(int address, int registerAddress, int numberBytes);
  I2C_STATUS read(uint8_t address, uint8_t numberBytes, uint8_t *dataBuffer);
  I2C_STATUS read(uint8_t address, uint8_t registerAddress, uint8_t numberBytes, uint8_t *dataBuffer);


private:
  uint8_t start();
  uint8_t sendAddress(uint8_t);
  uint8_t sendByte(uint8_t);
  uint8_t receiveByte(uint8_t);
  uint8_t stop();
  void lockUp();
  uint8_t returnStatus;
  uint8_t nack;
  uint8_t data[MAX_BUFFER_SIZE];
  static uint8_t bytesAvailable;
  static uint8_t bufferIndex;
  static uint8_t totalBytes;
  static uint16_t timeOutDelay;

};

extern I2C I2c;

#endif

#endif

