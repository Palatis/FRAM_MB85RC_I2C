/**************************************************************************/
/*!
    @file     FRAM_MB85RC_I2C.h
    @author   SOSAndroid.fr (E. Ha.)

    @section  HISTORY

    v1.0 - First release
	v1.0.1 - Robustness enhancement
	v1.0.2 - fix constructor, introducing byte move in memory
	v1.0.3 - fix writeLong() function
	v1.0.4 - fix constructor call error
	v1.0.5 - Enlarge density chip support by making check more flexible, Error codes not anymore hardcoded, add connect example, add Cypress FM24 & CY15B series comment.
	v1.1.0b - adding support for devices without device IDs + 4K & 16 K devices support

    Driver for the MB85RC I2C FRAM from Fujitsu.

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, SOSAndroid.fr (E. Ha.)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#ifndef _FRAM_MB85RC_I2C_H_
#define _FRAM_MB85RC_I2C_H_

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Wire.h>

// Enabling debug I2C - comment to disable / normal operations
#ifndef SERIAL_DEBUG
#define SERIAL_DEBUG 1
#endif

// IDs
//Manufacturers codes
#define FUJITSU_MANUFACT_ID		0x00A
#define CYPRESS_MANUFACT_ID		0x004
#define MANUALMODE_MANUFACT_ID	0xF00
#define MANUALMODE_PRODUCT_ID	0xF00
#define MANUALMODE_DENSITY_ID	0xF00

// The density codes gives the memory's adressing scheme
#define DENSITY_MB85RC04V		0x00 // 4K
#define DENSITY_MB85RC64TA		0x03 // 64K
#define DENSITY_MB85RC256V		0x05 // 512K
#define DENSITY_MB85RC512T		0x06 // 512K
#define DENSITY_MB85RC1MT		0x07 // 1M

#define DENSITY_CY15B128J		0x01 // 128K - FM24V01A also
#define DENSITY_CY15B256J		0x02 // 256K - FM24V02A also
#define DENSITY_FM24V05			0x03 // 512K
#define DENSITY_FM24V10			0x04 // 1024K

// Devices MB85RC16, MB85RC16V, MB85RC64A, MB85RC64V and MB85RC128A do not support Device ID reading
// 			FM24W256,FM24CL64B, FM24C64B, FM24C16B, FM24C04B, FM24CL04B

#define MAXADDRESS_04	512
#define MAXADDRESS_16	2048
#define MAXADDRESS_64	8192
#define MAXADDRESS_128	16384
#define MAXADDRESS_256	32768
#define MAXADDRESS_512	65536
#define MAXADDRESS_1024	65536 // 1M devices are in fact managed as 2 512 devices from lib point of view > create 2 instances of the object with each a differnt address

// Adresses
#define MB85RC_ADDRESS_A000		0x50
#define MB85RC_ADDRESS_A001		0x51
#define MB85RC_ADDRESS_A010		0x52
#define MB85RC_ADDRESS_A011		0x53
#define MB85RC_ADDRESS_A100		0x54
#define MB85RC_ADDRESS_A101		0x55
#define MB85RC_ADDRESS_A110		0x56
#define MB85RC_ADDRESS_A111		0x57
#define MB85RC_DEFAULT_ADDRESS	MB85RC_ADDRESS_A000

//Special commands
#define MASTER_CODE	0xF8
#define SLEEP_MODE	0x86 //Cypress codes, not used here
#define HIGH_SPEED	0x08 //Cypress codes, not used here

// Managing Write protect pin
#define MANAGE_WP false //false if WP pin remains not connected
#define DEFAULT_WP_PIN	13 //write protection pin - active high, write enabled when low
#define DEFAULT_WP_STATUS  false //false means protection is off - write is enabled

// Error management
#define ERROR_SUCCESS				0 // Success
#define ERROR_OUT_OF_RANGE			1 // Data too long to fit the transmission buffer on Arduino
#define ERROR_NACK_ON_ADDRESS		2 // received NACK on transmit of address
#define ERROR_NACK_ON_DATA			3 // received NACK on transmit of data
#define ERROR_SERIAL_UNAVAILABLE	4 // Serial seems not available
#define ERROR_DEVICE_ID				5 // Not referenced device ID
#define ERROR_UNUSED				6 // Unused
#define ERROR_CHIP_UNIDENTIFIED		7 // Fram chip unidentified
#define ERROR_TOO_SHORT				8 // Number of bytes asked to read null
#define ERROR_INVALID_BIT_POS		9 // Bit position out of range
#define ERROR_NOT_PERMITTED			10 // Not permitted operation
#define ERROR_OUT_OF_RANGE1			11 // Memory address out of range

class FRAM_MB85RC_I2C {
public:
	FRAM_MB85RC_I2C(void) {
		_framInitialised = false;
		_manualMode = false;
		i2c_addr = MB85RC_DEFAULT_ADDRESS;
		wpPin = DEFAULT_WP_PIN;
		byte result = FRAM_MB85RC_I2C::initWP(DEFAULT_WP_STATUS);
	}

	FRAM_MB85RC_I2C(uint8_t address, boolean wp) {
		_framInitialised = false;
		_manualMode = false;
		i2c_addr = address;
		wpPin = DEFAULT_WP_PIN;
		byte result = FRAM_MB85RC_I2C::initWP(wp);
	}

	FRAM_MB85RC_I2C(uint8_t address, boolean wp, int pin) {
		_framInitialised = false;
		_manualMode = false;
		i2c_addr = address;
		wpPin = pin;
		byte result = FRAM_MB85RC_I2C::initWP(wp);
	}

	FRAM_MB85RC_I2C(uint8_t address, boolean wp, int pin, uint16_t chipDensity) {
		//This constructor provides capability for chips without the device IDs implemented
		_framInitialised = false;
		_manualMode = true;
		i2c_addr = address;
		wpPin = pin;
		density = chipDensity;

		byte result = FRAM_MB85RC_I2C::initWP(wp);
	}

	void begin(void) {
		byte deviceFound = FRAM_MB85RC_I2C::checkDevice();

		#if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
		if (!Serial) Serial.begin(9600);
		if (Serial){
			Serial.println("FRAM_MB85RC_I2C object created");
			Serial.print("I2C device address 0x");
			Serial.println(i2c_addr, HEX);
			Serial.print("WP pin number ");
			Serial.println(wpPin, DEC);
			Serial.print("Write protect management: ");
			if(MANAGE_WP) {
				Serial.println("true");
			}
			else {
				Serial.println("false");
			}
			if(deviceFound == ERROR_SUCCESS) {
				Serial.println("Memory Chip initialized");
				FRAM_MB85RC_I2C::deviceIDs2Serial();
			}
			else {
				Serial.println("Memory Chip NOT FOUND");
			}
			Serial.println("...... ...... ......");
		}
		#endif
	}

	/**************************************************************************/
	/*!
	    Check if device is connected at address @i2c_addr
		returns 0 = device found
				7 = device not found
	*/
	/**************************************************************************/
	byte checkDevice(void) {
		byte result;
		if (_manualMode) {
			result = setDeviceIDs();
		} else {
			result = getDeviceIDs();
		}

		if ((result == ERROR_SUCCESS) && ((manufacturer == FUJITSU_MANUFACT_ID) || (manufacturer == CYPRESS_MANUFACT_ID) || (manufacturer == MANUALMODE_MANUFACT_ID)) && (maxaddress != 0)) {
			_framInitialised = true;
		} else {
			result = ERROR_CHIP_UNIDENTIFIED;
			_framInitialised = false;
		}
		return result;
	}

	/**************************************************************************/
	/*!
	    @brief  Reads one bit from the specified FRAM address

	    @params[in] framAddr
	                The 16-bit address to read from in FRAM memory
	    @params[in] bitNb
	                The bit position to read
		@params[out] *bit
					value of the bit: 0 | 1
	    @returns
					return code of Wire.endTransmission()
					return code 9 if bit position is larger than 7
	*/
	/**************************************************************************/
	byte readBit(uint16_t framAddr, uint8_t bitNb, byte *bit) {
		byte result;
		if (bitNb > 7) {
			result = ERROR_INVALID_BIT_POS;
		} else {
			uint8_t buffer[1];
			result = FRAM_MB85RC_I2C::readArray(framAddr, 1, buffer);
			*bit = bitRead(buffer[0], bitNb);
		}
		return result;
	}

	/**************************************************************************/
	/*!
	    @brief  Set one bit to the specified FRAM address

	    @params[in] framAddr
	                The 16-bit address to read from in FRAM memory
	    @params[in] bitNb
	                The bit position to set
	    @returns
					return code of Wire.endTransmission()
					return code 9 if bit position is larger than 7
	*/
	/**************************************************************************/
	byte setOneBit(uint16_t framAddr, uint8_t bitNb) {
		byte result;
		if (bitNb > 7)  {
			result = ERROR_INVALID_BIT_POS;
		} else {
			uint8_t buffer[1];
			result = FRAM_MB85RC_I2C::readArray(framAddr, 1, buffer);
			bitSet(buffer[0], bitNb);
			result = FRAM_MB85RC_I2C::writeArray(framAddr, 1, buffer);
		}
		return result;
	}

	/**************************************************************************/
	/*!
	    @brief  Clear one bit to the specified FRAM address

	    @params[in] framAddr
	                The 16-bit address to read from in FRAM memory
	    @params[in] bitNb
	                The bit position to clear
	    @returns
					return code of Wire.endTransmission()
					return code 9 if bit position is larger than 7
	*/
	/**************************************************************************/
	byte clearOneBit(uint16_t framAddr, uint8_t bitNb) {
		byte result;
		if (bitNb > 7) {
			result = ERROR_INVALID_BIT_POS;
		} else {
			uint8_t buffer[1];
			result = FRAM_MB85RC_I2C::readArray(framAddr, 1, buffer);
			bitClear(buffer[0], bitNb);
			result = FRAM_MB85RC_I2C::writeArray(framAddr, 1, buffer);
		}
		return result;
	}

	/**************************************************************************/
	/*!
	    @brief  Toggle one bit to the specified FRAM address

	    @params[in] framAddr
	                The 16-bit address to read from in FRAM memory
	    @params[in] bitNb
	                The bit position to toggle
	    @returns
					return code of Wire.endTransmission()
					return code 9 if bit position is larger than 7
	*/
	/**************************************************************************/
	byte toggleBit(uint16_t framAddr, uint8_t bitNb) {
		byte result;
		if (bitNb > 7) {
			result = ERROR_INVALID_BIT_POS;
		} else {
			uint8_t buffer[1];
			result = FRAM_MB85RC_I2C::readArray(framAddr, 1, buffer);

			if ( (buffer[0] & (1 << bitNb)) == (1 << bitNb) ) {
				bitClear(buffer[0], bitNb);
			} else {
				bitSet(buffer[0], bitNb);
			}
			result = FRAM_MB85RC_I2C::writeArray(framAddr, 1, buffer);
		}
		return result;
	}

	/**************************************************************************/
	/*!
	    @brief  Reads an array of bytes from the specified FRAM address

	    @params[in] i2cAddr
	                The I2C address of the FRAM memory chip (1010+A2+A1+A0)
	    @params[in] framAddr
	                The 16-bit address to read from in FRAM memory
		@params[in] items
					number of items to read from memory chip
		@params[out] values[]
					array to be filled in by the memory read
	    @returns
					return code of Wire.endTransmission()
	*/
	/**************************************************************************/
	byte readArray (uint16_t framAddr, byte items, uint8_t values[]) {
		if ((framAddr >= maxaddress) || ((framAddr + (uint16_t) items - 1) >= maxaddress)) return ERROR_OUT_OF_RANGE1;
		byte result;
		if (items == 0) {
			result = ERROR_TOO_SHORT; //number of bytes asked to read null
		} else {
			FRAM_MB85RC_I2C::I2CAddressAdapt(framAddr);
			result = Wire.endTransmission();
			Wire.requestFrom(i2c_addr, (uint8_t)items);
			for (byte i=0; i < items; i++) {
				values[i] = Wire.read();
			}
		}
		return result;
	}

	/**************************************************************************/
	/*!
	    @brief  Writes an array of bytes from a specific address

	    @params[in] i2cAddr
	                The I2C address of the FRAM memory chip (1010+A2+A1+A0)
	    @params[in] framAddr
	                The 16-bit address to write to in FRAM memory
	    @params[in] items
	                The number of items to write from the array
		@params[in] values[]
	                The array of bytes to write
		@returns
					return code of Wire.endTransmission()
	*/
	/**************************************************************************/
	byte writeArray (uint16_t framAddr, byte items, uint8_t values[]) {
		if ((framAddr >= maxaddress) || ((framAddr + (uint16_t) items - 1) >= maxaddress)) return ERROR_OUT_OF_RANGE1;
		FRAM_MB85RC_I2C::I2CAddressAdapt(framAddr);
		for (byte i=0; i < items ; i++) {
			Wire.write(values[i]);
		}
		return Wire.endTransmission();
	}

	/**************************************************************************/
	/*!
	    @brief  Reads one byte from the specified FRAM address

	    @params[in] i2cAddr
	                The I2C address of the FRAM memory chip (1010+A2+A1+A0)
	    @params[in] framAddr
	                The 16-bit address to read from in FRAM memory
		@params[out] *values
					data read from memory
	    @returns
					return code of Wire.endTransmission()
	*/
	/**************************************************************************/
	byte readByte (uint16_t framAddr, uint8_t *value) {
		uint8_t buffer[1];
		byte result = FRAM_MB85RC_I2C::readArray(framAddr, 1, buffer);
		*value = buffer[0];
		return result;
	}

	/**************************************************************************/
	/*!
	    @brief  Writes a single byte to a specific address

	    @params[in] i2cAddr
	                The I2C address of the FRAM memory chip (1010+A2+A1+A0)
	    @params[in] framAddr
	                The 16-bit address to write to in FRAM memory
		@params[in] value
	                One byte to write
		@returns
					return code of Wire.endTransmission()
	*/
	/**************************************************************************/
	byte writeByte (uint16_t framAddr, uint8_t value) {
		uint8_t buffer[] = {value};
		return FRAM_MB85RC_I2C::writeArray(framAddr, 1, buffer);
	}

	/**************************************************************************/
	/*!
	    @brief  Copy a byte from one address to another in the memory scope

	    @params[in] i2cAddr
	                The I2C address of the FRAM memory chip (1010+A2+A1+A0)
	    @params[in] origAddr
	                The 16-bit address to read from in FRAM memory
		@params[in] destAddr
					The 16-bit address to write in FRAM memory
	    @returns
					return code of Wire.endTransmission()
	*/
	/**************************************************************************/
	byte copyByte (uint16_t origAddr, uint16_t destAddr) {
		uint8_t buffer[1];
		byte result = readByte(origAddr, buffer);
		result = writeByte(destAddr, buffer[0]);
		return result;
	}

	/**************************************************************************/
	/*!
	    @brief  Reads a 16bits value from the specified FRAM address

	    @params[in] framAddr
	                The 16-bit address to read from in FRAM memory
		@params[out] value
					16bits word
	    @returns
					return code of Wire.endTransmission()
	*/
	/**************************************************************************/
	byte readWord(uint16_t framAddr, uint16_t *value) {
		uint8_t buffer[2];
		byte result = FRAM_MB85RC_I2C::readArray(framAddr, 2, buffer);
		*value = *reinterpret_cast<uint16_t *>(buffer);
		return result;
	}

	/**************************************************************************/
	/*!
	    @brief  Write a 16bits value from the specified FRAM address

	    @params[in] framAddr
	                The 16-bit address to read from in FRAM memory
		@params[in] value
					16bits word
	    @returns
					return code of Wire.endTransmission()
	*/
	/**************************************************************************/
	byte writeWord(uint16_t framAddr, uint16_t value) {
		uint8_t *buffer = reinterpret_cast<uint8_t *>(&value);
		return FRAM_MB85RC_I2C::writeArray(framAddr, 2, buffer);
	}

	/**************************************************************************/
	/*!
	    @brief  Read a 32bits value from the specified FRAM address

	    @params[in] framAddr
	                The 16-bit address to read from FRAM memory
		@params[in] value
					32bits word
	    @returns
					return code of Wire.endTransmission()
	*/
	/**************************************************************************/
	byte readLong(uint16_t framAddr, uint32_t *value) {
		uint8_t buffer[4];
		byte result = FRAM_MB85RC_I2C::readArray(framAddr, 4, buffer);
		*value = *reinterpret_cast<uint32_t *>(buffer);
		return result;
	}

	/**************************************************************************/
	/*!
	    @brief  Write a 32bits value to the specified FRAM address

	    @params[in] framAddr
	                The 16-bit address to write to FRAM memory
		@params[in] value
					32bits word
	    @returns
					return code of Wire.endTransmission()
	*/
	/**************************************************************************/
	byte writeLong(uint16_t framAddr, uint32_t value) {
		uint8_t *buffer = reinterpret_cast<uint8_t *>(&value);
		return FRAM_MB85RC_I2C::writeArray(framAddr, 4, buffer);
	}

	/**************************************************************************/
	/*!
	    @brief  Reads the Manufacturer ID and the Product ID frm the IC

	    @params[in]   idtype
					  1: Manufacturer ID, 2: ProductID, 3:density code, 4:density
		@params[out]  *id
	                  The 16 bits ID value
	    @returns
					  0: success
					  1: error
	*/
	/**************************************************************************/
	byte getOneDeviceID(uint8_t idType, uint16_t *id) {
		byte result;
		const uint8_t manuf = 1;
		const uint8_t prod = 2;
		const uint8_t densc = 3;
		const uint8_t densi = 4;

		switch (idType) {
		case manuf:
			*id = manufacturer;
			result = ERROR_SUCCESS;
			break;
		case prod:
			*id = productid;
			result = ERROR_SUCCESS;
			break;
		case densc:
			*id = densitycode;
			result = ERROR_SUCCESS;
			break;
		case densi:
			*id = density;
			result = ERROR_SUCCESS;
			break;
		default:
			*id = 0;
			result = ERROR_DEVICE_ID;
			break;
		}
		return result;
	}

	/**************************************************************************/
	/*!
	    @brief  Return the readiness of the memory chip

	    @params[in]  none
		@returns
					  boolean
					  true : ready
					  false : not ready
	*/
	/**************************************************************************/
	boolean	isReady(void) {
		return _framInitialised;
	}

	/**************************************************************************/
	/*!
	    @brief  Return tu Write Protect status

	    @params[in]  none
		@returns
					  boolean
					  true : write protect enabled
					  false: wirte protect disabled
	*/
	/**************************************************************************/
	boolean	getWPStatus(void) {
		return wpStatus;
	}

	/**************************************************************************/
	/*!
	    @brief  Enable write protect function of the chip by pulling up WP pin

	    @params[in]   MANAGE_WP
	                  WP management switch defined in header file
	    @params[in]   wpPin
	                  pin number for WP pin
		@param[out]	  wpStatus
		@returns
					  0: success
					  1: error, WP not managed
	*/
	/**************************************************************************/
	byte enableWP(void) {
		byte result;
		if (MANAGE_WP) {
			digitalWrite(wpPin,HIGH);
			wpStatus = true;
			result = ERROR_SUCCESS;
		} else {
			result = ERROR_NOT_PERMITTED;
		}
		return result;
	}

	/**************************************************************************/
	/*!
	    @brief  Disable write protect function of the chip by pulling up WP pin

	    @params[in]   MANAGE_WP
	                  WP management switch defined in header file
	    @params[in]   wpPin
	                  pin number for WP pin
		@param[out]	  wpStatus
		@returns
					  0: success
					  1: error, WP not managed
	*/
	/**************************************************************************/
	byte disableWP(void) {
		byte result;
		if (MANAGE_WP) {
			digitalWrite(wpPin,LOW);
			wpStatus = false;
			result = ERROR_SUCCESS;
		} else {
			result = ERROR_NOT_PERMITTED;
		}
		return result;
	}

	/**************************************************************************/
	/*!
	    @brief  Erase device by overwriting it to 0x00

	    @params[in]   SERIAL_DEBUG
	                  Outputs erasing results to Serial
		@returns
					  0: success
					  1-4: error writing at a certain position
	*/
	/**************************************************************************/
	byte eraseDevice(void) {
		byte result = 0;
		uint16_t i = 0;

		#ifdef SERIAL_DEBUG
		if (Serial)
			Serial.println("Start erasing device");
		#endif

		while((i < maxaddress) && (result == 0)) {
			result = FRAM_MB85RC_I2C::writeByte(i, 0x00);
			i++;
		}

		#if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
		if (Serial && result !=0) {
			Serial.print("ERROR: device erasing stopped at position ");
			Serial.println(i, DEC);
			Serial.println("...... ...... ......");
		} else {
			Serial.println("device erased");
			Serial.println("...... ...... ......");
		}
		#endif
		return result;
	}

 private:
	uint8_t	i2c_addr;
	boolean	_framInitialised;
	boolean	_manualMode;
	uint16_t	manufacturer;
	uint16_t	productid;
	uint16_t	densitycode;
	uint16_t	density;
	uint16_t	maxaddress;

	int	wpPin;
	boolean	wpStatus;

	/**************************************************************************/
	/*!
	    @brief  Reads the Manufacturer ID and the Product ID from the IC and populate class' variables for devices supporting that feature

	    @params[in]   none
		@params[out]  manufacturerID
	                  The 12-bit manufacturer ID (Fujitsu = 0x00A)
	    @params[out]  productID
	                  The memory density (bytes 11..8) and proprietary
	                  Product ID fields (bytes 7..0). Should be 0x510 for
	                  the MB85RC256V for instance.
		@param[out]	  The memory densitycode (bytes 11..8)
					  from 0x03 (64K chip) to 0x07 (1M chip)
		@param[out]	  The memory density got from density code
					  from 64 to 1024K
		@param[out]	  The memory max address of storage slot
	    @returns
					  return code of Wire.endTransmission() or interpreted error.
	*/
	/**************************************************************************/
	byte getDeviceIDs(void) {
		uint8_t localbuffer[3] = { 0, 0, 0 };
		uint8_t result;

		/* Get device IDs sequence 	*/
		/* 1/ Send 0xF8 to the I2C bus as a write instruction. bit 0: 0 => 0xF8 >> 1 */
		/* Send 0xF8 to 12C bus. Bit shift to right as beginTransmission() requires a 7bit. beginTransmission() 0 for write => 0xF8 */
		/* Send device address as 8 bits. Bit shift to left as we are using a simple write()                                        */
		/* Send 0xF9 to I2C bus. By requesting 3 bytes to read, requestFrom() add a 1 bit at the end of a 7 bits address => 0xF9    */
		/* See p.10 of http://www.fujitsu.com/downloads/MICRO/fsa/pdf/products/memory/fram/MB85RC-DS501-00017-3v0-E.pdf             */

		Wire.beginTransmission(MASTER_CODE >> 1);
		Wire.write((byte)(i2c_addr << 1));
		result = Wire.endTransmission(false);

		Wire.requestFrom(MASTER_CODE >> 1, 3);
		localbuffer[0] = (uint8_t) Wire.read();
		localbuffer[1] = (uint8_t) Wire.read();
		localbuffer[2] = (uint8_t) Wire.read();

		/* Shift values to separate IDs */
		manufacturer = (localbuffer[0] << 4) + (localbuffer[1] >> 4);
		densitycode = (uint16_t)(localbuffer[1] & 0x0F);
		productid = ((localbuffer[1] & 0x0F) << 8) + localbuffer[2];

		if (manufacturer == FUJITSU_MANUFACT_ID) {
			switch (densitycode) {
			case DENSITY_MB85RC04V:
				density = 4;
				maxaddress = MAXADDRESS_04;
				break;
			case DENSITY_MB85RC64TA:
				density = 64;
				maxaddress = MAXADDRESS_64;
				break;
			case DENSITY_MB85RC256V:
				density = 256;
				maxaddress = MAXADDRESS_256;
				break;
			case DENSITY_MB85RC512T:
				density = 512;
				maxaddress = MAXADDRESS_512;
				break;
			case DENSITY_MB85RC1MT:
				density = 1024;
				maxaddress = MAXADDRESS_1024;
				break;
			default:
				density = 0; /* means error */
				maxaddress = 0; /* means error */
				if (result == 0) result = ERROR_CHIP_UNIDENTIFIED; /*device unidentified, comminication ok*/
				break;
			}
		} else if (manufacturer == CYPRESS_MANUFACT_ID) {
			switch (densitycode) {
			case DENSITY_CY15B128J:
				density = 128;
				maxaddress = MAXADDRESS_128;
				break;
			case DENSITY_CY15B256J:
				density = 256;
				maxaddress = MAXADDRESS_256;
				break;
			case DENSITY_FM24V05:
				density = 512;
				maxaddress = MAXADDRESS_512;
				break;
			case DENSITY_FM24V10:
				density = 1024;
				maxaddress = MAXADDRESS_1024;
				break;
			default:
				density = 0; /* means error */
				maxaddress = 0; /* means error */
				if (result == 0) result = ERROR_CHIP_UNIDENTIFIED; /*device unidentified, comminication ok*/
				break;
			}
		} else {
			density = 0; /* means error */
			maxaddress = 0; /* means error */
			if (result == 0) result = ERROR_CHIP_UNIDENTIFIED; /*device unidentified, comminication ok*/
		}

		return result;
	}

	/**************************************************************************/
	/*!
	    @brief  set devices IDs for chip that does not support the feature as this has not been implemented in every chips by manufacturers

	    @params[in]   none

		@params[out]  manufacturerID set as "manual mode"

		@param[out]	  The memory max address of storage slot
	    @returns
					  return ERROR_SUCCESS, ERROR_CHIP_UNIDENTIFIED, ERROR_NOT_PERMITTED codes
	*/
	/**************************************************************************/
	byte setDeviceIDs(void) {
		if(_manualMode) {
			switch(density) {
				case 4:
					maxaddress = MAXADDRESS_04;
					break;
				case 16:
					maxaddress = MAXADDRESS_16;
					break;
				case 64:
					maxaddress = MAXADDRESS_64;
					break;
				case 128:
					maxaddress = MAXADDRESS_128;
					break;
				case 256:
					maxaddress = MAXADDRESS_256;
					break;
				case 512:
					maxaddress = MAXADDRESS_512;
					break;
				case 1024:
					maxaddress = MAXADDRESS_1024;
					break;
				default:
					maxaddress = 0; /* means error */
					break;
			}
			densitycode = MANUALMODE_DENSITY_ID;
			productid = MANUALMODE_PRODUCT_ID;
			manufacturer = MANUALMODE_MANUFACT_ID;
			if (maxaddress !=0) {
				return ERROR_SUCCESS;
			} else {
				return ERROR_CHIP_UNIDENTIFIED;
			}
		} else {
			return ERROR_NOT_PERMITTED;
		}
	}

	/**************************************************************************/
	/*!
	    @brief  Init write protect function for class constructor

	    @params[in]   MANAGE_WP
	                  WP management switch defined in header file
	    @params[in]   wpPin
	                  pin number for WP pin
	    @params[in]   wp
	                  Boolean for startup WP
		@param[out]	  wpStatus
		@returns
					  0: success, init done
					  1: error - should never happen
	*/
	/**************************************************************************/
	byte initWP(boolean wp) {
		byte result;
		if (MANAGE_WP) {
			pinMode(wpPin, OUTPUT);
			if (wp) {
				result = FRAM_MB85RC_I2C::enableWP();
			} else {
				result = FRAM_MB85RC_I2C::disableWP();
			}
		} else {
			wpStatus = false;
			result = ERROR_SUCCESS;
		}
		return result;
	}

	/**************************************************************************/
	/*!
	    @brief  Utility function to print out memory chip IDs to serial if Debug enabled

	    @params[in]   SERIAL_DEBUG
		@param[out]	  none
		@returns
					  0: success
					  4: error, debug not activated or Serial not available
	*/
	/**************************************************************************/
	byte deviceIDs2Serial(void) {
		byte result = ERROR_SERIAL_UNAVAILABLE;
		#ifdef SERIAL_DEBUG
		if (Serial){
			Serial.println("FRAM Device IDs");
			Serial.print("Manufacturer 0x"); Serial.println(manufacturer, HEX);
			Serial.print("ProductID 0x"); Serial.println(productid, HEX);
			Serial.print("Density code 0x"); Serial.println(densitycode, HEX);
			Serial.print("Density "); Serial.print(density, DEC); Serial.println("K");
			if ((manufacturer != MANUALMODE_MANUFACT_ID) && (density > 0))  Serial.println("Device identfied automatically");
			if ((manufacturer == MANUALMODE_MANUFACT_ID) && (density > 0))  Serial.println("Device properties set");
			Serial.println("...... ...... ......");
			result = ERROR_SUCCESS;
		}
		#endif
		return result;
	}

	/**************************************************************************/
	/*!
	    @brief 	Adapts the I2C calls (chip address + memory pointer) according to chip datasheet
				4K chips : 1 MSB of memory address as LSB of device address + 8 bits memory address
				16K chips : 3 MSB of memory address as LSB of device address + 8 bits memory address
				64K and more chips : full chipp address & 16 bits memory address

	    @params[in]  address : memory address
		@param[out]	 none
		@returns	 void
	*/
	/**************************************************************************/
	void I2CAddressAdapt(uint16_t framAddr) {
		uint8_t chipaddress;

		switch(density) {
		case 4:
			chipaddress = (i2c_addr | ((framAddr >> 8) & 0x1));
			break;
		case 16:
			chipaddress = (i2c_addr | ((framAddr >> 8) & 0x7));
			break;
		default:
			chipaddress = i2c_addr;
			break;
		}

		#if defined(SERIAL_DEBUG) && (SERIAL_DEBUG == 1)
		Serial.print("Calculated address 0x");
		Serial.println(chipaddress, HEX);
		#endif

		if (density < 64) {
			Wire.beginTransmission(chipaddress);
			Wire.write(framAddr & 0xFF);
		} else {
			Wire.beginTransmission(chipaddress);
			Wire.write(framAddr >> 8);
			Wire.write(framAddr & 0xFF);
		}
		return;
	}
};

#endif
