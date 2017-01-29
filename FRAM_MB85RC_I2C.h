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

#include <utility/twi.h>
#include <Wire.h>

#include "FRAM_defines.h"
#include "FRAM_WriteProtectManagers.h"
#include "FRAM_DeviceIdentifiers.h"

// Enabling debug I2C - comment to disable / normal operations
//#define DEBUB_SERIAL_FRAM_MB85RC_I2C Serial

// Managing Write protect pin
#define MANAGE_WP false //false if WP pin remains not connected
#define DEFAULT_WP_PIN	13 //write protection pin - active high, write enabled when low
#define DEFAULT_WP_STATUS  false //false means protection is off - write is enabled

template < typename WriteProtectT, typename DeviceIdentifierT >
class FRAM_MB85RC_I2C_T {
public:
	// This constructor probes the i2c bus for a device with device IDs implemented
	FRAM_MB85RC_I2C_T(uint8_t const address = MB85RC_DEFAULT_ADDRESS, bool const wp = DEFAULT_WP_STATUS, uint8_t const pin = DEFAULT_WP_PIN) :
		_i2c_addr(address),
		_wp(wp, pin),
		_device_id(false, 0)
	{
	}

	// This constructor provides capability for chips without the device IDs implemented
	FRAM_MB85RC_I2C_T(uint8_t const address, bool const wp, uint8_t const pin, uint16_t const density) :
		_i2c_addr(address),
		_wp(wp, pin),
		_device_id(true, density)
	{
	}

	void begin(void) {
		_wp.init();

		#if defined(DEBUB_SERIAL_FRAM_MB85RC_I2C)
		byte deviceFound =
		#endif
			checkDevice();

		#if defined(DEBUB_SERIAL_FRAM_MB85RC_I2C)
		if (DEBUB_SERIAL_FRAM_MB85RC_I2C) {
			DEBUB_SERIAL_FRAM_MB85RC_I2C.println(F("FRAM_MB85RC_I2C object created"));
			DEBUB_SERIAL_FRAM_MB85RC_I2C.print(F("I2C device address 0x"));
			DEBUB_SERIAL_FRAM_MB85RC_I2C.println(_i2c_addr, HEX);
			_wp.describe();
			if (deviceFound == ERROR_SUCCESS) {
				DEBUB_SERIAL_FRAM_MB85RC_I2C.println(F("Memory Chip initialized"));
				_device_id.describe();
			} else {
				DEBUB_SERIAL_FRAM_MB85RC_I2C.println(F("Memory Chip NOT FOUND"));
			}
			DEBUB_SERIAL_FRAM_MB85RC_I2C.println(F("...... ...... ......"));
		}
		#endif
	}

	/**************************************************************************/
	/*!
	    Check if device is connected at address @_i2c_addr
		returns 0 = device found
				7 = device not found
	*/
	/**************************************************************************/
	byte checkDevice(void) {
		return _device_id.checkDevice(_i2c_addr);
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
	byte readBit(uint16_t const & framAddr, uint8_t const bitNb, byte * bit) {
		byte result;
		if (bitNb > 7) {
			result = ERROR_INVALID_BIT_POS;
		} else {
			uint8_t buffer[1];
			result = readArray(framAddr, 1, buffer);
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
	byte setOneBit(uint16_t const & framAddr, uint8_t const bitNb) {
		byte result;
		if (bitNb > 7)  {
			result = ERROR_INVALID_BIT_POS;
		} else {
			uint8_t buffer[1];
			result = readArray(framAddr, 1, buffer);
			bitSet(buffer[0], bitNb);
			result = writeArray(framAddr, 1, buffer);
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
	byte clearOneBit(uint16_t const & framAddr, uint8_t const bitNb) {
		byte result;
		if (bitNb > 7) {
			result = ERROR_INVALID_BIT_POS;
		} else {
			uint8_t buffer[1];
			result = readArray(framAddr, 1, buffer);
			bitClear(buffer[0], bitNb);
			result = writeArray(framAddr, 1, buffer);
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
	byte toggleBit(uint16_t const & framAddr, uint8_t const bitNb) {
		byte result;
		if (bitNb > 7) {
			result = ERROR_INVALID_BIT_POS;
		} else {
			uint8_t buffer[1];
			result = readArray(framAddr, 1, buffer);

			if ( (buffer[0] & (1 << bitNb)) == (1 << bitNb) ) {
				bitClear(buffer[0], bitNb);
			} else {
				bitSet(buffer[0], bitNb);
			}
			result = writeArray(framAddr, 1, buffer);
		}
		return result;
	}

	/**************************************************************************/
	/*!
	    @brief  Reads an array of bytes from the specified FRAM address without
				any checks! (Unsafe!)

	    @params[in] framAddr
	                The 16-bit address to read from in FRAM memory
		@params[in] items
					number of items to read from memory chip
		@params[out] values
					array to be filled in by the memory read
	    @returns
					return code of Wire.endTransmission()
	*/
	/**************************************************************************/
	byte readArrayUnsafe(uint16_t const & framAddr, uint8_t const items, uint8_t * const values) {
		_framAddressAdapt(framAddr);
		byte result = Wire.endTransmission();
		Wire.requestFrom(_i2c_addr, items);
		for (byte i=0; i < items; i++)
			values[i] = Wire.read();
		return result;
	}

	/**************************************************************************/
	/*!
	    @brief  Reads an array of bytes from the specified FRAM address.
				This method does range check and length check, and tries to read
				all the data even if length exceeds TWI_BUFFER_LENGTH.

	    @params[in] framAddr
	                The 16-bit address to read from in FRAM memory
		@params[in] items
					number of items to read from memory chip
		@params[out] values
					array to be filled in by the memory read
	    @returns
					return code of Wire.endTransmission()
	*/
	/**************************************************************************/
	byte readArray(uint16_t framAddr, uint16_t items, uint8_t * values) {
		if (items >= _device_id.getMaxAddress())
			return ERROR_TOO_LONG;
		if ((framAddr >= _device_id.getMaxAddress()) || ((framAddr + items - 1) >= _device_id.getMaxAddress()))
			return ERROR_OUT_OF_RANGE;
		if (items == 0)
			return ERROR_TOO_SHORT; //number of bytes asked to read null

		byte result = 0;
		while ((items != 0) && (result == 0)) {
			uint8_t bytes = min(items, TWI_BUFFER_LENGTH);
			result = readArrayUnsafe(framAddr, bytes, values);
			framAddr += bytes;
			values += bytes;
			items -= bytes;
		}
		return result;
	}

	/**************************************************************************/
	/*!
	    @brief  Writes an array of bytes to the specific FRAM address without
				any checks! (Unsafe!)

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
	byte writeArrayUnsafe(uint16_t const & framAddr, byte const items, uint8_t const * const values) {
		_framAddressAdapt(framAddr);
		for (byte i=0; i < items ; i++) {
			Wire.write(values[i]);
		}
		return Wire.endTransmission();
	}

	/**************************************************************************/
	/*!
	    @brief  Writes an array of bytes to the specific FRAM address.
				This method does range check and length check, and tries to write
				all the data even if length exceeds TWI_BUFFER_LENGTH.
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
	byte writeArray(uint16_t framAddr, uint16_t items, uint8_t const * values) {
		if (items >=  _device_id.getMaxAddress())
			return ERROR_TOO_LONG;
		if ((framAddr >=  _device_id.getMaxAddress()) || ((framAddr + items - 1) >= _device_id.getMaxAddress()))
		 	return ERROR_OUT_OF_RANGE;
		if (items == 0)
			return ERROR_TOO_SHORT; // number of bytes asked to write null

		byte result = 0;
		while ((items != 0) && (result == 0)) {
			uint8_t bytes = min(items, TWI_BUFFER_LENGTH);
			result = writeArrayUnsafe(framAddr, bytes, values);
			framAddr += bytes;
			values += bytes;
			items -= bytes;
		}
		return result;
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
	__attribute__ ((always_inline, deprecated)) inline
	byte readByte (uint16_t const & framAddr, uint8_t * const value) {
		return readArray(framAddr, sizeof(uint8_t), value);
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
	__attribute__ ((always_inline, deprecated)) inline
	byte writeByte (uint16_t const & framAddr, uint8_t const value) {
		return writeArray(framAddr, sizeof(uint8_t), &value);
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
	byte copyByte (uint16_t const & origAddr, uint16_t const & destAddr) {
		uint8_t buffer;
		byte result = readByte(origAddr, &buffer);
		result = writeByte(destAddr, buffer);
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
	__attribute__ ((always_inline, deprecated)) inline
	byte readWord(uint16_t const & framAddr, uint16_t * const value) {
		return readArray(framAddr, sizeof(uint16_t), reinterpret_cast<uint8_t *>(value));
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
	__attribute__ ((always_inline, deprecated)) inline
	byte writeWord(uint16_t const & framAddr, uint16_t const & value) {
		return writeArray(framAddr, sizeof(uint16_t), reinterpret_cast<uint8_t const *>(&value));
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
	__attribute__ ((always_inline, deprecated)) inline
	byte readLong(uint16_t const & framAddr, uint32_t * const value) {
		return readArray(framAddr, sizeof(uint32_t), reinterpret_cast<uint8_t * const>(value));
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
	__attribute__ ((always_inline, deprecated)) inline
	byte writeLong(uint16_t const & framAddr, uint32_t const & value) {
		return writeArray(framAddr, sizeof(uint32_t), reinterpret_cast<uint8_t const * const>(&value));
	}

	/**************************************************************************/
	/*!
	    @brief  Read a 32bits value from the specified FRAM address

	    @params[in] framAddr
	                The 16-bit address to read from FRAM memory
		@params[out] value
					Any tope of value
	    @returns
					return code of Wire.endTransmission()
	*/
	/**************************************************************************/
	template < typename T >
	__attribute__ ((always_inline))	inline
	byte readFrom(uint16_t const & framAddr, T & value) {
		return readArray(framAddr, sizeof(T), reinterpret_cast<uint8_t * const>(&value));
	}

	/**************************************************************************/
	/*!
	    @brief  Write a value to the specified FRAM address

	    @params[in] framAddr
	                The 16-bit address to write to FRAM memory
		@params[in] value
					Any type of value
	    @returns
					return code of Wire.endTransmission()
	*/
	/**************************************************************************/
	template < typename T >
	__attribute__ ((always_inline)) inline
	byte writeTo(uint16_t const framAddr, T const & value) {
		return writeArray(framAddr, sizeof(T), reinterpret_cast<uint8_t const * const>(&value));
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
	byte getOneDeviceID(uint8_t const idType, uint16_t * const id) {
		return _device_id.getOneDeviceID(idType, id);
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
	__attribute__ ((always_inline)) inline
	boolean	isReady(void) {
		return _device_id.isFramInitialized();
	}

	/**************************************************************************/
	/*!
	    @brief  Return tu Write Protect status

		@returns
					  boolean
					  true : write protect enabled
					  false: wirte protect disabled
	*/
	/**************************************************************************/
	__attribute__ ((always_inline)) inline
	boolean	getWPStatus(void) {
		return _wp.status();
	}

	/**************************************************************************/
	/*!
	    @brief  Enable write protect function of the chip by pulling up WP pin

		@returns
					0: success
					1: error, WP not managed
	*/
	/**************************************************************************/
	__attribute__ ((always_inline)) inline
	byte enableWP(void) {
		return _wp.enable();
	}

	/**************************************************************************/
	/*!
	    @brief  Disable write protect function of the chip by pulling up WP pin

		@returns
					  0: success
					  1: error, WP not managed
	*/
	/**************************************************************************/
	__attribute__ ((always_inline)) inline
	byte disableWP(void) {
		return _wp.disable();
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

		#if defined(DEBUB_SERIAL_FRAM_MB85RC_I2C)
		if (DEBUB_SERIAL_FRAM_MB85RC_I2C)
			DEBUB_SERIAL_FRAM_MB85RC_I2C.println(F("Start erasing device"));
		#endif

		uint16_t addr = 0x0000;
		uint32_t length = _device_id.getMaxAddress();
		while (length != 0) {
			uint8_t bytes = min(length, TWI_BUFFER_LENGTH);
			_framAddressAdapt(addr);
			while(bytes-- && (result == 0)) {
				result = Wire.write(0x00);
			}
			addr += bytes;
			length -= bytes;
		}

		#if defined(DEBUB_SERIAL_FRAM_MB85RC_I2C)
		if (DEBUB_SERIAL_FRAM_MB85RC_I2C && result != 0) {
			DEBUB_SERIAL_FRAM_MB85RC_I2C.print(F("ERROR: device erasing stopped at position "));
			DEBUB_SERIAL_FRAM_MB85RC_I2C.println(i, DEC);
			DEBUB_SERIAL_FRAM_MB85RC_I2C.println(F("...... ...... ......"));
		} else {
			DEBUB_SERIAL_FRAM_MB85RC_I2C.println(F("device erased"));
			DEBUB_SERIAL_FRAM_MB85RC_I2C.println(F("...... ...... ......"));
		}
		#endif
		return result;
	}

 private:
	uint8_t	_i2c_addr;

	WriteProtectT _wp;
	DeviceIdentifierT _device_id;

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
	void _framAddressAdapt(uint16_t const & framAddr) {
		_i2c_addr = _device_id.framAddressAdapt(_i2c_addr, framAddr);
	}
};

// backward compatibility
#if MANAGE_WP
typedef FRAM_MB85RC_I2C_T<WriteProtect_Dynamic, DeviceIdentifier_Probe> FRAM_MB85RC_I2C;
#else
typedef FRAM_MB85RC_I2C_T<WriteProtect_Unmanaged, DeviceIdentifier_Probe> FRAM_MB85RC_I2C;
#endif

#endif
