#ifndef _FRAM_DEVICE_IDENTIFIERS_H_
#define _FRAM_DEVICE_IDENTIFIERS_H_

#include <Arduino.h>

#include <Wire.h>

#include "FRAM_defines.h"

// a DeviceIdentifier that supports both auto and manual modes
class DeviceIdentifier_Probe {
public:
	typedef uint32_t fram_address_t;

	DeviceIdentifier_Probe(bool const manual, uint16_t const & density):
		_manual(manual),
		_framInitialized(false),
		_density(density)
	{ }

	inline __attribute((always_inline))
	uint8_t describe() {
		#if defined(DEBUB_SERIAL_FRAM_MB85RC_I2C)
		if (DEBUB_SERIAL_FRAM_MB85RC_I2C) {
			DEBUB_SERIAL_FRAM_MB85RC_I2C.println(F("FRAM Device IDs"));
			DEBUB_SERIAL_FRAM_MB85RC_I2C.print(F("Manufacturer 0x"));
			DEBUB_SERIAL_FRAM_MB85RC_I2C.println(_manufacturer, HEX);
			DEBUB_SERIAL_FRAM_MB85RC_I2C.print(F("ProductID 0x"));
			DEBUB_SERIAL_FRAM_MB85RC_I2C.println(_productid, HEX);
			DEBUB_SERIAL_FRAM_MB85RC_I2C.print(F("Density code 0x"));
			DEBUB_SERIAL_FRAM_MB85RC_I2C.println(_densitycode, HEX);
			DEBUB_SERIAL_FRAM_MB85RC_I2C.print(F("Density "));
			DEBUB_SERIAL_FRAM_MB85RC_I2C.print(_density, DEC);
			DEBUB_SERIAL_FRAM_MB85RC_I2C.println('K');
			if ((_manufacturer != MANUALMODE_MANUFACT_ID) && (_density > 0))
				DEBUB_SERIAL_FRAM_MB85RC_I2C.println(F("Device identfied automatically"));
			if ((_manufacturer == MANUALMODE_MANUFACT_ID) && (_density > 0))
				DEBUB_SERIAL_FRAM_MB85RC_I2C.println(F("Device properties set"));
			DEBUB_SERIAL_FRAM_MB85RC_I2C.println(F("...... ...... ......"));
			return ERROR_SUCCESS;
		}
		return ERROR_SERIAL_UNAVAILABLE;
		#else
		return ERROR_SUCCESS;
		#endif
	}

	inline __attribute((always_inline))
	uint8_t checkDevice(uint8_t i2c_addr) {
		byte result;
		if (_manual) {
			result = setDeviceIDs();
		} else {
			result = getDeviceIDs(i2c_addr);
		}

		if ((result == ERROR_SUCCESS) && ((_manufacturer == FUJITSU_MANUFACT_ID) || (_manufacturer == CYPRESS_MANUFACT_ID) || (_manufacturer == MANUALMODE_MANUFACT_ID)) && (_maxaddress != 0)) {
			_framInitialized = true;
		} else {
			result = ERROR_CHIP_UNIDENTIFIED;
			_framInitialized = false;
		}
		return result;
	}

	uint8_t getOneDeviceID(uint8_t const idType, uint16_t * const id) {
		byte result = ERROR_SUCCESS;
		const uint8_t manuf = 1;
		const uint8_t prod = 2;
		const uint8_t densc = 3;
		const uint8_t densi = 4;

		switch (idType) {
		case manuf:
			*id = _manufacturer;
			break;
		case prod:
			*id = _productid;
			break;
		case densc:
			*id = _densitycode;
			break;
		case densi:
			*id = _density;
			break;
		default:
			*id = 0;
			result = ERROR_DEVICE_ID;
			break;
		}
		return result;
	}

	inline __attribute((always_inline))
	bool isInitialized() { return _framInitialized; }

	inline __attribute((always_inline))
	fram_address_t getMaxAddress() { return _maxaddress; }

	uint8_t framAddressAdapt(uint8_t const i2c_addr, fram_address_t const & framAddr) {
		uint8_t ret_addr = i2c_addr;
		if (_maxaddress <= 512) { // 4Kbit
			ret_addr = ((i2c_addr & 0b11111110) | ((framAddr >> 8) & 0b00000001));
		} else if (_maxaddress <= 4096) { // 16Kbit
			ret_addr = ((i2c_addr & 0b11111000) | ((framAddr >> 8) & 0b00000111));
		} else if (_maxaddress <= 65536) { // 64Kbit ~ 512Kbit
			// does nothing
		} else { // 1Mbit
			ret_addr = ((i2c_addr & 0b11111110) | ((framAddr >> 16) & 0b00000001));
		}

		#if defined(DEBUB_SERIAL_FRAM_MB85RC_I2C)
		DEBUB_SERIAL_FRAM_MB85RC_I2C.print("Calculated address 0x");
		DEBUB_SERIAL_FRAM_MB85RC_I2C.println(_i2c_addr, HEX);
		#endif

		if (_maxaddress <= 4096) {
			Wire.beginTransmission(ret_addr);
			Wire.write(framAddr & 0xFF);
		} else {
			Wire.beginTransmission(ret_addr);
			Wire.write(framAddr >> 8);
			Wire.write(framAddr & 0xFF);
		}

		return ret_addr;
	}

private:
	bool _manual;
	bool _framInitialized;
	uint16_t _manufacturer;
	uint16_t _productid;
	uint16_t _densitycode;
	uint16_t _density;
	uint32_t _maxaddress;

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
	byte getDeviceIDs(uint8_t i2c_addr) {
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
		_manufacturer = (localbuffer[0] << 4) + (localbuffer[1] >> 4);
		_densitycode = (uint16_t)(localbuffer[1] & 0x0F);
		_productid = ((localbuffer[1] & 0x0F) << 8) + localbuffer[2];

		if (_manufacturer == FUJITSU_MANUFACT_ID) {
			switch (_densitycode) {
			case DENSITY_MB85RC04V:
				_density = 4;
				_maxaddress = MAXADDRESS_04;
				break;
			case DENSITY_MB85RC64TA:
				_density = 64;
				_maxaddress = MAXADDRESS_64;
				break;
			case DENSITY_MB85RC256V:
				_density = 256;
				_maxaddress = MAXADDRESS_256;
				break;
			case DENSITY_MB85RC512T:
				_density = 512;
				_maxaddress = MAXADDRESS_512;
				break;
			case DENSITY_MB85RC1MT:
				_density = 1024;
				_maxaddress = MAXADDRESS_1024;
				break;
			default:
				_density = 0; /* means error */
				_maxaddress = 0; /* means error */
				if (result == 0) result = ERROR_CHIP_UNIDENTIFIED; /*device unidentified, comminication ok*/
				break;
			}
		} else if (_manufacturer == CYPRESS_MANUFACT_ID) {
			switch (_densitycode) {
			case DENSITY_CY15B128J:
				_density = 128;
				_maxaddress = MAXADDRESS_128;
				break;
			case DENSITY_CY15B256J:
				_density = 256;
				_maxaddress = MAXADDRESS_256;
				break;
			case DENSITY_FM24V05:
				_density = 512;
				_maxaddress = MAXADDRESS_512;
				break;
			case DENSITY_FM24V10:
				_density = 1024;
				_maxaddress = MAXADDRESS_1024;
				break;
			default:
				_density = 0; /* means error */
				_maxaddress = 0; /* means error */
				if (result == 0) result = ERROR_CHIP_UNIDENTIFIED; /*device unidentified, comminication ok*/
				break;
			}
		} else {
			_density = 0; /* means error */
			_maxaddress = 0; /* means error */
			if (result == 0) result = ERROR_CHIP_UNIDENTIFIED; /*device unidentified, comminication ok*/
		}

		return result;
	}

	/**************************************************************************/
	/*!
		@brief  set devices IDs for chip that does not support the feature as
				this has not been implemented in every chips by manufacturers

		@params[in]	none

		@returns	ERROR_SUCCESS on success,
					ERROR_CHIP_UNIDENTIFIED, ERROR_NOT_PERMITTED codes
	*/
	/**************************************************************************/
	uint8_t setDeviceIDs(void) {
		if(!_manual)
			return ERROR_NOT_PERMITTED;

		switch(_density) {
			case 4:
				_maxaddress = MAXADDRESS_04;
				break;
			case 16:
				_maxaddress = MAXADDRESS_16;
				break;
			case 64:
				_maxaddress = MAXADDRESS_64;
				break;
			case 128:
				_maxaddress = MAXADDRESS_128;
				break;
			case 256:
				_maxaddress = MAXADDRESS_256;
				break;
			case 512:
				_maxaddress = MAXADDRESS_512;
				break;
			case 1024:
				_maxaddress = MAXADDRESS_1024;
				break;
			default:
				_maxaddress = 0; /* means error */
				break;
		}
		_densitycode = MANUALMODE_DENSITY_ID;
		_productid = MANUALMODE_PRODUCT_ID;
		_manufacturer = MANUALMODE_MANUFACT_ID;

		return _maxaddress == 0 ?
			ERROR_CHIP_UNIDENTIFIED :
			ERROR_SUCCESS;
	}
};

// a DeviceIdentifier that only supports manual modes (omitting all the probing)
// codes, results a smaller footprint in both code space and memory
template < typename address_t, uint32_t maxaddress >
class DeviceIdentifier_Static {
public:
	typedef address_t fram_address_t;

	inline __attribute((always_inline))
	DeviceIdentifier_Static(bool const /* manual */, uint32_t const & /* density */)
	{ }

	inline __attribute((always_inline))
	uint8_t describe() {
		#if defined(DEBUG_SERIAL_FRAM_MB85RC_I2C)
		if (DEBUG_SERIAL_FRAM_MB85RC_I2C) {
			DEBUB_SERIAL_FRAM_MB85RC_I2C.println(F("FRAM Device IDs"));
			DEBUG_SERIAL_FRAM_MB85RC_I2C.println(F("Manual mode: true"));
			DEBUB_SERIAL_FRAM_MB85RC_I2C.print(F("Density "));
			DEBUB_SERIAL_FRAM_MB85RC_I2C.print(maxaddress / 8 / 1024, DEC);
			DEBUB_SERIAL_FRAM_MB85RC_I2C.println('K');
			DEBUB_SERIAL_FRAM_MB85RC_I2C.println(F("...... ...... ......"));
			return ERROR_SUCCESS;
		}
		return ERROR_SERIAL_UNAVAILABLE;
		#endif
		return ERROR_SUCCESS;
	}

	inline __attribute((always_inline))
	uint8_t checkDevice(uint8_t const /* i2c_addr */) { return ERROR_SUCCESS; }

	inline __attribute((always_inline))
	bool isInitialized() { return true; }

	inline __attribute((always_inline))
	fram_address_t getMaxAddress() { return maxaddress; }

	uint8_t getOneDeviceID(uint8_t const idType, uint16_t * const id) {
		byte result = ERROR_SUCCESS;
		const uint8_t manuf = 1;
		const uint8_t prod = 2;
		const uint8_t densc = 3;
		const uint8_t densi = 4;

		switch (idType) {
		case manuf:
			*id = MANUALMODE_MANUFACT_ID;
			break;
		case prod:
			*id = MANUALMODE_PRODUCT_ID;
			break;
		case densc:
			*id = MANUALMODE_DENSITY_ID;
			break;
		case densi:
			*id = maxaddress / 8 / 1024;
			break;
		default:
			*id = 0;
			result = ERROR_DEVICE_ID;
			break;
		}
		return result;
	}

	inline __attribute((always_inline)) // force inline to optimize out uint32_t for framAddr
	uint8_t framAddressAdapt(uint8_t const i2c_addr, uint32_t const & framAddr) {
		uint8_t ret_addr = i2c_addr;
		if (maxaddress <= 512) { // 4Kbit
			ret_addr = ((i2c_addr & 0b11111110) | ((framAddr >> 8) & 0b00000001));
		} else if (maxaddress <= 4096) { // 16Kbit
			ret_addr = ((i2c_addr & 0b11111000) | ((framAddr >> 8) & 0b00000111));
		} else if (maxaddress <= 65536) { // 64Kbit ~ 512Kbit
			// does nothing
		} else { // 1Mbit
			ret_addr = ((i2c_addr & 0b11111110) | ((framAddr >> 16) & 0b00000001));
		}

		#if defined(DEBUB_SERIAL_FRAM_MB85RC_I2C)
		DEBUB_SERIAL_FRAM_MB85RC_I2C.print("Calculated address 0x");
		DEBUB_SERIAL_FRAM_MB85RC_I2C.println(ret_addr, HEX);
		#endif

		if (maxaddress <= 4096) {
			Wire.beginTransmission(ret_addr);
			Wire.write(framAddr & 0xFF);
		} else {
			Wire.beginTransmission(ret_addr);
			Wire.write(framAddr >> 8);
			Wire.write(framAddr & 0xFF);
		}

		return ret_addr;
	}
};

#endif
