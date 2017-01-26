#ifndef _WRITE_PROTECT_MANAGERS_H_
#define _WRITE_PROTECT_MANAGERS_H_

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "FRAM_defines.h"

// This WP manager allows you to change the pin on-the-fly.
// Although I don't have no idea why someone would like to do that...
class WriteProtect_Dynamic {
public:
	__attribute__ ((always_inline)) inline
	WriteProtect_Dynamic(bool wp, uint8_t pin):
		_wp(wp),
		_pin(pin)
	{
	}

	__attribute__ ((always_inline)) inline
	void init() {
		pinMode(_pin, OUTPUT);
		digitalWrite(_pin, _wp);
	}

	__attribute__ ((always_inline)) inline
	void describe() {
		#if defined(DEBUG_SERIAL_FRAM_MB85RC_I2C)
		DEBUB_SERIAL_FRAM_MB85RC_I2C.print(F("Write protect management: true"));
		DEBUB_SERIAL_FRAM_MB85RC_I2C.print(F("WP pin number "));
		DEBUB_SERIAL_FRAM_MB85RC_I2C.println(_pin, DEC);
		#endif
	}

	__attribute__ ((always_inline)) inline
	uint8_t enable() {
		digitalWrite(_pin, HIGH);
		_wp = true;
		return ERROR_SUCCESS;
	}

	__attribute__ ((always_inline)) inline
	uint8_t disable() {
		digitalWrite(_pin, LOW);
		_wp = true;
		return ERROR_SUCCESS;
	}

	__attribute__ ((always_inline)) inline
	bool status() {
		return _wp;
	}

private:
	bool _wp;
	uint8_t _pin;
};

// This WP manager should be more effective, since there's a very rare chance
// that the user would like to change wp on runtime.
template < uint8_t pin >
class WriteProtect_Static {
public:
	__attribute__ ((always_inline)) inline
	WriteProtect_Static(bool wp, uint8_t):
	 	_wp(wp)
	{ }

	__attribute__ ((always_inline)) inline
	void init() {
		pinMode(pin, OUTPUT);
		digitalWrite(pin, _wp);
	}

	__attribute__ ((always_inline)) inline
	void describe() {
		#if defined(DEBUG_SERIAL_FRAM_MB85RC_I2C)
		DEBUB_SERIAL_FRAM_MB85RC_I2C.print(F("Write protect management: true"));
		DEBUB_SERIAL_FRAM_MB85RC_I2C.print(F("WP pin number "));
		DEBUB_SERIAL_FRAM_MB85RC_I2C.println(pin, DEC);
		#endif
	}

	__attribute__ ((always_inline)) inline
	uint8_t enable() {
		digitalWrite(pin, HIGH);
		_wp = true;
		return ERROR_SUCCESS;
	}

	__attribute__ ((always_inline)) inline
	uint8_t disable() {
		digitalWrite(pin, LOW);
		_wp = false;
		return ERROR_SUCCESS;
	}

	__attribute__ ((always_inline)) inline
	bool status() { return _wp; }

private:
	bool _wp;
};

class WriteProtect_Unmanaged {
public:
	__attribute__ ((always_inline)) inline
	WriteProtect_Unmanaged(bool, uint8_t) { }

	__attribute__ ((always_inline)) inline
	void init() { }

	__attribute__ ((always_inline)) inline
	void describe() {
		#if defined(DEBUG_SERIAL_FRAM_MB85RC_I2C)
		DEBUB_SERIAL_FRAM_MB85RC_I2C.print(F("Write protect management: false"));
		#endif
	}

	__attribute__ ((always_inline)) inline
	uint8_t enable() { return ERROR_NOT_PERMITTED; }

	__attribute__ ((always_inline)) inline
	uint8_t disable() { return ERROR_NOT_PERMITTED; }

	__attribute__ ((always_inline)) inline
	bool status() { return false; }
};

#endif
