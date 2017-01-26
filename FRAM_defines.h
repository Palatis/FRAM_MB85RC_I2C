#ifndef _FRAM_DEFINES_H_
#define _FRAM_DEFINES_H_

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

#endif
