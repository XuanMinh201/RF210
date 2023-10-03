/*
	This file is part of the KX023-1025-IMU library.
	Copyright (c) 2021 Good Solutions Sweden AB. All rights reserved.

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
	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef __KIONIX_KX023_h__
#define __KIONIX_KX023_h__

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "kx023_reg_map.h"

#define KX023_DEFAULT_ADDR 0x1E

typedef enum
{
	RANGE_2G = 0,
	RANGE_4G = 1,
	RANGE_8G = 2,
} AccelerationRange_t;

typedef enum
{
	DATARATE_12_5HZ = 0,
	DATARATE_25HZ = 1,
	DATARATE_50HZ = 2,
	DATARATE_100HZ = 3,
	DATARATE_200HZ = 4,
	DATARATE_400HZ = 5,
	DATARATE_800HZ = 6,
	DATARATE_1600HZ = 7,
	DATARATE_0_781HZ = 8,
	DATARATE_1_563HZ = 9,
	DATARATE_3_125HZ = 10,
	DATARATE_6_25HZ = 11,
} KX023_OutputDatarate_t;

typedef enum
{
	LOWPOWER = 0x00,
	HIGHPOWER = 0x40,
} KX023_PowerMode_t;

class KX023
{
public:
	KX023(TwoWire &wire = Wire, uint8_t slaveAddress = KX023_DEFAULT_ADDR);
	KX023(int csPin, SPIClass &spi = SPI);
	virtual ~KX023();

	int begin(void);
	void end(void);

	uint32_t getInterruptStatus(void);
	void clearInterrupt(void);

	// Accelerometer
	int configContinuousReading(KX023_PowerMode_t powerMode = LOWPOWER, AccelerationRange_t accelerationRange = RANGE_2G, KX023_OutputDatarate_t outputDataRate = DATARATE_50HZ); // powerMode 0 = LowPower, 1 = HighPower; accelerationRange 0 = +/-2g, 1 = +/-4g, 2 = +/-8g; outputDataRate = 0(12.5Hz),1(25Hz),2(50Hz),3(100Hz),4(200Hz),5(400Hz),6(800Hz),7(1600Hz),8(0.781Hz),9(1.563Hz),10(3.125Hz),11(6.25Hz)
	virtual int readAcceleration(float &x, float &y, float &z); // Results are in G (earth gravity).
	virtual int readAcceleration(float *x, float *y, float *z); // Results are in G (earth gravity).

	// Single/Double Tap detection
	int configSingleTapDetection(void);

private:
	int readRegister(uint8_t address);
	int readRegisters(uint8_t address, uint8_t *data, size_t length);
	int writeRegister(uint8_t address, uint8_t value);

	void setStandbyMode(void);
	void setOperatingMode(void);

private:
	TwoWire *_wire;
	SPIClass *_spi;
	uint8_t _slaveAddress;
	int _csPin;
	AccelerationRange_t _accelerationRange;

	SPISettings _spiSettings;
};

#endif /* __KIONIX_KX023_h__ */
