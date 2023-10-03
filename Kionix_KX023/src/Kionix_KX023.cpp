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

#include "Kionix_KX023.h"

KX023::KX023(TwoWire &wire, uint8_t slaveAddress) : _wire(&wire),
																										_spi(NULL),
																										_slaveAddress(slaveAddress)
{
}

KX023::KX023(int csPin, SPIClass &spi) : _wire(NULL),
																				 _spi(&spi),
																				 _csPin(csPin),
																				 _spiSettings(10E6, MSBFIRST, SPI_MODE0)
{
}

KX023::~KX023()
{
}

int KX023::begin(void)
{
	if (_spi != NULL)
	{
		pinMode(_csPin, OUTPUT);
		digitalWrite(_csPin, HIGH);
		_spi->begin();
	}
	else
	{
		_wire->beginTransmission(_slaveAddress);
		if (_wire->endTransmission() == 0)
		{
			return 0;
		}

		_wire->begin();
	}

	// Check WHO_AM_I register
	if (readRegister(KX023_Who_AM_I_REG) != 0x15)
	{
		end();
		return 0;
	}

	// Check COTR register
	// uint8_t cotr_reg_val = (uint8_t)readRegister(KX023_COTR_REG);
	// if (!((cotr_reg_val == 0x55) || (cotr_reg_val == 0xAA)))
	// {
	// 	end();
	// 	return 0;
	// }

	return 1;
}

void KX023::end(void)
{
	if (_spi != NULL)
	{
		_spi->end();
		digitalWrite(_csPin, LOW);
		pinMode(_csPin, INPUT);
	}
	else
	{
		writeRegister(KX023_CNTL1_REG, 0x18);
		_wire->endTransmission();
	}
}

uint32_t KX023::getInterruptStatus(void)
{
	uint8_t reg[3];
	readRegisters(KX023_INS1_REG, reg, 3);

	return ((reg[0] << 0) | (reg[1] << 8) | (reg[2] << 16));
}

void KX023::clearInterrupt(void)
{
	readRegister(KX023_INT_REL_REG);
}

int KX023::configContinuousReading(KX023_PowerMode_t powerMode, AccelerationRange_t accelerationRange, KX023_OutputDatarate_t outputDataRate)
{
	this->_accelerationRange = accelerationRange;

	if (powerMode == LOWPOWER && (outputDataRate == DATARATE_400HZ || outputDataRate == DATARATE_400HZ || outputDataRate == DATARATE_400HZ))
	{
		// Data rate not supported in low power mode
		return 0;
	}

	writeRegister(KX023_ODCNTL_REG, outputDataRate);

	int activateValue = 0x80 | (uint8_t)powerMode;

	if (this->_accelerationRange == RANGE_4G)
	{
		activateValue = activateValue | 0x08;
	}
	if (this->_accelerationRange == RANGE_8G)
	{
		activateValue = activateValue | 0x10;
	}

	writeRegister(KX023_CNTL1_REG, activateValue);

	return 1;
}

int KX023::readAcceleration(float &x, float &y, float &z)
{
	int16_t data[3];

	if (!readRegisters(KX023_XOUTL_REG, (uint8_t *)data, sizeof(data)))
	{
		x = NAN;
		y = NAN;
		z = NAN;

		return 0;
	}
	if (this->_accelerationRange == RANGE_2G)
	{
		x = data[0] * 2.0 / 32768.0;
		y = data[1] * 2.0 / 32768.0;
		z = data[2] * 2.0 / 32768.0;
	}
	if (this->_accelerationRange == RANGE_4G)
	{
		x = data[0] * 4.0 / 32768.0;
		y = data[1] * 4.0 / 32768.0;
		z = data[2] * 4.0 / 32768.0;
	}
	if (this->_accelerationRange == RANGE_8G)
	{
		x = data[0] * 8.0 / 32768.0;
		y = data[1] * 8.0 / 32768.0;
		z = data[2] * 8.0 / 32768.0;
	}

	return 1;
}

int KX023::readAcceleration(float *x, float *y, float *z)
{
	int16_t data[3];

	if (!readRegisters(KX023_XOUTL_REG, (uint8_t *)data, sizeof(data)))
	{
		*x = NAN;
		*y = NAN;
		*z = NAN;

		return 0;
	}
	if (this->_accelerationRange == RANGE_2G)
	{
		*x = data[0] * 2.0 / 32768.0;
		*y = data[1] * 2.0 / 32768.0;
		*z = data[2] * 2.0 / 32768.0;
	}
	if (this->_accelerationRange == RANGE_4G)
	{
		*x = data[0] * 4.0 / 32768.0;
		*y = data[1] * 4.0 / 32768.0;
		*z = data[2] * 4.0 / 32768.0;
	}
	if (this->_accelerationRange == RANGE_8G)
	{
		*x = data[0] * 8.0 / 32768.0;
		*y = data[1] * 8.0 / 32768.0;
		*z = data[2] * 8.0 / 32768.0;
	}

	return 1;
}

int KX023::configSingleTapDetection(void)
{
	/* Set STANBY Mode */
	setStandbyMode();

	/* Tap/double tap setting -> OTDT in CNTL3 */
	uint8_t cntl3_reg_value = readRegister(KX023_CNTL3_REG);
	cntl3_reg_value &= 0b11000111; // Clear OTDT2:0
	// cntl3_reg_value |= (0x00 << 3); // 50 Hz
	// cntl3_reg_value |= (0x01 << 3); // 100 Hz
	// cntl3_reg_value |= (0x02 << 3); // 200 Hz
	cntl3_reg_value |= (0x03 << 3); // 400 Hz (Default)
	// cntl3_reg_value |= (0x04 << 3); // 12.5 Hz
	// cntl3_reg_value |= (0x05 << 3); // 25 Hz
	// cntl3_reg_value |= (0x06 << 3); // 800 Hz
	// cntl3_reg_value |= (0x07 << 3); // 1600 Hz
	writeRegister(KX023_CNTL3_REG, cntl3_reg_value);

	/* Settings for the physical interrupt pin INT1 */
	uint8_t inc1_reg_value = readRegister(KX023_INC1_REG);
	inc1_reg_value |= 0b00111000; // Set IEN1, IEA1, IEL1
	writeRegister(KX023_INC1_REG, inc1_reg_value);

	/* Tap/Double Tap interrupt reported on physical interrupt pin INT1 */
	uint8_t inc4_reg_value = readRegister(KX023_INC4_REG);
	inc4_reg_value |= (0x01 << 2); // Set TDTI1
	writeRegister(KX023_INC4_REG, inc4_reg_value);

	/* Enable detecting of tap events -> TDTE in CNTL1 */
	uint8_t cntl1_reg_value = readRegister(KX023_CNTL1_REG);
	cntl1_reg_value |= (0x01 << 2); // Set TDTE
	writeRegister(KX023_CNTL1_REG, cntl1_reg_value);

	/* Set OPERATING Mode */
	setOperatingMode();
}

int KX023::readRegister(uint8_t address)
{
	uint8_t value;

	if (readRegisters(address, &value, sizeof(value)) != 1)
	{
		return -1;
	}

	return value;
}

int KX023::readRegisters(uint8_t address, uint8_t *data, size_t length)
{
	if (_spi != NULL)
	{
		_spi->beginTransaction(_spiSettings);
		digitalWrite(_csPin, LOW);
		_spi->transfer(0x80 | address);
		_spi->transfer(data, length);
		digitalWrite(_csPin, HIGH);
		_spi->endTransaction();
	}
	else
	{
		_wire->beginTransmission(_slaveAddress);
		_wire->write(address);

		if (_wire->endTransmission(false) != 0)
		{
			return -1;
		}

		if (_wire->requestFrom(_slaveAddress, length) != length)
		{
			return 0;
		}

		for (size_t i = 0; i < length; i++)
		{
			*data++ = _wire->read();
		}
	}
	return 1;
}

int KX023::writeRegister(uint8_t address, uint8_t value)
{
	if (_spi != NULL)
	{
		_spi->beginTransaction(_spiSettings);
		digitalWrite(_csPin, LOW);
		_spi->transfer(address);
		_spi->transfer(value);
		digitalWrite(_csPin, HIGH);
		_spi->endTransaction();
	}
	else
	{
		_wire->beginTransmission(_slaveAddress);
		_wire->write(address);
		_wire->write(value);
		if (_wire->endTransmission() != 0)
		{
			return 0;
		}
	}
	return 1;
}

void KX023::setStandbyMode(void)
{
	uint8_t cntl1_reg_value = readRegister(KX023_CNTL1_REG);
	cntl1_reg_value &= ~0x80; // Clear PC
	writeRegister(KX023_CNTL1_REG, cntl1_reg_value);
}

void KX023::setOperatingMode(void)
{
	uint8_t cntl1_reg_value = readRegister(KX023_CNTL1_REG);
	cntl1_reg_value |= 0x80; // Set PC
	writeRegister(KX023_CNTL1_REG, cntl1_reg_value);
}
