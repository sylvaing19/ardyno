#include "DynamixelMotor.h"

DynamixelDevice::DynamixelDevice(DynamixelInterface &aInterface, DynamixelID aID):
	mInterface(aInterface), mID(aID), mStatusReturnLevel(255)
{
	mStatus = DYN_STATUS_OK;
	if (mID == BROADCAST_ID)
	{
		mStatusReturnLevel = 0;
	}
}

uint8_t DynamixelDevice::statusReturnLevel()
{
	if (mStatusReturnLevel == 255)
	{
		init();
	}
	return mStatusReturnLevel;
}

void DynamixelDevice::statusReturnLevel(uint8_t aSRL)
{
	write(DYN_ADDRESS_SRL, aSRL);
	if (status() == DYN_STATUS_OK)
	{
		mStatusReturnLevel = aSRL;
	}
}

uint16_t DynamixelDevice::model()
{
	uint16_t result;
	read(DYN_ADDRESS_ID, result);
	return result;
}

uint8_t DynamixelDevice::firmware()
{
	uint8_t result;
	read(DYN_ADDRESS_FIRMWARE, result);
	return result;
}

void DynamixelDevice::communicationSpeed(uint32_t aSpeed)
{
	uint8_t value = 2000000 / aSpeed - 1;
	if (value != 0) // forbid 2MBd rate, because it is out of spec, and can be difficult to undo
	{
		write(DYN_ADDRESS_BAUDRATE, value);
	}
}

DynamixelStatus DynamixelDevice::init()
{
	mStatusReturnLevel = 1;
	DynamixelStatus status = ping();
	if (status != DYN_STATUS_OK)
	{
		mStatusReturnLevel = 0;
		return status;
	}
	status = read(DYN_ADDRESS_SRL, mStatusReturnLevel);
	if (status & (DYN_STATUS_COM_ERROR | DYN_STATUS_TIMEOUT))
	{
		mStatusReturnLevel = 0;
	}
	else if (status != DYN_STATUS_OK)
	{
		mStatusReturnLevel = 0;
		return status;
	}
	return DYN_STATUS_OK;
}



DynamixelMotor::DynamixelMotor(DynamixelInterface &aInterface, DynamixelID aId):
	DynamixelDevice(aInterface, aId)
{}


void DynamixelMotor::wheelMode()
{
	jointMode(0,0);
}

void DynamixelMotor::jointMode(uint16_t aCWLimit, uint16_t aCCWLimit)
{
	uint32_t data = (aCWLimit | (uint32_t(aCCWLimit) << 16));
	write(DYN_ADDRESS_CW_LIMIT, data);
}

void DynamixelMotor::enableTorque(bool aTorque)
{
	write(DYN_ADDRESS_ENABLE_TORQUE, uint8_t(aTorque?1:0));
}

DynamixelStatus DynamixelMotor::alarmShutdown(uint8_t aMode)
{
	aMode &= B01111111;
	return write(DYN_ADDRESS_ALARM_SHUTDOWN, aMode);
}

DynamixelStatus DynamixelMotor::speed(uint16_t aSpeed)
{
	return write(DYN_ADDRESS_GOAL_SPEED, aSpeed);
}

DynamixelStatus DynamixelMotor::torqueLimit(uint16_t aTorque)
{
	return write(DYN_ADDRESS_TORQUE_LIMIT, aTorque);
}

DynamixelStatus DynamixelMotor::goalPosition(uint16_t aPosition)
{
	return write(DYN_ADDRESS_GOAL_POSITION, aPosition);
}

DynamixelStatus DynamixelMotor::goalPositionDegree(uint16_t posDeg)
{
	return goalPosition(posDeg * 3.41);
}

void DynamixelMotor::recoverTorque()
{
	uint16_t currentPosition = -1;
	if (!(read(DYN_ADDRESS_CURRENT_POSITION, currentPosition) & DYN_STATUS_COM_ERROR))
	{
		if (currentPosition != (uint16_t)-1)
		{
			write(DYN_ADDRESS_GOAL_POSITION, currentPosition);
		}
	}

	uint16_t torque = 0x3FF;
	if (read(DYN_ADDRESS_MAX_TORQUE, torque) & DYN_STATUS_COM_ERROR)
	{
		write(DYN_ADDRESS_TORQUE_LIMIT, 0x3FF);
	}
	else
	{
		write(DYN_ADDRESS_TORQUE_LIMIT, torque);
	}
	enableTorque();
}

DynamixelStatus DynamixelMotor::resetSecuritySettings()
{
	DynamixelStatus status = 0;
	status |= write(DYN_ADDRESS_TEMP_LIMIT, (uint8_t)85);
	status |= write(DYN_ADDRESS_LOW_VOLTAGE_LIMIT, (uint8_t)60);
	status |= write(DYN_ADDRESS_HIGH_VOLTAGE_LIMIT, (uint8_t)140);
	status |= write(DYN_ADDRESS_MAX_TORQUE, (uint16_t)0x3FF);
	status |= write(DYN_ADDRESS_ALARM_LED, (uint8_t)36);
	status |= write(DYN_ADDRESS_ALARM_SHUTDOWN, (uint8_t)36);
	return status;
}

void DynamixelMotor::setId(uint8_t newId)
{
	write(DYN_ADDRESS_ID, newId);
	mID = newId;
}

void DynamixelMotor::led(uint8_t aState)
{
	write(DYN_ADDRESS_LED, aState);
}

uint16_t DynamixelMotor::currentPosition()
{
	uint16_t currentPosition;
	if (read(DYN_ADDRESS_CURRENT_POSITION, currentPosition) & DYN_STATUS_COM_ERROR)
	{
		return UINT16_MAX;
	}
	else
	{
		return currentPosition;
	}
}

uint16_t DynamixelMotor::currentPositionDegree()
{
	return (uint16_t)((float)currentPosition() / 3.41);
}

