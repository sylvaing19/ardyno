/**
 * \file DynamixelMotor.h
 * \brief Define classe for dynamixel motor
*/

#ifndef DYNAMIXEL_MOTOR_H
#define DYNAMIXEL_MOTOR_H

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "DynamixelInterface.h"

class DynamixelDevice
{
public:
	
	DynamixelDevice(DynamixelInterface &aInterface, DynamixelID aId);
	
	DynamixelStatus init();
	
	DynamixelStatus status()
	{
		return mStatus;
	}
	
	DynamixelID id()
	{
		return mID;
	}
	
	uint8_t statusReturnLevel();
	void statusReturnLevel(uint8_t aSRL);
	
	uint16_t model();
	uint8_t firmware();
	
	void communicationSpeed(uint32_t aSpeed);
	
	
	template<class T>
	inline DynamixelStatus read(uint8_t aAddress, T& aData)
	{
		return mStatus=mInterface.read<T>(mID, aAddress, aData, mStatusReturnLevel);
	}
	
	template<class T>
	inline DynamixelStatus write(uint8_t aAddress, const T& aData)
	{
		return mStatus=mInterface.write<T>(mID, aAddress, aData, mStatusReturnLevel);
	}
	
	template<class T>
	inline DynamixelStatus regWrite(uint8_t aAddress, const T& aData)
	{
		return mStatus=mInterface.regWrite<T>(mID, aAddress, aData, mStatusReturnLevel);
	}
	
	DynamixelStatus ping()
	{
		return mStatus=mInterface.ping(mID);
	}
	
	DynamixelStatus action()
	{
		return mStatus=mInterface.action(mID, mStatusReturnLevel);
	}
	
	DynamixelStatus reset()
	{
		return mStatus=mInterface.reset(mID, mStatusReturnLevel);
	}
	
private:
	
	DynamixelInterface &mInterface;
	uint8_t mStatusReturnLevel;
	DynamixelStatus mStatus;

protected:
	DynamixelID mID;
};


class DynamixelMotor:public DynamixelDevice
{
public:
	
	DynamixelMotor(DynamixelInterface &aInterface, DynamixelID aId);
	
	void wheelMode();
	void jointMode(uint16_t aCWLimit=0, uint16_t aCCWLimit=0x3FF);
	
	void enableTorque(bool aTorque=true);
	DynamixelStatus alarmShutdown(uint8_t aMode = 0x04);
	DynamixelStatus speed(uint16_t aSpeed);
	DynamixelStatus torqueLimit(uint16_t aTorque);
	DynamixelStatus goalPosition(uint16_t aPosition);
	DynamixelStatus goalPositionDegree(uint16_t posDeg);

	void setId(uint8_t newId);
	
	void led(uint8_t aState);

	uint16_t currentPosition();
	uint16_t currentPositionDegree();
};

#endif
