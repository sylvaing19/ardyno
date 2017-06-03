#ifndef DYNAMIXEL_INTERFACE_H
#define DYNAMIXEL_INTERFACE_H

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Dynamixel.h"

#define COM_TIMEOUT 2 // unit: ms [WARNING: standard timeout is 50]

/**
 * \class  DynamixelInterface
 * \brief Represent a dynamixel bus
*/
class DynamixelInterface
{
public:
	/**
	* \brief Constructor
	* \param[in] aStreamController : stream controller that abstract real stream
	* \param[in] aDirectionPin : direction pin, use NO_DIR_PORT if you do not one (default)
	* \param[in] aTranferOwnership : if true, the stream will be deleted in the destructor
	*/
	DynamixelInterface(HardwareSerial &aStream, uint8_t aDirectionPin = NO_DIR_PORT, bool aTranferOwnership = false);

	/**
	* \brief Start interface
	* \param[in] aBaud : Baudrate
	*
	* Start the interface with desired baudrate, call once before using the interface
	*/
	void begin(unsigned long aBaud);

	/**
	* \brief Send a packet on bus
	* \param[in] aPacket : Packet to send
	*
	* The function wait for the packet to be completly sent (using Stream.flush)
	*/
	void sendPacket(const DynamixelPacket &aPacket);

	/**
	* \brief Receive a packet on bus
	* \param[out] aPacket : Received packet. mData field must be previously allocated
	*
	* The function wait for a new packet on the bus. Timeout depends of timeout of the underlying stream.
	* Return error code in case of communication error (timeout, checksum error, ...)
	*/
	void receivePacket(DynamixelPacket &aPacket, uint8_t answerSize = 0);
	
	/**
	* \brief Send the current packet, wait for the answer and update the mStatus acordingly
	* \param[in] aExpectStatus : should be "aStatusReturnLevel>0 && aID!=BROADCAST_ID"
	*/
	void transaction(bool aExpectStatus, uint8_t memorySize = 0);
	
	//sizeof(T) must be lower than DYN_INTERNAL_BUFFER_SIZE, and in any case lower than 256
	template<class T>
	inline DynamixelStatus read(uint8_t aID, uint8_t aAddress, T& aData, uint8_t aStatusReturnLevel=2);
	template<class T>
	inline DynamixelStatus write(uint8_t aID, uint8_t aAddress, const T& aData, uint8_t aStatusReturnLevel=2);
	template<class T>
	inline DynamixelStatus regWrite(uint8_t aID, uint8_t aAddress, const T& aData, uint8_t aStatusReturnLevel=2);
	
	DynamixelStatus read(uint8_t aID, uint8_t aAddress, uint8_t aSize, uint8_t *aPtr, uint8_t aStatusReturnLevel=2);
	DynamixelStatus write(uint8_t aID, uint8_t aAddress, uint8_t aSize, const uint8_t *aPtr, uint8_t aStatusReturnLevel=2);
	DynamixelStatus regWrite(uint8_t aID, uint8_t aAddress, uint8_t aSize, const uint8_t *aPtr, uint8_t aStatusReturnLevel=2);
	DynamixelStatus syncWrite(uint8_t nID, const uint8_t *aID, uint8_t aAddress, uint8_t aSize, const uint8_t *aPtr, uint8_t aStatusReturnLevel=2);
	
	DynamixelStatus ping(uint8_t aID);
	DynamixelStatus action(uint8_t aID=BROADCAST_ID, uint8_t aStatusReturnLevel=2);
	DynamixelStatus reset(uint8_t aID, uint8_t aStatusReturnLevel=2);
	
private:

	/** \brief Switch stream to read (receive)) mode */
	void readMode();

	/** \brief Switch stream to write (send) mode */
	void writeMode();

	void setReadMode(HardwareSerial &);
	void setWriteMode(HardwareSerial &);
	
	DynamixelPacket mPacket;
	static const uint8_t NO_DIR_PORT = 255;
	HardwareSerial &mStream;
	const uint8_t mDirectionPin;
	bool mStreamOwner;
};


template<class T>
DynamixelStatus DynamixelInterface::read(uint8_t aID, uint8_t aAddress, T& aData, uint8_t aStatusReturnLevel)
{
	return read(aID, aAddress, uint8_t(sizeof(T)), (uint8_t*)&aData, aStatusReturnLevel);
}

template<class T>
DynamixelStatus DynamixelInterface::write(uint8_t aID, uint8_t aAddress, const T& aData, uint8_t aStatusReturnLevel)
{
	return write(aID, aAddress, uint8_t(sizeof(T)), (const uint8_t*)&aData, aStatusReturnLevel);
}

template<class T>
DynamixelStatus DynamixelInterface::regWrite(uint8_t aID, uint8_t aAddress, const T& aData, uint8_t aStatusReturnLevel)
{
	return regWrite(aID, aAddress, uint8_t(sizeof(T)), (const uint8_t*)aData, aStatusReturnLevel);
}


#endif
