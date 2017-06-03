#include "DynamixelInterface.h"


DynamixelInterface::DynamixelInterface(HardwareSerial &aStream, uint8_t aDirectionPin, bool aTranferOwnership) :
	mStream(aStream), mDirectionPin(aDirectionPin), mStreamOwner(aTranferOwnership)
{
	if (mDirectionPin != NO_DIR_PORT)
	{
		digitalWrite(mDirectionPin, LOW);
		pinMode(mDirectionPin, OUTPUT);
	}
}

void DynamixelInterface::begin(unsigned long aBaud)
{
	mStream.begin(aBaud);
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) // Teensy 3.0 3.1 3.2 3.5 3.6
	if (&mStream == &Serial1)
	{
		UART0_C1 |= UART_C1_LOOPS | UART_C1_RSRC; // Connect internally RX and TX for half duplex
		CORE_PIN1_CONFIG |= PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin
	}
	else if (&mStream == &Serial2)
	{
		UART1_C1 |= UART_C1_LOOPS | UART_C1_RSRC; // Connect internally RX and TX for half duplex
		CORE_PIN10_CONFIG |= PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin
	}
	else if (&mStream == &Serial3)
	{
		UART2_C1 |= UART_C1_LOOPS | UART_C1_RSRC; // Connect internally RX and TX for half duplex
		CORE_PIN8_CONFIG |= PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin
	}
#if defined(__MK64FX512__) || defined(__MK66FX1M0__) // Teensy 3.5 ou 3.6
	else if (&mStream == &Serial4)
	{
		UART3_C1 |= UART_C1_LOOPS | UART_C1_RSRC; // Connect internally RX and TX for half duplex
		CORE_PIN32_CONFIG |= PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin
	}
	else if (&mStream == &Serial5)
	{
		UART4_C1 |= UART_C1_LOOPS | UART_C1_RSRC; // Connect internally RX and TX for half duplex
		CORE_PIN33_CONFIG |= PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin
	}
	else if (&mStream == &Serial6)
	{
		UART5_C1 |= UART_C1_LOOPS | UART_C1_RSRC; // Connect internally RX and TX for half duplex
		CORE_PIN48_CONFIG |= PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin
	}
#endif
#endif
	mStream.setTimeout(COM_TIMEOUT); //warning : response delay seems much higher than expected for some operation (eg writing eeprom)
	readMode();
}

void DynamixelInterface::sendPacket(const DynamixelPacket &aPacket)
{
	writeMode();

	mStream.write(0xFF);
	mStream.write(0xFF);
	mStream.write(aPacket.mID);
	mStream.write(aPacket.mLength);
	mStream.write(aPacket.mInstruction);
	uint8_t n = 0;
	if (aPacket.mAddress != 255)
	{
		mStream.write(aPacket.mAddress);
		++n;
	}
	if (aPacket.mDataLength != 255)
	{
		mStream.write(aPacket.mDataLength);
		++n;
	}
	if (aPacket.mLength>(2 + n))
	{
		if (aPacket.mIDListSize == 0)
		{
			mStream.write(aPacket.mData, aPacket.mLength - 2 - n);
		}
		else
		{
			uint8_t *ptr = aPacket.mData;
			for (uint8_t i = 0; i<aPacket.mIDListSize; ++i)
			{
				mStream.write(aPacket.mIDList[i]);
				mStream.write(ptr, aPacket.mDataLength);
				ptr += aPacket.mDataLength;
			}
		}
	}
	mStream.write(aPacket.mCheckSum);
	mStream.flush();
	mStream.clear();
	readMode();
}

void DynamixelInterface::receivePacket(DynamixelPacket &aPacket, uint8_t answerSize)
{
	static uint8_t buffer[3];
	aPacket.mIDListSize = 0;
	aPacket.mAddress = 255;
	aPacket.mDataLength = 255;
	if (mStream.readBytes(buffer, 2)<2)
	{
		aPacket.mStatus = DYN_STATUS_COM_ERROR | DYN_STATUS_TIMEOUT;
		return;
	}
	if (buffer[0] != 255 || buffer[1] != 255)
	{
		aPacket.mStatus = DYN_STATUS_COM_ERROR;
		return;
	}
	if (mStream.readBytes(buffer, 3)<3)
	{
		aPacket.mStatus = DYN_STATUS_COM_ERROR | DYN_STATUS_TIMEOUT;
		return;
	}
	if (aPacket.mID != buffer[0])
	{
		aPacket.mStatus = DYN_STATUS_COM_ERROR;
		return;
	}
	aPacket.mLength = buffer[1];
	if (aPacket.mLength > 2 && aPacket.mLength - 2 != answerSize)
	{
		aPacket.mStatus = DYN_STATUS_COM_ERROR;
		return;
	}
	aPacket.mStatus = buffer[2];
	if (aPacket.mLength>2 && (int)mStream.readBytes(reinterpret_cast<char*>(aPacket.mData), aPacket.mLength - 2)<(aPacket.mLength - 2))
	{
		aPacket.mStatus = DYN_STATUS_COM_ERROR | DYN_STATUS_TIMEOUT;
		return;
	}
	if (mStream.readBytes(reinterpret_cast<char*>(&(aPacket.mCheckSum)), 1)<1)
	{
		aPacket.mStatus = DYN_STATUS_COM_ERROR | DYN_STATUS_TIMEOUT;
		return;
	}
	if (aPacket.checkSum() != aPacket.mCheckSum)
	{
		aPacket.mStatus = DYN_STATUS_COM_ERROR | DYN_STATUS_CHECKSUM_ERROR;
	}
}

void DynamixelInterface::transaction(bool aExpectStatus, uint8_t answerSize)
{
	sendPacket(mPacket);
	if(aExpectStatus)
	{
		receivePacket(mPacket, answerSize);
	}
	else
	{
		mPacket.mStatus=DYN_STATUS_OK;
	}
}

DynamixelStatus DynamixelInterface::read(uint8_t aID, uint8_t aAddress, uint8_t aSize, uint8_t *aPtr, uint8_t aStatusReturnLevel)
{
	mPacket=DynamixelPacket(aID, DYN_READ, 4, aPtr, aAddress, aSize);
	transaction(aStatusReturnLevel>0 && aID!=BROADCAST_ID, aSize);
	return mPacket.mStatus;
}

DynamixelStatus DynamixelInterface::write(uint8_t aID, uint8_t aAddress, uint8_t aSize, const uint8_t *aPtr, uint8_t aStatusReturnLevel)
{
	mPacket=DynamixelPacket(aID, DYN_WRITE, aSize+3, aPtr, aAddress);
	transaction(aStatusReturnLevel>1 && aID!=BROADCAST_ID);
	return mPacket.mStatus;
}

DynamixelStatus DynamixelInterface::regWrite(uint8_t aID, uint8_t aAddress, uint8_t aSize, const uint8_t *aPtr, uint8_t aStatusReturnLevel)
{
	mPacket=DynamixelPacket(aID, DYN_REG_WRITE, aSize+3, aPtr, aAddress);
	transaction(aStatusReturnLevel>1 && aID!=BROADCAST_ID);
	return mPacket.mStatus;
}

DynamixelStatus DynamixelInterface::syncWrite(uint8_t nID, const uint8_t *aID, uint8_t aAddress, uint8_t aSize, const uint8_t *aPtr, uint8_t aStatusReturnLevel)
{
	mPacket=DynamixelPacket(BROADCAST_ID, DYN_SYNC_WRITE, (aSize+1)*nID+4, aPtr, aAddress, aSize, nID, aID);
	transaction(false);
	return mPacket.mStatus;
}

DynamixelStatus DynamixelInterface::ping(uint8_t aID)
{
	mPacket=DynamixelPacket(aID, DYN_PING, 2, NULL);
	transaction(true);
	return mPacket.mStatus;
}

DynamixelStatus DynamixelInterface::action(uint8_t aID, uint8_t aStatusReturnLevel)
{
	mPacket=DynamixelPacket(aID, DYN_ACTION, 2, NULL);
	transaction(aStatusReturnLevel>1 && aID!=BROADCAST_ID);
	return mPacket.mStatus;
}

DynamixelStatus DynamixelInterface::reset(uint8_t aID, uint8_t aStatusReturnLevel)
{
	mPacket=DynamixelPacket(aID, DYN_RESET, 2, NULL);
	transaction(aStatusReturnLevel>1 && aID!=BROADCAST_ID);
	return mPacket.mStatus;
}

void DynamixelInterface::readMode()
{
	if (mDirectionPin != NO_DIR_PORT)
	{
		digitalWrite(mDirectionPin, LOW);
	}
	else
	{
		setReadMode(mStream);
	}
}

void DynamixelInterface::writeMode()
{
	if (mDirectionPin != NO_DIR_PORT)
	{
		digitalWrite(mDirectionPin, HIGH);
	}
	else
	{
		setWriteMode(mStream);
	}
}

void DynamixelInterface::setReadMode(HardwareSerial & aSerial)
{
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) // Teensy 3.0 3.1 3.2 3.5 3.6
	if (&mStream == &Serial1)
	{
		UART0_C3 &= ~UART_C3_TXDIR;
	}
	else if (&mStream == &Serial2)
	{
		UART1_C3 &= ~UART_C3_TXDIR;
	}
	else if (&mStream == &Serial3)
	{
		UART2_C3 &= ~UART_C3_TXDIR;
	}
#if defined(__MK64FX512__) || defined(__MK66FX1M0__) // Teensy 3.5 ou 3.6
	else if (&mStream == &Serial4)
	{
		UART3_C3 &= ~UART_C3_TXDIR;
	}
	else if (&mStream == &Serial5)
	{
		UART4_C3 &= ~UART_C3_TXDIR;
	}
	else if (&mStream == &Serial6)
	{
		UART5_C3 &= ~UART_C3_TXDIR;
	}
#endif
#endif
}

void DynamixelInterface::setWriteMode(HardwareSerial & aSerial)
{
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) // Teensy 3.0 3.1 3.2 3.5 3.6
	if (&mStream == &Serial1)
	{
		UART0_C3 |= UART_C3_TXDIR;
	}
	else if (&mStream == &Serial2)
	{
		UART1_C3 |= UART_C3_TXDIR;
	}
	else if (&mStream == &Serial3)
	{
		UART2_C3 |= UART_C3_TXDIR;
	}
#if defined(__MK64FX512__) || defined(__MK66FX1M0__) // Teensy 3.5 ou 3.6
	else if (&mStream == &Serial4)
	{
		UART3_C3 |= UART_C3_TXDIR;
	}
	else if (&mStream == &Serial5)
	{
		UART4_C3 |= UART_C3_TXDIR;
	}
	else if (&mStream == &Serial6)
	{
		UART5_C3 |= UART_C3_TXDIR;
	}
#endif
#endif
}
