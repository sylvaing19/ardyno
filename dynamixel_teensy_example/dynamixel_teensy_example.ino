#include "Dynamixel.h"
#include "DynamixelInterface.h"
#include "DynamixelMotor.h"

void setup()
{
	Serial.println("Dynamixel Teensy example - begin");
}

void loop()
{
	DynamixelInterface dInterface(Serial1);	// Stream
	dInterface.begin(9600, 50); // baudrate, timeout
	DynamixelMotor motor(dInterface, 0);	// Interface , ID

	motor.init(); // This will get the returnStatusLevel of the servo
	Serial.printf("Status return level = %u\n", motor.statusReturnLevel());
	motor.jointMode(); // Set the angular limits of the servo. Set to [min, max] by default
	motor.enableTorque();

	while (true)
	{
		motor.goalPositionDegree(140);
		delay(1000);
		motor.goalPositionDegree(160);
		delay(1000);
	}

}
