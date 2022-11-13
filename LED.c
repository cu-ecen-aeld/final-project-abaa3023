// author: Abijith Ananda Krishnan
// references: 
// Hardware setup - https://osoyoo.com/driver/pi3_start_learning_kit_lesson_16/pirsensor.c
// WiringPi setup - http://wiringPi.com/download-and-install/
// Pin mapping    - https://projects.drogon.net/raspberry-pi/wiringpi/pins/


#include <wiringPi.h> // wiringPi library
#include <stdio.h>    
#include <stdlib.h>

#define PIRpin 0 // PIR sensor input GPIO17
#define LEDpin 1 // LED pin output GPIO18

int main()
{
		// WiringPiSetup provides the mapping from virtual pin numbers 0 to 16
		// to the real underlying Broadcomm GPIO pin numbers
		if(wiringPiSetup() == -1)
		{
			printf("setup wiringPi failed!");
			exit(1);
		}
		else
		{
			printf("wiringPi setup success");
		}
		
		pinMode(LEDpin, OUTPUT); // LED pin is output
		pinMode(PIRpin, INPUT); // PIR pin is input
		printf("PIN mode set");
		
		while(1)
		{
				if(!(digitalRead(PIRpin))) // digitalRead returns 0 if detetced
				{	
					printf("Detected\n");	
					digitalWrite(LEDpin, HIGH);
					delay(1000);
				}
				else
				{
					printf("Not detected\n");
					digitalWrite(LEDpin, LOW);
					delay(1000);
				}
		}
		return 0;
}
