#include <Servo.h>
//#include <math.h>
#define legcount 4
#define legServocount 3
#define tailServocount 2

Servo legservo[legcount][legServocount];
Servo tailservo[tailServocount];
int legpin[4][3] ={
				{2,3,4},
				{5,6,9},
				{10,11,12},
				{22,24,26},
				};
int tailpin[tailServocount] = {28,30};
int tailhome[tailServocount] = {90,90};
float offset[legcount][legServocount] = {
										//{95,85,90},
										{95,0,90},
										{90,95,100},
										{90,87,90},
										{85,90,90}
										//{85,180,90}
										
};
void setup()
{

	//leg servos
	for(int i= 0; i<legcount; i++)
	{
		for(int j = 0; j<legServocount; j++)
		{
			legservo[i][j].attach(legpin[i][j]);	
		}
		
	}
	
	//tail servos
	for(int i=0; i<tailServocount;i++)
	{
		tailservo[i].attach(tailpin[i]);
	}
}

void loop()
{
	for(int i= 0; i<legcount; i++)
	{
		for(int j = 0; j<legServocount; j++)
		{
			legservo[i][j].write(offset[i][j]);	
		}
		
	}
	for(int i= 0; i<legcount; i++)
	{
	//	tailservo[i].write(tailhome[i]);	
	}
	tailservo[0].write(0);
	tailservo[1].write(180);
	

}

