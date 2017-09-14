// Pitch Back :  lowering the rear end of the robot


// JOINT ANGLES are in degrees
// this code uses inverse kinematics to first derive the angles, then store them
// for further use.
// it also incorporates SINUSOIDAL delays in the tail movement...
// to reduce the  jerk


#include <Servo.h>

#define totalServocount 1	//total numbers of motor in the robot
#define legServocount 3 	//number of motors in a leg
#define tailServocount 2 	//number of motors in a tail
#define legcount 4 			//total number of legs
#define step_iter 18 		//number of iterations in a step :  standard
#define max_step_count  8 	// max number of steps in a gait : standard

// All dimensions are in m and degrees
//Robot's properties
double clen = 0.115;
double tlen = 0.065;
double abd_len = 0.045; 	//hip abduction joint length
double chassis_len = 0.24; 	//length of the chassis
double height = 0.178 + abd_len;//0.16 + abd_len;
double step = 0.08;
double back_step = 0.04; 	// the position of foot behinfd shoulder in standing
double step_height = 0.02;
double spread = 0;//0.04;  		// Spreading of legs: +ve implies towards outside, -ve implies towards inside
double pitch = 8; 			// the anglle of pitching backwards in DEGREES
int iter_delay = 3;    		// Delay after completion of iteration of all servos
int servoloop_delay = 5; 	// delay after changing the state of the servo
int servo_delay_gain = 2 ; // proptional gain for finding the delay depending on change in state
int tail_iter_delay = 20;
int tail_delay = 50;
int tail_amp = 90;
int tail_off = 25; 			//offset to the tail: NOTE: It is always added to the angle

//using the array approach for defining properties of the servos
//leg servos
int legpin[4][3] =
{
	{2, 3, 4},
	{5, 6, 9},
	{10, 11, 12},
	{22, 24, 26},
};
int leghome[4][3] =
{
	{0, 0, 0},
	{0, 0, 0},
	{0, 0, 0},
	{0, 0, 0}
};

//tail servos
int tailpin[tailServocount] = {28, 30};
int tailhome[tailServocount] = {90, 90};

//Defining servo objects
Servo legservo[legcount][legServocount];
Servo tailservo[tailServocount];

//offset of joint angles
float offset[legcount][legServocount] = {
										{90,85,93},
										{95,93,95},
										{90,87,90},
										{90,90,95}};
										
//defining the variables used in gait execution
//arrays for storing angles
float leg0[legServocount][step_iter*max_step_count];
float leg1[legServocount][step_iter*max_step_count];
float leg2[legServocount][step_iter*max_step_count];
float leg3[legServocount][step_iter*max_step_count];

int step_count = 2; 	// step count for crawl gait 1.2
double x = 0;	    	// side coordinate of the leg
double z = height; 		// veritcle coordinates of the foot
double y = 0; 			// forward coordinate of the foot
double curr_pitch = 0; 	// current pitch 
int loop_iter = 0;
float hip_abd = 0;
//******************THE SETUP FUNCTION********************
void setup()
{
	//Setting up the motors

	for(int i = 0; i < legcount; i++)
	{
		for(int j = 0; j < legServocount; j++)
		{
			legservo[i][j].attach(legpin[i][j]);
		}
	}
	for(int i = 0; i < tailServocount; i++)
	{
		tailservo[i].attach(tailpin[i]);
	}
	//serial monitor for debugging
	Serial.begin(9600);

}

//**************THE LOOP:  CONTAINS THE GAIT*******************
void loop()
{
	if(loop_iter == 0)
	{

		//Generating Angles for pitch back
		// step_iter = 18 : standard
		// step_count = 2
		// ptich back : Angle : 8 deg
		// for leg 0
		for (int i = 0; i < (step_iter * step_count) + 1 ; i++)
		{
			
			if(i < (step_iter * 1) + 1) //step 1
			{
				 curr_pitch = i*pitch/step_iter;
			}
			else if (i > (step_iter * 1) && i < (step_iter * 2) + 1) //step 2
			{
				 curr_pitch = ((step_iter*2) - i)*pitch/step_iter;				 
			}
			
			x = spread;
			y = -1*height*sin(curr_pitch*M_PI/180);
			z = height * cos(curr_pitch*M_PI/180) ;

			//finding the angles
			double psi = atan2(x, z);
			double z1 = (z / cos(psi)) - abd_len;
			double cosphi = ((z1 * z1 + y * y - tlen * tlen - clen * clen) / (2 * clen * tlen));
			double phi = atan2(sqrt((1 - cosphi * cosphi)), cosphi);
			double k2 = clen * sin(phi);
			double k1 = tlen + clen * cos(phi);
			double alpha = atan2(k2, k1);
			double theta  = atan2(y, z1) - alpha;

			leg0[0][i] = lrint((psi) * 180 / M_PI);
			leg0[1][i] = lrint((-theta) * 180 / M_PI);
			leg0[2][i] = lrint((phi ) * 180 / M_PI);
			Serial.println(leg0[1][i]);
		}

		// for leg 1
		for (int i = 0; i < (step_iter * step_count) + 1 ; i++)
		{
			
			if(i < (step_iter * 1) + 1) //step 1
			{
				 curr_pitch = i*pitch/step_iter;
			}
			else if (i > (step_iter * 1) && i < (step_iter * 2) + 1) //step 2
			{
				 curr_pitch = ((step_iter*2) - i)*pitch/step_iter;				 
			}
			
			x = spread;
			y = chassis_len * (1 - cos(curr_pitch*M_PI/180));
			z = height - chassis_len * sin(curr_pitch*M_PI/180) ;

			//finding the angles
			double psi = atan2(x, z);
			double z1 = (z / cos(psi)) - abd_len;
			double cosphi = ((z1 * z1 + y * y - tlen * tlen - clen * clen) / (2 * clen * tlen));
			double phi = atan2(sqrt((1 - cosphi * cosphi)), cosphi);
			double k2 = clen * sin(phi);
			double k1 = tlen + clen * cos(phi);
			double alpha = atan2(k2, k1);
			double theta  = atan2(y, z1) - alpha;
			leg1[0][i] = lrint((-psi) * 180 / M_PI);
			leg1[1][i] = lrint(theta * 180 / M_PI);
			leg1[2][i] = lrint((-phi ) * 180 / M_PI);

		}

		// for leg 2
		for (int i = 0; i < (step_iter * step_count) + 1 ; i++)
		{
			
			if(i < (step_iter * 1) + 1) //step 1
			{
				 curr_pitch = i*pitch/step_iter;
			}
			else if (i > (step_iter * 1) && i < (step_iter * 2) + 1) //step 2
			{
				 curr_pitch = ((step_iter*2) - i)*pitch/step_iter;				 
			}
			
			x = spread;
			y = chassis_len * (1 - cos(curr_pitch*M_PI/180));
			z = height - chassis_len * sin(curr_pitch*M_PI/180) ;

			//finding the angles
			double psi = atan2(x, z);
			double z1 = (z / cos(psi)) - abd_len;
			double cosphi = ((z1 * z1 + y * y - tlen * tlen - clen * clen) / (2 * clen * tlen));
			double phi = atan2(sqrt((1 - cosphi * cosphi)), cosphi);
			double k2 = clen * sin(phi);
			double k1 = tlen + clen * cos(phi);
			double alpha = atan2(k2, k1);
			double theta  = atan2(y, z1) - alpha;
			leg2[0][i] = lrint((psi) * 180 / M_PI);
			leg2[1][i] = lrint((-theta) * 180 / M_PI);
			leg2[2][i] = lrint((phi) * 180 / M_PI);

		}

		// for leg 3
		for (int i = 0; i < (step_iter * step_count) + 1 ; i++)
		{
			
			if(i < (step_iter * 1) + 1) //step 1
			{
				 curr_pitch = i*pitch/step_iter;
			}
			else if (i > (step_iter * 1) && i < (step_iter * 2) + 1) //step 2
			{
				 curr_pitch = ((step_iter*2) - i)*pitch/step_iter;				 
			}
			
			x = spread;
			y = -1*height*sin(curr_pitch*M_PI/180);
			z = height * cos(curr_pitch*M_PI/180) ;

			//finding the angles
			double psi = atan2(x, z);
			double z1 = (z / cos(psi)) - abd_len;
			double cosphi = ((z1 * z1 + y * y - tlen * tlen - clen * clen) / (2 * clen * tlen));
			double phi = atan2(sqrt((1 - cosphi * cosphi)), cosphi);
			double k2 = clen * sin(phi);
			double k1 = tlen + clen * cos(phi);
			double alpha = atan2(k2, k1);
			double theta  = atan2(y, z1) - alpha;
			leg3[0][i] = lrint((-psi) * 180 / M_PI);
			leg3[1][i] = lrint(theta * 180 / M_PI);
			leg3[2][i] = lrint((-phi ) * 180 / M_PI);
		}
		//updating loop_iter
		loop_iter++;
	}

	//set HIP ABDUCTION ANGLE
	/*
	hip_abd = 0;
	legservo[0][0].write(offset[0][0] + 5 );
	delay(servoloop_delay);
	legservo[1][0].write(offset[1][0] - 5);
	delay(servoloop_delay);
	legservo[2][0].write(offset[2][0] + 10  );
	delay(servoloop_delay);
	legservo[3][0].write(offset[3][0] - 5);
	delay(servoloop_delay);
	*/
	tailservo[0].write(75);
	delay(tail_delay);

//---------Execution--------\\

// begining leg movement
	int i = 0;
	for(int j = 0; j < legServocount; j++)
	{
		legservo[0][j].write(offset[0][j] + leg0[j][i]);
		delay(servo_delay_gain * sqrt(pow((leg0[j][i] - leg0[j][legServocount - 1]), 2)));

		legservo[3][j].write(offset[3][j] + leg3[j][i]);
		delay(servo_delay_gain * sqrt(pow((leg3[j][i] - leg3[j][legServocount - 1]), 2)));

		legservo[1][j].write(offset[1][j] + leg1[j][i]);
		delay(servo_delay_gain * sqrt(pow((leg1[j][i] - leg1[j][legServocount - 1]), 2)));

		legservo[2][j].write(offset[2][j] + leg2[j][i]);
		delay(servo_delay_gain * sqrt(pow((leg2[j][i] - leg2[j][legServocount - 1]), 2)));

	}
	delay(iter_delay);

// rest of the leg movement (without tail)
	for(int i = 1; i < (step_iter * step_count) + 1; i++)
	{

		for(int j = 0; j < legServocount; j++)
		{
			legservo[0][j].write(offset[0][j] + leg0[j][i]);
			delay(servo_delay_gain * sqrt(pow((leg0[j][i] - leg0[j][i - 1]), 2)));

			legservo[3][j].write(offset[3][j] + leg3[j][i]);
			delay(servo_delay_gain * sqrt(pow((leg3[j][i] - leg3[j][i - 1]), 2)));

			legservo[1][j].write(offset[1][j] + leg1[j][i]);
			delay(servo_delay_gain * sqrt(pow((leg1[j][i] - leg1[j][i - 1]), 2)));

			legservo[2][j].write(offset[2][j] + leg2[j][i]);
			delay(servo_delay_gain * sqrt(pow((leg2[j][i] - leg2[j][i - 1]), 2)));

		}
		if(i == (step_iter*step_count/2))
		delay(1000);
	
		delay(iter_delay);
	}


}
