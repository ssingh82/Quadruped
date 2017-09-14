// Crawl Gait 1  Exection of the robot
// Crawl Gait 1 : front legs move, then trunk then the rear legs
// Tail :  only Yaw movement is allowed

// assuming hip abduction and flexion joint are co-incident
// JOINT ANGLES are in degrees
// this code uses inverse kinematics to first derive the angles, then store them
// for further use.
// it also incorporates SINUSOIDAL delays in the tail movement...
// to reduce the  jerk


#include <Servo.h> 

#define totalServocount 14 //total numbers of motor in the robot
#define legServocount 3 //number of motors in a leg
#define tailServocount 2 //number of motors in a tail
#define legcount 4 //total number of legs
#define step_iter 18 //number of iterations in a step :  standard
#define max_step_count  8 // max number of steps in a gait : standard

// All dimensions are in m and degrees
//Robot's properties
double clen = 0.115;
double tlen = 0.065;
double height = 0.16;
double stand_height = clen + tlen -0.01; // = 0.18
double crouch = 0.04;
double step = 0.08;
double back_step = 0.04; // the position of foot behinfd shoulder in standing
double step_height = 0.02;
int iter_delay = 10;
int servoloop_delay = 5;
double tail_delay = 30;
 //offset to the tail: NOTE: It is always added to the angle



//using the array approach for defining properties of the servos
//leg servos
int legpin[4][3] ={
				{2,3,4},
				{5,6,9},
				{10,11,12},
				{22,24,26},
				};
int leghome[4][3] = {
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0, 0, 0}
					};

//tail servos
int tailpin[tailServocount] = {28,30};
int tailhome[tailServocount] = {90,90};

//Defining servo objects
Servo legservo[legcount][legServocount];
Servo tailservo[tailServocount];

//Delay while executing servo loops


//defining joint anlgles
//current joint angles
float psi[legcount] = {0,0,0,0}; 
float theta[legcount] = {0,0,0,0};
float phi[legcount] = {0,0,0,0};

//offset of joint angles
float offset[legcount][legServocount] = {
										{95,85,93},
										{95,93,95},
										{95,87,95},
										{85,90,95}};

//defining the variables used in gait execution
//arrays for storing angles
float leg0[legServocount][step_iter*max_step_count];
float leg1[legServocount][step_iter*max_step_count];
float leg2[legServocount][step_iter*max_step_count];
float leg3[legServocount][step_iter*max_step_count];

int step_count = 1; // step count for crawl gait 1.2 
double z = height; // veritcle coordinates of the foot
double y = 0; // forward coordinate of the foot
int loop_iter = 0;
float hip_abd = 0;
//******************THE SETUP FUNCTION********************
void setup()
{
  //Setting up the motors

	for(int i= 0; i<legcount; i++)
	{
		for(int j = 0; j<legServocount; j++)
		{
			legservo[i][j].attach(legpin[i][j]);	
		}
	}
	for(int i=0; i<tailServocount;i++)
	{
		tailservo[i].attach(tailpin[i]);
	}
	//serial monitor for debugging
	Serial.begin(9600);

}


//**************THE LOOP:  CONTAINS THE GAIT*******************
void loop()
{
	if(loop_iter==0)
	{

	//Generating Angles for Crawl GAit 1
	// step_iter = 18 : standard
	// step_count = 5 : for crawl gait 1-2
	//GAIT: 0-2-3-1
	// for leg 0
	for (int i = 0; i<(step_iter*step_count) + 1 ; i++)
	{ 
	    if(i<(step_iter*1)+1) //step 1
		{	    
			y = 0;
	    	z = stand_height - crouch*sin((i)*10*M_PI/180) ;
		}
		else if (i> (step_iter*1) && i<(step_iter*2)+1) //step 2
		{
			y = 0;
	    	z = stand_height - crouch*sin((i/2)*10*M_PI/180); 		
		}

	    double cosphi = ((z*z + y*y - tlen*tlen - clen*clen)/(2*clen*tlen));
	    double phi = atan2(sqrt((1 - cosphi*cosphi)), cosphi);
	    double k2 = clen*sin(phi);
	    double k1 = tlen + clen*cos(phi);
	    double alpha = atan2(k2,k1);
	    double theta  = atan2(y,z) - alpha;
	    leg0[1][i] = lrint((-theta)*180/M_PI);
	    leg0[2][i] = lrint((phi )*180/M_PI);
		Serial.println(leg0[1][i]);
    }
    
	// for leg 3
	for (int i = 0; i<(step_iter*step_count) + 1 ; i++)
	{ 
	    if(i<(step_iter*1)+1) //step 1
		{	    
			y = 0;
	    	z = stand_height - crouch*sin((i)*10*M_PI/180) ;
		}
		else if (i> (step_iter*1) && i<(step_iter*2)+1) //step 2
		{
			y = 0;
	    	z = stand_height - crouch*sin((i/2)*10*M_PI/180); 		
		}

	    double cosphi = ((z*z + y*y - tlen*tlen - clen*clen)/(2*clen*tlen));
	    double phi = atan2(sqrt((1 - cosphi*cosphi)), cosphi);
	    double k2 = clen*sin(phi);
	    double k1 = tlen + clen*cos(phi);
	    double alpha = atan2(k2,k1);
	    double theta = atan2(y,z) - alpha;
	    leg3[1][i] = lrint(theta*180/M_PI);
	    leg3[2][i] = lrint((-phi )*180/M_PI);
		
    }
    
	// for leg 1
	for (int i = 0; i<(step_iter*step_count) + 1 ; i++)
	{ 
	    if(i<(step_iter*1)+1) //step 1
		{	    
			y = 0;
	    	z = stand_height - crouch*sin((i)*10*M_PI/180) ;
		}
		else if (i> (step_iter*1) && i<(step_iter*2)+1) //step 2
		{
			y = 0;
	    	z = stand_height - crouch*sin((i/2)*10*M_PI/180); 		
		}
	    double cosphi = ((z*z + y*y - tlen*tlen - clen*clen)/(2*clen*tlen));
	    double phi = atan2(sqrt((1 - cosphi*cosphi)), cosphi);
	    double k2 = clen*sin(phi);
	    double k1 = tlen + clen*cos(phi);
	    double alpha = atan2(k2,k1);
	    double theta = atan2(y,z) - alpha;
	    leg1[1][i] = lrint(theta*180/M_PI);
	    leg1[2][i] = lrint((-phi )*180/M_PI);
		
    }

	// for leg 2
	for (int i = 0; i<(step_iter*step_count) + 1 ; i++)
	{ 
	    if(i<(step_iter*1)+1) //step 1
		{	    
			y = 0;
	    	z = stand_height - crouch*sin((i)*10*M_PI/180) ;
		}
		else if (i> (step_iter*1) && i<(step_iter*2)+1) //step 2
		{
			y = 0;
	    	z = stand_height - crouch*sin((i/2)*10*M_PI/180); 		
		}
	    double cosphi = ((z*z + y*y - tlen*tlen - clen*clen)/(2*clen*tlen));
	    double phi = atan2(sqrt((1 - cosphi*cosphi)), cosphi);
	    double k2 = clen*sin(phi);
	    double k1 = tlen + clen*cos(phi);
	    double alpha = atan2(k2,k1);
	    double theta  = atan2(y,z) - alpha;
	    leg2[1][i] = lrint((-theta)*180/M_PI);
	    leg2[2][i] = lrint((phi)*180/M_PI);
		
    }
 	//updating loop_iter
    loop_iter++;
	}
	//set HIP ABDUCTION ANGLE
	hip_abd = 0;
	legservo[0][0].write(offset[0][0] + hip_abd ); 
	delay(servoloop_delay);
	legservo[1][0].write(offset[1][0] - hip_abd); 
	delay(servoloop_delay);
	legservo[2][0].write(offset[2][0] + hip_abd  ); 
	delay(servoloop_delay);
	legservo[3][0].write(offset[3][0] - hip_abd); 
	delay(servoloop_delay);
	
	tailservo[1].write(90);	
	tailservo[0].write(90);
	delay(tail_delay);
	
//---------Execution--------\\
//leg movement (along with tail)
	for(int i = 0; i<(step_iter*step_count)+1; i++)
	{
		if(i == 0)
		delay(1000);
		for(int j = 1; j<legServocount; j++)
		{
		legservo[0][j].write(offset[0][j] + leg0[j][i]); 
		delay(servoloop_delay);
		legservo[3][j].write(offset[3][j] + leg3[j][i]); 
		delay(servoloop_delay);
		legservo[1][j].write(offset[1][j] + leg1[j][i]); 
		delay(servoloop_delay);
		legservo[2][j].write(offset[2][j] + leg2[j][i]); 
		delay(servoloop_delay);
		
		}
		if(i == (step_iter*step_count/2))
		delay(1000);
	
		delay(iter_delay);
	}


}




