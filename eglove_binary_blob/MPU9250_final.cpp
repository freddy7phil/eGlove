//GCC & MRAA Lib
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <string.h>
#include "mraa.h"

//User Lib
#include "I2Cdev_mraa.hpp"
#include "MPU9250_DMP.hpp"
#include "helper_3dmath.hpp"

#ifndef MPU9250_FINAL_HPP
#define MPU9250_FINAL_HPP
	#include "MPU9250_final.hpp"
#endif


//Select the Print Variable
/*
RAW_DATA
QUATERNION
EULER_ANGLES
*/
//#define QUATERNION
#define EULER_ANGLES

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
using namespace std;
MPU9250 mpu;

//User Variables
//Array to Store ANSI Colour
char Ansi_Colour[10];
char Ansi_Colour_Bold[10];

//Loop Count variables
uint8_t i = 0;


sig_atomic_t volatile isrunning = 1;

//MRAA variables  
mraa_gpio_context S0;
mraa_gpio_context S1;
mraa_gpio_context S2;
mraa_gpio_context S3;
mraa_i2c_context MPU9250_i2c;

//MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//using namespace std;

void MPU9250_GPIO_Init()
{
	S0 = mraa_gpio_init(32);
	S1 = mraa_gpio_init(33);
	S2 = mraa_gpio_init(46);
	S3 = mraa_gpio_init(47);

	
	mraa_gpio_dir(S0, MRAA_GPIO_OUT);
	mraa_gpio_dir(S1, MRAA_GPIO_OUT);
	mraa_gpio_dir(S2, MRAA_GPIO_OUT);
	mraa_gpio_dir(S3, MRAA_GPIO_OUT);
}

/*
void MPU9250_I2C_Init()
{
  if (MPU9250_i2c == NULL) 
	{
		printf("MPU9250 I2C initialization failed, exit...\n");
		exit(1);
	}
	
	printf("MPU9250 I2C initialized successfully\n");
	
	mraa_i2c_address(MPU9250_i2c, MPU9250_DEFAULT_ADDRESS);
	printf("MPU9250 I2C Address set to 0x%x\n", MPU9250_DEFAULT_ADDRESS);
}
*/

void MPU9250_MUX_Select(uint8_t mux_channel)
{
	if(mux_channel == 0)
	{
		mraa_gpio_write(S0, 1);
	  	mraa_gpio_write(S1, 0);
		mraa_gpio_write(S2, 1);
		mraa_gpio_write(S3, 0);
	}
	
	else if(mux_channel == 1)
	{
		mraa_gpio_write(S0, 0);
		mraa_gpio_write(S1, 1);
		mraa_gpio_write(S2, 1);
		mraa_gpio_write(S3, 0);
	}
	
	else if(mux_channel == 2)
	{
		mraa_gpio_write(S0, 1);
		mraa_gpio_write(S1, 1);
		mraa_gpio_write(S2, 1);
		mraa_gpio_write(S3, 0);
	}
	
	else if(mux_channel == 3)
	{
		mraa_gpio_write(S0, 0);
		mraa_gpio_write(S1, 0);
		mraa_gpio_write(S2, 0);
		mraa_gpio_write(S3, 1);
	}

	else if(mux_channel == 4)
	{
		mraa_gpio_write(S0, 1);
		mraa_gpio_write(S1, 0);
		mraa_gpio_write(S2, 0);
		mraa_gpio_write(S3, 1);
	}
		
	else
	{
		printf("Wrong Channel\n");
	}
}

                 
void MPU9250_Text_Colour(uint8_t mux_channel)
{
  	if(mux_channel == 0)
	{
    	strcpy(Ansi_Colour, ANSI_COLOUR_RED);
    	strcpy(Ansi_Colour_Bold, ANSI_COLOUR_RED_BOLD);
	}
	
	else if(mux_channel == 1)
	{
    	strcpy(Ansi_Colour, ANSI_COLOUR_YELLOW);
    	strcpy(Ansi_Colour_Bold, ANSI_COLOUR_YELLOW_BOLD);
	}
	
	else if(mux_channel == 2)
	{
    	strcpy(Ansi_Colour, ANSI_COLOUR_CYAN);
    	strcpy(Ansi_Colour_Bold, ANSI_COLOUR_CYAN_BOLD);
	}
	
	else if(mux_channel == 3)
	{
    	strcpy(Ansi_Colour, ANSI_COLOUR_GREEN);
    	strcpy(Ansi_Colour_Bold, ANSI_COLOUR_GREEN_BOLD);
	}

	else if(mux_channel == 4)
	{
    	strcpy(Ansi_Colour, ANSI_COLOUR_BLUE);
    	strcpy(Ansi_Colour_Bold, ANSI_COLOUR_BLUE_BOLD);
	}

	else
	{
		printf("Wrong Channel\n");
	}
}


//Signal Handler
void sig_handler(int signum)
{
	if(signum == SIGINT)
	{
		isrunning = 0;
	}
}


void MPU9250_Setup()
{
	for(i = 0; i < SEN_COUNT; i++)
	{
		MPU9250 mpu;
    	printf("%s------------------------------------------------------------%s\n", ANSI_COLOUR_WHITE_BOLD, ANSI_COLOUR_RESET);
 		//if(i==2)
		//continue;   
		//Select the Multiplexer Channel
		MPU9250_MUX_Select(i);
    
    	//Select Text Colour
    	MPU9250_Text_Colour(i);

		printf("%s[  SEN%d  ] Initializing I2C... %s\n", Ansi_Colour, i, ANSI_COLOUR_RESET);
		//Initialize the MPU
		mpu.initialize();
		//Report Connection
		mpu.testConnection() ? printf("%s[  SEN%d  ] MPU9250 connection successful %s\n", Ansi_Colour, i, ANSI_COLOUR_RESET) : printf("%s[  SEN%d  ] MPU9250 connection Failed %s\n", Ansi_Colour, i, ANSI_COLOUR_RESET);

		printf("%s[  SEN%d  ] Initializing DMP... %s\n", Ansi_Colour, i, ANSI_COLOUR_RESET);
		//Initialize the DMP
		devStatus = mpu.dmpInitialize();
    	std::cout << "DMP Status";
    	//Make sure it worked (returns 0 if so)
    	if (devStatus == 0) 
    	{
        	//Turn on the DMP, now that it's ready
        	printf("%s[  SEN%d  ] Enabling DMP... %s\n", Ansi_Colour, i, ANSI_COLOUR_RESET);
        	mpu.setDMPEnabled(true);

        	//Set our DMP Ready flag so the main loop() function knows it's okay to use it
        	printf("%s[  SEN%d  ] DMP Ready! %s\n", Ansi_Colour, i, ANSI_COLOUR_RESET);
        	dmpReady = true;

        	//Get expected DMP packet size for later comparison
        	packetSize = mpu.dmpGetFIFOPacketSize();
        
    	}

    	else 
    	{
        	//ERROR!
        	//1 = initial memory load failed
        	//2 = DMP configuration updates failed
        	//(If it's going to break, usually the code will be 1)
    		printf("DMP Initialization failed (code ");
        	std::cout << devStatus;
        	printf(")\n");
    	}
    	
    	printf("%s[  SEN%d  ] Ready %s\n", Ansi_Colour, i, ANSI_COLOUR_RESET);
    
    	printf("%s------------------------------------------------------------%s\n", ANSI_COLOUR_WHITE_BOLD, ANSI_COLOUR_RESET);
	}
	printf("%s\n\tInitialization Complete: All Systems are GO!!!%s\n\n", ANSI_COLOUR_WHITE_BOLD, ANSI_COLOUR_RESET);
}


void MPU9250_Loop()
{
	//Select the Multiplexer Channel
	MPU9250_MUX_Select(i);

  	//Select Text Colour
  	MPU9250_Text_Colour(i);
	
	//If programming failed, don't try to do anything
    if(!dmpReady) return;

    //Get current FIFO count
    fifoCount = mpu.getFIFOCount();

    //Wait for correct available data length, should be a VERY short wait
    while(fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    //Read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    #ifdef QUATERNION
    	//Display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        std::cout << Ansi_Colour << "[ SEN" << i << " ]" << ANSI_COLOUR_RESET << ".."<<fifoCount;
        std::cout << "\t";
        std::cout << Ansi_Colour << q.w << ANSI_COLOUR_RESET;
        std::cout << "\t";
        std::cout << Ansi_Colour << q.x << ANSI_COLOUR_RESET;
        std::cout << "\t";
        std::cout << Ansi_Colour << q.y << ANSI_COLOUR_RESET;
        std::cout << "\t";
        std::cout << Ansi_Colour << q.z << "\n" << ANSI_COLOUR_RESET << endl;

    #endif
    
    #ifdef EULER_ANGLES
        //Display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        std::cout << "Euler Angles\t"<<".."<<fifoCount<<"\t";
        std::cout << Ansi_Colour << "[ SEN" << i << " ]" << ANSI_COLOUR_RESET;
        std::cout << "\t";
        std::cout << Ansi_Colour << (euler[0] * 180/M_PI) << ANSI_COLOUR_RESET;
        std::cout << "\t";
        std::cout << Ansi_Colour << (euler[1] * 180/M_PI) << ANSI_COLOUR_RESET;
        std::cout << "\t";
        std::cout << Ansi_Colour << (euler[2] * 180/M_PI) << "\n" << ANSI_COLOUR_RESET;
    #endif
}


int main(int argc, char **argv)
{
	printf("%s------------------------------------------------------------%s\n", ANSI_COLOUR_MAGENTA_BOLD, ANSI_COLOUR_RESET);
	printf("%s-----------------------eGlove Project-----------------------%s\n", ANSI_COLOUR_MAGENTA_BOLD, ANSI_COLOUR_RESET);
	printf("%s------------------------------------------------------------%s\n", ANSI_COLOUR_MAGENTA_BOLD, ANSI_COLOUR_RESET);
	sleep(1); //1s delay
	
	mraa_init();
	
	MPU9250_GPIO_Init();

	//MPU9250_i2c = mraa_i2c_init(1);
	usleep(1000); //1ms delay

	//MPU9250_I2C_Init();
	
	signal(SIGINT, &sig_handler);
	usleep(1000); //1ms delay
	
	//IMU Init
	//DMP Init
	MPU9250_Setup();
  
	printf("%s------------------------------------------------------------%s\n", ANSI_COLOUR_WHITE_BOLD, ANSI_COLOUR_RESET);
    i = 0;
	//while(isrunning)
	while (i<2)
	{
	     
		//for(i = 0; i < SEN_COUNT; i++)
		//{  
			//if(i == 2)
			//continue;
			MPU9250_Loop();
			i = i +1;
		//}

	}
	return MRAA_SUCCESS;
}

