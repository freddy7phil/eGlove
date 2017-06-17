//GCC & MRAA Lib
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include "mraa.h"

//User Lib
#include "Final.h"

int main(int argc, char **argv)
{
	printf("%s------------------------------------------------------------%s\n", ANSI_COLOUR_MAGENTA_BOLD, ANSI_COLOUR_RESET);
	printf("%s-----------------------eGlove Project-----------------------%s\n", ANSI_COLOUR_MAGENTA_BOLD, ANSI_COLOUR_RESET);
	printf("%s------------------------------------------------------------%s\n", ANSI_COLOUR_MAGENTA_BOLD, ANSI_COLOUR_RESET);
	sleep(1); //1s delay
	
	mraa_init();
	
	MPU9250_GPIO_Init();

	MPU9250_i2c = mraa_i2c_init(1);
	
	signal(SIGINT, &sig_handler);
	usleep(1000); //1ms delay
	
	MPU9250_I2C_Init();
	sleep(1); //1s delay

	MPU9250_I2C_Config(SEN_COUNT);	
	sleep(1); //1s delay
  
	printf("%s------------------------------------------------------------%s\n", ANSI_COLOUR_WHITE_BOLD, ANSI_COLOUR_RESET);
	
	#ifdef GESTURE_SCAN
		//Initialize Database
		MPU9250_Alpha_Struct();
	#endif

	//Store First Time Instance
	Timer = clock();
  
	while(isrunning)
	{
	
		for(i = 0; i < SEN_COUNT; i++)
		{  

			MPU9250_Loop();
			
			#ifdef RAW_ANGLES
				//Print Raw Angles
				printf("%s[  SEN%d  ] Raw Angles:  Raw_X:%6.2lf    Raw_Y:%6.2lf %s\n", Ansi_Colour_Bold, i, r[i].Raw_X, r[i].Raw_Y, ANSI_COLOUR_RESET);
			#endif
			
			#ifdef COMPLEMENTARY_ANGLES
				//Print Complementary Filter Angles
				printf("%s[  SEN%d  ] CF Angles:  C_X:%6.2lf    C_Y:%6.2lf %s\n", Ansi_Colour_Bold, i, c[i].C_X, c[i].C_Y, ANSI_COLOUR_RESET);
			#endif

			s[i].Roll = c[i].C_X;
			s[i].Pitch = c[i].C_Y;
			
			#ifdef EULER_ANGLES
    			//Print Euler Angles
    			printf("\r%s[  SEN%d  ] Euler Angles:  Roll:%6.2lf    Pitch:%6.2lf %s", Ansi_Colour_Bold, i, s[i].Roll, s[i].Pitch, ANSI_COLOUR_RESET);
      		#endif
			

			
//			printf("%s------------------------------------------------------------%s\n", ANSI_COLOUR_WHITE_BOLD, ANSI_COLOUR_RESET);
    
			dt = (double)(clock() - Timer) / CLOCKS_PER_SEC;
	
			#ifdef LOOP_TIME	
				//Print Loop-time
				printf("\rTime elapsed in s: %f", dt);
			#endif
	
  			Timer = clock();    
//			printf("%s------------------------------------------------------------%s\n", ANSI_COLOUR_WHITE_BOLD, ANSI_COLOUR_RESET);
		}

	#ifdef GESTURE_SCAN	
		MPU9250_Gesture_Scan();
	#endif

	}
	return MRAA_SUCCESS;
}

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

void MPU9250_I2C_Init()
{
  if (MPU9250_i2c == NULL) 
	{
		printf("MPU9250 I2C initialization failed, exit...\n");
		exit(1);
	}
	
	printf("MPU9250 I2C initialized successfully\n");
	
	mraa_i2c_address(MPU9250_i2c, MPU_ADDR);
	printf("MPU9250 I2C Address set to 0x%x\n", MPU_ADDR);
}

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
	
void MPU9250_I2C_Config(uint8_t sen_count)
{
	for(i = 0; i < sen_count; i++)
	{
    	printf("%s------------------------------------------------------------%s\n", ANSI_COLOUR_WHITE_BOLD, ANSI_COLOUR_RESET);
    
		//Select the Multiplexer Channel
		MPU9250_MUX_Select(i);
    
    	//Select Text Colour
    	MPU9250_Text_Colour(i);
		
		//Reset all the Registers
  		mraa_i2c_address(MPU9250_i2c, MPU_ADDR);
		MPU9250_I2C_Write(PWR_MGMT_1, PWR_RESET);
		printf("%s[  SEN%d  ] Reset %s\n", Ansi_Colour, i, ANSI_COLOUR_RESET);
		usleep(100000); //100ms delay
	
		mraa_i2c_address(MPU9250_i2c, MPU_ADDR);
 		MPU9250_I2C_Write(PWR_MGMT_1, DEVICE_ON);
		printf("%s[  SEN%d  ] Switched ON %s\n", Ansi_Colour, i, ANSI_COLOUR_RESET);
		usleep(100000); //100ms delay
	
		mraa_i2c_address(MPU9250_i2c, MPU_ADDR);
  		uint8_t data = mraa_i2c_read_byte_data(MPU9250_i2c, WHO_AM_I); //Should return 0x71
  		printf("%s[  SEN%d  ] Who am I: 0x%x %s\n", Ansi_Colour, i, data, ANSI_COLOUR_RESET);
		usleep(100000); //100ms delay
		
		if (data != 0x71) 
		{ 
			// Read "WHO_AM_I" register
    		printf("Error reading [  SEN%d  ]!", i);
    		while (1);
  		}
	
 		MPU9250_I2C_Write(SMPRT_DIV, SAMPLE_RATE);
	
 		MPU9250_I2C_Write(CONFIG, DLPF_CFG);

 		//Set the Gyroscope Scale to 250°/s
 		MPU9250_I2C_Write(GYRO_CONFIG, GYRO_250);
	
		//Set the Accelerometer Scale to 2G
 		MPU9250_I2C_Write(ACCEL_CONFIG, ACCEL_2G);
    		
		printf("%s[  SEN%d  ] Ready %s\n", Ansi_Colour, i, ANSI_COLOUR_RESET);
    
    	printf("%s------------------------------------------------------------%s\n", ANSI_COLOUR_WHITE_BOLD, ANSI_COLOUR_RESET);
    
		sleep(1); //1s delay
	}
	printf("%s\n\tInitialization Complete: All Systems are GO!!!%s\n\n", ANSI_COLOUR_WHITE_BOLD, ANSI_COLOUR_RESET);
}

void MPU9250_I2C_Write(uint8_t address, uint8_t value)
{	
	//Set MPU Device Address
	mraa_i2c_address(MPU9250_i2c, MPU_ADDR);
	
	//Write Command and Data
	mraa_i2c_write_byte_data(MPU9250_i2c, value, address);
}

void MPU9250_I2C_Read(uint8_t address, uint8_t *value)
{	
	//Set ALS Device Address
	mraa_i2c_address(MPU9250_i2c, MPU_ADDR);
	
	//Write Command and Read Data
	*value = mraa_i2c_read_byte_data(MPU9250_i2c, address);
}

int16_t MPU9250_Get_Measurement(uint8_t addrL, uint8_t addrH)
{	
	//Read & Store the Lower Byte
	MPU9250_I2C_Read(addrL, &Measurement_L);
	
	//Read & Store the Higher Byte
	MPU9250_I2C_Read(addrH, &Measurement_H);
  
  	return (int16_t)((Measurement_H << 8) + Measurement_L);
}

void MPU9250_Filter_Init()
{
	//Get Accelerometer Measurements
	Accel_X = MPU9250_Get_Measurement(ACCEL_XOUT_L, ACCEL_XOUT_H);
  	Accel_Y = MPU9250_Get_Measurement(ACCEL_YOUT_L, ACCEL_YOUT_H);
  	Accel_Z = MPU9250_Get_Measurement(ACCEL_ZOUT_L, ACCEL_ZOUT_H);
	
	Roll = atan(Accel_Y / sqrt((Accel_X * Accel_X) + (Accel_Z * Accel_Z))) * (180.0 / PI);
  	Pitch = atan2(-Accel_X, Accel_Z) * (180.0 / PI);
							
	MPU9250_Kalman_Get_Angle(Roll);
	MPU9250_Kalman_Get_Angle(Pitch);
							
	r[i].Raw_X = Roll;
	r[i].Raw_Y = Pitch;
							
	c[i].C_X = Roll;
	c[i].C_Y = Pitch;
}

void MPU9250_Loop()
{
	//Select the Multiplexer Channel
	MPU9250_MUX_Select(i);

	//Initialize Filter for all Sensors once
    while(Init_Count < SEN_COUNT)
	{
		MPU9250_Filter_Init();
		Init_Count++;
	}

  	//Select Text Colour
  	MPU9250_Text_Colour(i);
				
	//Get Accelerometer Measurements
	Accel_X = MPU9250_Get_Measurement(ACCEL_XOUT_L, ACCEL_XOUT_H);
  	Accel_Y = MPU9250_Get_Measurement(ACCEL_YOUT_L, ACCEL_YOUT_H);
  	Accel_Z = MPU9250_Get_Measurement(ACCEL_ZOUT_L, ACCEL_ZOUT_H);

	//Get Gyroscope Measurements
  	Gyro_X = MPU9250_Get_Measurement(GYRO_XOUT_L, GYRO_XOUT_H);
  	Gyro_Y = MPU9250_Get_Measurement(GYRO_YOUT_L, GYRO_YOUT_H);
  	Gyro_Z = MPU9250_Get_Measurement(GYRO_ZOUT_L, GYRO_ZOUT_H);
    
    #ifdef RAW_DATA 
		//Print Accelerometer Values
		printf("%s[  SEN%d  ] Accelerometer: X:%6d    Y:%6d   Z:%6d %s\n", Ansi_Colour, i, Accel_X, Accel_Y, Accel_Z, ANSI_COLOUR_RESET);
		
		//Print Gyroscope Values
		printf("%s[  SEN%d  ] Gyroscope:     X:%6d    Y:%6d   Z:%6d %s\n", Ansi_Colour, i, Gyro_X, Gyro_Y, Gyro_Z, ANSI_COLOUR_RESET);
    #endif
  	
  	Roll = atan(Accel_Y / sqrt((Accel_X * Accel_X) + (Accel_Z * Accel_Z))) * (180.0 / PI);
  	Pitch = atan2(-Accel_X, Accel_Z) * (180.0 / PI);
                  
  	Gyro_X_Rate = Gyro_X / 131.0;
  	Gyro_Y_Rate = Gyro_Y / 131.0;
                  
  	// This fixes the transition problem when the accelerometer Angle jumps between -180 and 180 degrees
  	if ((Pitch < -90 && k[i].K_Y > 90) || (Pitch > 90 && k[i].K_Y < -90)) 
  	{
    	MPU9250_Kalman_Get_Angle(Pitch);
    	c[i].C_Y = Pitch;
    	k[i].K_Y = Pitch;
    	r[i].Raw_Y = Pitch;
  	} 
     
  	else
  	{
    	k[i].K_Y = MPU9250_Kalman_Update(Pitch, Gyro_Y_Rate, dt); // Calculate the Angle using a Kalman filter
  	}
                  
  	if (abs(k[i].K_Y) > 90)
  	{
    	Gyro_X_Rate = -Gyro_X_Rate; // Invert Rate, so it fits the restriced accelerometer reading
  	}
       
  	k[i].K_X = MPU9250_Kalman_Update(Roll, Gyro_X_Rate, dt); // Calculate the Angle using a Kalman filter
  
  	r[i].Raw_X += Gyro_X_Rate * dt; // Calculate gyro Angle without any filter
  	r[i].Raw_Y += Gyro_Y_Rate * dt;
    
	c[i].C_X = 0.93 * (c[i].C_X + Gyro_X_Rate * dt) + 0.07 * Roll; // Calculate the Angle using a Complimentary filter
  	c[i].C_Y = 0.93 * (c[i].C_Y + Gyro_Y_Rate * dt) + 0.07 * Pitch;

  	// Reset the gyro Angle when it has drifted too much
  	if (r[i].Raw_X < -180 || r[i].Raw_X > 180)
  	{
    	r[i].Raw_X = k[i].K_X;
  	}
                 
  	if (r[i].Raw_Y < -180 || r[i].Raw_Y > 180)
  	{
    	r[i].Raw_Y = k[i].K_Y;
  	}
}


void MPU9250_Kalman_Get_Angle(double newAngle)
{
  Angle = newAngle;                    
}
                  
double MPU9250_Kalman_Update(double newAngle, double newRate, double dt)
{
	// Discrete Kalman filter time update equations - Time Update ("Predict")
  	// Update xhat - Project the state ahead
  	/* Step 1 */
  	Rate = newRate - Bias;
  	Angle += dt * Rate;

  	// Update estimation error covariance - Project the error covariance ahead
  	/* Step 2 */
  	P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_ANGLE);
  	P[0][1] -= dt * P[1][1];
  	P[1][0] -= dt * P[1][1];
  	P[1][1] += Q_BIAS * dt;

  	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
  	// Calculate Kalman gain - Compute the Kalman gain
  	/* Step 4 */
  	S = P[0][0] + R_MEASURE;
  
  	/* Step 5 */
  	K[0] = P[0][0] / S;
  	K[1] = P[1][0] / S;

  	// Calculate Angle and Bias - Update estimate with measurement zk (newAngle)
  	/* Step 3 */
  	y = newAngle - Angle;
  
  	/* Step 6 */
  	Angle += K[0] * y;
  	Bias += K[1] * y;

  	// Calculate estimation error covariance - Update the error covariance
  	/* Step 7 */
  	P[0][0] -= K[0] * P[0][0];
  	P[0][1] -= K[0] * P[0][1];
  	P[1][0] -= K[1] * P[0][0];
  	P[1][1] -= K[1] * P[0][1];

  	return Angle;
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

//Structure Variables
/*-----Alphabet Database-----*/
/*-------------A-------------*/
void MPU9250_Alpha_Struct()
{	
	/*----------------A----------------*/
	alphabets[0].minS[0].Roll = 25.0f;
	alphabets[0].maxS[0].Roll = 55.0f;
	alphabets[0].minS[0].Pitch = -180.0f;
	alphabets[0].maxS[0].Pitch = 180.0f;
		
	alphabets[0].minS[1].Roll = -100.0f;
	alphabets[0].maxS[1].Roll = -40.0f;
	alphabets[0].minS[1].Pitch = -180.0f;
	alphabets[0].maxS[1].Pitch = 180.0f;

	alphabets[0].minS[2].Roll = -100.0f;
	alphabets[0].maxS[2].Roll = -40.0f;
	alphabets[0].minS[2].Pitch = -180.0f;
	alphabets[0].maxS[2].Pitch = 180.0f;

	alphabets[0].minS[3].Roll = -100.0f;
	alphabets[0].maxS[3].Roll = -40.0f;
	alphabets[0].minS[3].Pitch = -180.0f;
	alphabets[0].maxS[3].Pitch = 180.0f;

	alphabets[0].minS[4].Roll = -100.0f;
	alphabets[0].maxS[4].Roll = -40.0f;
	alphabets[0].minS[4].Pitch = -180.0f;
	alphabets[0].maxS[4].Pitch = 180.0f;
		
		
	/*----------------B----------------*/
	alphabets[1].minS[0].Roll = 20.0f;
	alphabets[1].maxS[0].Roll = 50.0f;
	alphabets[1].minS[0].Pitch = -40.0f;
	alphabets[1].maxS[0].Pitch = 0.0f;

		
	alphabets[1].minS[1].Roll = 50.0f;
	alphabets[1].maxS[1].Roll = 100.0f;
	alphabets[1].minS[1].Pitch = -150.0f;
	alphabets[1].maxS[1].Pitch = 150.0f;

	alphabets[1].minS[2].Roll = 40.0f;
	alphabets[1].maxS[2].Roll = 80.0f;
	alphabets[1].minS[2].Pitch = -150.0f;
	alphabets[1].maxS[2].Pitch = 150.0f; 
		
	alphabets[1].minS[3].Roll = 60.0f;
	alphabets[1].maxS[3].Roll = 100.0f;
	alphabets[1].minS[3].Pitch = -150.0f;
	alphabets[1].maxS[3].Pitch = 150.0f;
		
	alphabets[1].minS[4].Roll = 60.0f;
	alphabets[1].maxS[4].Roll = 100.0f;
	alphabets[1].minS[4].Pitch = -150.0f;
	alphabets[1].maxS[4].Pitch = 150.0f;
		
		
	/*----------------C----------------*/
	alphabets[2].minS[0].Roll = 0.0f;
	alphabets[2].maxS[0].Roll = 30.0f;
	alphabets[2].minS[0].Pitch = -150.0f;
	alphabets[2].maxS[0].Pitch = -50.0f;
		
	alphabets[2].minS[1].Roll = 0.0f;
	alphabets[2].maxS[1].Roll = 30.0f;
	alphabets[2].minS[1].Pitch = -150.0f;
	alphabets[2].maxS[1].Pitch = 150.0f;

	alphabets[2].minS[2].Roll = -30.0f;
	alphabets[2].maxS[2].Roll = 0.0f;
	alphabets[2].minS[2].Pitch = -150.0f;
	alphabets[2].maxS[2].Pitch = 150.0f; 
	 
	alphabets[2].minS[3].Roll = -20.0f;
	alphabets[2].maxS[3].Roll = 0.0f;
	alphabets[2].minS[3].Pitch = -150.0f;
	alphabets[2].maxS[3].Pitch = 150.0f;
		
	alphabets[2].minS[4].Roll = 0.0f;
	alphabets[2].maxS[4].Roll = 40.0f;
	alphabets[2].minS[4].Pitch = -150.0f;
	alphabets[2].maxS[4].Pitch = 150.0f;


	/*----------------D----------------*/
	alphabets[3].minS[0].Roll = -10.0f;
	alphabets[3].maxS[0].Roll = 40.0f;
	alphabets[3].minS[0].Pitch = -150.0f;
	alphabets[3].maxS[0].Pitch = -50.0f;
		
	alphabets[3].minS[1].Roll = 60.0f;
	alphabets[3].maxS[1].Roll = 110.0f;
	alphabets[3].minS[1].Pitch = -50.0f;
	alphabets[3].maxS[1].Pitch = 50.0f;

	alphabets[3].minS[2].Roll = -80.0f;
	alphabets[3].maxS[2].Roll = -30.0f;
	alphabets[3].minS[2].Pitch = 0.0f;
	alphabets[3].maxS[2].Pitch = 80.0f; 
	 
	alphabets[3].minS[3].Roll = -70.0f;
	alphabets[3].maxS[3].Roll = -30.0f;
	alphabets[3].minS[3].Pitch = -10.0f;
	alphabets[3].maxS[3].Pitch = 40.0f;
		
	alphabets[3].minS[4].Roll = -70.0f;
	alphabets[3].maxS[4].Roll = 30.0f;
	alphabets[3].minS[4].Pitch = -150.0f;
	alphabets[3].maxS[4].Pitch = 150.0f;


	/*----------------E----------------*/
	alphabets[4].minS[0].Roll = -10.0f;
	alphabets[4].maxS[0].Roll = 30.0f;
	alphabets[4].minS[0].Pitch = -100.0f;
	alphabets[4].maxS[0].Pitch = 100.0f;
		
	alphabets[4].minS[1].Roll = -40.0f;
	alphabets[4].maxS[1].Roll = 0.0f;
	alphabets[4].minS[1].Pitch = -50.0f;
	alphabets[4].maxS[1].Pitch = 0.0f;

	alphabets[4].minS[2].Roll = -80.0f;
	alphabets[4].maxS[2].Roll = -30.0f;
	alphabets[4].minS[2].Pitch = 0.0f;
	alphabets[4].maxS[2].Pitch = 80.0f; 
	 
	alphabets[4].minS[3].Roll = -70.0f;
	alphabets[4].maxS[3].Roll = -30.0f;
	alphabets[4].minS[3].Pitch = -10.0f;
	alphabets[4].maxS[3].Pitch = 40.0f;
		
	alphabets[4].minS[4].Roll = -70.0f;
	alphabets[4].maxS[4].Roll = 30.0f;
	alphabets[4].minS[4].Pitch = -100.0f;
	alphabets[4].maxS[4].Pitch = 100.0f;


	/*----------------F----------------*/
	alphabets[5].minS[0].Roll = 40.0f;
	alphabets[5].maxS[0].Roll = 80.0f;
	alphabets[5].minS[0].Pitch = -150.0f;
	alphabets[5].maxS[0].Pitch = 0.0f;
		
	alphabets[5].minS[1].Roll = -40.0f;
	alphabets[5].maxS[1].Roll = 0.0f;
	alphabets[5].minS[1].Pitch = -50.0f;
	alphabets[5].maxS[1].Pitch = 0.0f;

	alphabets[5].minS[2].Roll = 40.0f;
	alphabets[5].maxS[2].Roll = 80.0f;
	alphabets[5].minS[2].Pitch = 0.0f;
	alphabets[5].maxS[2].Pitch = 100.0f; 
	 
	alphabets[5].minS[3].Roll = 30.0f;
	alphabets[5].maxS[3].Roll = 80.0f;
	alphabets[5].minS[3].Pitch = 0.0f;
	alphabets[5].maxS[3].Pitch = 100.0f;
		
	alphabets[5].minS[4].Roll = 50.0f;
	alphabets[5].maxS[4].Roll = 100.0f;
	alphabets[5].minS[4].Pitch = 0.0f;
	alphabets[5].maxS[4].Pitch = 100.0f;


	/*----------------G----------------*/
	alphabets[6].minS[0].Roll = 60.0f;
	alphabets[6].maxS[0].Roll = 100.0f;
	alphabets[6].minS[0].Pitch = 110.0f;
	alphabets[6].maxS[0].Pitch = 170.0f;
		
	alphabets[6].minS[1].Roll = 55.0f;
	alphabets[6].maxS[1].Roll = 95.0f;
	alphabets[6].minS[1].Pitch = -10.0f;
	alphabets[6].maxS[1].Pitch = 30.0f;

	alphabets[6].minS[2].Roll = -100.0f;
	alphabets[6].maxS[2].Roll = -55.0f;
	alphabets[6].minS[2].Pitch = -180.0f;
	alphabets[6].maxS[2].Pitch = 180.0f; 
	 
	alphabets[6].minS[3].Roll = -95.0f;
	alphabets[6].maxS[3].Roll = -55.0f;
	alphabets[6].minS[3].Pitch = -180.0f;
	alphabets[6].maxS[3].Pitch = 180.0f;
		
	alphabets[6].minS[4].Roll = -90.0f;
	alphabets[6].maxS[4].Roll = -40.0f;
	alphabets[6].minS[4].Pitch = -180.0f;
	alphabets[6].maxS[4].Pitch = 180.0f;


	/*----------------H----------------*/
	alphabets[7].minS[0].Roll = 15.0f;
	alphabets[7].maxS[0].Roll = 75.0f;
	alphabets[7].minS[0].Pitch = -80.0f;
	alphabets[7].maxS[0].Pitch = -30.0f;
		
	alphabets[7].minS[1].Roll = 55.0f;
	alphabets[7].maxS[1].Roll = 95.0f;
	alphabets[7].minS[1].Pitch = -180.0f;
	alphabets[7].maxS[1].Pitch = 180.0f;

	alphabets[7].minS[2].Roll = -90.0f;
	alphabets[7].maxS[2].Roll = -50.0f;
	alphabets[7].minS[2].Pitch = -180.0f;
	alphabets[7].maxS[2].Pitch = 180.0f; 
	 
	alphabets[7].minS[3].Roll = -80.0f;
	alphabets[7].maxS[3].Roll = -40.0f;
	alphabets[7].minS[3].Pitch = -180.0f;
	alphabets[7].maxS[3].Pitch = 180.0f;
		
	alphabets[7].minS[4].Roll = 65.0f;
	alphabets[7].maxS[4].Roll = 105.0f;
	alphabets[7].minS[4].Pitch = -180.0f;
	alphabets[7].maxS[4].Pitch = 180.0f;

	
	/*----------------I----------------*/
	alphabets[8].minS[0].Roll = -100.0f;
	alphabets[8].maxS[0].Roll = 100.0f;
	alphabets[8].minS[0].Pitch = -75.0f;
	alphabets[8].maxS[0].Pitch = -25.0f;
		
	alphabets[8].minS[1].Roll = -80.0f;
	alphabets[8].maxS[1].Roll = -20.0f;
	alphabets[8].minS[1].Pitch = -100.0f;
	alphabets[8].maxS[1].Pitch = -40.0f;

	alphabets[8].minS[2].Roll = -110.0f;
	alphabets[8].maxS[2].Roll = -40.0f;
	alphabets[8].minS[2].Pitch = -60.0f;
	alphabets[8].maxS[2].Pitch = 0.0f; 
	 
	alphabets[8].minS[3].Roll = -80.0f;
	alphabets[8].maxS[3].Roll = -30.0f;
	alphabets[8].minS[3].Pitch = -100.0f;
	alphabets[8].maxS[3].Pitch = 100.0f;
		
	alphabets[8].minS[4].Roll = 55.0f;
	alphabets[8].maxS[4].Roll = 95.0f;
	alphabets[8].minS[4].Pitch = -180.0f;
	alphabets[8].maxS[4].Pitch = 180.0f;

	
	/*----------------J----------------*/
	alphabets[9].minS[0].Roll = 55.0f;
	alphabets[9].maxS[0].Roll = 95.0f;
	alphabets[9].minS[0].Pitch = 30.0f;
	alphabets[9].maxS[0].Pitch = 70.0f;
		
	alphabets[9].minS[1].Roll = -50.0f;
	alphabets[9].maxS[1].Roll = 50.0f;
	alphabets[9].minS[1].Pitch = -100.0f;
	alphabets[9].maxS[1].Pitch = 100.0f;

	alphabets[9].minS[2].Roll = -50.0f;
	alphabets[9].maxS[2].Roll = 50.0f;
	alphabets[9].minS[2].Pitch = 100.0f;
	alphabets[9].maxS[2].Pitch = 180.0f; 
	 
	alphabets[9].minS[3].Roll = -50.0f;
	alphabets[9].maxS[3].Roll = 50.0f;
	alphabets[9].minS[3].Pitch = 50.0f;
	alphabets[9].maxS[3].Pitch = 100.0f;
		
	alphabets[9].minS[4].Roll = -30.0f;
	alphabets[9].maxS[4].Roll = 30.0f;
	alphabets[9].minS[4].Pitch = 100.0f;
	alphabets[9].maxS[4].Pitch = 180.0f;

	
	/*----------------K----------------*/
	alphabets[10].minS[0].Roll = 30.0f;
	alphabets[10].maxS[0].Roll = 70.0f;
	alphabets[10].minS[0].Pitch = -180.0f;
	alphabets[10].maxS[0].Pitch = 180.0f;
		
	alphabets[10].minS[1].Roll = 20.0f;
	alphabets[10].maxS[1].Roll = 50.0f;
	alphabets[10].minS[1].Pitch = -30.0f;
	alphabets[10].maxS[1].Pitch = 30.0f;

	alphabets[10].minS[2].Roll = -50.0f;
	alphabets[10].maxS[2].Roll = 50.0f;
	alphabets[10].minS[2].Pitch = 50.0f;
	alphabets[10].maxS[2].Pitch = 120.0f; 
	 
	alphabets[10].minS[3].Roll = -70.0f;
	alphabets[10].maxS[3].Roll = -30.0f;
	alphabets[10].minS[3].Pitch = 50.0f;
	alphabets[10].maxS[3].Pitch = 180.0f;
		
	alphabets[10].minS[4].Roll = -50.0f;
	alphabets[10].maxS[4].Roll = -10.0f;
	alphabets[10].minS[4].Pitch = 50.0f;
	alphabets[10].maxS[4].Pitch = 180.0f;

	
	/*----------------L----------------*/
	alphabets[11].minS[0].Roll = 10.0f;
	alphabets[11].maxS[0].Roll = 40.0f;
	alphabets[11].minS[0].Pitch = -180.0f;
	alphabets[11].maxS[0].Pitch = -50.0f;
		
	alphabets[11].minS[1].Roll = 50.0f;
	alphabets[11].maxS[1].Roll = 90.0f;
	alphabets[11].minS[1].Pitch = -50.0f;
	alphabets[11].maxS[1].Pitch = 50.0f;

	alphabets[11].minS[2].Roll = -110.0f;
	alphabets[11].maxS[2].Roll = -50.0f;
	alphabets[11].minS[2].Pitch = -180.0f;
	alphabets[11].maxS[2].Pitch = -50.0f; 
	 
	alphabets[11].minS[3].Roll = -100.0f;
	alphabets[11].maxS[3].Roll = -50.0f;
	alphabets[11].minS[3].Pitch = 50.0f;
	alphabets[11].maxS[3].Pitch = 180.0f;
		
	alphabets[11].minS[4].Roll = -80.0f;
	alphabets[11].maxS[4].Roll = -40.0f;
	alphabets[11].minS[4].Pitch = 50.0f;
	alphabets[11].maxS[4].Pitch = 180.0f;

	
	/*----------------M----------------*/
	alphabets[12].minS[0].Roll = -80.0f;
	alphabets[12].maxS[0].Roll = -10.0f;
	alphabets[12].minS[0].Pitch = 40.0f;
	alphabets[12].maxS[0].Pitch = -40.0f;
		
	alphabets[12].minS[1].Roll = -20.0f;
	alphabets[12].maxS[1].Roll = 20.0f;
	alphabets[12].minS[1].Pitch = 20.0f;
	alphabets[12].maxS[1].Pitch = 60.0f;

	alphabets[12].minS[2].Roll = -30.0f;
	alphabets[12].maxS[2].Roll = 30.0f;
	alphabets[12].minS[2].Pitch = 50.0f;
	alphabets[12].maxS[2].Pitch = 150.0f; 
	 
	alphabets[12].minS[3].Roll = -20.0f;
	alphabets[12].maxS[3].Roll = 20.0f;
	alphabets[12].minS[3].Pitch = 50.0f;
	alphabets[12].maxS[3].Pitch = 100.0f;
		
	alphabets[12].minS[4].Roll = -30.0f;
	alphabets[12].maxS[4].Roll = 30.0f;
	alphabets[12].minS[4].Pitch = 100.0f;
	alphabets[12].maxS[4].Pitch = 150.0f;

	
	/*----------------N----------------*/
	alphabets[13].minS[0].Roll = -70.0f;
	alphabets[13].maxS[0].Roll = 0.0f;
	alphabets[13].minS[0].Pitch = -40.0f;
	alphabets[13].maxS[0].Pitch = 20.0f;
		
	alphabets[13].minS[1].Roll = -20.0f;
	alphabets[13].maxS[1].Roll = 20.0f;
	alphabets[13].minS[1].Pitch = 20.0f;
	alphabets[13].maxS[1].Pitch = 60.0f;

	alphabets[13].minS[2].Roll = -30.0f;
	alphabets[13].maxS[2].Roll = 30.0f;
	alphabets[13].minS[2].Pitch = 80.0f;
	alphabets[13].maxS[2].Pitch = 120.0f; 
	 
	alphabets[13].minS[3].Roll = -40.0f;
	alphabets[13].maxS[3].Roll = 0.0f;
	alphabets[13].minS[3].Pitch = 70.0f;
	alphabets[13].maxS[3].Pitch = 120.0f;
		
	alphabets[13].minS[4].Roll = -30.0f;
	alphabets[13].maxS[4].Roll = 30.0f;
	alphabets[13].minS[4].Pitch = 90.0f;
	alphabets[13].maxS[4].Pitch = 130.0f;

	
	/*----------------O----------------*/
	alphabets[14].minS[0].Roll = 25.0f;
	alphabets[14].maxS[0].Roll = 55.0f;
	alphabets[14].minS[0].Pitch = -120.0f;
	alphabets[14].maxS[0].Pitch = -40.0f;
		
	alphabets[14].minS[1].Roll = -40.0f;
	alphabets[14].maxS[1].Roll = -20.0f;
	alphabets[14].minS[1].Pitch = -50.0f;
	alphabets[14].maxS[1].Pitch = 0.0f;

	alphabets[14].minS[2].Roll = -80.0f;
	alphabets[14].maxS[2].Roll = -40.0f;
	alphabets[14].minS[2].Pitch = -50.0f;
	alphabets[14].maxS[2].Pitch = 50.0f; 
	 
	alphabets[14].minS[3].Roll = -80.0f;
	alphabets[14].maxS[3].Roll = -30.0f;
	alphabets[14].minS[3].Pitch = -50.0f;
	alphabets[14].maxS[3].Pitch = 50.0f;
		
	alphabets[14].minS[4].Roll = -80.0f;
	alphabets[14].maxS[4].Roll = -10.0f;
	alphabets[14].minS[4].Pitch = 20.0f;
	alphabets[14].maxS[4].Pitch = 80.0f;

	
	/*----------------P----------------*/
	alphabets[15].minS[0].Roll = -70.0f;
	alphabets[15].maxS[0].Roll = -30.0f;
	alphabets[15].minS[0].Pitch = -180.0f;
	alphabets[15].maxS[0].Pitch = 180.0f;
		
	alphabets[15].minS[1].Roll = -75.0f;
	alphabets[15].maxS[1].Roll = -25.0f;
	alphabets[15].minS[1].Pitch = -180.0f;
	alphabets[15].maxS[1].Pitch = 180.0f;

	alphabets[15].minS[2].Roll = -75.0f;
	alphabets[15].maxS[2].Roll = -25.0f;
	alphabets[15].minS[2].Pitch = -180.0f;
	alphabets[15].maxS[2].Pitch = 180.0f; 
	 
	alphabets[15].minS[3].Roll = -50.0f;
	alphabets[15].maxS[3].Roll = 50.0f;
	alphabets[15].minS[3].Pitch = -180.0f;
	alphabets[15].maxS[3].Pitch = 180.0f;
		
	alphabets[15].minS[4].Roll = -40.0f;
	alphabets[15].maxS[4].Roll = 40.0f;
	alphabets[15].minS[4].Pitch = -180.0f;
	alphabets[15].maxS[4].Pitch = 180.0f;

	
	/*----------------Q----------------*/
	alphabets[16].minS[0].Roll = -90.0f;
	alphabets[16].maxS[0].Roll = 90.0f;
	alphabets[16].minS[0].Pitch = -180.0f;
	alphabets[16].maxS[0].Pitch = 180.0f;
		
	alphabets[16].minS[1].Roll = -110.0f;
	alphabets[16].maxS[1].Roll = -70.0f;
	alphabets[16].minS[1].Pitch = -50.0f;
	alphabets[16].maxS[1].Pitch = 50.0f;

	alphabets[16].minS[2].Roll = 30.0f;
	alphabets[16].maxS[2].Roll = 80.0f;
	alphabets[16].minS[2].Pitch = -180.0f;
	alphabets[16].maxS[2].Pitch = 180.0f; 
	 
	alphabets[16].minS[3].Roll = 30.0f;
	alphabets[16].maxS[3].Roll = 80.0f;
	alphabets[16].minS[3].Pitch = -180.0f;
	alphabets[16].maxS[3].Pitch = 180.0f;
		
	alphabets[16].minS[4].Roll = 30.0f;
	alphabets[16].maxS[4].Roll = 80.0f;
	alphabets[16].minS[4].Pitch = -180.0f;
	alphabets[16].maxS[4].Pitch = 180.0f;

	
	/*----------------R----------------*/
	alphabets[17].minS[0].Roll = 30.0f;
	alphabets[17].maxS[0].Roll = 60.0f;
	alphabets[17].minS[0].Pitch = -100.0f;
	alphabets[17].maxS[0].Pitch = 0.0f;
		
	alphabets[17].minS[1].Roll = 40.0f;
	alphabets[17].maxS[1].Roll = 80.0f;
	alphabets[17].minS[1].Pitch = -50.0f;
	alphabets[17].maxS[1].Pitch = 50.0f;

	alphabets[17].minS[2].Roll = 25.0f;
	alphabets[17].maxS[2].Roll = 75.0f;
	alphabets[17].minS[2].Pitch = -20.0f;
	alphabets[17].maxS[2].Pitch = 40.0f; 
	 
	alphabets[17].minS[3].Roll = -100.0f;
	alphabets[17].maxS[3].Roll = -50.0f;
	alphabets[17].minS[3].Pitch = -180.0f;
	alphabets[17].maxS[3].Pitch = 180.0f;
		
	alphabets[17].minS[4].Roll = -80.0f;
	alphabets[17].maxS[4].Roll = -40.0f;
	alphabets[17].minS[4].Pitch = -180.0f;
	alphabets[17].maxS[4].Pitch = 180.0f;

	
	/*----------------S----------------*/
	alphabets[18].minS[0].Roll = 30.0f;
	alphabets[18].maxS[0].Roll = 70.0f;
	alphabets[18].minS[0].Pitch = -70.0f;
	alphabets[18].maxS[0].Pitch = 30.0f;
		
	alphabets[18].minS[1].Roll = -90.0f;
	alphabets[18].maxS[1].Roll = -40.0f;
	alphabets[18].minS[1].Pitch = -180.0f;
	alphabets[18].maxS[1].Pitch = 0.0f;

	alphabets[18].minS[2].Roll = -90.0f;
	alphabets[18].maxS[2].Roll = -60.0f;
	alphabets[18].minS[2].Pitch = -180.0f;
	alphabets[18].maxS[2].Pitch = 180.0f; 
	 
	alphabets[18].minS[3].Roll = -90.0f;
	alphabets[18].maxS[3].Roll = -50.0f;
	alphabets[18].minS[3].Pitch = -180.0f;
	alphabets[18].maxS[3].Pitch = 180.0f;
		
	alphabets[18].minS[4].Roll = -90.0f;
	alphabets[18].maxS[4].Roll = -50.0f;
	alphabets[18].minS[4].Pitch = -180.0f;
	alphabets[18].maxS[4].Pitch = 180.0f;

	
	/*----------------T----------------*/
	alphabets[19].minS[0].Roll = 30.0f;
	alphabets[19].maxS[0].Roll = 90.0f;
	alphabets[19].minS[0].Pitch = -180.0f;
	alphabets[19].maxS[0].Pitch = 180.0f;
	
	alphabets[19].minS[1].Roll = -40.0f;
	alphabets[19].maxS[1].Roll = 40.0f;
	alphabets[19].minS[1].Pitch = -80.0f;
	alphabets[19].maxS[1].Pitch = -30.0f;

	alphabets[19].minS[2].Roll = 30.0f;
	alphabets[19].maxS[2].Roll = 70.0f;
	alphabets[19].minS[2].Pitch = 0.0f;
	alphabets[19].maxS[2].Pitch = 50.0f; 
	 
	alphabets[19].minS[3].Roll = 50.0f;
	alphabets[19].maxS[3].Roll = 90.0f;
	alphabets[19].minS[3].Pitch = -100.0f;
	alphabets[19].maxS[3].Pitch = 100.0f;
		
	alphabets[19].minS[4].Roll = 40.0f;
	alphabets[19].maxS[4].Roll = 90.0f;
	alphabets[19].minS[4].Pitch = -100.0f;
	alphabets[19].maxS[4].Pitch = 180.0f;

	
	/*----------------U----------------*/
	alphabets[20].minS[0].Roll = 15.0f;
	alphabets[20].maxS[0].Roll = 45.0f;
	alphabets[20].minS[0].Pitch = -65.0f;
	alphabets[20].maxS[0].Pitch = -15.0f;
		
	alphabets[20].minS[1].Roll = 70.0f;
	alphabets[20].maxS[1].Roll = 90.0f;
	alphabets[20].minS[1].Pitch = -40.0f;
	alphabets[20].maxS[1].Pitch = 40.0f;

	alphabets[20].minS[2].Roll = 55.0f;
	alphabets[20].maxS[2].Roll = 75.0f;
	alphabets[20].minS[2].Pitch = 40.0f;
	alphabets[20].maxS[2].Pitch = 70.0f; 
	 
	alphabets[20].minS[3].Roll = -90.0f;
	alphabets[20].maxS[3].Roll = -50.0f;
	alphabets[20].minS[3].Pitch = -180.0f;
	alphabets[20].maxS[3].Pitch = 180.0f;
		
	alphabets[20].minS[4].Roll = -90.0f;
	alphabets[20].maxS[4].Roll = 0.0f;
	alphabets[20].minS[4].Pitch = -180.0f;
	alphabets[20].maxS[4].Pitch = 180.0f;

	
	/*----------------V----------------*/
	alphabets[21].minS[0].Roll = 20.0f;
	alphabets[21].maxS[0].Roll = 50.0f;
	alphabets[21].minS[0].Pitch = -60.0f;
	alphabets[21].maxS[0].Pitch = -20.0f;
		
	alphabets[21].minS[1].Roll = 70.0f;
	alphabets[21].maxS[1].Roll = 90.0f;
	alphabets[21].minS[1].Pitch = -180.0f;
	alphabets[21].maxS[1].Pitch = -20.0f;

	alphabets[21].minS[2].Roll = 40.0f;
	alphabets[21].maxS[2].Roll = 70.0f;
	alphabets[21].minS[2].Pitch = 70.0f;
	alphabets[21].maxS[2].Pitch = 100.0f; 
	 
	alphabets[21].minS[3].Roll = -90.0f;
	alphabets[21].maxS[3].Roll = -50.0f;
	alphabets[21].minS[3].Pitch = -180.0f;
	alphabets[21].maxS[3].Pitch = 180.0f;
		
	alphabets[21].minS[4].Roll = -90.0f;
	alphabets[21].maxS[4].Roll = -40.0f;
	alphabets[21].minS[4].Pitch = -180.0f;
	alphabets[21].maxS[4].Pitch = 180.0f;

	
	/*----------------W----------------*/
	alphabets[22].minS[0].Roll = 15.0f;
	alphabets[22].maxS[0].Roll = 45.0f;
	alphabets[22].minS[0].Pitch = -40.0f;
	alphabets[22].maxS[0].Pitch = 20.0f;
		
	alphabets[22].minS[1].Roll = 60.0f;
	alphabets[22].maxS[1].Roll = 90.0f;
	alphabets[22].minS[1].Pitch = -120.0f;
	alphabets[22].maxS[1].Pitch = -60.0f;

	alphabets[22].minS[2].Roll = 50.0f;
	alphabets[22].maxS[2].Roll = 90.0f;
	alphabets[22].minS[2].Pitch = 60.0f;
	alphabets[22].maxS[2].Pitch = 100.0f; 
	 
	alphabets[22].minS[3].Roll = 50.0f;
	alphabets[22].maxS[3].Roll = 90.0f;
	alphabets[22].minS[3].Pitch = 0.0f;
	alphabets[22].maxS[3].Pitch = 90.0f;
		
	alphabets[22].minS[4].Roll = -90.0f;
	alphabets[22].maxS[4].Roll = -50.0f;
	alphabets[22].minS[4].Pitch = -180.0f;
	alphabets[22].maxS[4].Pitch = 180.0f;

	
	/*----------------X----------------*/
	alphabets[23].minS[0].Roll = -50.0f;
	alphabets[23].maxS[0].Roll = 90.0f;
	alphabets[23].minS[0].Pitch = -60.0f;
	alphabets[23].maxS[0].Pitch = -20.0f;
		
	alphabets[23].minS[1].Roll = 0.0f;
	alphabets[23].maxS[1].Roll = 60.0f;
	alphabets[23].minS[1].Pitch = -70.0f;
	alphabets[23].maxS[1].Pitch = -10.0f;

	alphabets[23].minS[2].Roll = -50.0f;
	alphabets[23].maxS[2].Roll = 50.0f;
	alphabets[23].minS[2].Pitch = 25.0f;
	alphabets[23].maxS[2].Pitch = 65.0f; 
	 
	alphabets[23].minS[3].Roll = -90.0f;
	alphabets[23].maxS[3].Roll = -40.0f;
	alphabets[23].minS[3].Pitch = -180.0f;
	alphabets[23].maxS[3].Pitch = 180.0f;
		
	alphabets[23].minS[4].Roll = -90.0f;
	alphabets[23].maxS[4].Roll = -40.0f;
	alphabets[23].minS[4].Pitch = -180.0f;
	alphabets[23].maxS[4].Pitch = 180.0f;

	
	/*----------------Y----------------*/
	alphabets[24].minS[0].Roll = 0.0f;
	alphabets[24].maxS[0].Roll = 90.0f;
	alphabets[24].minS[0].Pitch = -180.0f;
	alphabets[24].maxS[0].Pitch = -50.0f;
		
	alphabets[24].minS[1].Roll = -90.0f;
	alphabets[24].maxS[1].Roll = -40.0f;
	alphabets[24].minS[1].Pitch = -180.0f;
	alphabets[24].maxS[1].Pitch = 180.0f;

	alphabets[24].minS[2].Roll = -90.0f;
	alphabets[24].maxS[2].Roll = -50.0f;
	alphabets[24].minS[2].Pitch = -180.0f;
	alphabets[24].maxS[2].Pitch = 180.0f; 
	 
	alphabets[24].minS[3].Roll = -80.0f;
	alphabets[24].maxS[3].Roll = -30.0f;
	alphabets[24].minS[3].Pitch = -180.0f;
	alphabets[24].maxS[3].Pitch = 180.0f;
		
	alphabets[24].minS[4].Roll = 40.0f;
	alphabets[24].maxS[4].Roll = 80.0f;
	alphabets[24].minS[4].Pitch = 0.0f;
	alphabets[24].maxS[4].Pitch = 180.0f;

	
	/*----------------Z----------------*/
	alphabets[25].minS[0].Roll = -40.0f;
	alphabets[25].maxS[0].Roll = 40.0f;
	alphabets[25].minS[0].Pitch = -180.0f;
	alphabets[25].maxS[0].Pitch = 0.0f;
		
	alphabets[25].minS[1].Roll = -25.0f;
	alphabets[25].maxS[1].Roll = 25.0f;
	alphabets[25].minS[1].Pitch = -180.0f;
	alphabets[25].maxS[1].Pitch = 180.0f;

	alphabets[25].minS[2].Roll = -60.0f;
	alphabets[25].maxS[2].Roll = 60.0f;
	alphabets[25].minS[2].Pitch = -180.0f;
	alphabets[25].maxS[2].Pitch = 180.0f; 
	 
	alphabets[25].minS[3].Roll = -60.0f;
	alphabets[25].maxS[3].Roll = 60.0f;
	alphabets[25].minS[3].Pitch = 0.0f;
	alphabets[25].maxS[3].Pitch = 180.0f;
		
	alphabets[25].minS[4].Roll = -60.0f;
	alphabets[25].maxS[4].Roll = 60.0f;
	alphabets[25].minS[4].Pitch = 0.0f;
	alphabets[25].maxS[4].Pitch = 180.0f;
	}

// compares our reading with defined reading for each character.
void MPU9250_Gesture_Scan()
{
	//Scan Current Alphabets for Matching Pattern
	for(m = 0; m < ALPHA_COUNT; m++)
	{
		for(n = 0; n < SEN_COUNT; n++)
		{
			MPU9250_Sensor_Scan(n);
		}

//		printf("Scan Match: %d\n", scan_match);
		
		if(scan_match == SEN_COUNT)
		{
			char Gesture = MPU9250_Gesture_Select(m);
			#ifdef GESTURE_SCAN
				printf("The Gesture is: %c\n", Gesture);
			#endif
		}
/*
		else
		{
			printf("\rThe Gesture is: -");
		}
*/
		scan_match = 0;
	}
}

//Scan all Sensors
void MPU9250_Sensor_Scan(int sen_count)
{
	if((s[n].Roll > alphabets[m].minS[n].Roll && 
	  	s[n].Roll < alphabets[m].maxS[n].Roll) &&
	   (s[n].Pitch > alphabets[m].minS[n].Pitch && 
		s[n].Pitch < alphabets[m].maxS[n].Pitch))
	{
		++scan_match;
  	}
}

//Return the Detected Gesture
char MPU9250_Gesture_Select(uint8_t gesture)
{
	switch(gesture)
	{
		case 0:
			return 'A';
			break;
		case 1:
			return 'B';
			break;
		case 2:
			return 'C';
			break;
		case 3:
			return 'D';
			break;
		case 4:
			return 'E';
			break;
		case 5:
			return 'F';
			break;
		case 6:
			return 'G';
			break;
		case 7:
			return 'H';
			break;
		case 8:
			return 'I';
			break;
		case 9:
			return 'J';
			break;
		case 10:
			return 'K';
			break;
		case 11:
			return 'L';
			break;
		case 12:
			return 'M';
			break;
		case 13:
			return 'N';
			break;
		case 14:
			return 'O';
			break;
		case 15:
			return 'P';
			break;
		case 16:
			return 'Q';
			break;
		case 17:
			return 'R';
			break;
		case 18:
			return 'S';
			break;
		case 19:
			return 'T';
			break;
		case 20:
			return 'U';
			break;
		case 21:
			return 'V';
			break;
		case 22:
			return 'W';
			break;
		case 23:
			return 'X';
			break;
		case 24:
			return 'Y';
			break;
		case 25:
			return 'Z';
			break;
		default :
			return '_';
			break;
			
				
	}
}