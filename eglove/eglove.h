#ifndef EGLOVE_H
#define EGLOVE_H

/*---------------Global Macros---------------*/
#define SEN_COUNT    5 //No. of Sensors
#define PI           3.14159265358979323846f

//Select the Print Variable

/*
RAW_DATA
RAW_ANGLES
COMPLEMENTARY_ANGLES
EULER_ANGLES
LOOP_TIME
*/

//#define RAW_ANGLES
//#define RAW_DATA
//#define COMPLEMENTARY_ANGLES
#define EULER_ANGLES
#define LOOP_TIME

//Kalman Filter Constants
#define Q_ANGLE   0.001f //Process Noise Variance for the Accelerometer
#define Q_BIAS    0.003f //Process Noise Variance for the Gyro Bias
#define R_MEASURE 0.03f  //Measurement Noise Variance

//ANSI Colour Codes       
#define ANSI_COLOUR_RED          "\x1b[31m"
#define ANSI_COLOUR_GREEN        "\x1b[32m"
#define ANSI_COLOUR_YELLOW       "\x1b[33m"
#define ANSI_COLOUR_BLUE         "\x1b[34m"
#define ANSI_COLOUR_MAGENTA      "\x1b[35m"
#define ANSI_COLOUR_CYAN         "\x1b[36m"
#define ANSI_COLOUR_RESET        "\x1b[0m"

#define ANSI_COLOUR_RED_BOLD     "\033[1m\033[31m"      /* Bold Red */
#define ANSI_COLOUR_GREEN_BOLD   "\033[1m\033[32m"      /* Bold Green */
#define ANSI_COLOUR_YELLOW_BOLD  "\033[1m\033[33m"      /* Bold Yellow */
#define ANSI_COLOUR_BLUE_BOLD    "\033[1m\033[34m"      /* Bold Blue */
#define ANSI_COLOUR_MAGENTA_BOLD "\033[1m\033[35m"      /* Bold Magenta */
#define ANSI_COLOUR_CYAN_BOLD    "\033[1m\033[36m"      /* Bold Cyan */
#define ANSI_COLOUR_WHITE_BOLD   "\033[1m\033[37m"      /* Bold White */

/*-------------MPU9250 Registers-------------*/
//Self Test registers R/W
#define SELF_TEST_X  0x0D
#define SELF_TEST_Y  0x0E
#define SELF_TEST_Z  0x0F
#define SELF_TEST_A  0x0A

//Configuration Registers
#define SMPRT_DIV    0x19 //Sample Rate Divider
#define CONFIG       0x1A //FSYNC & DLPF config
#define GYRO_CONFIG  0x1B //Self-Test & Scale select
#define ACCEL_CONFIG 0x1C //Self-Test & Scale select

//Accelerometer Measurement Registers
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

//Temperature Measurement Registers
//Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 35
#define TEMP_OUT_H   0x41
#define TEMP_OUT_L   0x42

//Gyroscope Measurement Registers
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48

//Power Management Registers
#define PWR_MGMT_1   0x6B
#define PWR_MGMT_2   0x6C

//Device i.d. Register
#define WHO_AM_I     0x75

/*
R1 - 0x69
R2 - 0x68
*/
#define MPU_ADDR     0x68

//Reset the Registers and Power
#define PWR_RESET    0x80
#define DEVICE_ON    0x01

//Accelerometer Scale
#define ACCEL_2G     0x00
#define ACCEL_4G     0x08
#define ACCEL_8G     0x10
#define ACCEL_16G    0x18

//Gyroscope Scale
#define GYRO_250     0x00
#define GYRO_500     0x08
#define GYRO_1000    0x10
#define GYRO_2000    0x18

//Sample Rate @25Hz
#define SAMPLE_RATE  0x07

//5Hz LPF
#define DLPF_CFG     0x00

void MPU9250_GPIO_Init();
void MPU9250_I2C_Init();
void MPU9250_MUX_Select(uint8_t mux_channel);
void MPU9250_I2C_Config(uint8_t sen_count);
void MPU9250_I2C_Write(uint8_t address, uint8_t value);
void MPU9250_I2C_Read(uint8_t address, uint8_t *value);
int16_t MPU9250_Get_Measurement(uint8_t addrL, uint8_t addrH);
void MPU9250_Loop();
void MPU9250_Filter_Init();
void MPU9250_Kalman_Get_Angle(double newAngle);
double MPU9250_Kalman_Update(double newAngle, double newRate, double dt);
void MPU9250_Text_Colour(uint8_t mux_channel);
void sig_handler(int signum);

//Acceleometer Measurement Storage Variables
double Accel_X = 0.0;
double Accel_Y = 0.0;
double Accel_Z = 0.0;

//Gyroscope Measurement Storage Variables
double Gyro_X = 0.0;
double Gyro_Y = 0.0;
double Gyro_Z = 0.0;

//Gyroscope Measurements in deg/s
double Gyro_X_Rate = 0.0f;
double Gyro_Y_Rate = 0.0f;

//Temperature Measurement Storage Variable
float Temperature = 0.0f;

//Variable to Store LOW and HIGH Register values
uint8_t Measurement_L = 0;
uint8_t Measurement_H = 0;

//Kalman Filter Variables
double Angle = 0.0f; //The Angle calculated by the Kalman Filter - Part of the 2x1 State Vector
double Bias = 0.0f;  //The Gyro Bias calculated by the Kalman Filter - Part of the 2x1 State Vector
double Rate = 0.0f;  //UnBiased Rate calculated from the Rate and the calculated Bias

double dt = 0.0f;    //Delta-Time, time difference between each filter update

double P[2][2]; //Error Covariance Matrix - This is a 2x2 Matrix
double K[2];    //Kalman Gain - This is a 2x1 Vector
double y;       //Angle Difference
double S;       //Estimate Error

//Structure to Store Raw Angles
struct Raw_Angles
{
	double Raw_X;
	double Raw_Y;	
}r[SEN_COUNT];

//Structure to store Complementary Filter Angles
struct Complementary_Filter_Angles 
{
	double C_X;
	double C_Y;	
}c[SEN_COUNT];

//Structure to Store Kalman Filter Angles
struct Kalman_Filter_Angles
{
	double K_X;
	double K_Y;	
}k[SEN_COUNT];

//Variable to Store Euler Angles
double Roll  = 0.0;
double Pitch = 0.0;

//Structure to Store Euler Angles
struct Euler
{
	double Roll;
	double Pitch;
}s[SEN_COUNT];

//Array to Store ANSI Colour
char Ansi_Colour[10];
char Ansi_Colour_Bold[10];

//Loop Count variables
uint8_t i = 0;
uint8_t Init_Count = 0;

//Timer Variables
struct timeval tv;
uint64_t timer1, timer2;

sig_atomic_t volatile isrunning = 1;

//MRAA variables  
mraa_gpio_context S0;
mraa_gpio_context S1;
mraa_gpio_context S2;
mraa_gpio_context S3;
mraa_i2c_context MPU9250_i2c;

#endif