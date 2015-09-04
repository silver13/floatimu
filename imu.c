/*

this file is a replacement imu for the bradwii port for hubsan 
and other quads based on the mini54 cpu.
  
by silverx 2015

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// library headers
#include <stdbool.h>
#include "lib_timers.h"
#include "lib_fp.h"

// project file headers
#include "bradwii.h"
#include "gyro.h"
#include "accelerometer.h"
#include "baro.h"
#include "imu.h"
#include "compass.h"

extern globalstruct global;
extern usersettingsstruct usersettings;



//#define SMALL_ANGLE_APPROX
#ifdef SMALL_ANGLE_APPROX
// Small angle approximation rotation 
// with simple sin and cos
#define ssin(val) (val)
#define scos(val) 1.0f


#else
// version without small angle approximations defines for sin and cos
// SMALL_ANGLE_APPROX not defined

// proper rotation matrix with float sinf and cosf
//#define _sinf(val) sinf(val)
//#define _cosf(val) cosf(val)

// proper rotation matrix with approximations for sin and cos
// smaller size then above version
#define _sinf(val) (val)
#define _cosf(val)  (1 - ((val)*(val))/2) 
#endif

// gravity 1G size from accelerometer
// should be smaller then 3000
// this is the same as the int size

#define ACC_1G 1172.0f


// read the acc and gyro a bunch of times and get an average of how far off they are.
// assumes the aircraft is sitting level and still.
// If both==false, only gyro is calibrated and accelerometer calibration not touched.
void calibrategyroandaccelerometer(bool both)
{
#ifdef X4_BUILD
    uint8_t ledstatus;
#endif
	// do some readings to initialize the gyro filters
	for (int x = 0; x < 15; ++x) {   
	readgyro();
	readacc();
	}
	// start the low pass at the first readout rather then at zero
	 for (int x = 0; x < 3; ++x) {
            usersettings.gyrocalibration[x]= -global.gyrorate[x];
						if(both)
                usersettings.acccalibration[x]= -global.acc_g_vector[x];
		}
	 /*
    for (int x = 0; x < 3; ++x) {
        usersettings.gyrocalibration[x] = 0;
        if(both)
            usersettings.acccalibration[x] = 0;
    }
*/
    fixedpointnum totaltime = 0;

    // calibrate the gyro and acc
    while (totaltime < (FIXEDPOINTCONSTANT(4) << TIMESLIVEREXTRASHIFT)) // 4 seconds
    {
        readgyro();
        if(both) {
            readacc();
            global.acc_g_vector[ZINDEX] -= FIXEDPOINTONE; // vertical vector should be at 1g
        }

        calculatetimesliver();
        totaltime += global.timesliver;
#ifdef X4_BUILD
        // Rotating LED pattern
        ledstatus = (uint8_t)((totaltime >> (FIXEDPOINTSHIFT+TIMESLIVEREXTRASHIFT-3))& 0x3);
        switch(ledstatus) {
        case 0:
            x4_set_leds(X4_LED_FL);
            break;
        case 1:
            x4_set_leds(X4_LED_FR);
            break;
        case 2:
            x4_set_leds(X4_LED_RR);
            break;
        case 3:
            x4_set_leds(X4_LED_RL);
            break;
        }
#endif
        for (int x = 0; x < 3; ++x) {
            lib_fp_lowpassfilter(&usersettings.gyrocalibration[x], -global.gyrorate[x], global.timesliver, FIXEDPOINTONEOVERONE, TIMESLIVEREXTRASHIFT);
            if(both)
                lib_fp_lowpassfilter(&usersettings.acccalibration[x], -global.acc_g_vector[x], global.timesliver, FIXEDPOINTONEOVERONE, TIMESLIVEREXTRASHIFT);
        }
    }
}

static float GEstG[3] = {0 , 0 , ACC_1G};

void initimu(void)
{
    // calibrate both sensors if we didn't load any data from eeprom
    if (global.usersettingsfromeeprom == 0)
        calibrategyroandaccelerometer(true);
    else // only gyro
        calibrategyroandaccelerometer(false);

   global.estimateddownvector[XINDEX] = 0;
   global.estimateddownvector[YINDEX] = 0;
 //   global.estimateddownvector[ZINDEX] = FIXEDPOINTONE;

  //  global.estimatedwestvector[XINDEX] = FIXEDPOINTONE;
  //  global.estimatedwestvector[YINDEX] = 0;
  //  global.estimatedwestvector[ZINDEX] = 0;

// init reference vector with actual gravity vector		
	readacc();
	for ( int i = 0 ; i < 2;i++)
	{
	global.acc_g_vector[i] = global.acc_g_vector[i] + usersettings.acccalibration[i];
  GEstG[i] = global.acc_g_vector[i]>>6;
	}
	
//lastbarorawaltitude = global.altitude = global.barorawaltitude;
//global.altitudevelocity = 0;

}


void vectorcopy( float *vector1 , float *vector2);


static unsigned long gp_timer ;

float tofloat( fixedpointnum fp)
{
  float ans =  fp * 1.5258789e-5; // (1/65536)
  return ans;
}


float calcmagnitude( float  vector[3] )
{
 float accmag = 0; 
 for (uint8_t axis = 0; axis < 3; axis++) {
		accmag  +=  vector[axis] * vector[axis];
	}	
	accmag = sqrtf ( accmag  );
	return accmag;
}


void vectorcopy( float *vector1 , float *vector2)
{
	  for (int axis = 0; axis < 3; axis++) {
			vector1[axis]  = vector2[axis];
		}	
}


void imucalculateestimatedattitude(void)
{	 
  float EstG[3];	
	float acc[3]; // holds float accel vector
	float deltatime; // time in seconds
	float gyros[3];
	
vectorcopy ( &EstG[0] , &GEstG[0] );
	 
	 deltatime = (float)lib_timers_gettimermicrosecondsandreset(&gp_timer) * 0.000001f ; // time in seconds
	 
	 deltatime = deltatime* 0.92; // correction factor
																// unknown reason 
    readgyro();
    readacc();

    // correct the gyro and acc readings to remove error      
	 // x & y accel offsets only
    for ( int x = 0; x < 3; ++x) { // was 3
        global.gyrorate[x] = global.gyrorate[x] + usersettings.gyrocalibration[x];
        global.acc_g_vector[x] = global.acc_g_vector[x] + usersettings.acccalibration[x];
				acc[x]= global.acc_g_vector[x]>>6; 
    }

// deadzone for yaw rate	
//global.gyrorate[2] = ( 0 || global.gyrorate[2] > 40000 || global.gyrorate[2]< -40000 ) *  global.gyrorate[2] ;


for ( int i = 0 ; i < 3; i++)
{		
	gyros[i]  = tofloat( global.gyrorate[i] ) * deltatime * 0.01745329;
}
		
#ifndef SMALL_ANGLE_APPROX	
 // This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
	float mat[3][3];
  float tempvect[3];
	float cosx, sinx, cosy, siny, cosz, sinz;
	float coszcosx, coszcosy, sinzcosx, coszsinx, sinzsinx;
vectorcopy ( &tempvect[0] , & EstG[0] );

// the signs are differnt due to different conventions
// for positive/negative angles in various multiwii forks this is based on
	cosx = _cosf( gyros[1]);
	sinx = _sinf( gyros[1]);
	cosy = _cosf( -gyros[0]);
	siny = _sinf( -gyros[0]);
	cosz = _cosf( -gyros[2]);
	sinz = _sinf( -gyros[2]);

	coszcosx = cosz * cosx;
	coszcosy = cosz * cosy;
	sinzcosx = sinz * cosx;
	coszsinx = sinx * cosz;
	sinzsinx = sinx * sinz;

	mat[0][0] = coszcosy;
	mat[0][1] = -cosy * sinz;
	mat[0][2] = siny;
	mat[1][0] = sinzcosx + (coszsinx * siny);
	mat[1][1] = coszcosx - (sinzsinx * siny);
	mat[1][2] = -sinx * cosy;
	mat[2][0] = (sinzsinx) - (coszcosx * siny);
	mat[2][1] = (coszsinx) + (sinzcosx * siny);
	mat[2][2] = cosy * cosx;

	EstG[0] = tempvect[0] * mat[0][0] + tempvect[1] * mat[1][0] + tempvect[2] * mat[2][0];
	EstG[1] = tempvect[0] * mat[0][1] + tempvect[1] * mat[1][1] + tempvect[2] * mat[2][1];
	EstG[2] = tempvect[0] * mat[0][2] + tempvect[1] * mat[1][2] + tempvect[2] * mat[2][2];

#endif // end rotation matrix

#ifdef SMALL_ANGLE_APPROX
// this is a rotation with small angle approximation
  float deltagyroangle;	 // holds float gyro angle in rad
	// Rotate Estimated vector(s), ROLL
  EstG[2] =  scos(gyros[0]) * EstG[2] - ssin(gyros[0]) * EstG[0];
  EstG[0] =  ssin(gyros[0]) * EstG[2] + scos(gyros[0]) * EstG[0];
		
  // Rotate Estimated vector(s), PITCH
  EstG[1] =  scos(gyros[1]) * EstG[1] + ssin(gyros[1]) * EstG[2];
  EstG[2] = -ssin(gyros[1]) * EstG[1] + scos(gyros[1]) * EstG[2];

  // Rotate Estimated vector(s), YAW
  EstG[0] =  scos(gyros[2]) * EstG[0] - ssin(gyros[2]) * EstG[1];
  EstG[1] =  ssin(gyros[2]) * EstG[0] + scos(gyros[2]) * EstG[1];

#endif // end small angle approx

// yaw not tested ( maybe for yaw hold? )
// includes deadzone 
// global.currentestimatedeulerattitude[YAWINDEX] += ( 0 || global.gyrorate[2] > 40000 || global.gyrorate[2]< -40000 ) * ( (float) ( global.gyrorate[2] ) * deltatime + 0.5f);

// yaw without deadzone
// not tested , i am not sure what the yaw angle is even used for
 global.currentestimatedeulerattitude[YAWINDEX] += ( (float) ( global.gyrorate[2] ) * deltatime);

   lib_fp_constrain180(&global.currentestimatedeulerattitude[YAWINDEX]);	


// global.estimateddownvector[ZINDEX] < 0 
// in pilotcontrol.c fix for inverted(not tested)
global.estimateddownvector[ZINDEX] = (EstG[2]>0)? 1111:-1111 ;

// orientation vector magnitude
float mag = 0;
mag = calcmagnitude( &EstG[0] );
	
// normalize orientation vector
if (1) 
	{
	 for (int axis = 0; axis < 3; axis++) 
		{
			EstG[axis] =  EstG[axis] / ( mag / ACC_1G );
		}	
	}
//debug4 = mag;
	
// calc acc mag
float accmag;
	
accmag = calcmagnitude( &acc[0] );	
	
//	debug2 = accmag;

//normvector( acc , accmag, normal);
	// normalize acc
 for (int axis = 0; axis < 3; axis++) 
	{
	  acc[axis] =  acc[axis] / (accmag / ACC_1G) ;
	}	

// test acc mag
//debug5 = calcmagnitude( &acc[0] );
	
/* Set the Gyro Weight for Gyro/Acc complementary filter */
/* Increasing this value would reduce and delay Acc influence on the output of the filter*/
	// times for 3ms loop time
	// filter time changes linearily with loop time
	// 0.970  0.1s
	// 0.988  0.25s
	// 0.994  0.5 s 
	// 0.996  0.75 s
	// 0.997  1.0 sec
	// 0.998  1.5 sec 
	// 0.9985 2 sec
	// 0.999  3 sec
	// 0.99925  4 s
#define GYR_CMPF_FACTOR 0.998f // was 0.998

#define DISABLE_ACC 0

#define ACC_MIN 0.8f
#define ACC_MAX 1.2f 	
static unsigned int count = 0;
	
if ( ( accmag > ACC_MIN * ACC_1G ) && ( accmag < ACC_MAX * ACC_1G ) && !DISABLE_ACC ) 
		{
    //for (axis = 0; axis < 3; axis++)
			if ( count >= 10 ) // loop time = 3ms so 30ms wait
			{
			//x4_set_leds( 0xFF);
			EstG[0] =  EstG[0] * GYR_CMPF_FACTOR + (float)acc[0] * ( 1.0f - GYR_CMPF_FACTOR );
			EstG[1] =  EstG[1] * GYR_CMPF_FACTOR + (float)acc[1] * ( 1.0f - GYR_CMPF_FACTOR );
			EstG[2] =  EstG[2] * GYR_CMPF_FACTOR + (float)acc[2] * ( 1.0f - GYR_CMPF_FACTOR );
			}
			count++;
   }
		else 
		{// acc mag out of bounds
			//x4_set_leds( 0x00);
			count = 0;
		}


 vectorcopy ( &GEstG[0] , &EstG[0]);
	 
// convert our vectors to euler angles
		
	 global.currentestimatedeulerattitude[ROLLINDEX] = lib_fp_atan2(
				FIXEDPOINTCONSTANT(EstG[0]*8 ), 
				FIXEDPOINTCONSTANT(EstG[2]*8 ) ) ;	
/*
	 global.currentestimatedeulerattitude[PITCHINDEX] = lib_fp_atan2(
				FIXEDPOINTCONSTANT( EstG[1]*8),
				FIXEDPOINTCONSTANT( EstG[2]*8) );
*/

    if (lib_fp_abs(global.currentestimatedeulerattitude[ROLLINDEX]) > FIXEDPOINT45 && lib_fp_abs(global.currentestimatedeulerattitude[ROLLINDEX]) < FIXEDPOINT135) {
        global.currentestimatedeulerattitude[PITCHINDEX] = 
					lib_fp_atan2(EstG[1]*8, lib_fp_abs(EstG[0])*8);
   } else {
       global.currentestimatedeulerattitude[PITCHINDEX] = 
						lib_fp_atan2(
						EstG[1]*8, 
						EstG[2]*8);
   }
	
}

