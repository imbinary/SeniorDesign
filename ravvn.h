/*
 * ravvn.h
 *
 *  Created on: Aug 26, 2015
 *      Author: Chris
 */

#ifndef RAVVN_H_
#define RAVVN_H_

//*****************************************************************************
//
// Structure to hold the BSM data and provide access to other tasks.
//
//*****************************************************************************
typedef struct rBSMDataStruct
{
	float latitude;
	float longitude;
	int16_t elevation;
	float pAccuracy;
	float speed;
	uint16_t heading;
	uint16_t steeringAngle;
	int16_t longAccel;
	int16_t latAccel;
	int16_t vertAccel;
	float yawRate;
	bool brake;
	uint16_t length;
	uint16_t width;
    float btime;
    int date;
    int ID;
} rBSMData_t;



//todo toggle for device type
#define  DTYPE  0    // 1 for vehicle, 0 for infrastructure
#define ADXL 0
#define DEMO 1
#define DEVID 1
#endif /* RAVVN_H_ */

extern rBSMData_t g_rBSMData;

