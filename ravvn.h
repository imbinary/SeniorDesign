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
	float latitiude;
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

} rBSMData_t;




#define  DTYPE  1    // 1 for vehicle, 0 for infrastructure

#endif /* RAVVN_H_ */

extern rBSMData_t g_rBSMData;

