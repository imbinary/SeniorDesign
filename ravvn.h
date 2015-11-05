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
	uint16_t longAccel;
	uint16_t latAccel;
	uint16_t vertAccel;
	float yawRate;
	bool brake;
	uint16_t length;
	uint16_t width;
    float time;
    int date;

} rBSMData_t;

struct AMessage
{
    char ucMessageID;
    int8_t dir;
    int8_t size;
    int8_t color;
    uint16_t time;
} xMessage;


#define  DTYPE  1    // 1 for vehicle, 0 for infrastructure

#endif /* RAVVN_H_ */

extern rBSMData_t g_rBSMData;
//extern xMessage pxMessage;
