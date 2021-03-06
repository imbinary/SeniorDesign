/*
 * util.h
 *
 *  Created on: Aug 10, 2015
 *      Author: chris
 */

#ifndef UTIL_H_
#define UTIL_H_

typedef struct Vector {
	float x;
	float y;
} Vector;

typedef struct Intersection {
	Vector intersectPoint;
	bool linesAreParallel;
	float parameter1, parameter2;
}Intersection;



/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  Function prototypes                                           :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

char** str_split(char* a_str, const char a_delim);
char *strsep(char* line, const char* delims);
int sstr_split(char result[][25],char* a_str, const char a_delim);
double deg2rad(double);
double rad2deg(double);
double distance(double lat1, double lon1, double lat2, double lon2, char unit);
int16_t direction(double lat1, double lon1, double lat2, double lon2, char unit);
double deg2dec(double deg);
int8_t nmea_validateChecksum(char *strPtr, uint16_t bufSize);
const char * nmea_generateChecksum(char *strPtr, char *dstStr);
Intersection intersectVectors(Vector line1Start, Vector line1Dir,
		Vector line2Start, Vector line2Dir);
void delay(int milliseconds);
int reset();
#endif /* UTIL_H_ */
