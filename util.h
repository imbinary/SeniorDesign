/*
 * util.h
 *
 *  Created on: Aug 10, 2015
 *      Author: chris
 */

#ifndef UTIL_H_
#define UTIL_H_

char** str_split(char* a_str, const char a_delim);
char *strsep(char* line, const char* delims);
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  Function prototypes                                           :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double deg2rad(double);
double rad2deg(double);
double distance(double lat1, double lon1, double lat2, double lon2, char unit);
int16_t direction(double lat1, double lon1, double lat2, double lon2, char unit);
double deg2dec(double deg);
int8_t nmea_validateChecksum(char *strPtr, uint16_t bufSize);
const char * nmea_generateChecksum(char *strPtr, char *dstStr);

#endif /* UTIL_H_ */
