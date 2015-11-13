/*
 * util.c
 *
 *  Created on: Aug 10, 2015
 *      Author: chris
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/uartstdio.h"
#include "util.h"
#include <math.h>

char** str_split(char* a_str, const char a_delim) {
	char** result = 0;
	int8_t count = 0;
	char* tmp = a_str;
	char* last_comma = 0;
	char delim[2];
	delim[0] = a_delim;
	delim[1] = 0;
	//char *mystring;

	/* Count how many elements will be extracted. */
	while (*tmp) {
		if (a_delim == *tmp) {
			count++;
			last_comma = tmp;
		}
		tmp++;
	}
	//if(count==0)
	//	return NULL;
	/* Add space for trailing token. */
	count += last_comma < (a_str + strlen(a_str) - 1);

	/* Add space for terminating null string so caller
	 knows where the list of returned strings ends. */
	count++;

	result = pvPortMalloc(sizeof(char*) * count);

	if (result) {
		size_t idx = 0;
		tmp = a_str;
		char* token = strsep(tmp, delim);

		while (token) {
			assert(idx < count);
			result[idx] = pvPortMalloc(strlen(token) + 1);
			memcpy(result[idx], token, strlen(token) + 1);
			idx++;
			token = strsep(0, delim);
		}
		assert(idx == count - 1);
		result[idx] = 0;
	}
	return result;
}

/*
 * strtok version that handles null fields
 */
char *strsep(char* line, const char* delims) {
	static char *saveline = NULL;
	char *p;
	int n;

	if (line != NULL)
		saveline = line;

	/*
	 *see if we have reached the end of the line
	 */
	if (saveline == NULL || *saveline == '\0')
		return (NULL);
	/*
	 *return the number of characters that aren't delims
	 */
	n = strcspn(saveline, delims);
	p = saveline; /*save start of this token*/

	saveline += n; /*bump past the delim*/

	if (*saveline != '\0') /*trash the delim if necessary*/
		*saveline++ = '\0';

	return (p);
}

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::                                                                         :*/
/*::  This routine calculates the distance between two points (given the     :*/
/*::  latitude/longitude of those points). It is being used to calculate     :*/
/*::  the distance between two locations using GeoDataSource(TM) products.   :*/
/*::                                                                         :*/
/*::  Definitions:                                                           :*/
/*::    South latitudes are negative, east longitudes are positive           :*/
/*::                                                                         :*/
/*::  Passed to function:                                                    :*/
/*::    lat1, lon1 = Latitude and Longitude of point 1 (in decimal degrees)  :*/
/*::    lat2, lon2 = Latitude and Longitude of point 2 (in decimal degrees)  :*/
/*::    unit = the unit you desire for results                               :*/
/*::           where: 'M' is statute miles (default)                         :*/
/*::                  'K' is kilometers                                      :*/
/*::                  'N' is nautical miles                                  :*/
/*::  Worldwide cities and other features databases with latitude longitude  :*/
/*::  are available at http://www.geodatasource.com                          :*/
/*::                                                                         :*/
/*::  For enquiries, please contact sales@geodatasource.com                  :*/
/*::                                                                         :*/
/*::  Official Web site: http://www.geodatasource.com                        :*/
/*::                                                                         :*/
/*::           GeoDataSource.com (C) All Rights Reserved 2015                :*/
/*::                                                                         :*/
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

#define pi 3.14159265358979323846

double distance(double lat1, double lon1, double lat2, double lon2, char unit) {
	double theta, dist;
	theta = lon1 - lon2;
	dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2))
			+ cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
	dist = acos(dist);
	dist = rad2deg(dist);
	dist = dist * 60 * 1.1515;
	switch (unit) {
	case 'M':
		break;
	case 'K':
		dist = dist * 1.609344;
		break;
	case 'N':
		dist = dist * 0.8684;
		break;
	case 'm':
		dist = dist * 1609.344;
		break;
	}
	return (dist);
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts decimal degrees to radians             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double deg2rad(double deg) {
	return (deg * pi / 180);
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts radians to decimal degrees             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double rad2deg(double rad) {
	return (rad * 180 / pi);
}

int16_t direction(double lat1, double lon1, double lat2, double lon2, char unit) {
	double y, x;
	double theta = lon2 - lon1;
	int16_t direction;
	y = sin(deg2rad(theta)) * cos(deg2rad(lat2));
	x = cos(deg2rad(lat1)) * sin(deg2rad(lat2))
			- sin(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
	direction = rad2deg(atan2(y, x));
	if (direction <= 0)
		direction += 360;
	return direction;
}

double deg2dec(double deg) {
	if (deg == 0)
		return 0;
	int intp = deg;  //DDMM
	int min = intp - ((intp / 100) * 100); // MM
	double decp = deg - intp; // .m
	decp = (((double) min + decp)) / 60;
	return ((double) (intp / 100)) + (decp);
	//return deg;
}

// this takes a nmea gps string, and validates it againts the checksum
// at the end if the string. (requires first byte to be $)
int8_t nmea_validateChecksum(char *strPtr, uint16_t bufSize) {
	int p;
	char c;
	uint8_t chksum;
	uint8_t nmeaChk;
	int8_t flagValid;
	char hx[5] = "0x00";

	flagValid = 1; // we start true, and make it false if things are not right

	if (strPtr[0] != '$') {
		flagValid = 0;
	}

	// if we are still good, test all bytes
	if (flagValid == 1) {
		c = strPtr[1]; // get first chr
		chksum = c;
		p = 2;
		while ((c != '*') && (p < bufSize)) {
			c = strPtr[p]; // get next chr
			if (c != '*') {
				chksum = chksum ^ c;
			}
			p++;
		}
		// at this point we are either at * or at end of string
		hx[2] = strPtr[p];
		hx[3] = strPtr[p + 1];
		hx[4] = 0x00;
		nmeaChk = strtol(hx, NULL, 16);
		if (chksum != nmeaChk) {
			flagValid = 0;
		}
	}

	return flagValid;
}

// this returns a single binary byte that is the checksum
// you must convert it to hex if you are going to print it or send it
const char * nmea_generateChecksum(char *strPtr, char *dstStr) {
	int p;
	char c;
	uint8_t chksum;

	c = strPtr[0]; // get first chr
	chksum = c;
	p = 1;
	while (c != 0x00) {
		c = strPtr[p]; // get next chr
		if (c != 0x00) {
			chksum = chksum ^ c;
		}
		p++;
	}
	sprintf(&dstStr[0], "$%s*%02x", strPtr, chksum);
	return dstStr;
}

float dotproduct(Vector vec1, Vector vec2){
	return (vec1.x * vec2.x) + (vec1.y * vec2.y);
}


Vector rotate90CW (Vector vec){
	Vector tmp;
	tmp.x = vec.y;
	tmp.y = vec.x * -1;
   return tmp;
}

Vector rotate90CCW (Vector vec){
	Vector tmp;
	tmp.x = vec.y * -1;
	tmp.y = vec.x;
   return tmp;
}

Vector vectoradd (Vector vec1, Vector vec2){
	Vector tmp;
	tmp.x = vec1.x + vec2.x;
	tmp.y = vec1.y + vec2.y;
	return tmp;
}

Vector vectorsub (Vector vec1, Vector vec2){
	Vector tmp;
	tmp.x = vec1.x - vec2.x;
	tmp.y = vec1.y - vec2.y;
	return tmp;
}

Vector vectormult (Vector vec1, float scalar){
	Vector tmp;
	tmp.x = vec1.x * scalar;
	tmp.y = vec1.y * scalar;
	return tmp;
}

Intersection intersectVectors(Vector line1Start, Vector line1Dir,
		Vector line2Start, Vector line2Dir) {

	Intersection returnValue;
	Vector line2DirPerp = rotate90CW(line2Dir);
	float line1DirDotLine2DirPerp = dotproduct(line1Dir, line2DirPerp);

	if (line1DirDotLine2DirPerp != 0) {
		returnValue.linesAreParallel = false;
		float line2StartToLine1StartDotLine2DirPerp = dotproduct( vectorsub(line1Start, line2Start), line2DirPerp);

		returnValue.parameter1 = line2StartToLine1StartDotLine2DirPerp / -line1DirDotLine2DirPerp;

		// now the same for line 2!
		Vector line1DirPerp = rotate90CW(line1Dir);
		float line2DirDotLine1DirPerp = dotproduct(line2Dir, line1DirPerp);

		// no need to check for parallel vectors, we already checked...
		float line1StartToLine2StartDotLine1DirPerp = dotproduct( vectorsub(line2Start, line1Start), line1DirPerp);

		returnValue.parameter2 = line1StartToLine2StartDotLine1DirPerp / -line2DirDotLine1DirPerp;

		// we can calculate the intersectionpoint with either line 1 or line 2

		returnValue.intersectPoint = vectormult(vectoradd(line1Start, line1Dir), returnValue.parameter1);

		// you can also check if parameter1 and parameter2 are
		// between 0 and 1 and input that information in the
		// intersection struct here...

		return returnValue;
	}
	else
		returnValue.linesAreParallel = true;
// when the lines are parallel, there is no intersection point and also no parameter1 and parameter2!
	return returnValue;
}
