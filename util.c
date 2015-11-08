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
	if(count==0)
		return NULL;
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
			result[idx] = pvPortMalloc(strlen(token)+1);
			memcpy(result[idx],token,strlen(token)+1);
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
char *strsep(char* line, const char* delims)
{
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
  dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
  dist = acos(dist);
  dist = rad2deg(dist);
  dist = dist * 60 * 1.1515;
  switch(unit) {
    case 'M':
      break;
    case 'K':
      dist = dist * 1.609344;
      break;
    case 'N':
      dist = dist * 0.8684;
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

int16_t direction(double lat1, double lon1, double lat2, double lon2, char unit){
	double y,x;
	double theta = lon2 - lon1;
	int16_t direction;
	y = sin(deg2rad(theta)) * cos(deg2rad(lat2));
	x = cos(deg2rad(lat1)) * sin(deg2rad(lat2)) - sin(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
	direction = rad2deg(atan2(y, x));
	if(direction <= 0)
		direction += 360;
	return direction;
}

double deg2dec(double deg){
	if (deg == 0)
		return 0;
	int intp = deg;  //DDMM
	int min = intp - ((intp/100)*100); // MM
	double decp = deg - intp; // .m
	decp = (((double) min + decp))/60;
	return ( (double) (intp/100) )+(  decp );
	//return deg;
}
