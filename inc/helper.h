#include <cmath>
#include <iostream>
#include <algorithm>
#include <iostream>
#include <regex>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include "config.h"

//Degrees to pwm 1000-2000 range for servos
int angleToPwm(float angleDeg);
// Convert angle in degrees to radians
double deg2rad(double deg);
// Convert angle in radians to degrees
double rad2deg(double rad);
//calculate distance between long/lat points using Vincenty formula
double vincenty(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg);
//calculate destination point given start point, bearing and distance using Vincenty formula
void vincentyDirect(double lat1_deg, double lon1_deg, double bearing_deg, double distance_m,double &lat2_deg, double &lon2_deg);
//read waypoints.csv to match name