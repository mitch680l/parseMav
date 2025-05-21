#include <cmath>
#include <iostream>
#include <algorithm>
#include <iostream>
#include <regex>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
int angleToPwm(float angleDeg);
double deg2rad(double deg);
double rad2deg(double rad);
double vincenty(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg);
void vincentyDirect(double lat1_deg, double lon1_deg, double bearing_deg, double distance_m,double &lat2_deg, double &lon2_deg);
bool getWaypointCoords(const std::string& name,double* lat, double* lon, double* alt);