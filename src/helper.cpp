#include "helper.h"
int angleToPwm(float angleDeg, double modifier) {
    return static_cast<int>(PWM_MIN + modifier * angleDeg);
}


double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

double vincenty(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {
    const double a = 6378137.0;               // WGS-84 major axis
    const double f = 1 / 298.257223563;       // WGS-84 flattening
    const double b = (1 - f) * a;

    double phi1 = deg2rad(lat1_deg);
    double phi2 = deg2rad(lat2_deg);
    double L = deg2rad(lon2_deg - lon1_deg);

    double U1 = atan((1 - f) * tan(phi1));
    double U2 = atan((1 - f) * tan(phi2));

    double sinU1 = sin(U1), cosU1 = cos(U1);
    double sinU2 = sin(U2), cosU2 = cos(U2);

    double lambda = L;
    double lambda_prev;
    double sinSigma, cosSigma, sigma, sinAlpha, cosSqAlpha, cos2SigmaM, C;
    int iterLimit = 100;
    do {
        double sinLambda = sin(lambda);
        double cosLambda = cos(lambda);
        sinSigma = sqrt(
            pow(cosU2 * sinLambda, 2) +
            pow(cosU1 * sinU2 - sinU1 * cosU2 * cosLambda, 2)
        );
        if (sinSigma == 0) return 0;  // coincident points

        cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda;
        sigma = atan2(sinSigma, cosSigma);
        sinAlpha = cosU1 * cosU2 * sinLambda / sinSigma;
        cosSqAlpha = 1 - sinAlpha * sinAlpha;
        cos2SigmaM = (cosSqAlpha != 0) ?
            (cosSigma - 2 * sinU1 * sinU2 / cosSqAlpha) : 0;

        C = f / 16 * cosSqAlpha * (4 + f * (4 - 3 * cosSqAlpha));
        lambda_prev = lambda;
        lambda = L + (1 - C) * f * sinAlpha *
                 (sigma + C * sinSigma *
                 (cos2SigmaM + C * cosSigma *
                 (-1 + 2 * cos2SigmaM * cos2SigmaM)));
    } while (fabs(lambda - lambda_prev) > 1e-12 && --iterLimit > 0);

    if (iterLimit == 0)
        return -1; 

    double uSq = cosSqAlpha * (a * a - b * b) / (b * b);
    double A = 1 + uSq / 16384 *
               (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
    double B = uSq / 1024 *
               (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));
    double deltaSigma = B * sinSigma *
                        (cos2SigmaM + B / 4 *
                        (cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM) -
                        B / 6 * cos2SigmaM *
                        (-3 + 4 * sinSigma * sinSigma) *
                        (-3 + 4 * cos2SigmaM * cos2SigmaM)));

    double s = b * A * (sigma - deltaSigma);
    return s;
}

void vincentyDirect(double lat1_deg, double lon1_deg, double bearing_deg, double distance_m,double &lat2_deg, double &lon2_deg) {
    const double a = 6378137.0;               
    const double f = 1 / 298.257223563;       
    const double b = (1 - f) * a;

    double alpha1 = deg2rad(bearing_deg);
    double sinAlpha1 = sin(alpha1);
    double cosAlpha1 = cos(alpha1);

    double phi1 = deg2rad(lat1_deg);
    //double sinU1 = sin((1 - f) * phi1);
    //double cosU1 = cos((1 - f) * phi1);

    double tanU1 = (1 - f) * tan(phi1);
    double U1 = atan(tanU1);
    double sigma1 = atan2(tanU1, cosAlpha1);

    double sinU1_ = sin(U1), cosU1_ = cos(U1);

    double sinAlpha = cosU1_ * sinAlpha1;
    double cosSqAlpha = 1 - sinAlpha * sinAlpha;

    double uSq = cosSqAlpha * (a*a - b*b) / (b*b);
    double A = 1 + uSq / 16384 *
    (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
    double B = uSq / 1024 *
    (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));

    double sigma = distance_m / (b * A);
    double sigmaP;
    double cos2SigmaM, sinSigma, cosSigma, deltaSigma;

    int iterLimit = 100;
    do {
    cos2SigmaM = cos(2 * sigma1 + sigma);
    sinSigma = sin(sigma);
    cosSigma = cos(sigma);
    deltaSigma = B * sinSigma *
        (cos2SigmaM + B / 4 *
        (cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM) -
        B / 6 * cos2SigmaM *
        (-3 + 4 * sinSigma * sinSigma) *
        (-3 + 4 * cos2SigmaM * cos2SigmaM)));

    sigmaP = sigma;
    sigma = distance_m / (b * A) + deltaSigma;
    } while (fabs(sigma - sigmaP) > 1e-12 && --iterLimit > 0);

    if (iterLimit == 0) {
    std::cout << "Vincenty direct formula did not converge.\n";
    lat2_deg = lon2_deg = NAN;
    return;
    }

    double tmp = sinU1_ * sinSigma - cosU1_ * cosSigma * cosAlpha1;
    double phi2 = atan2(
    sinU1_ * cosSigma + cosU1_ * sinSigma * cosAlpha1,
    (1 - f) * sqrt(sinAlpha * sinAlpha + tmp * tmp)
    );
    double lambda = atan2(
    sinSigma * sinAlpha1,
    cosU1_ * cosSigma - sinU1_ * sinSigma * cosAlpha1
    );

    double C = f / 16 * cosSqAlpha * (4 + f * (4 - 3 * cosSqAlpha));
    double L = lambda - (1 - C) * f * sinAlpha *
    (sigma + C * sinSigma *
    (cos2SigmaM + C * cosSigma *
    (-1 + 2 * cos2SigmaM * cos2SigmaM)));

    double lon2 = deg2rad(lon1_deg) + L;

    lat2_deg = rad2deg(phi2);
    lon2_deg = rad2deg(lon2);
}

std::string getMissionStateName(int num) {
    switch(num) {
        case 0:
        return "UKNOWN";
        break;

        case 1:
        return "NO MISSION";
        break;

        case 2:
        return "NOT STARTED";
        break;

        case 3:
        return "ACTIVE";
        break;

        case 4:
        return "PAUSED";
        break;

        default:
        return "COMPLETE";
        break;
    }
}