#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H


#include <cmath>
#include <numeric>
#include <vector>

/**
 * @brief A struct to represent a cartesian coordinate.
 * @author Logan Wolf
 */

struct cart {

        double x;
        double y;

    };

/**
 * @brief A struct to represent a polar coordinate.
 * @author Logan Wolf
 */
struct polar {

    double theta;
    double r;

};

/**
 * @brief A struct to hold a rolling average.
 * @author Logan Wolf
 */


/**
 * @brief Returns the sign of a given value.
 * 
 * @param val The value to check.
 * @return int Returns 1 if positive, -1 if negative, and 0 if zero.
 */
inline int sgn(double val) {
    return ((0) < val) - (val < (0));
}

/**
 * @brief Converts polar coordinates to cartesian coordinates.
 * 
 * @param r The radius or distance from the origin.
 * @param theta The angle in radians.
 * @return cart The corresponding cartesian coordinates.
 */
inline cart PtoC(double r, double theta){
    
    cart p;
    
    p.x = r*cos(theta);
    p.y = r*sin(theta);

    return p;

}

/**
 * @brief Converts cartesian coordinates to polar coordinates.
 * 
 * @param x The x-coordinate.
 * @param y The y-coordinate.
 * @return polar The corresponding polar coordinates.
 */
inline polar CtoP(double x, double y)  {

    polar p;

    p.r = sqrtf(pow(x, 2)+pow(y, 2));
    p.theta = atan2f(y, x);
    
    return p;

}

/**
 * @brief Calculates a rolling average by adding a new value and removing the oldest one.
 * 
 * @param var The new value to add.
 * @param data The current data set.
 * @return avg The updated rolling average and data set.
 */
inline double rollAverage(double var, std::vector<double>& data){

    data.emplace_back(var);
    data.erase(data.begin());
    

    double average = std::accumulate(data.begin(),data.end(), (double)0)/data.size();

    return average;

}

inline double distanceFromSecant(double x1, double y1,
                          double x2, double y2,
                          double x0, double y0)
{
    double vx = x2 - x1;
    double vy = y2 - y1;

    double wx = x0 - x1;
    double wy = y0 - y1;

    double cross = vx * wy - vy * wx;
    double dist = std::abs(cross) / std::sqrt(vx*vx + vy*vy);

    return dist;
}

inline double distanceBetweenPoints(cart P1, cart P2){

    double X = P2.x - P1.x;
    double Y = P2.y - P1.y;

    double hyp = std::sqrt(pow(X,2)+pow(Y,2));

    return hyp;

}


    
inline double wrapHeading(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}


#endif