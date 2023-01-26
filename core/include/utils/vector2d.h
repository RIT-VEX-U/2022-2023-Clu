#pragma once

#include <cmath>

#ifndef PI
#define PI 3.141592654
#endif
/**
 * Vector2D is an x,y pair
 * Used to represent 2D locations on the field. 
 * It can also be treated as a direction and magnitude
*/
class Vector2D
{
public:

    /**
     * Data structure representing an X,Y coordinate
     */
    struct point_t
    {
        double x, y; ///< x and y position in space


        /** 
         * dist calculates the euclidian distance between this point and another point using the pythagorean theorem
         * @return the euclidian distance between this and other
         */
        double dist(const point_t other)
        {
          return sqrt(pow(this->x - other.x, 2) + pow(this->y - other.y, 2));
        }

        /**
         * Vector2D addition operation on points
         * @return this + other (this.x + other.x, this.y + other.y)
         */
        point_t operator+(const point_t &other)
        {
          point_t p 
          {
            .x = this->x + other.x,
            .y = this->y + other.y
          };
          return p;
        }

        /**
         * Vector2D subtraction operation on points
         * @return this - other (this.x - other.x, this.y - other.y)
         */
        point_t operator-(const point_t &other)
        {
          point_t p 
          {
            .x = this->x - other.x,
            .y = this->y - other.y
          };
          return p;
        }
    };

    /**
     * Construct a vector object.
     * 
     * @param dir Direction, in radians. 'foward' is 0, clockwise positive when viewed from the top.
     * @param mag Magnitude.
     */ 
    Vector2D(double dir, double mag);
    
    /**
     * Construct a vector object from a cartesian point.
     * 
     * @param p point_t.x , point_t.y
     */
    Vector2D(point_t p);

    /**
     * Get the direction of the vector, in radians.
     * '0' is forward, clockwise positive when viewed from the top.
     * 
     * Use r2d() to convert.
     * @return the direction of the vetctor in radians
     */
    double get_dir() const;

    /**
     * @return the magnitude of the vector
     */
    double get_mag() const;

    /**
     * @return the X component of the vector; positive to the right.
     */
    double get_x() const;

    /**
     * @return the Y component of the vector, positive forward.
     */
    double get_y() const;

    /**
     * Changes the magnitude of the vector to 1
     * @return the normalized vector
    */
    Vector2D normalize();

    /**
    * Returns a point from the vector
    * @return the point represented by the vector
    */
    Vector2D::point_t point();

    /**
     * Multiply the components of a vector by x
     * (self.x * x, self.y * x)
     * @param x the scalar value by which to multiply the components of the vector
     * @return the vector after scaling by s
    */
    Vector2D operator*(const double &x);
    /**
     * Add the components of two vectors together
     * Vector2D + Vector2D = (self.x + other.x, self.y + other.y)
     * @return the sum of the vectors
    */
    Vector2D operator+(const Vector2D &other);
    /**
     * Subtract the components of two vectors together
     * Vector2D - Vector2D = (self.x - other.x, self.y - other.y)
     * @return the difference of the vectors
    */
    Vector2D operator-(const Vector2D &other);

private:

    double dir, mag;

};

/**
 * General function for converting degrees to radians
 * @param deg the angle in degrees
 * @return the angle in radians
 */
double deg2rad(double deg);

/**
 * General function for converting radians to degrees
 * @param rad the angle in radians
 * @return the angle in degrees

 */
double rad2deg(double r);
