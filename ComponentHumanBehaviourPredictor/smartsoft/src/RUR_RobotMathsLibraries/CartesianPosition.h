/******************************************************************************/
/*                                                                            */
/*  Name:           CartesianPosition.h                                       */
/*  Purpose:        Header for a class Representing a position in Cartesian   */
/*                  coordinates                                               */
/*  Author:         Mark Burgin                                               */
/*  Date:           21/12/11                                                  */
/*  Copyright:      Copyright (C), RURobots Ltd, 2011                         */
/*                                                                            */
/*  This class provides storage for a Cartesian Position stored as            */
/*  x,y,z,roll,pitch,yaw values.  'Roll' is defined as the rotation around    */
/*  the fixed X-axis, 'pitch' is the rotation around the fixed Y-axis, and    */
/*  'yaw' is the rotation around the fixed Z-axis.                            */
/*  It is a specialised version of the Vector class that can contain only     */
/*  6 elements and adds methods for manipulating Cartesian Positions and      */
/*  converting them to and from Transformation types.                         */
/*                                                                            */
/******************************************************************************/


#ifndef CartesianPosition_H_
#define CartesianPosition_H_

#include "Vector.h"

namespace RUR_RobotMaths
{

class CartesianPosition : public Vector
{
    // Class Constructors
public:
    CartesianPosition();
public:
    CartesianPosition(double arrayValues[]);
public:
    CartesianPosition(const Vector& vector);
public:
    CartesianPosition(double x, double y, double z, double wx, double wy, double wz);
public:
    static CartesianPosition Zeros();
public:
    static CartesianPosition Ones();
public:
    static CartesianPosition MinusOnes();

    // Class Operators
public:
    CartesianPosition& operator =(const Vector& vector);
};

} // namespace RUR_Maths

#endif /* #ifndef CartesianPosition_H_ */
