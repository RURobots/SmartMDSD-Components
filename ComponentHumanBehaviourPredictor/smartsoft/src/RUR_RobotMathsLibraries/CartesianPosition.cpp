/******************************************************************************/
/*                                                                            */
/*  Name:           CartesianPosition.cpp                                     */
/*  Purpose:        Implementation of a class to store a Cartesian            */
/*                  position             .                                    */
/*  Author:         Mark Burgin                                               */
/*  Date:           21/12/11                                                  */
/*  Copyright:      Copyright (C), RURobots Ltd, 2011                         */
/*                                                                            */
/******************************************************************************/

#include <math.h>
#include <limits>
#include "Constants.h"
using namespace RUR_RobotMaths;
#include "MathsException.h"
#include "CartesianPosition.h"

namespace RUR_RobotMaths
{

    // Constructors and Destructors

/* public */CartesianPosition::CartesianPosition() : Vector(6)
{
}

/* public */CartesianPosition::CartesianPosition(double arrayValues[]) : Vector(arrayValues, 6)
{
}

/* public */CartesianPosition::CartesianPosition(const Vector& vector) : Vector(vector)
{
    if (vector.Size() != 6)
    {
        // Delete the data
        this->count = 0;
        delete values;

        throw MathsException("CartesianPosition must be initialised with 6 elements");
    }
}

/* public */CartesianPosition::CartesianPosition(double x, double y, double z, double wx, double wy, double wz) : Vector(6)
{
    values[0] = x;
    values[1] = y;
    values[2] = z;
    values[3] = wx;
    values[4] = wy;
    values[5] = wz;
}

/* public, static */CartesianPosition CartesianPosition::Zeros()
{
	return Vector::Zeros(6);
}

/* public, static */CartesianPosition CartesianPosition::Ones()
{
	return Vector::Ones(6);
}

/* public, static */CartesianPosition CartesianPosition::MinusOnes()
{
    return Vector::MinusOnes(6);
}

/* public */CartesianPosition& CartesianPosition::operator =(const Vector& vector)
{
    if (vector.Size() != 6)
    {
        throw MathsException("CartesianPosition must be initialised with 6 elements");
    }
    else
    {
        Vector::operator =(vector);
    }

    return (*this);
}

} // namespace RUR_RobotMaths
