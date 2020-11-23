/******************************************************************************/
/*                                                                            */
/*  Name:           Transformation.cpp                                        */
/*  Purpose:        Implementation of a 3D transformation class               */
/*  Author:         Mark Burgin                                               */
/*  Date:           12/12/11                                                  */
/*  Copyright:      Copyright (C), RURobots Ltd, 2011                         */
/*                                                                            */
/******************************************************************************/

#include <math.h>
#include <limits>
using namespace std;
#include "Transformation.h"
#include "Constants.h"
using namespace RUR_RobotMaths;

/* public */Transformation::Transformation() : Matrix(4,4)
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            rows[i][j] = 0.0;
        }
    }

    rows[0][0] = 1.0;
    rows[1][1] = 1.0;
    rows[2][2] = 1.0;
    rows[3][3] = 1.0;
}

/* public, virtual */Transformation::~Transformation()
{
}

/* public */Transformation::Transformation(const Matrix& matrix) : Matrix(matrix)
{
}

/* public, static */Transformation Transformation::Zeros()
{
	return Matrix::Zeros(4, 4);
}

/* public, static */Transformation Transformation::Ones()
{
	return Matrix::Ones(4, 4);
}

/* public, static */Transformation Transformation::MinusOnes()
{
	return Matrix::MinusOnes(4, 4);
}
/* public, static */Transformation Transformation::Identity()
{
	return Matrix::Identity(4, 4);
}

/* public, virtual*/CartesianPosition Transformation::Transform(const CartesianPosition& position) const
{
    Transformation startPosition;
    CartesianPosition returnValue;

    startPosition.FromCartesianPosition(position);

    Transformation result = (*this) * startPosition;

    returnValue = result.ToCartesianPosition();

    return returnValue;
}

/* public, virtual*/CartesianPosition Transformation::InverseTransform(const CartesianPosition& position) const
{
    Transformation startPosition;
    startPosition.FromCartesianPosition(position);
    Transformation inverseTranslation;
    Transformation inverseRotation;

    inverseTranslation[0][3] = -1.0 * rows[0][3];
    inverseTranslation[1][3] = -1.0 * rows[1][3];
    inverseTranslation[2][3] = -1.0 * rows[2][3];

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            inverseRotation[j][i] = rows[i][j];
        }
    }

    Transformation result = inverseRotation * inverseTranslation * startPosition;

    return result.ToCartesianPosition();
}

/* public, virtual */Transformation& Transformation::FromCartesianPosition(const CartesianPosition& position)
{
    rows[3][0] = 0.0;
    rows[3][1] = 0.0;
    rows[3][2] = 0.0;
    rows[3][3] = 1.0;

    rows[0][3] = position[0];
    rows[1][3] = position[1];
    rows[2][3] = position[2];

    double ca = cos(position[5]);
    double sa = sin(position[5]);
    double cb = cos(position[4]);
    double sb = sin(position[4]);
    double cc = cos(position[3]);
    double sc = sin(position[3]);

    rows[0][0] = ca * cb;
    rows[0][1] = ca * sb * sc - sa * cc;
    rows[0][2] = ca * sb * cc + sa * sc;
    rows[1][0] = sa * cb;
    rows[1][1] = sa * sb * sc + ca * cc;
    rows[1][2] = sa * sb * cc - ca * sc;
    rows[2][0] = -sb;
    rows[2][1] = cb * sc;
    rows[2][2] = cb * cc;

    return *this;
}

/* public, virtual */CartesianPosition Transformation::ToCartesianPosition()
{
    CartesianPosition returnValue;

    returnValue[0] = rows[0][3];
    returnValue[1] = rows[1][3];
    returnValue[2] = rows[2][3];

    returnValue[4] = atan2(-1.0 * rows[2][0], sqrt(rows[0][0] * rows[0][0] + rows[1][0] * rows[1][0]));
    double cb = cos(returnValue[4]);

    if (fabs(cb) < 0.000001)
    {
        returnValue[3] = 0.0; // Arbitrary choice of yaw angle
        if (returnValue[4] > 0)
        {
            // At +90 degrees
            returnValue[4] = Constants::PiOverTwo;
        }
        else
        {
            // At -90 degrees
            returnValue[4] = -1.0 * Constants::PiOverTwo;
        }
        returnValue[5] = -1.0 * atan2(rows[0][1], rows[1][1]);
    }
    else
    {
        returnValue[3] = atan2(rows[2][1] / cb, rows[2][2] / cb);
        returnValue[5] = atan2(rows[1][0] / cb, rows[0][0] / cb);
//        returnValue[3] = atan2(rows[2][1], rows[2][2]);
//        returnValue[5] = atan2(rows[1][0], rows[0][0]);
    }

    return returnValue;
}

/*public, virtual */bool Transformation::IsValid()
{
	for (int column = 0; column < 3; column++)
	{
		double sum = 0.0;
		for (int row = 0; row < 3; row++)
		{
			sum += rows[row][column] * rows[row][column];
		}

		if (fabs(sum  - 1.0) > (numeric_limits<double>::epsilon() * numeric_limits<double>::epsilon()))
		{
			return false;
		}

		if (rows[3][column] != 0.0)
		{
			return false;
		}
	}

	return true;
}

