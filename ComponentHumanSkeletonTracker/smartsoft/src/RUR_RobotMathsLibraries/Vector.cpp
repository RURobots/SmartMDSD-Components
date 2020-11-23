/******************************************************************************/
/*                                                                            */
/*	Name:           Vector.cpp                                                */
/*  Purpose:        General purpose vector maths class implementation         */
/*  Author:         Mark Burgin                                               */
/*  Date:           12/12/11                                                  */
/*	Copyright:      Copyright (C), RURobots Ltd, 2011                         */
/*                                                                            */
/******************************************************************************/

#include <math.h>
#include <limits>
//#include <unistd.h>
#include <string.h> // Only used for the ToString method
#include "stdio.h" // Only used for Print method
#include "Constants.h"
#include "MathsException.h"
#include "Vector.h"

namespace RUR_RobotMaths
{
    // Constructors and Destructors

/* public */Vector::Vector(int size)
{
    if (size <= 0)
    {
        throw MathsException("Cannot have a Vector with 0 or negative number of rows");
    }

    count = size;

    try
    {
        values = new double[size];
    }
    catch (const exception& e)
    {
        throw MathsException("Error allocating memory for Vector", e);
    }
}

/* public */Vector::Vector(double arrayValues[], int arraySize)
{
    if (arraySize == 0)
    {
        throw MathsException("Cannot have a Vector with 0 rows");
    }

    this->count = arraySize;

    try
    {
    	values = new double[arraySize];
    }
    catch (const exception& e)
    {
        throw MathsException("Error allocating memory for Vector", e);
    }

    for (int i = 0; i < arraySize; i++)
    {
        this->values[i] = arrayValues[i];
    }
}

/* public */Vector::Vector(const Vector& vector)
{
    this->count = vector.count;

    try
    {
        values = new double[vector.count];
    }
    catch (const exception& e)
    {
        throw MathsException("Error allocating memory for Vector", e);
    }

    for (int i = 0; i < vector.count; i++)
    {
        this->values[i] = vector.values[i];
    }
}

/* public, virtual */Vector::~Vector()
{
    delete values;

    count = 0;
}

/* public, static */Vector Vector::Zeros(int i)
{
    if (i <= 0)
    {
        throw MathsException("Cannot have a Vector with 0 or negative number of rows");
    }

    Vector returnValue(i);

    for (int index = 0; index < i; index++)
    {
        returnValue.SetValue(index, 0.0);
    }

    return returnValue;
}

/* public, static */Vector Vector::Ones(int i)
{
    if (i <= 0)
    {
        throw MathsException("Cannot have a Vector with 0 or a negative number of rows");
    }

    Vector returnValue(i);

    for (int index = 0; index < i; index++)
    {
        returnValue.SetValue(index, 1.0);
    }

    return returnValue;
}

/* public, static */Vector Vector::MinusOnes(int i)
{
    if (i <= 0)
    {
        throw MathsException("Cannot have a Vector with 0 or a negative number of rows");
    }

    Vector returnValue(i);

    for (int index = 0; index < i; index++)
    {
        returnValue.SetValue(index, -1.0);
    }

    return returnValue;
}


    // Methods

/* public, virtual */int Vector::Size() const
{
    return count;
}

/* public virtual */double Vector::GetValue(int i) const
{
    if ((i >= count) || (i < 0))
    {
        throw MathsException("Index out of range getting Vector element");
    }

    return values[i];
}

/* public, virtual */void Vector::SetValue(int i, double value)
{
    if ((i >= count) || (i < 0))
    {
        throw MathsException("Index out of range getting Vector element");
    }

    values[i] = value;
}

/* public, virtual */Vector Vector::Multiply(double x) const
{
    Vector returnValue(count);

    for (int index = 0; index < count; index++)
    {
        returnValue.SetValue(index, this->GetValue(index) * x);
    }

    return returnValue;
}

/* public, virtual */Vector Vector::Divide(double x) const
{
    Vector returnValue(count);

    for (int index = 0; index < count; index++)
    {
        returnValue.SetValue(index, this->GetValue(index) / x);
    }

    return returnValue;
}

/* public, virtual */Vector Vector::Plus(const Vector& x) const
{
    if (this->count != x.count)
    {
        throw MathsException("Cannot add Vectors of different sizes");
    }

    Vector returnValue(count);

    for (int index = 0; index < count; index++)
    {
        returnValue.SetValue(index, this->GetValue(index) + x.GetValue(index));
    }

    return returnValue;
}

/* public, virtual */Vector Vector::Minus(const Vector& x) const
{
    if (this->count != x.count)
    {
        throw MathsException("Cannot subtract Vectors of different sizes");
    }

    Vector returnValue(count);

    for (int index = 0; index < count; index++)
    {
        returnValue.SetValue(index, this->GetValue(index) - x.GetValue(index));
    }

    return returnValue;
}

/* public, virtual */double Vector::Dot(const Vector& x) const
{
    if (this->count != x.count)
    {
        throw MathsException("Cannot find dot product of Vectors of different sizes");
    }

    double returnValue = 0.0;
    for (int index = 0; index < count; index++)
    {
        returnValue += this->GetValue(index) * x.GetValue(index);
    }

    return returnValue;
}

/* public, virtual */Vector Vector::Cross(const Vector& x) const
{
    if ((this->count != 3) || (x.count != 3))
    {
        throw MathsException("Cannot find cross product of Vectors with size other than 3");
    }

    Vector result(3);

    result.SetValue(0, this->GetValue(1) * x.GetValue(2) - this->GetValue(2)
            * x.GetValue(1));
    result.SetValue(1, this->GetValue(2) * x.GetValue(0) - this->GetValue(0)
            * x.GetValue(2));
    result.SetValue(2, this->GetValue(0) * x.GetValue(1) - this->GetValue(1)
            * x.GetValue(0));

    return result;
}

/* public, friend */Vector Cross(const Vector& x, Vector y)
{
    return x.Cross(y);
}

/* public, virtual */double Vector::Norm(int n) const
{
    if (n < 0)
    {
        throw MathsException("Cannot calculate the n-norm of a Vector when n is a negative number");
    }

    double returnValue = 0;

    switch (n)
    {
    case 0:
    {
        // Infinity-norm
        for (int index = 0; index < count; index++)
        {
            if (fabs(this->GetValue(index)) > returnValue)
            {
                returnValue = fabs(this->GetValue(index));
            }
        }
        break;
    }
    case 1:
    {
        // Manhattan norm
        returnValue = 0;
        for (int index = 0; index < count; index++)
        {
            returnValue += fabs(this->GetValue(index));
        }
        break;
    }
    default:
    {
        returnValue = 0;
        for (int index = 0; index < count; index++)
        {
            returnValue += pow(this->GetValue(index), n);
        }
        returnValue = pow(returnValue, 1.0 / n);
        break;
    }
    }

    return returnValue;
}

/* public, virtual */void Vector::MakeZeros()
{
    for (int index = 0; index < count; index++)
    {
        values[index] = 0.0;
    }
}

/* public, virtual */void Vector::MakeOnes()
{
    for (int index = 0; index < count; index++)
    {
        values[index] = 1.0;
    }
}

/* public, virtual */void Vector::MakeMinusOnes()
{
    for (int index = 0; index < count; index++)
    {
        values[index] = -1.0;
    }
}

/* public, virtual */Vector& Vector::RecordMax(const Vector& vector, bool reset)
{
    if (reset)
    {
        for (int index = 0; index < vector.Size(); index++)
        {
            values[index] = -1.0 * numeric_limits<double>::max();
        }

        return *this;
    }

    for (int index = 0; index < vector.Size(); index++)
    {
        if (vector[index] > values[index])
        {
            values[index] = vector[index];
        }
    }

    return *this;
}

/* public, virtual */Vector& Vector::RecordMin(const Vector& vector, bool reset)
{
    if (reset)
    {
        for (int index = 0; index < vector.Size(); index++)
        {
            values[index] = numeric_limits<double>::max();
        }

        return *this;
    }

    for (int index = 0; index < vector.Size(); index++)
    {
        if (vector[index] < values[index])
        {
            values[index] = vector[index];
        }
    }

    return *this;
}

/* public, static */Vector Vector::PadToLength(int toLength)
{
    Vector newVector(toLength);

    int fromLength = this->Size();

    int smallestSize;
    if (fromLength < toLength)
    {
    	smallestSize = fromLength;
    }
    else
    {
    	smallestSize = toLength;
    }


	for (int i = 0; i < smallestSize; i++)   // truncates if output vector is smaller
	{
		newVector.SetValue(i, this->values[i]);
	}


	for (int i = fromLength; i < toLength; i++)   // pads if output vector is larger
	{
		newVector.SetValue(i, 0);
	}

    return newVector;
}

/* public, virtual */const char* Vector::ToString() const
{
	char* string = new char[Size() * 20 + 2]; // This will leak unless FreeStringMemory is called on it externally
	char value[20];

	string[0] = 0;
    strcat(string, "[");
    for (int i = 0;  i < Size(); i++)
    {
        if (i != 0)
        {
        	strcat(string, ", ");
        }
        sprintf(value, "%g", values[i]);
        strcat(string, value);
    }
    strcat(string, "]");

    return string;
}

/*public, static */void Vector::FreeStringMemory(const char* stringPointer)
{
	delete stringPointer;
}

/* public, virtual */void Vector::Print() const
{
    printf("[");
    for (int i = 0;  i < Size(); i++)
    {
        if (i != 0)
        {
            printf(", ");
        }
        printf("%g", values[i]);
    }
    printf("]\n");
}


    // Operators

/* public, virtual */double& Vector::operator[](const int i) const
{
    if ((i >= count) || (i < 0))
    {
        throw MathsException("Index out of range getting Vector element");
    }

    return values[i];
}

/* public */Vector Vector::operator*(double y) const
{
    return this->Multiply(y);
}

/* public, friend */Vector operator *(double x, Vector y)
{
    return y.Multiply(x);
}

/* public */Vector Vector::operator /(double y) const
{
    return this->Divide(y);
}

/* public */Vector Vector::operator +(const Vector& y) const
{
    return this->Plus(y);
}

/* public */Vector Vector::operator -(const Vector& y) const
{
    return this->Minus(y);
}

/* public */Vector& Vector::operator =(const Vector& vector)
{
    if (this->values != vector.values)
    {
        if (this->count != vector.count)
        {
            // Must resize the array
        	if (this->count != 0) //  Don't delete 0 sized vectors
        	{
        		delete values;
        	}

            this->count = vector.count;

            try
            {
				if (vector.count != 0) //  Don't allocate for 0 sized vectors
				{
					values = new double[vector.count];
            	}
            }
            catch (const exception& e)
            {
                throw MathsException("Error allocating memory for Vector", e);
            }
        }

        for (int i = 0; i < vector.count; i++)
        {
            this->values[i] = vector.values[i];
        }
    }

    return *this;
}

} // namespace RUR_RobotMaths
