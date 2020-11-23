/******************************************************************************/
/*                                                                            */
/*  Name:           Transformation.h                                          */
/*  Purpose:        Header for 3D transformation class                        */
/*  Author:         Mark Burgin                                               */
/*  Date:           12/12/11                                                  */
/*  Copyright:      Copyright (C), RURobots Ltd, 2011                         */
/*                                                                            */
/******************************************************************************/

#ifndef TRANSFORMATION_H_
#define TRANSFORMATION_H_

#include "CartesianPosition.h"
#include "Matrix.h"

namespace RUR_RobotMaths
{

class Transformation : public Matrix
{

	// Constructors and Destructors
public:
    Transformation();
public:
    virtual ~Transformation();
public:
    Transformation(const Matrix& matrix);
public:
	static Transformation Zeros();
public:
	static Transformation Ones();
public:
	static Transformation MinusOnes();
public:
	static Transformation Identity();

    // Class Methods
public:
    CartesianPosition Transform(const CartesianPosition& position) const;
public:
    CartesianPosition InverseTransform(const CartesianPosition& position) const;
public:
    virtual Transformation& FromCartesianPosition(const CartesianPosition& position);
public:
    virtual CartesianPosition ToCartesianPosition();
public:
	virtual bool IsValid();

    // Class Friends
    friend class TransformationTest; // Used only for unit testing
};

} // namespace RUR_RobotMaths;

#endif // #ifndef TRANSFORMATION_H_
