/******************************************************************************/
/*                                                                            */
/*  Name:           KalmanFilterWitouControls.h                               */
/*  Purpose:        Class that performs a Kalman Filter update for a system   */
/*                  that has no control vector                                */
/*  Author:         Mark Burgin                                               */
/*  Date:           06/12/17                                                  */
/*  Copyright:      Copyright (C), RURobots Ltd, 2017                         */
/*                                                                            */
/******************************************************************************/

#ifndef KalmanFilterWithoutControls_H_
#define KalmanFilterWithoutControls_H_

#include "Matrix.h"

namespace RUR_RobotMaths
{

class KalmanFilterWithoutControls
{

private:
	Matrix kalmanA;
private:
	Matrix kalmanC;

private:
	Matrix kalmanQ;
private:
	Matrix kalmanR;

private:
	int stateCount;
private:
	int controlCount;
private:
	int measurementCount;

private:
	Matrix kalmanATransposed;
private:
	Matrix kalmanCTransposed;
private:
	Matrix identityMatrix;
private:
	Matrix kalmanGain;

public:
	KalmanFilterWithoutControls(const Matrix& kalmanAToUse, const Matrix& kalmanCToUse, const Matrix& kalmanQToUse, const Matrix& kalmanRToUse);
	virtual ~KalmanFilterWithoutControls(void);

public:
	virtual Vector Update(const Vector& previousX, const Matrix& previousCovariance, bool hasMeasurements, const Vector& measurements, bool calculateCovariance, Matrix& newCovariance);

	friend class KalmanFilterWithoutControlsTest;
};

} // namespace RUR_Maths

#endif // #ifndef KalmanFilterWithoutControls_H_
