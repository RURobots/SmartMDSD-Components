/******************************************************************************/
/*                                                                            */
/*  Name:           KalmanFilterWithoutControls.cpp                           */
/*  Purpose:        Implementation of a class to that performs a Kalman		  */
/*                  Filter update for a system that has no control vector     */
/*  Author:         Mark Burgin                                               */
/*  Date:           06/12/17                                                  */
/*  Copyright:      Copyright (C), RURobots Ltd, 2017                         */
/*                                                                            */
/******************************************************************************/

// See "Probabilistic Robotics", Thrun et al.

#include "KalmanFilterWithoutControls.h"
#include "MathsException.h"

namespace RUR_RobotMaths
{

/* public */ KalmanFilterWithoutControls::KalmanFilterWithoutControls(const Matrix& kalmanAToUse, const Matrix& kalmanCToUse, const Matrix& kalmanQToUse, const Matrix& kalmanRToUse) :
	kalmanA(kalmanAToUse), kalmanC(kalmanCToUse), kalmanQ(kalmanQToUse), kalmanR(kalmanRToUse), kalmanATransposed(kalmanAToUse.Transpose()), kalmanCTransposed(kalmanCToUse.Transpose()), identityMatrix(Matrix::Identity(kalmanA.RowCount(), kalmanA.RowCount())), kalmanGain(kalmanA.RowCount(), kalmanQ.RowCount())
{
	// kalmanA					State transition matrix 	[stateN, stateN]
	// kalmanC					Measurement matrix 			[measN, stateN]
	// kalmanQ					Measurement covariance 		[measN, measN]
	// kalmanR					Process covariance 			[stateN, stateN]

	// Determine dimensionalities
	stateCount = kalmanA.RowCount();
	measurementCount = kalmanC.RowCount();

	// Check for consistent dimensionalities
	if ((stateCount < 1) || (measurementCount < 1))
	{
	    throw MathsException("The Kalman matrices cannot have zero dimensions");
	}
	if (kalmanA.ColumnCount() != stateCount)
	{
	    throw MathsException("The Kalman A matrix is not square or does not have the correct dimension (Should be [stateN, stateN])");
	}
	if (kalmanC.ColumnCount() != stateCount)
	{
	    throw MathsException("The Kalman C matrix does not have the correct dimensions (Should be [measN, stateN])");
	}
	if ((kalmanQ.RowCount() != measurementCount) || (kalmanQ.ColumnCount() != measurementCount))
	{
	    throw MathsException("The Kalman Q matrix is not square or does not have the correct dimension (Should be [measN, measN])");
	}
	if ((kalmanR.RowCount() != stateCount) || (kalmanR.ColumnCount() != stateCount))
	{
	    throw MathsException("The Kalman R matrix is not square or does not have the correct dimension (Should be [stateN, stateN])");
	}
}

/* public, virtual */KalmanFilterWithoutControls::~KalmanFilterWithoutControls(void)
{
}

/* public, virtual */ Vector KalmanFilterWithoutControls::Update(const Vector& previousX, const Matrix& previousCovariance, bool hasMeasurements, const Vector& measurements, bool calculateCovariance, Matrix& newCovariance)
{
	// previousX				Previous state vector			[stateN]
	// previousCovariance		Previous total covariance		[stateN, stateN]
	// hasMeasurements			bool inidcating if there are new measurements for this step
	// measurements				Measurement vector				[measN]
	// newCovariance			New total covariance matrix		[stateN, stateN] (on output)

	// Check for consistent dimensionalities
	if (previousX.Size() != stateCount)
	{
		throw MathsException("The state vector, previousX, has the wrong dimension (Should be [stateN])");
	}
	if ((previousCovariance.RowCount() != stateCount) || (previousCovariance.ColumnCount() != stateCount))
	{
		throw MathsException("The covariance matrix, previousCovariance, is not square or has the wrong dimension (Should be [stateN, stateN])");
	}
	if (hasMeasurements && (measurements.Size() != measurementCount))
	{
		throw MathsException("The measurement vector, measurements, has the wrong dimension (Should be [measN])");
	}

	// Prediction step
	Vector predictedX(stateCount);

	predictedX = kalmanA * previousX;
	Matrix predictedCovariance(stateCount, stateCount);

	if (calculateCovariance)
	{
		predictedCovariance = kalmanA * previousCovariance * kalmanATransposed + kalmanR;
	}

	// Correction step
	if (hasMeasurements)
	{
		Vector correctedX(stateCount);

		if (calculateCovariance)
		{
			kalmanGain = predictedCovariance * kalmanCTransposed * ((kalmanC * predictedCovariance * kalmanCTransposed) + kalmanQ).Inverse();
			correctedX = predictedX + kalmanGain * (measurements - (kalmanC * predictedX));
			newCovariance = (identityMatrix - (kalmanGain * kalmanC)) * predictedCovariance;
		}
		else
		{
			correctedX = predictedX + kalmanGain * (measurements - (kalmanC * predictedX));
		}
		return correctedX;
	}
	else
	{
		newCovariance = predictedCovariance;

		return predictedX;
	}
}

} // namespace RUR_RobotMaths


