/******************************************************************************/
/*                                                                            */
/*  Name:           BodyData.h                                                */
/*  Purpose:        Data structure representing the body data (skeleton).     */
/*  Author:         Mark Burgin                                               */
/*  Date:           27/03/2017                                                */
/*  Copyright:      Copyright (C), RURobots Ltd, 2017                         */
/*                                                                            */
/******************************************************************************/


#ifndef BODYDATA_H_
#define BODYDATA_H_

//#include "CartesianVelocity.h"

#define BODY_COUNT	6


class BodyData
{
public:
	enum Joint
	{
		SpineBase =	    0, 	// Base of the spine
		SpineMid =	    1, 	// Middle of the spine
		Neck =	        2, 	// Neck
		Head =	        3, 	// Head
		ShoulderLeft =  4, 	// Left shoulder
		ElbowLeft =     5, 	// Left elbow
		WristLeft =	    6, 	// Left wrist
		HandLeft =	    7, 	// Left hand
		ShoulderRight =	8, 	// Right shoulder
		ElbowRight =    9, 	// Right elbow
		WristRight =    10,	// Right wrist
		HandRight =	    11,	// Right hand
		HipLeft = 	    12,	// Left hip
		KneeLeft =	    13,	// Left knee
		AnkleLeft =     14, // Left ankle
		FootLeft =      15, // Left foot
		HipRight =	    16, // Right hip
		KneeRight =	    17, // Right knee
		AnkleRight =    18, // Right ankle
		FootRight =	    19, // Right foot
		SpineShoulder =	20, // Spine at the shoulder
		HandTipLeft =   21, // Tip of the left hand
		ThumbLeft =	    22, // Left thumb
		HandTipRight =  23, // Tip of the right hand
		ThumbRight =    24, // Right thumb
	};

public:
	enum HandTrackingState
	{
		Open,
		Closed,
		HandNotTracked,
		Lasso,
		Unknown
	};

public:
	enum BodyTrackingState
	{
		Tracked,
		Inferred,
		BodyNotTracked
	};

public:
	static const int numberOfJoints = 25;

	bool isTracked;
	HandTrackingState leftHandState;
	HandTrackingState rightHandState;

	float jointPosition2dX[numberOfJoints];
	float jointPosition2dY[numberOfJoints];

	float jointPosition3dX[numberOfJoints];
	float jointPosition3dY[numberOfJoints];
	float jointPosition3dZ[numberOfJoints];

	BodyTrackingState isJointTracked[numberOfJoints];
};

//typedef	RUR_RobotMaths::CartesianVelocity BodyVelocities[BodyData::numberOfJoints];
//typedef RUR_RobotMaths::CartesianPosition BodyPositions[BodyData::numberOfJoints];

#endif
