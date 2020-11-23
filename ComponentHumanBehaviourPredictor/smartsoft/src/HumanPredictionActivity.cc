//--------------------------------------------------------------------------
// Code generated by the SmartSoft MDSD Toolchain
// The SmartSoft Toolchain has been developed by:
//	
// Service Robotics Research Center
// University of Applied Sciences Ulm
// Prittwitzstr. 10
// 89075 Ulm (Germany)
//
// Information about the SmartSoft MDSD Toolchain is available at:
// www.servicerobotik-ulm.de
//
// This file is generated once. Modify this file to your needs. 
// If you want the toolchain to re-generate this file, please 
// delete it before running the code generator.
//--------------------------------------------------------------------------

#include <climits>
#include "CartesianPosition.h"
#include "Transformation.h"
using namespace RUR_RobotMaths;
#include "HumanPredictionActivity.hh"
#include "ComponentHumanBehaviourPredictor.hh"
#include "Skeleton.hh"
#include "BodyData.h"

#include <iostream>

HumanPredictionActivity::HumanPredictionActivity(SmartACE::SmartComponent *comp) 
:	HumanPredictionActivityCore(comp)
{
	std::cout << "constructor HumanPredictionActivity\n";
}
HumanPredictionActivity::~HumanPredictionActivity() 
{
	std::cout << "destructor HumanPredictionActivity\n";
}


void HumanPredictionActivity::on_HumanSkeletonsPushServiceIn(const DomainHumanTracking::CommHumanPositionsAndVelocities &input)
{
	COMP->humanBehaviourPredictorMutex.acquire();
	{
		//Save the input skeletons
		COMP->humanPositionsAndVelocities = input;

		// Perform the predictions and save the results
		CalculatePredictions(input, COMP->humanPositionPredictions);

		// Output the newest prediction
		COMP->humanPredictionsPushServiceOut->put(COMP->humanPositionPredictions);
	}
	COMP->humanBehaviourPredictorMutex.release();

}

int HumanPredictionActivity::on_entry()
{
	// do initialization procedures here, which are called once, each time the task is started
	// it is possible to return != 0 (e.g. when initialization fails) then the task is not executed further
	return 0;
}

int HumanPredictionActivity::on_execute()
{
	// this method is called from an outside loop,
	// hence, NEVER use an infinite loop (like "while(1)") here inside!!!
	// also do not use blocking calls which do not result from smartsoft kernel

	double decaySpeed = COMP->getGlobalState().getConfiguration().getDecaySpeed();

	COMP->humanBehaviourPredictorMutex.acquire();
	{
		for (int bodyIndex = 0; bodyIndex < MAX_BODIES; bodyIndex++)
		{
			for (int axisIndex = 0; axisIndex < 3; axisIndex++)
			{
				COMP->maxPosition[bodyIndex][axisIndex] -= decaySpeed / 30.0;
				COMP->minPosition[bodyIndex][axisIndex] += decaySpeed / 30.0;
			}
		}
	}
	COMP->humanBehaviourPredictorMutex.release();

	// it is possible to return != 0 (e.g. when the task detects errors), then the outer loop breaks and the task stops
	return 0;
}

int HumanPredictionActivity::on_exit()
{
	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	return 0;
}

void HumanPredictionActivity::CalculatePredictions(const DomainHumanTracking::CommHumanPositionsAndVelocities &input, DomainHumanTracking::CommHumanPositionPredictions &output)
{
	const bool displayDebugInfo = COMP->getGlobalState().getSettings().getDebug_info();		// Controls if debug information is printed

	// Calculate the transformation from sensor frame to world frame
	CartesianPosition sensorPosition;
	sensorPosition[0] = COMP->getGlobalState().getSensorPosition().getX();
	sensorPosition[1] = COMP->getGlobalState().getSensorPosition().getY();
	sensorPosition[2] = COMP->getGlobalState().getSensorPosition().getZ();
	sensorPosition[3] = COMP->getGlobalState().getSensorPosition().getRoll();
	sensorPosition[4] = COMP->getGlobalState().getSensorPosition().getElevation();
	sensorPosition[5] = COMP->getGlobalState().getSensorPosition().getAzimuth();
	Transformation sensorToWorld;
	sensorToWorld.FromCartesianPosition(sensorPosition);

	if (displayDebugInfo)
	{
		std::cout << "sensorToWorld:" << std::endl;
		sensorToWorld.Print();
	}

	if (displayDebugInfo && (input.getBodyDataSize() == 0))
	{
		std::cout << "Null data sent (no bodies)." << std::endl;
	}

	// Make the PredictedData array answer variable the required size
	output.resizePredictedData(input.getBodyDataSize());

	bool trackedFlags[BodyData::numberOfJoints];
	
	// Process all of the joints in all of the bodies
	for (int bodyIndex = 0; (bodyIndex < input.getBodyDataSize()) && (bodyIndex < MAX_BODIES); bodyIndex++)
	{
		CartesianPosition theBodyPositions[BodyData::numberOfJoints]; // this will store the joint positions for one body.
		CartesianPosition theBodyVelocities[BodyData::numberOfJoints]; // this will store the joint velocities for one body.
		DomainHumanTracking::CommPredictedData predictedData;
		for (int jointIndex = 0; jointIndex < input.getBodyDataElemAtPos(bodyIndex).getJointDataSize(); jointIndex++) {
			trackedFlags[jointIndex]= false;
			if ( input.getBodyDataElemAtPos(bodyIndex).getJointDataElemAtPos(jointIndex).getIsJointTracked() == DomainHumanTracking::JointTrackingStateType::TRACKED) {
				trackedFlags[jointIndex]= true; // note that the joint is tracked (for later check that all needed joints are tracked)
				CartesianPosition jointPosition;
				jointPosition = CartesianPosition::Zeros();
				jointPosition[0] = input.getBodyDataElemAtPos(bodyIndex).getJointDataElemAtPos(jointIndex).getJointPosition3d().getX();
				jointPosition[1] = input.getBodyDataElemAtPos(bodyIndex).getJointDataElemAtPos(jointIndex).getJointPosition3d().getY();
				jointPosition[2] = input.getBodyDataElemAtPos(bodyIndex).getJointDataElemAtPos(jointIndex).getJointPosition3d().getZ();
				jointPosition = sensorToWorld.Transform(jointPosition);
				jointPosition = COMP->worldToOutputFrame.InverseTransform(jointPosition);
				theBodyPositions[jointIndex] = jointPosition;

				CartesianPosition jointVelocity;
				CartesianPosition origin;
				jointVelocity = CartesianPosition::Zeros();
				origin = CartesianPosition::Zeros();
				jointVelocity[0] = COMP->duration * input.getBodyDataElemAtPos(bodyIndex).getJointDataElemAtPos(jointIndex).getJointVelocity3d().getX();
				jointVelocity[1] = COMP->duration * input.getBodyDataElemAtPos(bodyIndex).getJointDataElemAtPos(jointIndex).getJointVelocity3d().getY();
				jointVelocity[2] = COMP->duration * input.getBodyDataElemAtPos(bodyIndex).getJointDataElemAtPos(jointIndex).getJointVelocity3d().getZ();
				jointVelocity = sensorToWorld.Transform(jointVelocity);
				origin = sensorToWorld.Transform(origin);
				jointVelocity = COMP->worldToOutputFrame.InverseTransform(jointVelocity);
				origin = COMP->worldToOutputFrame.InverseTransform(origin);
				jointVelocity = jointVelocity - origin;	 // just need the rotation part of the velocity, so subtract the transformed origin offset
				theBodyVelocities[jointIndex] = jointVelocity;
			}
		} // Joint index loop
		// check that all needed joints have been tracked. If not, revert to simple behaviour prediction.
		if ( trackedFlags[BodyData::SpineBase] && trackedFlags[BodyData::SpineShoulder] &&
			trackedFlags[BodyData::ShoulderLeft] && trackedFlags[BodyData::ElbowLeft] && trackedFlags[BodyData::WristLeft] &&
			trackedFlags[BodyData::HandLeft] &&
			trackedFlags[BodyData::ShoulderRight] && trackedFlags[BodyData::ElbowRight] && trackedFlags[BodyData::WristRight] &&
			trackedFlags[BodyData::HandRight]) 
		{
			// all required joints present. Use skeleton extrapolation
            // first use simple extrapolation for head, neck, hips, legs as these do not take part in spine/arm kinematics
            for (int jointIndex = 0; jointIndex < input.getBodyDataElemAtPos(bodyIndex).getJointDataSize(); jointIndex++) {
                if (jointIndex == BodyData::Neck || jointIndex == BodyData::Head
                    || jointIndex == BodyData::HipLeft || jointIndex == BodyData::KneeLeft
                    || jointIndex == BodyData::AnkleLeft || jointIndex == BodyData::FootLeft
                    || jointIndex == BodyData::HipRight || jointIndex == BodyData::KneeRight
                    || jointIndex == BodyData::AnkleRight || jointIndex == BodyData::FootRight) {
                    if (trackedFlags[jointIndex]) {
                        CartesianPosition jointStartingPosition = theBodyPositions[jointIndex];
                        CartesianPosition jointEndingPosition = theBodyPositions[jointIndex] + theBodyVelocities[jointIndex] * COMP->duration;
                        for (int axisIndex = 0; axisIndex < 3; axisIndex++)
                        {
                            // Check start positions
                            if (jointStartingPosition[axisIndex] > COMP->maxPosition[bodyIndex][axisIndex])
                                COMP->maxPosition[bodyIndex][axisIndex] = jointStartingPosition[axisIndex];
        
                            if (jointStartingPosition[axisIndex] < COMP->minPosition[bodyIndex][axisIndex])
                                COMP->minPosition[bodyIndex][axisIndex] = jointStartingPosition[axisIndex];
        
                            // Check final positions
                            if (jointEndingPosition[axisIndex] > COMP->maxPosition[bodyIndex][axisIndex])
                                COMP->maxPosition[bodyIndex][axisIndex] = jointEndingPosition[axisIndex];
                            if (jointEndingPosition[axisIndex] < COMP->minPosition[bodyIndex][axisIndex])
                                COMP->minPosition[bodyIndex][axisIndex] = jointEndingPosition[axisIndex];
                        } // axis index loop
                    } // if tracked
                } // end if jointIndex select
            } // joint loop
            
            // next the spine/arm kinematics
            
			Skeleton Skel;
			Skeleton::PositionVelocityDataSet pvs;
			pvs.SpineBasePosition = fourvector(theBodyPositions[BodyData::SpineBase][0],	theBodyPositions[BodyData::SpineBase][1],	 theBodyPositions[BodyData::SpineBase][2]);
			pvs.SpineTopPosition = fourvector(theBodyPositions[BodyData::SpineShoulder][0],	 theBodyPositions[BodyData::SpineShoulder][1],	theBodyPositions[BodyData::SpineShoulder][2]);
			pvs.ShoulderLeftPosition = fourvector(theBodyPositions[BodyData::ShoulderLeft][0],	theBodyPositions[BodyData::ShoulderLeft][1],  theBodyPositions[BodyData::ShoulderLeft][2]);
			pvs.ElbowLeftPosition = fourvector(theBodyPositions[BodyData::ElbowLeft][0],	theBodyPositions[BodyData::ElbowLeft][1],	 theBodyPositions[BodyData::ElbowLeft][2]);
			pvs.WristLeftPosition = fourvector(theBodyPositions[BodyData::WristLeft][0],	theBodyPositions[BodyData::WristLeft][1],	 theBodyPositions[BodyData::WristLeft][2]);
			pvs.HandLeftPosition = fourvector(theBodyPositions[BodyData::HandLeft][0],	theBodyPositions[BodyData::HandLeft][1],  theBodyPositions[BodyData::HandLeft][2]);
			pvs.ShoulderRightPosition = fourvector(theBodyPositions[BodyData::ShoulderRight][0],	theBodyPositions[BodyData::ShoulderRight][1],	 theBodyPositions[BodyData::ShoulderRight][2]);
			pvs.ElbowRightPosition = fourvector(theBodyPositions[BodyData::ElbowRight][0],	theBodyPositions[BodyData::ElbowRight][1],	theBodyPositions[BodyData::ElbowRight][2]);
			pvs.WristRightPosition = fourvector(theBodyPositions[BodyData::WristRight][0],	theBodyPositions[BodyData::WristRight][1],	theBodyPositions[BodyData::WristRight][2]);
			pvs.HandRightPosition = fourvector(theBodyPositions[BodyData::HandRight][0],	theBodyPositions[BodyData::HandRight][1],	 theBodyPositions[BodyData::HandRight][2]);

			pvs.SpineBaseVelocity = fourvector(theBodyVelocities[BodyData::SpineBase][0], theBodyVelocities[BodyData::SpineBase][1], theBodyVelocities[BodyData::SpineBase][2]);
			pvs.SpineTopVelocity = fourvector(theBodyVelocities[BodyData::SpineShoulder][0],	theBodyVelocities[BodyData::SpineShoulder][1],	theBodyVelocities[BodyData::SpineShoulder][2]);
			pvs.ShoulderLeftVelocity = fourvector(theBodyVelocities[BodyData::ShoulderLeft][0],	 theBodyVelocities[BodyData::ShoulderLeft][1],	theBodyVelocities[BodyData::ShoulderLeft][2]);
			pvs.ElbowLeftVelocity = fourvector(theBodyVelocities[BodyData::ElbowLeft][0],	 theBodyVelocities[BodyData::ElbowLeft][1],	 theBodyVelocities[BodyData::ElbowLeft][2]);
			pvs.WristLeftVelocity = fourvector(theBodyVelocities[BodyData::WristLeft][0],	 theBodyVelocities[BodyData::WristLeft][1],	 theBodyVelocities[BodyData::WristLeft][2]);
			pvs.HandLeftVelocity = fourvector(theBodyVelocities[BodyData::HandLeft][0],	 theBodyVelocities[BodyData::HandLeft][1],	theBodyVelocities[BodyData::HandLeft][2]);
			pvs.ShoulderRightVelocity = fourvector(theBodyVelocities[BodyData::ShoulderRight][0],	 theBodyVelocities[BodyData::ShoulderRight][1],	 theBodyVelocities[BodyData::ShoulderRight][2]);
			pvs.ElbowRightVelocity = fourvector(theBodyVelocities[BodyData::ElbowRight][0],	 theBodyVelocities[BodyData::ElbowRight][1],  theBodyVelocities[BodyData::ElbowRight][2]);
			pvs.WristRightVelocity = fourvector(theBodyVelocities[BodyData::WristRight][0],	 theBodyVelocities[BodyData::WristRight][1],  theBodyVelocities[BodyData::WristRight][2]);
			pvs.HandRightVelocity = fourvector(theBodyVelocities[BodyData::HandRight][0],	 theBodyVelocities[BodyData::HandRight][1],	 theBodyVelocities[BodyData::HandRight][2]);

			cout << pvs << endl;
			Skel.setPose(pvs); // sets up the skeleton model
			double xmin, xmax, ymin, ymax, zmin, zmax;
			xmin = ymin = zmin = +10000000.0;
			xmax = ymax = zmax = -10000000.0;
			double t = COMP->duration / 4.0;
			int numSteps = 4;
			Skel.findPredictedEnvelopeLimits(t, numSteps, /* inputs/outputs */ xmin, xmax, ymin, ymax, zmin, zmax); // performs forward kinematics and determines the bounding cube.
			cout << "Predicted bounding box=" << endl;
			cout << "t =" << t << " numSteps=" << numSteps << endl;
			cout << "x = [" <<	xmin << " <-> " << xmax << "]" << endl;
			cout << "y = [" <<	ymin << " <-> " << ymax << "]" << endl;
			cout << "z = [" <<	zmin << " <-> " << zmax << "]" << endl;

			if (xmax > COMP->maxPosition[bodyIndex][0])
						COMP->maxPosition[bodyIndex][0] = xmax;
			if (xmin < COMP->minPosition[bodyIndex][0])
						COMP->minPosition[bodyIndex][0] = xmin;

			if (ymax > COMP->maxPosition[bodyIndex][1])
						COMP->maxPosition[bodyIndex][1] = ymax;
			if (ymin < COMP->minPosition[bodyIndex][1])
						COMP->minPosition[bodyIndex][1] = ymin;

			if (zmax > COMP->maxPosition[bodyIndex][2])
						COMP->maxPosition[bodyIndex][2] = zmax;
			if (zmin < COMP->minPosition[bodyIndex][2])
						COMP->minPosition[bodyIndex][2] = zmin;
		} else {
			// not all joints present. Use joint extrapolation
			// not all joints were tracked, so fall back to the simple prediction method
			// use the simple prediction method
			for (int jointIndex = 0; jointIndex < input.getBodyDataElemAtPos(bodyIndex).getJointDataSize(); jointIndex++) {
				if (trackedFlags[jointIndex]) {
					CartesianPosition jointStartingPosition = theBodyPositions[jointIndex];
					CartesianPosition jointEndingPosition = theBodyPositions[jointIndex] + theBodyVelocities[jointIndex] * COMP->duration;
					for (int axisIndex = 0; axisIndex < 3; axisIndex++)
					{
						// Check start positions
						if (jointStartingPosition[axisIndex] > COMP->maxPosition[bodyIndex][axisIndex])
							COMP->maxPosition[bodyIndex][axisIndex] = jointStartingPosition[axisIndex];
	
						if (jointStartingPosition[axisIndex] < COMP->minPosition[bodyIndex][axisIndex])
							COMP->minPosition[bodyIndex][axisIndex] = jointStartingPosition[axisIndex];
	
						// Check final positions
						if (jointEndingPosition[axisIndex] > COMP->maxPosition[bodyIndex][axisIndex])
							COMP->maxPosition[bodyIndex][axisIndex] = jointEndingPosition[axisIndex];
						if (jointEndingPosition[axisIndex] < COMP->minPosition[bodyIndex][axisIndex])
							COMP->minPosition[bodyIndex][axisIndex] = jointEndingPosition[axisIndex];
					} // axis index loop
				} // if tracked
			} // joint loop
		} // else clause
	
		// Construct the output data
		predictedData.setValid(true);
		predictedData.setXMaxLimit(COMP->maxPosition[bodyIndex][0]);
		predictedData.setXMinLimit(COMP->minPosition[bodyIndex][0]);
		predictedData.setYMaxLimit(COMP->maxPosition[bodyIndex][1]);
		predictedData.setYMinLimit(COMP->minPosition[bodyIndex][1]);
		predictedData.setZMaxLimit(COMP->maxPosition[bodyIndex][2]);
		predictedData.setZMinLimit(COMP->minPosition[bodyIndex][2]);

		// Save the predictions for this body
		output.setPredictedDataElemAtPos(bodyIndex, predictedData);
		if (displayDebugInfo) {
			std::cout << "X: " << predictedData.getXMinLimit() << " -> " << predictedData.getXMaxLimit() << std::endl;
			std::cout << "Y: " << predictedData.getYMinLimit() << " -> " << predictedData.getYMaxLimit() << std::endl;
			std::cout << "Z: " << predictedData.getZMinLimit() << " -> " << predictedData.getZMaxLimit() << std::endl;
		}
	} // Body index loop
}

