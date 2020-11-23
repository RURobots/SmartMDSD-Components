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

#include "Vector.h"
#include "CartesianPosition.h"
#include "Transformation.h"
using namespace RUR_RobotMaths;

#include "HumanPredictionsRequestAnswHandler.hh"
#include "ComponentSimpleHumanBehaviourPredictor.hh"

#define MAX_BODIES 6

HumanPredictionsRequestAnswHandler::HumanPredictionsRequestAnswHandler(IQueryServer *server)
:	HumanPredictionsRequestAnswHandlerCore(server)
{
	// Get the sensor position and calculate the transformation from sensor frame into world frame
}

void HumanPredictionsRequestAnswHandler::handleQuery(const Smart::QueryIdPtr &id, const DomainHumanTracking::CommHumanPositionPredictionsRequest& request) 
{
	std::cout << "Request for simple prediction received." << std::endl;

	const bool displayDebugInfo = COMP->getGlobalState().getSettings().getDebug_info();		// Controls if debug information is printed

	COMP->simpleHumanBehaviourPredictorMutex.acquire();
	{
		// Get required frame of reference and convert to world frame
		CartesianPosition outputFramePosition;
		outputFramePosition[0] = request.getFrameOfReference().get_x();
		outputFramePosition[1] = request.getFrameOfReference().get_y();
		outputFramePosition[2] = request.getFrameOfReference().get_z();
		outputFramePosition[3] = request.getFrameOfReference().get_roll();
		outputFramePosition[4] = request.getFrameOfReference().get_elevation();
		outputFramePosition[5] = request.getFrameOfReference().get_azimuth();
		COMP->worldToOutputFrame.FromCartesianPosition(outputFramePosition);

		if (displayDebugInfo)
		{
			std::cout << "toOutputFrame:" << std::endl;
			COMP->worldToOutputFrame.Print();
		}

		// Calculate the time duration for the prediction
		COMP->duration = (request.getTimeFrame().getSec() * 1000000ll + request.getTimeFrame().getUsec()) / 1000000.0;

		if (displayDebugInfo)
		{
			std::cout << "duration: " << COMP->duration << std::endl;
		}

		// Call the method that calculates new predictions
		COMP->simpleHumanPredictionActivity->CalculatePredictions(COMP->humanPositionsAndVelocities, COMP->humanPositionPredictions);

		this->server->answer(id, COMP->humanPositionPredictions);
	}
	COMP->simpleHumanBehaviourPredictorMutex.release();
}
