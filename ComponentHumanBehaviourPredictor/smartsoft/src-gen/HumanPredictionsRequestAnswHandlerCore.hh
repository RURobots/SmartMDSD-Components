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
// Please do not modify this file. It will be re-generated
// running the code generator.
//--------------------------------------------------------------------------
#ifndef _HUMANPREDICTIONSREQUESTANSWHANDLER_CORE_HH
#define _HUMANPREDICTIONSREQUESTANSWHANDLER_CORE_HH
		
#include "aceSmartSoft.hh"

#include <DomainHumanTracking/CommHumanPositionPredictions.hh>
#include <DomainHumanTracking/CommHumanPositionPredictionsRequest.hh>

// include the input interfaces (if any)

// include all interaction-observer interfaces
#include <HumanPredictionsRequestAnswHandlerObserverInterface.hh>

class HumanPredictionsRequestAnswHandlerCore 
:	public Smart::IInputHandler<std::pair<Smart::QueryIdPtr,DomainHumanTracking::CommHumanPositionPredictionsRequest>>
,	public Smart::TaskTriggerSubject
{
private:
virtual void handle_input(const std::pair<Smart::QueryIdPtr,DomainHumanTracking::CommHumanPositionPredictionsRequest> &input) override {
	this->handleQuery(input.first, input.second);
}


	virtual void updateAllCommObjects();

/**
 * Implementation of the Subject part of an InteractionObserver
 */
private:
	std::mutex interaction_observers_mutex;
	std::list<HumanPredictionsRequestAnswHandlerObserverInterface*> interaction_observers;
protected:
	void notify_all_interaction_observers();
public:
	void attach_interaction_observer(HumanPredictionsRequestAnswHandlerObserverInterface *observer);
	void detach_interaction_observer(HumanPredictionsRequestAnswHandlerObserverInterface *observer);

protected:
	
public:
	using IQueryServer = Smart::IQueryServerPattern<DomainHumanTracking::CommHumanPositionPredictionsRequest, DomainHumanTracking::CommHumanPositionPredictions>;
	using QueryId = Smart::QueryIdPtr;
	HumanPredictionsRequestAnswHandlerCore(IQueryServer *server);
	virtual ~HumanPredictionsRequestAnswHandlerCore() = default;
	
protected:
	IQueryServer *server;
	//this user-method has to be implemented in derived classes
	virtual void handleQuery(const QueryId &id, const DomainHumanTracking::CommHumanPositionPredictionsRequest& request) = 0;
};
#endif
