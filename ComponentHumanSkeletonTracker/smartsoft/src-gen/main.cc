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
#include <iostream>
#include "ComponentHumanSkeletonTracker.hh"

int main(int argc, char *argv[])
{
	std::cout << "main...\n";
	// initialize component infrastructure (loading ini-file, creating ports, tasks, etc.)
	ComponentHumanSkeletonTracker::instance()->init(argc, argv);
	// run component infrastructure until the component is commanded to shutdown
	ComponentHumanSkeletonTracker::instance()->run();
	// clean-up component's internal resources (deleting ports, tasks, etc.)
	ComponentHumanSkeletonTracker::instance()->fini();
	// destroy the component's singleton
	ComponentHumanSkeletonTracker::deleteInstance();
	std::cout << "... main() end. return 0." << std::endl;
	return 0;
}
