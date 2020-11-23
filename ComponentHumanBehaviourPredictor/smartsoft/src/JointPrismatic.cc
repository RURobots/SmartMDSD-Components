
/******************************************************************************/
/*                                                                            */
/*  Name:           JointPrismatic.cpp                                        */
/*  Purpose:        Implementation of a prismatic joint                       */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/

#include "JointPrismatic.hh"

Transform JointPrismatic::ForwardKinematics(double velocity, const double t) const {
    Transform T0,TZ;
    double OffsetAlongZ;
    // note this first stage is a bit different as it handles the linear velocity of the spine base. All the other velocities are angular.
    OffsetAlongZ = velocity * t ;
    TZ.unit();
    TZ.theMatrix[2][3] = OffsetAlongZ;
    T0 = T * TZ;  // add the offest along the z-axis to the DH Transform.
    return T0;
};

void JointPrismatic::loadLinear_OnlyTransform(fourvector StartLocation, fourvector unit_velocity_In) {
    unit_velocity = unit_velocity_In;
    toEndPointTransform.reset();
    toEndPointTransform.setOffset(StartLocation);
}

Transform JointPrismatic::ForwardKinematics_LinearOnly(double velocity, const double t) const {
    Transform T;
    T.reset();
    T.setOffset(unit_velocity * velocity * t + toEndPointTransform.get_offset());
    return T;
}




