
//******************************************************************************/
/*                                                                            */
/*  Name:           Geometry.hh                                               */
/*  Purpose:        Implementation of a various geometric functions           */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/

#ifndef Geometry_hpp
#define Geometry_hpp

#include <stdio.h>
#include <iostream>

#include "FourVector.hh"
#include "Transform.hh"
#include <vector>
#include <cmath>

using namespace std;

class Geometry {
public:

    const double rightAngleRads = M_PI / 2.0;
    
    double dotProduct(fourvector u, fourvector v) const;
    void crossProduct(fourvector u, fourvector v, fourvector& out) const;
    void CommonNormalNonParallel(fourvector P0, fourvector u, fourvector Q0, fourvector v, fourvector& Pout, fourvector& Qout, fourvector& computedWOut) const;
    void CommonNormalParallel(fourvector P0, fourvector Q0, fourvector v, fourvector& P, fourvector& Q, fourvector& w) const;
    void FindCommonNormalEndPoints(fourvector P0, fourvector u, fourvector Q0, fourvector& v,
                                   fourvector& P, fourvector& Q, fourvector& w) const;
    void RequiredRotation(fourvector fromVec, fourvector toVec, fourvector expectedAxis, double& theAngle) const;
    
    void obtainAxisAndAngle(fourvector vector1, fourvector axisPoint1, fourvector vector2, fourvector axisPoint2, fourvector& UnitRotationVector, double& RotationVectorAngle) const;
    fourvector ComputeRotateTangent(fourvector Leg, fourvector LegEndpointVelcocity, fourvector UnitAxis) const;
    void ComputeAxisLegRotationRate(const fourvector EndPosition, const fourvector EndVelocity, const fourvector AxisPointPosition, const fourvector AxisPointVelocity, const fourvector StartPointPosition, const fourvector StartPointVelocity,
                                /* outputs */ fourvector& JointUnitAxis, double& jointLegOpeningRate) const;
        
    // find angle which zeroreference must be rotated about axis to be in same plane as x
    // axis_unit ans zeroReference_unit must be unit vectors. Also zeroReference_unit  must be perpendicular to axis_unit.
    double obtainAngleAboutAxis(const fourvector& axis_unit, const fourvector& zeroReference_unit, const fourvector& x) const ;
    
    // obtainRadiusOfActionAndTangent
    // finds the perpendicular component of axis along the given leg. The tangent vector is also returned (i.e. the direction the leg
    // will move in if rotated along the axis.
    // If the radius length is zero, the tangentvector is zero and a false value returned.
    bool obtainRadiusOfActionAndTangent(const fourvector& axis, const fourvector& leg, fourvector& radiusVector, fourvector& tangentVector) const;
    
    // obtainRotateTangent
    // finds the direction of the leg movement for axis perpendicular to the baseline and leg, velocity in direction of increasing angle from baseline.
    // returns false if the axis cannot be obtained (i.e. baseline and leg parallel)
    bool obtainPlaneTangent(const fourvector& baseline, const fourvector& leg, fourvector& tangentUnitVector) const;
    
    
    fourvector normaliseVector(const fourvector& v, double& length) const;
    fourvector normaliseVector(const fourvector& v) const;
    
    Transform createRotationAboutAxis(fourvector& unitAxis, double angle) const;
                               
    friend class GeometryTest;
    
    
};


#endif /* Geometry_hpp */
