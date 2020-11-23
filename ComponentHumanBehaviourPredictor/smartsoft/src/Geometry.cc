
/******************************************************************************/
/*                                                                            */
/*  Name:           Geometry.cpp                                              */
/*  Purpose:        Implementation of a various geometric functions           */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/


#include "Geometry.hh"
#include <math.h>

double Geometry::dotProduct(fourvector u, fourvector v) const {
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2] ;
};

void Geometry::crossProduct(fourvector u, fourvector v, fourvector& out) const {
    out[0] = u[1]*v[2]-u[2]*v[1];
    out[1] = u[2]*v[0]-u[0]*v[2];
    out[2] = u[0]*v[1]-u[1]*v[0];
    //out[3] = 0;
};

void Geometry::CommonNormalNonParallel(fourvector P0, fourvector u, fourvector Q0, fourvector v,
                                       fourvector& Pout, fourvector& Qout, fourvector& computedWOut) const {
    fourvector w0  ;
    double uv, w0v, w0u, t, s;
    uv = dotProduct(u,v); //uv := u . v;
    crossProduct(u, v, computedWOut);
    w0 = Q0 - P0;
    w0v = dotProduct(w0,v); //w0v := w0 . v;
    w0u = dotProduct(w0,u); //w0v := w0 . u;
    t = (w0u*uv - w0v)/(1 + (-1)*uv*uv);
    s = (w0u - w0v*uv)/(1 + (-1)*uv*uv);
    Pout = P0 + u*s;
    Qout = Q0 + v*t;
};

void Geometry::CommonNormalParallel(fourvector P0, fourvector Q0, fourvector v, fourvector& P, fourvector& Q, fourvector& w) const {
    fourvector w0,  l  ;
    double projectl;
    w0 = Q0 - P0;
    projectl = dotProduct(w0,v); // w0 . v;
    l = v * projectl;
    Q = Q0 - l;
    w = Q - P0;
    P = P0;
};

void Geometry::FindCommonNormalEndPoints(fourvector P0, fourvector u, fourvector Q0, fourvector& v,
                                         fourvector& P, fourvector& Q, fourvector& w) const {
    double wlen;
    crossProduct(u, v, w); // w := u cross v;
    wlen = w.norm();
    if (wlen < 0.000001)
        return CommonNormalParallel(P0, Q0, v, P, Q, w);
    else
        return CommonNormalNonParallel(P0, u, Q0, v, P, Q, w);
};

void Geometry::RequiredRotation(fourvector fromVec, fourvector toVec, fourvector expectedAxis, double& theAngle) const {
    fourvector fromVec_unit = fromVec / fromVec.norm();
    fourvector toVec_unit = toVec / toVec.norm();
    fourvector crossVec, crossVecUnitLen;
    double cosAngle, crossVecLen, isParallelOrAnti,sinAngle;
    crossProduct(fromVec_unit, toVec_unit, crossVec);
    cosAngle = dotProduct(fromVec_unit, toVec_unit); // fromVec . toVec;
    if (cosAngle == 1.0) {
        theAngle=0;
        return;
    }
    if (cosAngle == -1.0) {
        theAngle=M_PI;
        return;
    }
    crossVecLen = crossVec.norm();
    crossVecUnitLen = crossVec/crossVecLen;
    isParallelOrAnti = dotProduct(crossVecUnitLen,expectedAxis);
    sinAngle = crossVecLen;
    if (sinAngle > 1.0)
        sinAngle = 1.0; // sometime sinAngle comes out to 1.00000000000002
    if (sinAngle < -1.0)
        sinAngle = -1.0; // sometime sinAngle comes out to -1.00000000000002
    theAngle = asin(sinAngle);
    if (cosAngle < 0.0)
        theAngle = M_PI - theAngle;
    if (isParallelOrAnti < 0.0)
        theAngle = -theAngle;
    return; // answer in theAngle
};



void Geometry::obtainAxisAndAngle(fourvector vector1, fourvector axisPoint1, fourvector vector2, fourvector axisPoint2, fourvector& UnitRotationVector, double& RotationVectorAngle) const {
    fourvector changeVector, RotationVector, line1, line2;
    double  sinAngle;
    line1 = vector1 - axisPoint1;
    line2 = vector2 - axisPoint2;
    changeVector = line2 - line1;
    crossProduct(line1, changeVector, RotationVector);
    sinAngle = RotationVector.norm()/(line1.norm() * changeVector.norm());
    RotationVectorAngle = asin(sinAngle);
    UnitRotationVector = RotationVector/RotationVector.norm();
}

void Geometry::ComputeAxisLegRotationRate(const fourvector EndPosition, const fourvector EndVelocity, const fourvector AxisPointPosition, const fourvector AxisPointVelocity, const fourvector StartPointPosition, const fourvector StartPointVelocity,
                                      /* outputs */ fourvector& JointUnitAxis, double& jointLegOpeningRate) const {
    fourvector FirstLeg, SecondLeg, JointAxis, StartLegRotationTangent, EndLegRotationTangent, StartLegRotationRate, EndLegRotationRate;
    fourvector FirstLegRelativeVelocity, SecondLegRelativeVelocity;
    fourvector StartLegRotationTangentUnitVector,  EndLegRotationTangentUnitVector;
    double StartLegOpeningRate,EndLegOpeningRate;
    FirstLeg = StartPointPosition - AxisPointPosition;
    SecondLeg = EndPosition - AxisPointPosition;
    crossProduct(FirstLeg, SecondLeg, JointAxis);
    JointUnitAxis = JointAxis/JointAxis.norm();
    
    FirstLegRelativeVelocity = StartPointVelocity - AxisPointVelocity;
    SecondLegRelativeVelocity = EndVelocity - AxisPointVelocity;
    
    crossProduct(FirstLeg, JointUnitAxis, StartLegRotationTangent);
    StartLegRotationTangentUnitVector = StartLegRotationTangent/StartLegRotationTangent.norm();
    StartLegOpeningRate = dotProduct(StartLegRotationTangentUnitVector,FirstLegRelativeVelocity)/FirstLeg.norm();
    
    crossProduct(JointUnitAxis, SecondLeg, EndLegRotationTangent);
    EndLegRotationTangentUnitVector = EndLegRotationTangent/EndLegRotationTangent.norm();
    EndLegOpeningRate = dotProduct(EndLegRotationTangentUnitVector,SecondLegRelativeVelocity)/SecondLeg.norm();
    
    jointLegOpeningRate = StartLegOpeningRate + EndLegOpeningRate;
    
}

fourvector Geometry::ComputeRotateTangent(fourvector Leg, fourvector LegEndpointVelcocity, fourvector UnitAxis) const {
    fourvector tangentVector, tangentUnitVector, resultVector;
    crossProduct(UnitAxis, Leg, tangentVector);
    double angularVelocity;
    tangentUnitVector = tangentVector/tangentVector.norm();
    angularVelocity = dotProduct(tangentUnitVector,LegEndpointVelcocity); //resultVector := tangentUnitVector . LegEndpointVelcocity;
    return tangentUnitVector * angularVelocity;
}

// find angle which zeroreference must be rotated about axis to be in same plane as x
// axis_unit and zeroReference_unit must be unit vectors. Also zeroReference_unit  must be perpendicular to axis_unit.
double Geometry::obtainAngleAboutAxis(const fourvector& axis_unit, const fourvector& zeroReference_unit, const fourvector& x) const  {
    //cout << "axis_unit=" << axis_unit << endl;
    //cout << "zeroReference_unit=" << zeroReference_unit << endl;
    //cout << "x=" << x << endl;

    double angle;
    double parallelComponentLength = dotProduct(x, axis_unit);
    fourvector radiusVector = x - axis_unit * parallelComponentLength; // radiusVector is x projected onto plane perpendicular to axis_unit
    double radiusLength;
    fourvector unitRadiusVector = normaliseVector(radiusVector, radiusLength) ;
    if (radiusLength > 0.0000001) {
        // unitRadiusVector and zeroReference are in the plane perpendicular to axis
        // calculate the angle between them. Need to use crossproduct and scalar product to get quadrant.
        fourvector crossVector;
        crossProduct(zeroReference_unit, unitRadiusVector, crossVector); // angle between reference vector and projected x
        double sinAngle = dotProduct(crossVector, axis_unit); // if this is negative then final angle is negative
        double cosangle = dotProduct(zeroReference_unit, unitRadiusVector); // always positive
        angle = atan2(sinAngle,cosangle);
    } else {
        angle = 0.0;
        return false;
    }
    return angle;
}

fourvector Geometry::normaliseVector(const fourvector& v, double& length) const {
    length = v.norm();
    if (length == 0.0)
        return v;
    return v / length;
}

fourvector Geometry::normaliseVector(const fourvector& v) const {
    double length = v.norm();
    if (length == 0.0)
        return v;
    return v / length;
}


// obtainRadiusOfActionAndTangent
// finds the perpendicular component of axis along the given leg. The tangent vector is also returned (i.e. the direction the leg
// will move in if rotated along the axis.
// If the radius length is zero, the tangentvector is zero and a false value returned.
bool Geometry::obtainRadiusOfActionAndTangent(const fourvector& axis, const fourvector& leg, fourvector& radiusVector, fourvector& tangentVector) const {
    double axisLength;
    fourvector unitAxis = normaliseVector(axis, axisLength);
    if (axisLength== 0.0) {
        tangentVector = fourvector(0.0, 0.0, 0.0);
        radiusVector  = fourvector(0.0, 0.0, 0.0);
        return false;
    }
    double parallelComponentLength = dotProduct(leg, unitAxis);
    radiusVector = leg - unitAxis * parallelComponentLength;
    double radiusLength;
    fourvector unitRadiusVector = normaliseVector(radiusVector, radiusLength) ;
    if (radiusLength > 0.0000001) {
        crossProduct(unitAxis, unitRadiusVector, tangentVector);
    } else {
        tangentVector = fourvector(0.0, 0.0, 0.0);
        return false;
    }
    return true;
}

// obtainRotateTangent
// finds the direction of the leg movement for axis perpendicular to the baseline and leg, velocity in direction of increasing angle from baseline.
// returns false if the axis cannot be obtained (i.e. baseline and leg parallel)
bool Geometry::obtainPlaneTangent(const fourvector& baseline, const fourvector& leg, fourvector& tangentUnitVector) const {
    fourvector axisVector;
    crossProduct(leg, baseline, axisVector);
    double axisVectorLen;
    fourvector axisUnitVector = normaliseVector(axisVector, axisVectorLen) ;
    if (axisVectorLen < 0.0000001 ) {
        return false;
    }
    fourvector tangentVector;
    crossProduct(leg, axisUnitVector, tangentVector);
    if (tangentVector.norm() < 0.0000001) {
        return false;
    }
    tangentUnitVector = normaliseVector(tangentVector) ;
    return true;
}

Transform Geometry::createRotationAboutAxis(fourvector& unitAxis, double angle) const {
    Transform U, S, C, E, E2, Q;
    U.unit();
    E.GenerateCrossProductMatrix(unitAxis);
    E2 = E * E;
    S = E * sin(angle);
    C = E2 * ( 1.0 - cos(angle));
    //T = U + S + C;
    Q = U;
    Q.add(S);
    Q.add(C);
    return Q;
}


