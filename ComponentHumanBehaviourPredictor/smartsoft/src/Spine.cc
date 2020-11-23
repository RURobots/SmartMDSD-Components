
/******************************************************************************/
/*                                                                            */
/*  Name:           Spine.cpp                                                 */
/*  Purpose:        Implementation of Spine kinematics                        */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/


#include "Spine.hh"

Transform Spine::findPredictedEnvelopeLimits(const double t,
                /* outputs */ double& xmin, double& xmax, double& ymin, double& ymax, double& zmin, double& zmax)  {

    fourvector spineBaseTravel;
    Transform SpineTopNewPosition = ForwardKinematics(t, spineBaseTravel);
    //cout << "SpineTopNewPosition=" << SpineTopNewPosition << endl;
    // first account for the initial base and top positions

    //cout << "xmin=" << xmin << " xmax=" << xmax << " ymin=" << ymin << " ymax=" << ymax << " zmin=" << zmin << " zmax=" << zmax << endl;
    fourvector pos = SpineTopNewPosition.get_offset();
    double x = pos[0]; double y=pos[1]; double z=pos[2];
    if (x < xmin)
        xmin = x;
    if (x > xmax)
        xmax = x;
    if (y < ymin)
        ymin = y;
    if (y > ymax)
        ymax = y;
    if (z < zmin)
        zmin = z;
    if (z > zmax)
        zmax = z;

    //cout << "xmin=" << xmin << " xmax=" << xmax << " ymin=" << ymin << " ymax=" << ymax << " zmin=" << zmin << " zmax=" << zmax << endl;
    
    pos = baseStartPosition + spineBaseTravel;
    x = pos[0]; y=pos[1]; z=pos[2];
    if (x < xmin)
        xmin = x;
    if (x > xmax)
        xmax = x;
    if (y < ymin)
        ymin = y;
    if (y > ymax)
        ymax = y;
    if (z < zmin)
        zmin = z;
    if (z > zmax)
        zmax = z;

    //cout << "xmin=" << xmin << " xmax=" << xmax << " ymin=" << ymin << " ymax=" << ymax << " zmin=" << zmin << " zmax=" << zmax << endl;

    return SpineTopNewPosition;
}

Transform Spine::getSpineTopTransform() const {
    Transform T;
    T.unit();
    T.setOffset(spineTop);
    return T;
}

Transform Spine::getBaseTransform() const {
    Transform T;
    T.unit();
    T.setOffset(basePositionCurrent);
    return T;
}

Spine::JointSet Spine::ExtractPositions() const {
    Spine::JointSet js;
    js.SpineBase = basePositionCurrent;
    js.SpineTop = spineTop;
    return js;
};

void Spine::setPose(const Spine::PositionVelocityDataSet& visionData) {
    fourvector axis1;
    fourvector zaxis = fourvector(0.0, 0.0, 1.0);
    Transform origin;
    origin.unit();
    spineTop = visionData.SpineTopPosition;

    baseStartPosition = visionData.SpineBasePosition;
    basePositionCurrent = baseStartPosition;
    baseVelocity = visionData.SpineBaseVelocity;
    //cout << "spine base=" << baseStartPosition << " spine top=" << visionData.SpineTopPosition << endl;
    topRelative = visionData.SpineTopPosition - baseStartPosition;
    //cout << "spine base->top=" <<  topRelative << endl;
    fourvector spinetopRelativeVelocity = visionData.SpineTopVelocity - visionData.SpineBaseVelocity;
    double spineTopSpeed = spinetopRelativeVelocity.norm();
    if (spineTopSpeed > 0.0) {
        
        //cout << "Base start position=" << baseStartPosition << endl;
        fourvector spinetopUnitRelativeVelocity = spinetopRelativeVelocity / spineTopSpeed;
        fourvector rotationAxis;
        G.crossProduct( zaxis, spinetopUnitRelativeVelocity, rotationAxis); // axis1 = velocity cross z-axis
        if (rotationAxis.norm() > 0.0) {
            spineRotationAxis = rotationAxis / rotationAxis.norm();
            fourvector radiusVector;
            fourvector tangentVector;
            G.obtainRadiusOfActionAndTangent(spineRotationAxis, topRelative, radiusVector, tangentVector);
            rotationRate = spineTopSpeed / radiusVector.norm();
        } else {
            //must be in z-direction, set axis to zero (so there will be no rotation).
            spineRotationAxis = fourvector(0.0, 0.0, 0.0);
            rotationRate = 0.0;
        }
        //cout << "Spine base position= " << baseStartPosition << endl;
        //cout << "Spine base velocity=" << baseVelocity << endl;
        //cout << "top relative=" << topRelative << endl;
        //cout << "axis=" << spineRotationAxis << endl;
        //cout << "rate=" << rotationRate << " rads/s = " << rotationRate * 180.0 / M_PI << " deg/s " << endl;
        //cout << "estimated spinetop travel distance after 1s =" << baseVelocity.norm() + topRelative.norm() * rotationRate << endl;
    }
    else {
        // spine top rotation is zero.
        rotationRate = 0.0;
        axis1 = fourvector(0.0, 0.0, 0.0);
        return;
    }
}


// note global coordinates used.
// FindDHTransforms  finds the DH params when fed the joint positions and axes for the home position of the spine.
//void Spine::FindDHTransforms(const Spine::PositionVelocityDataSet& visionData) {
//   setPose(visionData);
//}

Transform Spine::ForwardKinematics(const double t, fourvector& baseTravelDistance) {
    baseTravelDistance = baseVelocity * t;
    double angle = rotationRate * t;
    basePositionCurrent = baseStartPosition + baseTravelDistance;
    Transform spineTransform = G.createRotationAboutAxis(spineRotationAxis, angle);
    fourvector top = spineTransform * topRelative;
    //cout << "Spine::ForwardKinematics angle(deg)=" << angle * 180.0 / M_PI << " topRelative=" << topRelative << " rotated=" << top << endl;
    //cout << "baseTravelDistance =" << baseTravelDistance << endl;
    spineTop = top + basePositionCurrent;
    //cout << "spineTop =" << spineTop << endl;
   Transform T;
    T.unit();
    T.setOffset(spineTop);
    return T;
}

std::ostream& operator<<(std::ostream& os, const Spine::JointSet_Axes& x) {
    return os;
}

std::ostream& operator<<(std::ostream& os, const Spine::JointSet_Scalars& x) {
    return os;
}

std::ostream& operator<<(std::ostream& os, const Spine::PositionVelocityDataSet& x) {
    return os;
}

std::ostream& operator<<(std::ostream& os, const Spine::JointSet& x)  {
    return os;
}

/*
std::ostream& operator<<(std::ostream& os, const SkeletonBackAndArm::JointSet& x) {
    os << "========== JointSet ==========" << endl;
    os << "SpineBaseLinear=" << x.SpineBase << endl;
    os << "SpineTop=       " << x.SpineTop << endl;
    os << "Shoulder=      " << x.Shoulder << endl;
    os << "Elbow=          " << x.Elbow << endl;
    os << "Wrist=          " << x.Wrist << endl;
    os << "Hand=           " << x.Hand << endl;
    os << endl;
    return os;
}
*/

