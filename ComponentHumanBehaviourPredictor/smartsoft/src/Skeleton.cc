
/******************************************************************************/
/*                                                                            */
/*  Name:           Skeleton.cpp                                              */
/*  Purpose:        Implementation of a Skeleton kinematics class             */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/

#include "Skeleton.hh"
#include "Arm.hh"
#include "Transform.hh"
#include <limits>

void Skeleton::setPose(const Skeleton::PositionVelocityDataSet& visionData) {
    
    Transform spineTop;
    Spine::PositionVelocityDataSet SpineData;
    SpineData.SpineBasePosition = visionData.SpineBasePosition;
    SpineData.SpineBaseVelocity = visionData.SpineBaseVelocity;
    SpineData.SpineTopPosition = visionData.SpineTopPosition;
    SpineData.SpineTopVelocity = visionData.SpineTopVelocity;
    //cout << "SpineBasePosition =" << SpineData.SpineBasePosition << " SpineTopPosition=" << SpineData.SpineTopPosition << endl;
    //cout << "Base->Spine=" << SpineData.SpineTopPosition - SpineData.SpineBasePosition << endl;
    theSpine.setPose(SpineData);
    
    spineTop = theSpine.getSpineTopTransform();
    //cout << "spineTopTransform=" << spineTop << endl;
    //cout << "x=" << spineTop.get_x() << endl;
    //cout << "y=" << spineTop.get_y() << endl;
    //cout << "z=" << spineTop.get_z() << endl;
    //cout << "offset=" << spineTop.get_offset() << endl;

    Arm::PositionVelocityDataSet leftVisionData;
    Arm::PositionVelocityDataSet rightVisionData;
    
    leftVisionData.ShoulderPosition = visionData.ShoulderLeftPosition;
    leftVisionData.ShoulderVelocity = visionData.ShoulderLeftVelocity;
    leftVisionData.ElbowPosition = visionData.ElbowLeftPosition;
    leftVisionData.ElbowVelocity = visionData.ElbowLeftVelocity ;
    leftVisionData.WristPosition = visionData.WristLeftPosition;
    leftVisionData.WristVelocity = visionData.WristLeftVelocity ;
    leftVisionData.HandPosition = visionData.HandLeftPosition;
    leftVisionData.HandVelocity = visionData.HandLeftVelocity;
    
    //cout << "left arm=" << leftVisionData << endl;
    
    rightVisionData.ShoulderPosition = visionData.ShoulderRightPosition;
    rightVisionData.ShoulderVelocity = visionData.ShoulderRightVelocity ;
    rightVisionData.ElbowPosition = visionData.ElbowRightPosition;
    rightVisionData.ElbowVelocity = visionData.ElbowRightVelocity;
    rightVisionData.WristPosition = visionData.WristRightPosition;
    rightVisionData.WristVelocity = visionData.WristRightVelocity ;
    rightVisionData.HandPosition = visionData.HandRightPosition;
    rightVisionData.HandVelocity = visionData.HandRightVelocity;

    //cout << "right arm=" << rightVisionData << endl;
    
    Arm::handedness left = Arm::left;
    Arm::handedness right = Arm::right;
 
    theleftArm.setPose(left, leftVisionData, spineTop, SpineData.SpineTopVelocity);
    therightArm.setPose(right, rightVisionData, spineTop, SpineData.SpineTopVelocity);
}

void Skeleton::FindDHTransforms(JointSet jointPositions, JointSet jointAxes, fourvector SpineBaseVelocityUnitAxis) {
    
}

Skeleton::PoseTransforms Skeleton::ForwardKinematics(const double t) {
    Transform spineTopTransform;
    Arm::PoseTransforms lat, lat_global; // left arm, local and global coords
    Arm::PoseTransforms rat, rat_global; // right arm, local and global coords

    // separate the scalars for the spine , left arm and right arm so that forward kinetmatics can be performed on each
    fourvector spineBaseDisplacement;
    spineTopTransform = theSpine.ForwardKinematics(t, spineBaseDisplacement);
    
    lat = theleftArm.ForwardKinematics(t) ;
    rat = therightArm.ForwardKinematics(t) ;
    
    lat_global = theleftArm.applyTransform(spineTopTransform, lat);
    rat_global = therightArm.applyTransform(spineTopTransform, rat);

    Skeleton::PoseTransforms totals;
    totals.spineBaseLinear = theSpine.getBaseTransform();
    totals.spineTopRotate = theSpine.getSpineTopTransform();
    totals.LeftspineTopRotate = lat_global.spineTopRotate;
    totals.LeftShoulderZRotate = lat_global.ShoulderZRotate;
    totals.LeftShoulderXRotate = lat_global.ShoulderXRotate;
    totals.LeftShoulderUpperArmRotate = lat_global.ShoulderUpperArmRotate;
    totals.LeftElbowRotate = lat_global.ElbowRotate;
    totals.LeftElbowLowerArmRotate = lat_global.ElbowLowerArmRotate;
    totals.LeftWristRotate = lat_global.WristRotate;
    totals.LeftHandRotate = lat_global.HandRotate;
    totals.RightspineTopRotate = rat_global.spineTopRotate;
    totals.RightShoulderZRotate = rat_global.ShoulderZRotate;
    totals.RightShoulderXRotate = rat_global.ShoulderXRotate;
    totals.RightShoulderUpperArmRotate = rat_global.ShoulderUpperArmRotate;
    totals.RightElbowRotate = rat_global.ElbowRotate;
    totals.RightElbowLowerArmRotate = rat_global.ElbowLowerArmRotate;
    totals.RightWristRotate = rat_global.WristRotate;
    totals.RightHandRotate = rat_global.HandRotate;
    
    return totals;
}

Skeleton::JointSet Skeleton::ExtractPositions() const {
    Skeleton::JointSet js;
    Transform spinetop = theSpine.getSpineTopTransform();
    
    Spine::JointSet sajs =  theSpine.ExtractPositions();
    Arm::JointSet lajs =  theleftArm.ExtractPositions(); // in arm local coords
    Arm::JointSet lajs_global = theleftArm.applyTransform(spinetop, lajs); //in global coords
    Arm::JointSet rajs =  therightArm.ExtractPositions(); // in arm local coords
    Arm::JointSet rajs_global = theleftArm.applyTransform(spinetop, rajs); //in global coords
    
    js.SpineBase = sajs.SpineBase;
    js.SpineTopPosition = sajs.SpineTop;
    js.ShoulderLeftPosition = lajs_global.Shoulder;
    js.ShoulderRightPosition = rajs_global.Shoulder;
    js.ElbowLeftPosition = lajs_global.Elbow;
    js.ElbowRightPosition = rajs_global.Elbow;
    js.WristLeftPosition = lajs_global.Wrist;
    js.WristRightPosition = rajs_global.Wrist;
    js.HandLeftPosition = lajs_global.Hand;
    js.HandRightPosition = rajs_global.Hand;
    
    return js;
}

void Skeleton::findPredictedEnvelopeLimits(const double timePerStep, const int numSteps,
                /* outputs */ double& xmin, double& xmax, double& ymin, double& ymax, double& zmin, double& zmax)  {
    double timeOfTest;
    xmin = std::numeric_limits<double>::max();
    xmax = -xmin;
    ymin = xmin; ymax = xmax; zmin = xmin; zmax = xmax;
    for (int t=0; t<=numSteps; t++) {
        timeOfTest = t * timePerStep;
        //cout << endl << "==== predict time=" << timeOfTest << endl;
        Transform SpineTop;
        SpineTop = theSpine.findPredictedEnvelopeLimits(timeOfTest, xmin, xmax, ymin, ymax, zmin, zmax) ;
        //cout << "SpineTop=" << SpineTop << endl;
        //cout << "left" << endl;
        theleftArm.findPredictedEnvelopeLimits(SpineTop, timeOfTest, xmin, xmax, ymin, ymax, zmin, zmax) ;
        //cout << "right" << endl;
        therightArm.findPredictedEnvelopeLimits(SpineTop,timeOfTest, xmin, xmax, ymin, ymax, zmin, zmax) ;
    }
}

std::ostream& operator<<(std::ostream& os, const Skeleton& x) {
    os << "========== Skeleton ==========" << endl;
    os << endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Skeleton::PositionVelocityDataSet& x) {
    os << "========== Skeleton PV set ==========" << endl;
    os << "LeftHipPosition=       " << x.LeftHipPosition << endl;
    os << "LeftHipVelocity=       " << x.LeftHipVelocity << endl;
    os << "LeftHipPosition=       " << x.LeftHipPosition << endl;
    os << "LeftHipVelocity=       " << x.LeftHipVelocity << endl;
    os << "SpineBasePosition=     " << x.SpineBasePosition << endl;
    os << "SpineBaseVelocity=     " << x.SpineBaseVelocity << endl;
    os << "SpineTopPosition=      " << x.SpineTopPosition << endl;
    os << "SpineTopVelocity=      " << x.SpineTopVelocity << endl;
    os << "ShoulderLeftPosition=  " << x.ShoulderLeftPosition << endl;
    os << "ShoulderLeftVelocity=  " << x.ShoulderLeftVelocity << endl;
    os << "ElbowLeftPosition=     " << x.ElbowLeftPosition << endl;
    os << "ElbowLeftVelocity=     " << x.ElbowLeftVelocity << endl;
    os << "WristLeftPosition=     " << x.WristLeftPosition << endl;
    os << "WristLeftVelocity=     " << x.WristLeftVelocity << endl;
    os << "HandLeftPosition=      " << x.HandLeftPosition << endl;
    os << "HandLeftVelocity=      " << x.HandLeftVelocity << endl;
    os << "ShoulderRightPosition= " << x.ShoulderRightPosition << endl;
    os << "ShoulderRightVelocity= " << x.ShoulderRightVelocity << endl;
    os << "ElbowRightPosition=    " << x.ElbowRightPosition << endl;
    os << "ElbowRightVelocity=    " << x.ElbowRightVelocity << endl;
    os << "WristRightPosition=    " << x.WristRightPosition << endl;
    os << "WristRightVelocity=    " << x.WristRightVelocity << endl;
    os << "HandRightPosition=     " << x.HandRightPosition << endl;
    os << "HandRightVelocity=     " << x.HandRightVelocity << endl;
    os << endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Arm::PositionVelocityDataSet& x) {
    os << "========== Arm PV set ==========" << endl;
    os << "ShoulderPosition=  " << x.ShoulderPosition << endl;
    os << "ElbowPosition=     " << x.ElbowPosition << endl;
    os << "WristPosition=     " << x.WristPosition << endl;
    os << "HandPosition=      " << x.HandPosition << endl;
    os << "--------------------------------" << endl;
    os << "ShoulderVelocity=  " << x.ShoulderVelocity << endl;
    os << "ElbowVelocity=     " << x.ElbowVelocity << endl;
    os << "WristVelocity=     " << x.WristVelocity << endl;
    os << "HandVelocity=      " << x.HandVelocity << endl;
    os << endl;
    return os;
}
