
/******************************************************************************/
/*                                                                            */
/*  Name:           Arm.cpp                                                   */
/*  Purpose:        Implementation of human arm kinematics                    */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/

#include "Arm.hh"

void Arm::findPredictedEnvelopeLimits(const Transform& SpineTop, const double t,
                /* outputs */ double& xmin, double& xmax, double& ymin, double& ymax, double& zmin, double& zmax)  {
 
    ForwardKinematics(t);
    fourvector v = PTs.spineTopRotate.get_offset();
    fourvector pos = SpineTop * v;

    //cout << "xmin=" << xmin << " xmax=" << xmax << " ymin=" << ymin << " ymax=" << ymax << " zmin=" << zmin << " zmax=" << zmax << endl;

    //cout << "shoulder pos= " << pos << endl;
    double x = pos[0], y=pos[1], z=pos[2];
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

    pos = SpineTop * PTs.ElbowRotate.get_offset();
    //cout << "elbow pos=    " << pos << endl;
    x = pos[0]; y=pos[1]; z=pos[2];
    if (x < xmin) xmin = x;
    if (x > xmax) xmax = x;
    if (y < ymin) ymin = y;
    if (y > ymax) ymax = y;
    if (z < zmin) zmin = z;
    if (z > zmax) zmax = z;
    //cout << "xmin=" << xmin << " xmax=" << xmax << " ymin=" << ymin << " ymax=" << ymax << " zmin=" << zmin << " zmax=" << zmax << endl;
    
    pos = SpineTop * PTs.ElbowLowerArmRotate.get_offset();
    //cout << "wrist pos=    " << pos << endl;
    x = pos[0]; y=pos[1]; z=pos[2];

    if (x < xmin) xmin = x;
    if (x > xmax) xmax = x;
    if (y < ymin) ymin = y;
    if (y > ymax) ymax = y;
    if (z < zmin) zmin = z;
    if (z > zmax) zmax = z;
    //cout << "xmin=" << xmin << " xmax=" << xmax << " ymin=" << ymin << " ymax=" << ymax << " zmin=" << zmin << " zmax=" << zmax << endl;
    
    pos = SpineTop * PTs.HandRotate.get_offset();
    //cout << "hand pos=     " << pos << endl;
    x = pos[0]; y=pos[1]; z=pos[2];
    if (x < xmin) xmin = x;
    if (x > xmax) xmax = x;
    if (y < ymin) ymin = y;
    if (y > ymax) ymax = y;
    if (z < zmin) zmin = z;
    if (z > zmax) zmax = z;
    //cout << "xmin=" << xmin << " xmax=" << xmax << " ymin=" << ymin << " ymax=" << ymax << " zmin=" << zmin << " zmax=" << zmax << endl;
}

void Arm::setZeroPose(const handedness armSide, fourvector shoulderPosition, double upperArmLength, double lowerArmLength, double handLength) {
    P.spineTopRotate.dhparams.a = shoulderPosition[0]; // x-component
    P.spineTopRotate.dhparams.alpha = 0.0;
    P.spineTopRotate.dhparams.d = shoulderPosition[2]; // z-component;;
    P.spineTopRotate.dhparams.theta = 0;

    P.ShoulderZRotate.dhparams.a = 0;
    P.ShoulderZRotate.dhparams.alpha =  -M_PI / 2.0;;
    P.ShoulderZRotate.dhparams.d = 0;
    P.ShoulderZRotate.dhparams.theta = 0;
    
    P.ShoulderXRotate.dhparams.a = 0;
    P.ShoulderXRotate.dhparams.alpha = -M_PI / 2.0;
    P.ShoulderXRotate.dhparams.d = 0;
    P.ShoulderXRotate.dhparams.theta = -M_PI / 2.0;

    P.ShoulderUpperArmRotate.dhparams.a = 0;
    P.ShoulderUpperArmRotate.dhparams.alpha = M_PI / 2.0;
    //P.ShoulderUpperArmRotate.dhparams.alpha = -M_PI / 2.0;
    P.ShoulderUpperArmRotate.dhparams.d = upperArmLength;
    P.ShoulderUpperArmRotate.dhparams.theta = 0;

    P.ElbowRotate.dhparams.a = 0;
    P.ElbowRotate.dhparams.alpha = -M_PI / 2.0;
    P.ElbowRotate.dhparams.d = 0;
    P.ElbowRotate.dhparams.theta = 0;

    P.ElbowLowerArmRotate.dhparams.a = 0;
    P.ElbowLowerArmRotate.dhparams.alpha = M_PI / 2.0;
    P.ElbowLowerArmRotate.dhparams.d = lowerArmLength;
    P.ElbowLowerArmRotate.dhparams.theta = 0;
    
    P.WristRotate.dhparams.a = handLength;
    P.WristRotate.dhparams.alpha = 0;
    P.WristRotate.dhparams.d = 0;
    P.WristRotate.dhparams.theta = M_PI / 2.0;
    //
    P.HandRotate.dhparams.a = 0;
    P.HandRotate.dhparams.alpha = 0;
    P.HandRotate.dhparams.d = 0;
    P.HandRotate.dhparams.theta = -M_PI / 2.0;
 
    return;

}


// find the transform which rotates the spine-shoulder axis so it lies in the z-y plane as required for the zero pose.
Transform Arm::ProcessSpineTop(Transform initialReferenceFrame, const Transform& spineTopTransform, const handedness armSide, const Arm::PositionVelocityDataSet& d_world, const fourvector SpineTopVelocityIn) {
  
    Transform T;
    Transform origin;  // all positions relative to spine top....
    Transform shoulder_T;
    
    spineTop = spineTopTransform;
    //cout << "SpineTopTransform=" << spineTopTransform << endl;
    //cout << "SpineTopVelocity=" << SpineTopVelocityIn << endl;
    origin.unit();
    shoulder_T.unit();
    
    Transform jointRotate;
    fourvector zaxis(0.0, 0.0, 1.0);
    fourvector xaxis(1.0, 0.0, 0.0);
    spineTopPosition = spineTopTransform.get_offset();
    SpineTopVelocity = SpineTopVelocityIn;
    
    // first adjust the local vision data by removing the position & velocity component from the spine top
    localVisionData = d_world;
    localVisionData.ShoulderPosition = localVisionData.ShoulderPosition - spineTopPosition;
    localVisionData.ElbowPosition = localVisionData.ElbowPosition - spineTopPosition;
    localVisionData.WristPosition = localVisionData.WristPosition - spineTopPosition;
    localVisionData.HandPosition = localVisionData.HandPosition - spineTopPosition;
    localVisionData.ShoulderVelocity = localVisionData.ShoulderVelocity - SpineTopVelocity;
    localVisionData.ElbowVelocity = localVisionData.ElbowVelocity - SpineTopVelocity;
    localVisionData.WristVelocity = localVisionData.WristVelocity - SpineTopVelocity;
    localVisionData.HandVelocity = localVisionData.HandVelocity - SpineTopVelocity;
    
    //cout << "Arm::ProcessSpineTop" << endl;
    //cout << "arm positions relative to spine top" << endl;
    //cout << "===================================" << endl;
    //cout << "Shoulder=" << localVisionData.ShoulderPosition << endl;
    //cout << "Elbow=" << localVisionData.ElbowPosition << endl;
    //cout << "Wrist=" << localVisionData.WristPosition << endl;
    //cout << "Hand=" << localVisionData.HandPosition << endl;
    //cout << "-----------------------------------" << endl;
    //cout << "arm velocities relative to spine top" << endl;
    //cout << "Shoulder=" << localVisionData.ShoulderVelocity << endl;
    //cout << "Elbow=" << localVisionData.ElbowVelocity << endl;
    //cout << "Wrist=" << localVisionData.WristVelocity << endl;
    //cout << "Hand=" << localVisionData.HandVelocity << endl;

    // compute here the velocity components due to the spine tilt rate (which affects further out points more)
    // JF TODO

    double upperArmLength = (d_world.ElbowPosition - d_world.ShoulderPosition).norm();
    double lowerArmLength = (d_world.WristPosition - d_world.ElbowPosition).norm();
    double handLength = (d_world.HandPosition - d_world.WristPosition).norm();
    //cout << "upper arm length=" << upperArmLength << endl;
    //cout << "lower arm length=" << lowerArmLength << endl;
    //cout << "hand length     =" << handLength << endl;

    fourvector shoulder_spinetop = localVisionData.ShoulderPosition;
    //cout << "shoulder_spinetop =" << shoulder_spinetop << endl;
    
    // compute the transform required to rotate the arm about the z-axis so the shoulder lies along the x-axis
    // (because the zero pose requires the shoulder on the +x-axis
    
    // find angle of rotation about z-axis of shoulder with x-axis as zero reference
    //cout << "angle from " << shoulder_spinetop << " to " << xaxis << " around " << zaxis << endl;
    double requiredRotate = G.obtainAngleAboutAxis(zaxis, shoulder_spinetop, xaxis);
    //cout << "Spinetop alignment rotate=" << requiredRotate * 180.0 / M_PI << endl;
    jointRotate.GenerateRotationMatrix(zaxis, requiredRotate);
    //cout << "jointRotate x=" << jointRotate.get_x() << endl;
    //cout << "jointRotate y=" << jointRotate.get_y() << endl;
    //cout << "jointRotate z=" << jointRotate.get_z() << endl;
    fourvector shoulder_ZeroPose = jointRotate.rotate(shoulder_spinetop);
    //cout << "shoulder_ZeroPose (should have zero y-component) =" << shoulder_ZeroPose << endl;
    // shoulder_ZeroPose should be in x-z plane as required for zero pose configuration
    setZeroPose(armSide, shoulder_ZeroPose, upperArmLength, lowerArmLength, handLength);

    fourvector radiusVector;
    fourvector tangentVector;
    G.obtainRadiusOfActionAndTangent(zaxis, localVisionData.ShoulderPosition, radiusVector, tangentVector);
    double radiusLength = radiusVector.norm();
    double tangentialVelocity = G.dotProduct(tangentVector, localVisionData.ShoulderVelocity);
    rotateRates.spineTopRotate = tangentialVelocity / radiusLength;
    //cout << "shoulder-spine rotate rate=" <<  rotateRates.spineTopRotate << " over radius=" << radiusLength << endl;
    
    P.spineTopRotate.dhparams.theta -= requiredRotate;
    P.spineTopRotate.toEndPointTransform = P.spineTopRotate.ForwardKinematics(0.0, 0.0); // update the internal transforms to include the new theta value.
    //cout << "P.spineTopRotate offset=" << P.spineTopRotate.toEndPointTransform.get_offset() << endl;
    //cout << "P.spineTopRotate      x=" << P.spineTopRotate.toEndPointTransform.get_x() << endl;
    //cout << "P.spineTopRotate      y=" << P.spineTopRotate.toEndPointTransform.get_y() << endl;
    //cout << "P.spineTopRotate      z=" << P.spineTopRotate.toEndPointTransform.get_z() << endl;
    T = P.spineTopRotate.toEndPointTransform;
    //cout << "Arm::ProcessSpineTop T=" << T << endl;
    //cout << "T x-axis=" << T.get_x() << endl;
    //cout << "T y-axis=" << T.get_y() << endl;
    //cout << "T z-axis=" << T.get_z() << endl;
    return T;

}

Transform  Arm::ProcessShoulderZ(const Transform& TransformSoFar, const handedness armSide, const Arm::PositionVelocityDataSet& d_world) {
    // now obtain the transform for the shoulder rotate about Z (required to rotate the upper arm onto the y-z plane)
    // ==============================================================================================================
    // obtain angle from y-axis (subtracting angle already taken by shoulder-spine rotation)
    // first need to determine the direction of the
    //Transform jointRotate;
    //fourvector zaxis = cumulativeReferenceFrame.get_z();
    //fourvector yaxis = cumulativeReferenceFrame.get_y();
    //fourvector xaxis = cumulativeReferenceFrame.get_x();
    fourvector xaxis = TransformSoFar.get_x();
    fourvector yaxis = TransformSoFar.get_y();
    fourvector zaxis = TransformSoFar.get_z();
    //cout << "x-axis=" << xaxis << endl << "y-axis=" << yaxis << endl << "z-axis=" << zaxis << endl;

    fourvector elbowVelocity_world = localVisionData.ElbowVelocity;

    fourvector upperArmVector_world = localVisionData.ElbowPosition - localVisionData.ShoulderPosition;
    fourvector upperArmVector_world_unit = upperArmVector_world / upperArmVector_world.norm();
    //cout << "localVisionData.ElbowPosition = " << localVisionData.ElbowPosition << endl;
    //cout << "localVisionData.ShoulderPosition=" << localVisionData.ShoulderPosition << endl;
    //cout << "upper arm world=" << upperArmVector_world << endl;
    //cout << "upper arm world unit =" << upperArmVector_world_unit << endl;
    //cout << "compute angle from " << xaxis << " to " << upperArmVector_world_unit << " about axis=" << zaxis << endl;
    // find angle by which x-axis (direction of limb for zero angle of rotation) must be rotated about z axis  to match upperArmVector_world
    double shoulderZangle = G.obtainAngleAboutAxis(zaxis , xaxis, upperArmVector_world_unit);
    //cout << "ProcessShoulderZ angle=" << shoulderZangle * 180.0/ M_PI << endl;
    P.ShoulderZRotate.dhparams.theta += shoulderZangle;
    P.ShoulderZRotate.toEndPointTransform = P.ShoulderZRotate.ForwardKinematics(0.0, 0.0); // update the internal transforms to include the new theta value.
    Transform T = TransformSoFar * P.ShoulderZRotate.toEndPointTransform;
    //cout << "Arm::ProcessShoulderZ T=" << T << endl;
    
    // next the rate
    fourvector zaxisradiusVector;
    fourvector required_v_component;
    bool shoulderZFlag = G.obtainRadiusOfActionAndTangent(zaxis, upperArmVector_world, zaxisradiusVector, required_v_component);
    double Zangular_rate;
    if (shoulderZFlag)
        Zangular_rate = G.dotProduct(required_v_component, elbowVelocity_world) / zaxisradiusVector.norm();
    else
        Zangular_rate = 0;
    rotateRates.shoulderZRotate = Zangular_rate;
    
    //cout << "Shoulder angular rate= " << Zangular_rate * 180.8 / M_PI << endl;
    //cout << "Shoulder Z rotate In= " << TransformSoFar << endl;
    //cout << "Shoulder Z rotate T= " << P.ShoulderZRotate.toEndPointTransform << endl;
    //cout << "Shoulder Z output: " << endl;
    //cout << "offset=" << T.get_offset() << endl;
    //cout << "     x=" << T.get_x() << endl;
    //cout << "     y=" << T.get_y() << endl;
    //cout << "     z=" << T.get_z() << endl;
    return T;
}

Transform  Arm::ProcessShoulderX(const Transform& TransformSoFar, const handedness armSide, const Arm::PositionVelocityDataSet& d_world) {
    // now obtain the transform for the shoulder rotate about X
    // ========================================================
    // obtain angle from z-axis.
    //Transform jointRotate;
    //fourvector zaxis = cumulativeReferenceFrame.get_z();
    //fourvector yaxis = cumulativeReferenceFrame.get_y();
    //fourvector xaxis = cumulativeReferenceFrame.get_x();
    //fourvector xaxis = TransformSoFar.get_x();
    fourvector yaxis = TransformSoFar.get_y();
    //fourvector zaxis = TransformSoFar.get_z();

    fourvector upperArmVector_world = localVisionData.ElbowPosition - localVisionData.ShoulderPosition;
    fourvector upperArmVector_world_unit  = upperArmVector_world / upperArmVector_world.norm();
    fourvector elbowVelocity_world = localVisionData.ElbowVelocity - localVisionData.ShoulderVelocity;
    //cout << "upperArmVector_world="<< upperArmVector_world << endl;
    //cout << "upperArmVector_world_unit="<< upperArmVector_world_unit << endl;

    
    
    //cout << "elbowVelocity_world=" << elbowVelocity_world << endl;

    fourvector zReference = yaxis * -1; // the transform y axis points in the original -z axis direction.
    // find angle by which x-axis (the upperArmVector direction for a zero pose) must be rotated about z axis  to match upperArmVector_world
    double cosShoulderXangle =  G.dotProduct(upperArmVector_world_unit, zReference);
    if (cosShoulderXangle < -1.0) cosShoulderXangle = -1.0;
    if (cosShoulderXangle > +1.0) cosShoulderXangle = +1.0;
    double shoulderXangle = acos(cosShoulderXangle) - 90.0 * M_PI / 180.0 ; // zero is when arm is horizontal not vertical
    //cout << "Arm::ProcessShoulderX ProcessShoulderX angle=" << shoulderXangle * 180.0/ M_PI << endl;
    P.ShoulderXRotate.dhparams.theta += shoulderXangle;
    P.ShoulderXRotate.toEndPointTransform = P.ShoulderXRotate.ForwardKinematics(0.0, 0.0); // update the internal transforms to include the new theta value.
    Transform T = TransformSoFar * P.ShoulderXRotate.toEndPointTransform;
    
    fourvector required_v_component;
 
    bool shoulderXFlag = G.obtainPlaneTangent(zReference, upperArmVector_world, required_v_component);
    //cout << "zReference           = " << zReference << endl;
    //cout << "upperArmVector_world = " << upperArmVector_world << endl;
    //cout << "req v                = " << required_v_component << endl;
    double Xangular_rate;
    double upperArmVectorLen = upperArmVector_world.norm();
    if (shoulderXFlag)
        Xangular_rate = G.dotProduct(required_v_component, elbowVelocity_world) / upperArmVectorLen;
    else
        Xangular_rate = 0.0;
     
    //Xangular_rate = G.dotProduct(required_v_component, elbowVelocity_world) / upperArmVectorLen;
    rotateRates.shoulderXRotate = Xangular_rate;

    //cout << "Shoulder X rotate In= " << TransformSoFar << endl;
    //cout << "Shoulder X rotate T= " << P.ShoulderZRotate.toEndPointTransform << endl;
    //cout << "Shoulder X output: " << endl;
    //cout << "offset=" << T.get_offset() << endl;
    //cout << "     x=" << T.get_x() << endl;
    //cout << "     y=" << T.get_y() << endl;
    //cout << "     z=" << T.get_z() << endl;
    return T;
}

Transform Arm::ProcessUpperArm(const Transform& TransformSoFar, const handedness armSide, const Arm::PositionVelocityDataSet& d_world) {
    // now obtain the transform for the shoulder rotate upper arm axis.
    // ================================================================
    // Note This depends on upper arm and lower arm vectors.
    // first need the elbow axis (as that determines the rotation).
    //Transform jointRotate;
    Transform T;
    
    fourvector xaxis = TransformSoFar.get_x();
    fourvector yaxis = TransformSoFar.get_y();
    fourvector zaxis = TransformSoFar.get_z();
    //cout << "Arm::ProcessUpperArm offset=" << TransformSoFar.get_offset() << endl;
    //cout << "x-axis=" << TransformSoFar.get_x() << endl;
    //cout << "y-axis=" << TransformSoFar.get_y() << endl;
    //cout << "z-axis=" << TransformSoFar.get_z() << endl;

    fourvector lowerArmVector_world = localVisionData.WristPosition - localVisionData.ElbowPosition;
    fourvector upperArmVector_world = localVisionData.ElbowPosition - localVisionData.ShoulderPosition;
    fourvector wristVelocity_World = localVisionData.WristVelocity - localVisionData.ElbowVelocity;
    //cout << "upperArmVector_world=" << upperArmVector_world << endl;
    //cout << "lowerArmVector_world=" << lowerArmVector_world << endl;
    //cout << "upperArmVector_world_unit=" << G.normaliseVector(upperArmVector_world) << endl;
    //cout << "lowerArmVector_world_unit=" << G.normaliseVector(lowerArmVector_world) << endl;

   //fourvector elbowaxis;
    fourvector upperArmVector_unit = G.normaliseVector(upperArmVector_world);

    fourvector radiusVectorWrist;
    fourvector requiredWristTangentVector;
    // obtain the axis of elbow by cross product of upper am and lower arm vectors.
    elbowBent = G.obtainRadiusOfActionAndTangent(upperArmVector_world, lowerArmVector_world, radiusVectorWrist, requiredWristTangentVector);
    //cout << "Arm::ProcessUpperArm radiusVectorWrist=" << radiusVectorWrist << endl;
    //cout << "requiredWristTangentVector=" << requiredWristTangentVector << endl;
    if (elbowBent) {
        // elbow is bent so we have a reliable axis. Compute angle between this an the T2 y-axis (which is parallel to the zero rotate elbow axis)
        // now find the angle which yxaxis is rotated around axis upperArmVector_unit to reach requiredWristTangentVector (the elbow axis)
        double UpperArmRotateAngle = G.obtainAngleAboutAxis(upperArmVector_unit, yaxis, requiredWristTangentVector);
        //cout << "ProcessUpperArm angle=" << UpperArmRotateAngle * 180.0/M_PI << endl;
        P.ShoulderUpperArmRotate.dhparams.theta += UpperArmRotateAngle;
        P.ShoulderUpperArmRotate.toEndPointTransform = P.ShoulderUpperArmRotate.ForwardKinematics(0.0, 0.0); // update the internal transforms to include the new theta value.

        //cout << "P.ShoulderUpperArmRotate offset=" << P.ShoulderUpperArmRotate.toEndPointTransform.get_offset() << endl;
        //cout << "     x=" << P.ShoulderUpperArmRotate.toEndPointTransform.get_x() << endl;
        //cout << "     y=" << P.ShoulderUpperArmRotate.toEndPointTransform.get_y() << endl;
        //cout << "     z=" << P.ShoulderUpperArmRotate.toEndPointTransform.get_z() << endl;

        T = TransformSoFar * P.ShoulderUpperArmRotate.toEndPointTransform;
        
        double actionVelocity = G.dotProduct(requiredWristTangentVector, wristVelocity_World);
        double actionRate = actionVelocity / radiusVectorWrist.norm();
        //cout << "requiredWristTangentVector x=" << requiredWristTangentVector << endl;
        //cout << "wristVelocity_World         =" << wristVelocity_World << endl;
        //cout << "actionVelocity              =" << actionVelocity << endl;
        //cout << "actionRate                  =" << actionRate << endl;
        rotateRates.shoulderUpperArmRotate = actionRate ;
   } else {
        // elbow is straight so cannot compute the axis from that. Use a defalt zero pose and set the rotate rate to zero.
        rotateRates.shoulderUpperArmRotate = 0.0; // a straight arm will remain straight
        P.ShoulderUpperArmRotate.toEndPointTransform = P.ShoulderUpperArmRotate.ForwardKinematics(0.0, 0.0); // update the internal transforms. note: leave the DH theta where it is.
       T = TransformSoFar * P.ShoulderUpperArmRotate.toEndPointTransform;
    }

    //cout << "ShoulderUpperArmRotate rotate In= " << TransformSoFar << endl;
    //cout << "ShoulderUpperArmRotate rotate T= " << P.ShoulderUpperArmRotate.toEndPointTransform << endl;
    //cout << "ShoulderUpperArmRotate output: " << endl;
    //cout << "offset=" << T.get_offset() << endl;
    //cout << "     x=" << T.get_x() << endl;
    //cout << "     y=" << T.get_y() << endl;
    //cout << "     z=" << T.get_z() << endl;
   return T;
}

Transform Arm::ProcessElbow(const Transform& TransformSoFar, const handedness armSide, const Arm::PositionVelocityDataSet& d_world) {
    // now obtain the transform elbow
    // ==============================
    //Transform jointRotate;
    Transform T;

    //fourvector zaxis = cumulativeReferenceFrame.get_z();
    //fourvector yaxis = cumulativeReferenceFrame.get_y();
    //fourvector xaxis = cumulativeReferenceFrame.get_x();
    
    //fourvector xaxis = TransformSoFar.get_x();
    //fourvector yaxis = TransformSoFar.get_y();
    //fourvector zaxis = TransformSoFar.get_z();

    fourvector lowerArmVector_world = localVisionData.WristPosition - localVisionData.ElbowPosition;
    fourvector upperArmVector_world = localVisionData.ElbowPosition - localVisionData.ShoulderPosition;
    fourvector wristVelocity_World = localVisionData.WristVelocity - localVisionData.ElbowVelocity;
    fourvector lowerArmVectorUnit_world = lowerArmVector_world / lowerArmVector_world.norm();
    fourvector upperArmVectorUnit_world = upperArmVector_world  / upperArmVector_world.norm();

    //cout << "lowerArmVector_world=" << lowerArmVector_world << endl;
    //cout << "lowerArmVector_world_unit=" << lowerArmVectorUnit_world << endl;
    //cout << "upperArmVector_world=" << upperArmVector_world << endl;
    //cout << "upperArmVector_world_unit=" << upperArmVectorUnit_world << endl;

    
    if (elbowBent) {
        
        double cosElbowAngle = G.dotProduct(lowerArmVectorUnit_world, upperArmVectorUnit_world) ;
        if (cosElbowAngle < -1.0) cosElbowAngle = -1.0;
        if (cosElbowAngle > +1.0) cosElbowAngle = +1.0;
        double elbowAngle =  acos(cosElbowAngle);
        //cout << "ProcessElbow angle=" << elbowAngle * 180.0/M_PI << endl;
        
        P.ElbowRotate.dhparams.theta += -elbowAngle; // z-axis points back up the arm not down, so negate the angle....
        P.ElbowRotate.toEndPointTransform = P.ElbowRotate.ForwardKinematics(0.0, 0.0); // update the internal transforms to include the new theta value.
 
        //cout << "P.ElbowRotate offset=" << P.ElbowRotate.toEndPointTransform.get_offset() << endl;
        //cout << "     x=" << P.ElbowRotate.toEndPointTransform.get_x() << endl;
        //cout << "     y=" << P.ElbowRotate.toEndPointTransform.get_y() << endl;
        //cout << "     z=" << P.ElbowRotate.toEndPointTransform.get_z() << endl;
        
        T = TransformSoFar * P.ElbowRotate.toEndPointTransform;
                
        // next the rate.
        fourvector wrist_tangentVector;
        G.obtainPlaneTangent(upperArmVectorUnit_world, lowerArmVector_world, wrist_tangentVector);
        double elbowActionVelocity = G.dotProduct(wrist_tangentVector, wristVelocity_World);
        double elbowRate = elbowActionVelocity / lowerArmVector_world.norm();
        //cout << "wrist_tangentVector x=" << wrist_tangentVector << endl;
        //cout << "wristVelocity_World  =" << wristVelocity_World << endl;
        //cout << "elbowActionVelocity  =" << elbowActionVelocity << endl;
        //cout << "elbowRate                  =" << elbowRate << endl;
        rotateRates.elbowRotate = -elbowRate; // z-axis points back up the arm not down, so negate the angle....
    } else {
        // elbow is straight so cannot compute the axis from that. Use a defalt zero pose and set the rotate rate to zero.
        rotateRates.elbowRotate = 0.0; // a straight arm will remain straight
        // note leave the DH theta where it is.
        P.ElbowRotate.toEndPointTransform = P.ElbowRotate.ForwardKinematics(0.0, 0.0); // update the internal transforms to include the new theta value.
        T = TransformSoFar * P.ElbowRotate.toEndPointTransform;
    }

    //cout << "ElbowRotate rotate In= " << TransformSoFar << endl;
    //cout << "ElbowRotate rotate T= " << P.ElbowRotate.toEndPointTransform << endl;
    //cout << "ElbowRotate output: " << endl;
    //cout << "offset=" << T.get_offset() << endl;
    //cout << "     x=" << T.get_x() << endl;
    //cout << "     y=" << T.get_y() << endl;
    //cout << "     z=" << T.get_z() << endl;
    return T;
}

Transform  Arm::ProcessLowerArm(const Transform& TransformSoFar, const handedness armSide, const Arm::PositionVelocityDataSet& d_world) {
    // next the lower arm rotate
    // ==============================
    Transform jointRotate;
    Transform T;

    fourvector xaxis = TransformSoFar.get_x();
    fourvector yaxis = TransformSoFar.get_y();
    fourvector zaxis = TransformSoFar.get_z();

    fourvector lowerArmVector_world = localVisionData.WristPosition - localVisionData.ElbowPosition;
    fourvector handVector_world = localVisionData.HandPosition - localVisionData.WristPosition;
    fourvector handVelocity_world = localVisionData.HandVelocity - localVisionData.WristVelocity;
    fourvector lowerArmVector_unit = G.normaliseVector(lowerArmVector_world);
    
    //cout << "lowerArmVector_world=" << lowerArmVector_world << endl;
    //cout << "handVector_world    =" << handVector_world << endl;

    fourvector radiusVectorHand;
    fourvector requiredHandTangent;
    // find angle by which lowerArmVector_ZeroPose must be rotated about axis z_transformed to match upperArmVector_world
 
    wristBent = G.obtainRadiusOfActionAndTangent(lowerArmVector_world, handVector_world, radiusVectorHand, requiredHandTangent);
    
    //cout << "radiusVectorHand=" << radiusVectorHand << endl;
    //cout << "requiredHandTangent=" << requiredHandTangent << endl;
    //cout << "xaxis=" << xaxis << endl;
    //cout << "yaxis=" << yaxis << endl;
    //cout << "zaxis=" << zaxis << endl;
    
    fourvector yRef = yaxis * -1;
    // obtain the axis of elbow by cross product of upper am and lower arm vectors.
    if (wristBent) {
        // wrist is bent so we have a reliable axis. Compute angle between this an the y-axis (which is parallel to the zero rotate elbow axis)
        // now find the angle which zaxis is rotated around axis lowerArmVector_unit to reach requiredHandTangent
        //cout << "angle rotate from " << zaxis << " to " << requiredHandTangent << " around " << lowerArmVector_unit << endl;
        double lowerArmRotateAngle = G.obtainAngleAboutAxis(lowerArmVector_unit, yRef, requiredHandTangent);
        //cout << "lowerArmRotateAngle=" << lowerArmRotateAngle * 180.0 / M_PI << endl;
        // now find the angle with the x-axis
        P.ElbowLowerArmRotate.dhparams.theta += lowerArmRotateAngle;
        P.ElbowLowerArmRotate.toEndPointTransform = P.ElbowLowerArmRotate.ForwardKinematics(0.0, 0.0); // update the internal transforms to include the new theta value.
        T = TransformSoFar * P.ElbowLowerArmRotate.toEndPointTransform;
        
        // now to determine the rate of rotation...
        // find the tangent component of the velocity of the wrist along the axis of the upper arm.
        double actionVelocity = G.dotProduct(requiredHandTangent, handVelocity_world);
        rotateRates.elbowLowerArmRotate = actionVelocity / radiusVectorHand.norm();
   } else {
        // wrist is straight so cannot compute the axis from that. Use a default zero pose and set the rotate rate to zero.
        rotateRates.elbowLowerArmRotate = 0.0; // a straight arm will remain straight
        P.ElbowLowerArmRotate.toEndPointTransform = P.ElbowLowerArmRotate.ForwardKinematics(0.0, 0.0); // update the internal transforms. note: leave the DH theta where it is.
       T = TransformSoFar * P.ElbowLowerArmRotate.toEndPointTransform;
    }

    //cout << "ElbowLowerArmRotate rotate In= " << TransformSoFar << endl;
    //cout << "ElbowLowerArmRotate rotate T= " << P.ElbowLowerArmRotate.toEndPointTransform << endl;
    //cout << "ElbowLowerArmRotate output: " << endl;
    //cout << "offset=" << T.get_offset() << endl;
    //cout << "     x=" << T.get_x() << endl;
    //cout << "     y=" << T.get_y() << endl;
    //cout << "     z=" << T.get_z() << endl;
   return T;
}

Transform  Arm::ProcessWrist(const Transform& TransformSoFar, const handedness armSide, const Arm::PositionVelocityDataSet& d_world) {
    // now obtain the transform for the wrist
    // ======================================
    Transform T;
    
    fourvector xaxis = TransformSoFar.get_x();
    //fourvector yaxis = TransformSoFar.get_y();
    //fourvector zaxis = TransformSoFar.get_z();

    fourvector lowerArmVector_world = localVisionData.WristPosition - localVisionData.ElbowPosition;
    fourvector handVector_world = localVisionData.HandPosition - localVisionData.WristPosition;
    fourvector handVelocity_world = localVisionData.HandVelocity - localVisionData.WristVelocity;
    fourvector lowerArmVectorUnit_world = lowerArmVector_world / lowerArmVector_world.norm();
    fourvector handVectorUnit_world = handVector_world  / handVector_world.norm();
    fourvector wrist_tangentVector;
    //cout << "Arm::ProcessWrist" << endl;
    //cout << "handVector_world=" << handVector_world << endl;
    //cout << "handVector_world_unit=" << handVectorUnit_world << endl;


    if (wristBent) {
 
        double cosWristAngle = G.dotProduct(handVectorUnit_world, xaxis) ;
        if (cosWristAngle > 1.0) cosWristAngle = 1.0;
        if (cosWristAngle < -1.0) cosWristAngle = -1.0;

        double wristAngle = acos(cosWristAngle) - (90.0 * M_PI / 180.0); // 90 degrees from the z-axis equals zero wrist angle
        //cout << "wristRotateAngle=" << wristAngle * 180.0 / M_PI << endl;
        P.WristRotate.dhparams.theta += wristAngle;
        P.WristRotate.toEndPointTransform = P.WristRotate.ForwardKinematics(0.0, 0.0); // update the internal transforms to include the new theta value.
       // next the rate.
        
        fourvector hand_tangentVector;
        G.obtainPlaneTangent(lowerArmVectorUnit_world, handVector_world, hand_tangentVector);
        double wristActionVelocity = G.dotProduct(hand_tangentVector, handVelocity_world);
        rotateRates.wristRotate = wristActionVelocity / handVector_world.norm();
        
        T = TransformSoFar * P.WristRotate.toEndPointTransform;
        } else {
         // elbow is straight so cannot compute the axis from that. Use a defalt zero pose and set the rotate rate to zero.
         rotateRates.wristRotate = 0.0; // a straight arm will remain straight
         P.WristRotate.toEndPointTransform = P.WristRotate.ForwardKinematics(0.0, 0.0); // update the internal transforms to include the new theta value.
         // note leave the DH theta where it is.
        T = TransformSoFar * P.WristRotate.toEndPointTransform;
     }

    //cout << "WristRotate rotate In= " << TransformSoFar << endl;
    //cout << "WristRotate rotate T= " << P.WristRotate.toEndPointTransform << endl;
    //cout << "WristRotate output: " << endl;
    //cout << "offset=" << T.get_offset() << endl;
    //cout << "     x=" << T.get_x() << endl;
    //cout << "     y=" << T.get_y() << endl;
    //cout << "     z=" << T.get_z() << endl;
    return T;
}

Transform Arm::ProcessHand(const Transform& TransformSoFar, const handedness armSide, const Arm::PositionVelocityDataSet& d_world) {
    Transform T;
    // now obtain the transform for the hand - leave in zero pose position
    P.HandRotate.toEndPointTransform = P.HandRotate.ForwardKinematics(0.0, 0.0);
    T = TransformSoFar * P.HandRotate.toEndPointTransform;

    //cout << "HandRotate rotate In= " << TransformSoFar << endl;
    //cout << "HandRotate rotate T= " << P.HandRotate.toEndPointTransform << endl;
    //cout << "HandRotate output: " << endl;
    //cout << "offset=" << T.get_offset() << endl;
    //cout << "     x=" << T.get_x() << endl;
    //cout << "     y=" << T.get_y() << endl;
    //cout << "     z=" << T.get_z() << endl;
    return T;
}


void Arm::setPose(const handedness armSide, const Arm::PositionVelocityDataSet& d_world, const Transform& spineTopTransform, const fourvector& SpineTopVelocity) {

    spineTop = spineTopTransform;
    //cout << "Arm::setPose spineTopTransform=" << spineTop << endl;
    //cout << "x=" << spineTop.get_x() << endl;
    //cout << "y=" << spineTop.get_y() << endl;
    //cout << "z=" << spineTop.get_z() << endl;
    //cout << "offset=" << spineTop.get_offset() << endl;

    Transform T0, T1,T2,T3,T4,T5,T6,T7, T8;
    
    Transform initialReferenceFrame;
    initialReferenceFrame.unit();
    
    T0 = ProcessSpineTop(initialReferenceFrame, spineTopTransform, armSide, d_world, SpineTopVelocity);  // this is spinetop rotated about its z-axis to align spine-shoulder with y-z plane.
    //cout << "T0=" << T0 << endl;
    T1 = ProcessShoulderZ(T0, armSide, d_world);
    //cout << "T1=" << T1 << endl;
    T2 = ProcessShoulderX(T1, armSide, d_world);
    //cout << "T2=" << T2 << endl;
    T3 = ProcessUpperArm(T2, armSide, d_world);
    //cout << "T3=" << T3 << endl;
    T4 = ProcessElbow(T3, armSide, d_world);
    //cout << "T4=" << T4 << endl;
    T5 = ProcessLowerArm(T4, armSide, d_world);
    //cout << "T5=" << T5 << endl;
    T6 = ProcessWrist(T5, armSide, d_world);
    //cout << "T6=" << T6 << endl;
    T7 = ProcessHand(T6, armSide, d_world);
    //cout << "T7=" << T7 << endl;

}


// note these are arm local transforms, and joints positions and axes should be in arm local coordinates.
void Arm::FindDHTransforms(JointSet jointPositions, JointSet jointAxes, fourvector SpineBaseVelocityUnitAxis) {
    
}

Arm::JointSet Arm::ExtractPositions() const {
    Arm::JointSet js;
    js.Shoulder = PTs.spineTopRotate.get_offset();
    js.Elbow = PTs.ElbowRotate.get_offset();
    js.Wrist = PTs.WristRotate.get_offset();
    js.Hand = PTs.HandRotate.get_offset();;
    return js;
}

Arm::PoseTransforms Arm::ForwardKinematics(const double t) {
    Arm::PoseTransforms T;

    // first, compute the forawrd kiematics transform for each of the joints...
    T.spineTopRotate = P.spineTopRotate.ForwardKinematics(rotateRates.spineTopRotate, t);
    T.ShoulderZRotate = P.ShoulderZRotate.ForwardKinematics(rotateRates.shoulderZRotate, t);
    T.ShoulderXRotate = P.ShoulderXRotate.ForwardKinematics(rotateRates.shoulderXRotate, t);
    T.ShoulderUpperArmRotate = P.ShoulderUpperArmRotate.ForwardKinematics(rotateRates.shoulderUpperArmRotate, t);
    T.ElbowRotate = P.ElbowRotate.ForwardKinematics(rotateRates.elbowRotate, t);
    T.ElbowLowerArmRotate = P.ElbowLowerArmRotate.ForwardKinematics(rotateRates.elbowLowerArmRotate, t);
    T.WristRotate = P.WristRotate.ForwardKinematics(rotateRates.wristRotate, t);
    T.HandRotate = P.HandRotate.ForwardKinematics(rotateRates.handRotate, t);
    
    // now have the local transforms for each joint in T.
    // convert to cumulative transforms in world coordinates
    PTs.spineTopRotate = T.spineTopRotate;
    PTs.ShoulderZRotate = PTs.spineTopRotate * T.ShoulderZRotate;
    PTs.ShoulderXRotate = PTs.ShoulderZRotate * T.ShoulderXRotate;
    PTs.ShoulderUpperArmRotate = PTs.ShoulderXRotate * T.ShoulderUpperArmRotate;
    PTs.ElbowRotate = PTs.ShoulderUpperArmRotate * T.ElbowRotate;
    PTs.ElbowLowerArmRotate = PTs.ElbowRotate * T.ElbowLowerArmRotate;
    PTs.WristRotate = PTs.ElbowLowerArmRotate * T.WristRotate;
    PTs.HandRotate = PTs.WristRotate * T.HandRotate;

    return T;
}

Arm::PoseTransforms Arm::ForwardKinematics(const JointSet_Scalars angles) {
    Arm::PoseTransforms T;
    // first, compute the forawrd kiematics transform for each of the joints...
    T.spineTopRotate = P.spineTopRotate.ForwardKinematics(angles.spineTopRotate, 1.0);
    T.ShoulderZRotate = P.ShoulderZRotate.ForwardKinematics(angles.shoulderZRotate, 1.0);
    T.ShoulderXRotate = P.ShoulderXRotate.ForwardKinematics(angles.shoulderXRotate, 1.0);
    T.ShoulderUpperArmRotate = P.ShoulderUpperArmRotate.ForwardKinematics(angles.shoulderUpperArmRotate, 1.0);
    T.ElbowRotate = P.ElbowRotate.ForwardKinematics(angles.elbowRotate, 1.0);
    T.ElbowLowerArmRotate = P.ElbowLowerArmRotate.ForwardKinematics(angles.elbowLowerArmRotate, 1.0);
    T.WristRotate = P.WristRotate.ForwardKinematics(angles.wristRotate, 1.0);
    T.HandRotate = P.HandRotate.ForwardKinematics(angles.handRotate, 1.0);
    
    // now have the local transforms for each joint in T.
    // convert to cumulative transforms in world coordinates
    PTs.spineTopRotate = T.spineTopRotate;
    cout << T.spineTopRotate << endl;
    PTs.ShoulderZRotate = PTs.spineTopRotate * T.ShoulderZRotate;
    PTs.ShoulderXRotate = PTs.ShoulderZRotate * T.ShoulderXRotate;
    PTs.ShoulderUpperArmRotate = PTs.ShoulderXRotate * T.ShoulderUpperArmRotate;
    PTs.ElbowRotate = PTs.ShoulderUpperArmRotate * T.ElbowRotate;
    PTs.ElbowLowerArmRotate = PTs.ElbowRotate * T.ElbowLowerArmRotate;
    PTs.WristRotate = PTs.ElbowLowerArmRotate * T.WristRotate;
    PTs.HandRotate = PTs.WristRotate * T.HandRotate;
    
    return PTs;
}


Arm::PoseTransforms Arm::applyTransform(const Transform& T, const Arm::PoseTransforms& poses) const {
    Arm::PoseTransforms newPose;
    newPose.spineTopRotate = T * poses.spineTopRotate;
    newPose.ShoulderZRotate = T * poses.ShoulderZRotate;
    newPose.ShoulderXRotate = T * poses.ShoulderXRotate;
    newPose.ShoulderUpperArmRotate = T * poses.ShoulderUpperArmRotate;
    newPose.ElbowRotate = T * poses.ElbowRotate;
    newPose.ElbowLowerArmRotate = T * poses.ElbowLowerArmRotate;
    newPose.WristRotate = T * poses.WristRotate;
    newPose.HandRotate = T * poses.HandRotate;
    return newPose;
}

Arm::JointSet Arm::applyTransform(const Transform& T, const JointSet& joints) const {
    Arm::JointSet newJoints;
    newJoints.Shoulder = T * joints.Shoulder;
    newJoints.Elbow = T * joints.Elbow;
    newJoints.Wrist = T * joints.Wrist;
    newJoints.Hand = T * joints.Hand;
    return newJoints;
}
