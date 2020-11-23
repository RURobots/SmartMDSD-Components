
/******************************************************************************/
/*                                                                            */
/*  Name:           Skeleton.cpp                                              */
/*  Purpose:        Implementation of a 3D vector (with 4th unit element for  */
/*                  transform multiplications)                                */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/

#include "FourVector.hh"
#include <cmath>


fourvector::fourvector() {
    theVector[0] = 0;
    theVector[1] = 0;
    theVector[2] = 0;
    theVector[3] = 1.0;
}

fourvector::fourvector(double x, double y, double z, double t) {
    theVector[0] = x;
    theVector[1] = y;
    theVector[2] = z;
    theVector[3] = t;
}


double& fourvector::operator[](const int i) {
    return theVector[i];
}

const double& fourvector::operator[](int i) const {
    return theVector[i];
}

void fourvector::init(double a, double b, double c, double d) {
    theVector[0] = a;
    theVector[1] = b;
    theVector[2] = c;
    theVector[3] = d;
}

fourvector fourvector::operator -(const fourvector& x) const
{
    fourvector r;
    r.theVector[0] = theVector[0] - x.theVector[0];
    r.theVector[1] = theVector[1] - x.theVector[1];
    r.theVector[2] = theVector[2] - x.theVector[2];
    //r.theVector[3] = 0;
    return r;
};

fourvector fourvector::operator +(const fourvector& x) const {
    fourvector r;
    r.theVector[0] = theVector[0] + x.theVector[0];
    r.theVector[1] = theVector[1] + x.theVector[1];
    r.theVector[2] = theVector[2] + x.theVector[2];
    //r.theVector[3] = 0;
    return r;

};

fourvector fourvector::operator *(const double& x) const {
    fourvector r;
    r.theVector[0] = theVector[0] * x;
    r.theVector[1] = theVector[1] * x;
    r.theVector[2] = theVector[2] * x;
    //r.theVector[3] = 0;
    return r;
};

fourvector fourvector::operator /(const double& x) const {
    fourvector r;
    r.theVector[0] = theVector[0] / x;
    r.theVector[1] = theVector[1] / x;
    r.theVector[2] = theVector[2] / x;
    //r.theVector[3] = 0;
    return r;
};

double fourvector::norm() const {
    return sqrt(theVector[0]*theVector[0] + theVector[1]*theVector[1] + theVector[2]*theVector[2]);
}


std::ostream& operator<<(std::ostream& os, const fourvector& x) {
    //os << "[ " << x.theVector[0] << ", " << x.theVector[1] << ", " << x.theVector[2] << ", " << x.theVector[3] << " ]" ;
    //os << std::showpoint;
    std::ios::fmtflags old_settings = os.flags();
    os.setf(std::ios::fixed, std::ios::floatfield);
    os << std::showpos;
    os.precision(3);
    os << "[ " << std::setw(3) << x.theVector[0] << ", " <<  std::setw(3) << x.theVector[1] << ", " <<  std::setw(3) <<  x.theVector[2]  << " ]" ;
    //os.setf(0, std::ios::floatfield);
    //os << std::noshowpoint;
    //os.unsetf(std::ios::fixed | std::ios::scientific);
    os.flags(old_settings);
    return os;
}
