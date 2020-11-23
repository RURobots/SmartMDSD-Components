
/******************************************************************************/
/*                                                                            */
/*  Name:           Skeleton.hh                                               */
/*  Purpose:        Implementation of a 3D vector (with 4th unit element for  */
/*                  transform multiplications)                                */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/

#ifndef FourVector_hpp
#define FourVector_hpp

#include <stdio.h>
#include <iostream>
#include <iomanip>

class fourvector {
    double theVector[4];
public:
    fourvector();
    fourvector(double a, double y, double z, double t=1.0);
    fourvector operator -(const fourvector& x) const ;
    fourvector operator +(const fourvector& x) const ;
    fourvector operator *(const double& x) const ;
    fourvector operator /(const double& x) const ;
    double norm() const;
    double& operator[](const int i) ;
    const double &operator[](int i) const;
    void init(double a, double b, double c, double d);
    
    friend std::ostream& operator<<(std::ostream& os, const fourvector& x);
    
};
#endif /* FourVector_hpp */
