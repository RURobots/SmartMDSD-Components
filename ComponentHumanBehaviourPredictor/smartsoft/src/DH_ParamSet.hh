
/******************************************************************************/
/*                                                                            */
/*  Name:           DH_ParamSet.hh                                            */
/*  Purpose:        Implementation of class to store DH parameters            */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/

#ifndef DH_ParamSet_hpp
#define DH_ParamSet_hpp

#include <stdio.h>
#include <iostream>
#include <vector>

using namespace std;

class DH_ParamsSet {
public:
    double d;
    double theta;
    double a;
    double alpha;
    
    friend std::ostream& operator<<(std::ostream& os, const DH_ParamsSet& x);
    friend std::ostream& operator<<(std::ostream& os, const vector<DH_ParamsSet>& x);
};

#endif /* DH_ParamSet_hpp */
