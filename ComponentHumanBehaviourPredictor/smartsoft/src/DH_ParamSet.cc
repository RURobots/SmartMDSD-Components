
/******************************************************************************/
/*                                                                            */
/*  Name:           DH_ParamSet.cpp                                           */
/*  Purpose:        Implementation of class to store DH parameters            */
/*  Author:         John Foley                                                */
/*  Date:           16/10/2020                                                */
/*  Copyright:      Copyright ©, RURobots Ltd, 2020                           */
/*                                                                            */
/******************************************************************************/

#include "DH_ParamSet.hh"
#include <stdio.h>
#include <iostream>

std::ostream& operator<<(std::ostream& os, const DH_ParamsSet& x) {
    os << "========== DP Params ========" << std::endl;
    os << "d=" << x.d << " theta=" << x.theta << " a=" << x.a << " alpha=" << x.alpha << std::endl;
    os << "=============================" << std::endl;
    return os;
};

std::ostream& operator<<(std::ostream& os, const vector<DH_ParamsSet>& x) {
    os << "========== DP Params ========" << std::endl;
    for (std::vector<DH_ParamsSet>::const_iterator it = x.begin() ; it != x.end(); ++it)
    os << "d=" << it->d << " theta=" << it->theta << " a=" << it->a << " alpha=" << it->alpha << std::endl;
    os << "=============================" << std::endl;
   return os;
};

