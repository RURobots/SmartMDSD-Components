/******************************************************************************/
/*                                                                            */
/*  Name:           Constants.h                                               */
/*  Purpose:        Header defining useful constants                          */
/*  Author:         Mark Burgin                                               */
/*  Date:           23/01/12                                                  */
/*  Copyright:      Copyright (C), RURobots Ltd, 2012                         */
/*                                                                            */
/******************************************************************************/

#ifndef Constants_h_
#define Constants_h_

namespace RUR_RobotMaths
{

class Constants
{
public:
    static const double Pi;
public:
	static const double TwoPi;
public:
    static const double PiOverTwo;

public:
    static const double RadiansToDegrees;
public:
    static const double DegreesToRadians;

public:
    static const long long SecondsToNanoSeconds;
public:
    static const double NanoSecondsToSeconds;
};

} // namespace RUR_RobotMaths

#endif // #ifndef Constants_h_
