/******************************************************************************/
/*                                                                            */
/*  Name:           RurException.h                                            */
/*  Purpose:        Base exception class header                               */
/*  Author:         Mark Burgin                                               */
/*  Date:           12/12/11                                                  */
/*  Copyright:      Copyright (C), RURobots Ltd, 2011                         */
/*                                                                            */
/******************************************************************************/

#ifndef MathsException_h_
#define MathsException_h_

#include <typeinfo>
using namespace std;

namespace RUR_RobotMaths
{

class MathsException : public exception
{
    // Member variables
protected:
    exception* innerException;
protected:
    const char* exceptionText;

    // Constructors and Destructors
public:
    MathsException() throw();
public:
    MathsException(const char* text) throw();
public:
    MathsException(const char* text, const exception& inner) throw();
public:
    virtual ~MathsException() throw();

    // Methods
public:
    virtual const char* what() const throw();
public:
    virtual bool GetInnerException(exception& inner) const throw();
};

} // namespace RUR_RobotMaths

#endif // ifndef MathsException_h_
