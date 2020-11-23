/******************************************************************************/
/*                                                                            */
/*  Name:           RurException.cpp                                          */
/*  Purpose:        Base Exception class implementation                       */
/*  Author:         Mark Burgin                                               */
/*  Date:           12/12/11                                                  */
/*  Copyright:      Copyright (C), RURobots Ltd, 2011                         */
/*                                                                            */
/******************************************************************************/

#include <string.h>
#include <stdio.h>
#include <cstddef>
#include "MathsException.h"

using namespace RUR_RobotMaths;

/* public */MathsException::MathsException() throw()
{
    exceptionText = "No Exception!";
    innerException = NULL;
}

/* public */MathsException::MathsException(const char *text) throw()
{
    exceptionText = text;
    innerException = NULL;
}

/* public */MathsException::MathsException(const char *text, const exception& inner) throw()
{
    exceptionText = text;
    // ToDo Find a way of storing inner exception in RurException objects
//    innerException = new exception;
//    *innerException = inner;
    innerException = NULL;
}

/* public, virtual */MathsException::~MathsException() throw()
{
    if (innerException != NULL)
    {
        delete innerException;
    }
}

/* public, virtual */const char* MathsException::what() const throw()
{
    if (innerException != NULL)
    {
        char* extendedExceptionText = new char[strlen(exceptionText) + strlen(innerException->what()) + 18];

        sprintf(extendedExceptionText, "MathsException: %s (%s)", exceptionText, innerException->what());

        return extendedExceptionText;
    }
    else
    {
        char* extendedExceptionText = new char[strlen(exceptionText) + 15];

        sprintf(extendedExceptionText, "MathsException: %s", exceptionText);

        return extendedExceptionText;
    }
}

/* public, virtual */bool MathsException::GetInnerException(exception& inner) const throw()
{
    if (innerException != NULL)
    {
        inner = *innerException;
        return true;
    }
    else
    {
        return false;
    }
}

