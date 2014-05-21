#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <iomanip>

#include "ultimateTrackerThread.h"

IncomingEvent eventFromBottle(const Bottle &b)
{
    IncomingEvent ie;
    ie.fromBottle(b);
    return ie;
};

ultimateTrackerThread::ultimateTrackerThread(int _rate, const string &_name, const string &_robot, int _v) :
                       RateThread(_rate), name(_name), robot(_robot), verbosity(_v)
{
    int order  =4;
    int inputs =3;
    Matrix A=eye(order);
    Matrix H=zeros(inputs,order);
    Matrix Q=eye(order);
    Matrix R=eye(order);
    posVelEstimator = new Kalman(A,H,Q,R);

    inputPortTest = new BufferedPort<yarp::sig::Vector>;
}

bool ultimateTrackerThread::threadInit()
{
    inputPortTest -> open(("/"+name+"/test:i").c_str());
}

void ultimateTrackerThread::run()
{
    inputVector = inputPortTest -> read(false);

    if (inputVector != NULL)
    {

    }
}


int ultimateTrackerThread::printMessage(const int l, const char *f, ...) const
{
    if (verbosity>=l)
    {
        fprintf(stdout,"*** %s: ",name.c_str());

        va_list ap;
        va_start(ap,f);
        int ret=vfprintf(stdout,f,ap);
        va_end(ap);

        return ret;
    }
    else
        return -1;
}

void ultimateTrackerThread::threadRelease()
{
    delete posVelEstimator;
    posVelEstimator = 0;

    printMessage(0,"Closing ports..\n");
        closePort(inputPortTest);
        printMessage(1,"    inputPortTest successfully closed!\n");
}

// empty line to make gcc happy
