/**
 * Copyright (C) 2013 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
**/

/**
 * \defgroup utilsLib utilsLib
 *  
 * @ingroup periPersonalSpace
 *  
 * Utilities used throughout the modules and libraries.
 *
 * \author Alessandro Roncone and Matej Hoffmann
 *  
 * Date: first release 30/10/2013
 *  
 * Copyright (C) 2013 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
**/ 

#ifndef __UTILS_H__
#define __UTILS_H__

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>

#include <vector>
#include <map>
#include <list> 
#include <sstream>

#include "parzenWindowEstimator.h"

//namespace std {
//template<class _Tp, class _Alloc = std::allocator< _Tp > >
//class list;}

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace iCub::iKin;

using namespace std;

/**
 * Converts a matrix to a vector with 16 elements
**/
yarp::sig::Vector toVector(yarp::sig::Matrix m);


/**
 * Closes properly a given port
**/
void closePort(yarp::os::Contactable *_port);

/**
 * Retrieves a matrix from a bottle:
 * @param b  is the bottle
 * @param in is the index from which start acquiring values
 * @param r  is the number of rows of the matrix
 * @param c  is the number of cols of the matrix
**/
yarp::sig::Matrix matrixFromBottle(const yarp::os::Bottle b, int in, const int r, const int c);

/**
 * Retrieves a matrix from a bottle:
 * @param b    is the bottle
 * @param in   is the index from which start acquiring values
 * @param size is the size of the vector
**/
yarp::sig::Vector vectorFromBottle(const Bottle b, int in, const int size);

/**
 * Puts a matrix into a bottle, by cycling through its elements
 * and adding them as double
**/
void matrixIntoBottle(const yarp::sig::Matrix m, Bottle &b);

/**
 * Puts a vector into a bottle, by cycling through its elements
 * and adding them as double
**/
void vectorIntoBottle(const yarp::sig::Vector v, Bottle &b);


/**
* Converts an int to a string
**/
string int_to_string( const int a );

/**
* Computes the factorial using a recursive method
**/
unsigned int factorial(unsigned int n);

/**
* Struct that encloses all the information related to a taxel.
**/
struct IncomingEvent
{
    yarp::sig::Vector Pos;
    yarp::sig::Vector Vel;
    double Radius;          // average radius of the object
    string Src;             // the source of information the event is coming from

    /**
    * Constructor
    **/    
    IncomingEvent();

    /**
    * Constructor with Pos and Vel
    **/    
    IncomingEvent(const Vector &p, const Vector &v, const double r, const string &s);

    /**
    * Constructor from bottle
    **/   
    IncomingEvent(const Bottle &b);

    /**
    * Copy constructor
    **/
    IncomingEvent(const IncomingEvent &e);

    /**
    * 
    **/    
    Bottle toBottle();

    /**
    * 
    **/
    bool fromBottle(const Bottle &b);

    /**
    * Copy Operator
    **/
    IncomingEvent &operator=(const IncomingEvent &e);

    /**
    * Print Method
    **/
    void print();

    /**
    * toString Method
    **/
    string toString(int precision) const;
};

/**
* It has only two more members
**/
struct IncomingEvent4Taxel : public IncomingEvent
{
    double NRM;
    double TTC;

    /**
    * Constructor
    **/    
    IncomingEvent4Taxel();

    /**
    * Constructor with Pos and Vel
    **/    
    IncomingEvent4Taxel(const Vector &p, const Vector &v, const double r, const string &s);

    /**
    * Copy constructor
    **/
    IncomingEvent4Taxel(const IncomingEvent &e);
    IncomingEvent4Taxel(const IncomingEvent4Taxel &e);

    /**
    * Copy Operator
    **/
    IncomingEvent4Taxel &operator=(const IncomingEvent &e);

    /**
    * Copy Operator
    **/
    IncomingEvent4Taxel &operator=(const IncomingEvent4Taxel &e);

    /**
    * Print Method
    **/
    void print();
};

/**
* Struct that encloses all the information related to a taxel.
**/
struct Taxel
{
    int ID;                    // taxels' ID
    int Resp;                  // taxels' activation level (0-255)
    double rfAngle;            // angle of the receptive field [rad]
    yarp::sig::Vector Pos;     // taxel's position w.r.t. the limb
    yarp::sig::Vector WRFPos;  // taxel's position w.r.t. the root RF
    yarp::sig::Vector Norm;    // taxel's normal   w.r.t. the limb
    yarp::sig::Vector px;      // (u,v) projection in the image plane
    yarp::sig::Matrix RF;      // taxel's reference Frame (computed from Pos and Norm)
    
    IncomingEvent4Taxel Evnt;  // IncomingEvent as seen from the taxel's RF

    // parzenWindowEstimator pwe; // taxel's response by means of a parzen window estimator
    // parzenWindowEstimator buf; // buffer for storing temporary events

    parzenWindowEstimator2D pwe2D; // taxel's response by means of a 2D parzen window estimator

    /**
    * Default constructor
    **/    
    Taxel();

    /**
    * Constructor with Pos and Norm
    **/    
    Taxel(const Vector &p, const Vector &n);

    /**
    * Constructor with Pos, Norm and ID
    **/    
    Taxel(const Vector &p, const Vector &n, const int &i);

    /**
    * Copy Operator
    **/
    Taxel &operator=(const Taxel &t);

    /**
    * Print Method
    **/
    void print(int verbosity=0);

    /**
    * toString Method
    **/
    string toString(int precision=0);

    /**
    * init function
    **/
    void init();

    /**
    * Compute and set the taxel's reference frame (from its position and its normal vector)
    **/
    void setRF();

    /**
    * Resets the parzen window estimator
    **/
    bool resetParzenWindow();

    /**
    * Add or remove a sample from the pwe's histogram
    **/
    bool addSample(const IncomingEvent4Taxel ie);
    bool removeSample(const IncomingEvent4Taxel ie);

    /**
    * Computes the response of the taxel.
    **/
    bool computeResponse();

    /**
    * Check if the input sample is inside the Receptive field (i.e. the cone)
    **/
    bool insideRFCheck(const IncomingEvent4Taxel ie);

};

/**
* Class that encloses all the information related to a skinpart.
**/
class skinPart
{
 public:
    string name;
    vector<Taxel> taxel;
    int size;   // size of the skinPart if the patches were full - it differs from taxel.size()
             
    /**
    * Indexing variable used in the case of reducing the resolution - e.g. taking only triangle centers
    * The index into the vector is the taxel ID, the value stored is its representative
    **/
    vector<int> Taxel2Repr; 

    /**
    * Mapping in the opposite direction
    * Indexed by representative taxel IDs, it stores lists of the taxels being represented - e.g. all taxels of a triangle
    **/
    map<unsigned int, list<unsigned int> > Repr2TaxelList;
    /**
    * Constructor
    **/    
    skinPart();

    /**
    * Copy Operator
    **/
    skinPart &operator=(const skinPart &spw);

    /**
    * Print Method
    **/
    void print(int verbosity=0);

    /**
    * toString Method
    **/
    string toString(int precision=0);
};


/**
* Struct that encloses all the information related to the eyes.
* It's used for the projections.
**/
class eyeWrapper
{
public:
    string name;

    iCubEye *eye;

    double headVersion;
    Matrix *Prj;

public:
    /**
    * Constructor
    **/    
    eyeWrapper(string _name, double _hV, const ResourceFinder &_eyeCalibRF);

    /**
    * Copy Operator
    **/
    eyeWrapper &operator=(const eyeWrapper &ew);
};

#endif

// empty line to make gcc happy
