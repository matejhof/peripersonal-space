#include "fgtThread.h"

fgtThread::fgtThread(int _rate, const string &_name, const string &_robot, int _v) :
                       RateThread(_rate), name(_name), robot(_robot), verbosity(_v)
{
    stateFlag = 0;
    timeNow   = yarp::os::Time::now();

    fingerL.resize(2,0.0);
    fingerR.resize(2,0.0);
}

bool fgtThread::threadInit()
{
    imagePortInR. open(("/"+name+"/imageR:i").c_str());
    imagePortInL. open(("/"+name+"/imageL:i").c_str());
    imagePortOutR.open(("/"+name+"/imageR:o").c_str());
    imagePortOutL.open(("/"+name+"/imageL:o").c_str());

    doubleTouchPort.open(("/"+name+"/doubleTouch:i").c_str());
    outPort.open(("/"+name+"/out:o").c_str());


    // I know that I should remove this but it's harmless (and I'm overly lazy)
        if (robot=="icub")
        {
            Network::connect("/icub/camcalib/left/out",("/"+name+"/imageL:i").c_str());
            Network::connect("/icub/camcalib/right/out",("/"+name+"/imageR:i").c_str());
        }
        else
        {
            Network::connect("/icubSim/cam/left",("/"+name+"/imageL:i").c_str());
            Network::connect("/icubSim/cam/right",("/"+name+"/imageR:i").c_str());
        }

        Network::connect(("/"+name+"/imageL:o").c_str(),"/yvL");
        Network::connect(("/"+name+"/imageR:o").c_str(),"/yvR");
        Network::connect("/doubleTouch/status:o",("/"+name+"/doubleTouch:i").c_str());

    return true;
}

void fgtThread::run()
{
    yTrace("Running..");

    ImageOf<PixelRgb> *tmpL = imagePortInL.read(false);
    if(tmpL!=NULL)
    {
        imageInL = tmpL;
    }
    ImageOf<PixelRgb> *tmpR = imagePortInR.read(false);
    if(tmpR!=NULL)
    {
        imageInR = tmpR;
    }

    // process the doubleTouch
    // if(doubleTouchBottle = doubleTouchPort.read(false))
    // {
    //     if(doubleTouchBottle != NULL)
    //     {
    //         if (doubleTouchBottle->get(3).asString() != "")
    //         {
    //             doubleTouchStep = doubleTouchBottle->get(0).asInt();

    //             if(doubleTouchStep>1 && doubleTouchStep<7)
    //             {
                        if (imageInL!=NULL && imageInR!=NULL)
                        {
                            yTrace("Processing images..");
                            processImages(imageOutL,imageOutR);
                            sendImages();
                            sendFinger();
                        }
    //             }
    //         }
    //     }
    // }
}

bool fgtThread::sendFinger()
{

}

bool fgtThread::processImages(ImageOf<PixelRgb> &_oL, ImageOf<PixelRgb> &_oR)
{
    _oR.resize(*imageInR);
    _oL.resize(*imageInL);
    // 
    ImageOf<PixelMono> maskL;
    ImageOf<PixelMono> maskR;
    maskL.resize(*imageInL);
    maskR.resize(*imageInR);

    ImageOf<PixelRgb> imageOL=*imageInL;
    ImageOf<PixelRgb> imageOR=*imageInR;

    cv::Mat imgL((IplImage*)imageOL.getIplImage());
    cv::Mat imgR((IplImage*)imageOR.getIplImage());
    cv::Mat mskL((IplImage*)maskL.getIplImage());
    cv::Mat mskR((IplImage*)maskR.getIplImage());
    cv::Mat imgLHSV;
    cv::Mat imgRHSV;

    yTrace("I'm converting the images to in HSV");
    cvtColor(imgL,imgLHSV,CV_RGB2HSV);
    cvtColor(imgR,imgRHSV,CV_BGR2HSV);

    yTrace("I'm filtering according to the HSV");
    cv::inRange(imgLHSV, cv::Scalar(40,50,50), cv::Scalar(60,255,255),mskL);
    cv::inRange(imgRHSV, cv::Scalar(40,50,50), cv::Scalar(80,255,255),mskR);

    yTrace("I'm blurring them in order to remove noise.");
    medianBlur(mskL, mskL, 5);
    medianBlur(mskR, mskR, 5);

    // The mask could have more than one blob: find the biggest one
    // (hopefully the fingertip)
    yTrace("Let's find the biggest contour");
    vector<vector<cv::Point> > contours;
    double area = -1;
    cv::RotatedRect rect;

    contours.clear();
    cv::findContours(mskL,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    int idx=-1;
    int largestArea=0;

    if (contours.size()>0)
    {
        for (size_t i=0; i<contours.size(); i++)
        {
            area=cv::contourArea(contours[i]);
            if (area>largestArea)
            {
                largestArea=area;
                idx=i;
            }
        }

        if (largestArea>12)
        {
            // 5d: Find the center of mass of the biggest contour
            rect=cv::fitEllipse(contours[idx]);
            cv::Point fgtL=rect.center;
            fingerL[0] = fgtL.x;
            fingerL[1] = fgtL.y;

            cv::ellipse(imgL, rect, cv::Scalar(40,50,50), 2);
            cv::circle(imgL, fgtL, 3, cv::Scalar(175,125,0), -1);
        }
    }

    area = -1;
    contours.clear();
    cv::findContours(mskR,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    idx=-1;
    largestArea=0;

    if (contours.size()>0)
    {
        for (size_t i=0; i<contours.size(); i++)
        {
            area=cv::contourArea(contours[i]);
            if (area>largestArea)
            {
                largestArea=area;
                idx=i;
            }
        }

        if (largestArea>16)
        {
            // 5d: Find the center of mass of the biggest contour
            rect=cv::fitEllipse(contours[idx]);
            cv::Point fgtR=rect.center;
            fingerR[0] = fgtR.x;
            fingerR[1] = fgtR.y;

            cv::ellipse(imgR, rect, cv::Scalar(40,50,50), 2);
            cv::circle (imgR, fgtR, 3, cv::Scalar(175,125,0), -1);
        }
    }

    _oL = imageOL;
    _oR = imageOR;

    return true;
}

bool fgtThread::sendImages()
{
    imagePortOutL.prepare()=imageOutL;
    imagePortOutL.write();

    imagePortOutR.prepare()=imageOutR;
    imagePortOutR.write();

    return true;
}

int fgtThread::printMessage(const int l, const char *f, ...) const
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

void fgtThread::threadRelease()
{
}

// empty line to make gcc happy
