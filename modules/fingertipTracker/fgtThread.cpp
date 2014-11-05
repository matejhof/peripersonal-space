#include "fgtThread.h"

fgtThread::fgtThread(int _rate, const string &_name, const string &_robot, int _v) :
                       RateThread(_rate), name(_name), robot(_robot), verbosity(_v)
{
    stateFlag = 0;
    timeNow   = yarp::os::Time::now();
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

bool fgtThread::processImages(ImageOf<PixelMono> &_oL, ImageOf<PixelMono> &_oR)
{
    // _oR = *imageInR;
    // _oL = *imageInL;
    // 
    _oL.resize(*imageInL);
    _oR.resize(*imageInR);

    cv::Mat imgL((IplImage*)imageInL->getIplImage());
    cv::Mat imgR((IplImage*)imageInR->getIplImage());
    cv::Mat outL((IplImage*)_oL.getIplImage());
    cv::Mat outR((IplImage*)_oR.getIplImage());
    cv::Mat imgLHSV;
    cv::Mat imgRHSV;

    yTrace("I'm converting the images to their HSV scale");
    cvtColor(imgL,imgLHSV,CV_RGB2HSV);
    cvtColor(imgR,imgRHSV,CV_BGR2HSV);

    yTrace("I'm blurring them in order to remove noise.");
    medianBlur(imgLHSV, imgLHSV, 3);
    medianBlur(imgRHSV, imgRHSV, 3);

    yDebug("I'm filtering according to the HSV");
    cv::inRange(imgLHSV, cv::Scalar(40,50,50), cv::Scalar(60,255,255),outL);
    cv::inRange(imgRHSV, cv::Scalar(40,50,50), cv::Scalar(60,255,255),outR);

    // //remove some further noise
    // cv::Mat element = cv::getStructuringElement(CV_SHAPE_RECT,cvSize(2,2),cvPoint(1,1));  
    // cv::morphologyEx(outL, outL, 2, element, cvPoint(-1,-1), 4);

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
