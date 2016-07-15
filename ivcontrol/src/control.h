#ifndef NODE_EXAMPLE_TALKER_H
#define NODE_EXAMPLE_TALKER_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

#include "ivcontrol/pathmsg.h"
#include "std_msgs/Int16MultiArray.h"
#include "ivcontrol/controlmsg.h"
#include "control.h"

#define    M_PI    3.14159265358979323846
#define    MAXSIZE 1500
#define    FarDis  8
const int    kWidth      = 200;
const int    kHeight     = 800;
const double kResolution = 0.1;
const int    kResolutionReal = kResolution*100;                 //10
const int    kCarPositionW   = 100;
const int    kCarPositionH   = kHeight - 20*kResolutionReal;    //600
const int    kWidthReal      = kCarPositionW/kResolutionReal;   //10
const int    kHeightReal     = kCarPositionH/kResolutionReal;   //60

typedef struct Point
{
	unsigned short x;
	unsigned short y;
	unsigned char value;
	char           U;
}Point;
typedef struct ssMatrix
{
	int Num;
	Point point[MAXSIZE];
}ssMatrix;


  class control
  {
    public:
     //! Constructor.
    control(ros::NodeHandle mh);
    ~control();
   
    void beroad(ivcontrol::pathmsg arroad);
    ivcontrol::controlmsg followroad(ssMatrix roadmatrix);

    double GetMedianNum(double* bArray,int iFilterLen);
     //! Timer callback for publishing message.
  //  void timerCallback(const ros::TimerEvent& event);

    public:
    //! The timer variable used to go to callback function at specified rate.
  //  ros::Timer timer_;


    ssMatrix road;
    ivcontrol::controlmsg resultcontrol;
    std_msgs::Int16MultiArray controlmsg;
    double Kp;
    double Kd;
    
    
    //! Message publisher.
    ros::Publisher pub_;


    //! The actual message.
    std::string message_;
   
    //! The first integer to use in addition.
    int a_;

    //! The second integer to use in addition.
    int b_;
  };
//}

#endif // NODE_EXAMPLE_TALKER_H
