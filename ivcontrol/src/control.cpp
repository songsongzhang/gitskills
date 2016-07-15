#include "control.h"

const unsigned int data_sz = 4;
static double                    angleArry[9];
static double tempresult;
static int lastangle;
 control::control(ros::NodeHandle mh)
 {
    Kp = 1.5;
    Kd = 4.5;
    mh.param("Kpparam",Kp,Kp);
    mh.param("Kdparam",Kd,Kd);
    memset(&resultcontrol,0,sizeof(resultcontrol));
    memset(&controlmsg,0,sizeof(controlmsg));
    controlmsg.data.resize(data_sz);
    lastangle = 0;
    for(int i = 0; i<9; i++)
    {
        angleArry[i] = 0;
    }
 }

 control::~control()
 {

 }

void control::beroad(ivcontrol::pathmsg arroad)
{
	road.Num = arroad.number;
	for(int i = 0; i<arroad.number; i++)
	{
		road.point[i].x = arroad.x[i];
		road.point[i].y = arroad.y[i];
		road.point[i].value = arroad.value[i];
		road.point[i].U = arroad.U[i];
	}
}

ivcontrol::controlmsg control::followroad(ssMatrix roadmatrix)
{
    ivcontrol::controlmsg    result;
    memset(&result,0,sizeof(result));
    int angle = 0;
    int deltaDis = kCarPositionW - 1 - roadmatrix.point[0].y;
    int deltaAng = 0;
    double deAng = 0;
    if((kCarPositionH - roadmatrix.point[roadmatrix.Num-1].x)*kResolution>FarDis)
    {
        deltaAng = roadmatrix.point[0].y - roadmatrix.point[FarDis*kResolutionReal].y;
    }
    else
    {
        deltaAng = roadmatrix.point[0].y - roadmatrix.point[roadmatrix.Num-1].y;
    }
    double angtemp = deltaAng*kResolution;
    if(angtemp < 0.000001 && angtemp > -0.000001)
    {
        deAng = 0;
    }
    else
    {
    	deAng = atan(angtemp/FarDis)*180/M_PI;
    }


    for(int j = 0; j<8; j++)
    {
        angleArry[j] = angleArry[j+1];
    }
    angleArry[8] = deAng;
    double tempangle[9];
    for(int j = 0; j<9; j++)
    {
	    tempangle[j] = angleArry[j];
    }
    tempresult = GetMedianNum(tempangle,9);
    ROS_INFO("deAng1 is : [%f]",deAng);
    ROS_INFO("deAng2 is : [%f]",tempresult);
//角度影响因子在６左右
//偏差影响因子在
    angle = Kp*deltaDis +Kd*tempresult;
    int rusult = 0;
    
    
    rusult = (angle+lastangle)/2;
    rusult = (int) (rusult/10)*10;
   // ROS_INFO("I tempresult: [%d]", tempresult);
    //angle = (int) (angle/5)*5;
    result.angle = rusult;
    result.torque = 20;
    result.speed = 10;
    result.state = 0;
    lastangle = angle;


	return result;
}


double control::GetMedianNum(double* bArray,int iFilterLen)
{
    int i,j;
    double bTemp;
    for(j=0;j<iFilterLen-1;j++)
    {
        for(i=0;i<iFilterLen-j-1;i++)
        {
            if(bArray[i]>bArray[i+1])
            {
                bTemp = bArray[i];
                bArray[i] = bArray[i+1];
                bArray[i+1] = bTemp;
            }
        }
    }
    if((iFilterLen & 1)>0)
    {
        bTemp = bArray[(iFilterLen+1)/2];
    }
    else
    {
        bTemp = (bArray[iFilterLen/2]+bArray[iFilterLen/2+1])/2;
    }
    return bTemp;
}
