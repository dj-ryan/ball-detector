/****************************************************************************
*
*   Copyright (c) 2011 Carrick Detweiler
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*
* Started off from: http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
*
******************************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

#include <ball_detector/ballLocation.h>
#include <ball_detector/ballDebug.h>
#include <ball_detector/debugTimes.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
using namespace std;

namespace enc = sensor_msgs::image_encodings;

//Define to enable debugging (images, times, etc)
#define BALLDETECTOR_DEBUG



class BallDetector{
public:
  BallDetector();
  ~BallDetector(){
  }
private:
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  cv::Scalar lowThresh, highThresh;

  void updateHighThreshold(const geometry_msgs::Vector3::ConstPtr& thresh);
  void updateLowThreshold(const geometry_msgs::Vector3::ConstPtr& thresh);


  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub_;

  //Publishers for the min/max hsv values we see in the center of the window
  ros::Publisher lowh_pub_, lows_pub_, lowv_pub_;
  ros::Publisher meanh_pub_, means_pub_, meanv_pub_;
  ros::Publisher highh_pub_, highs_pub_, highv_pub_;

  //subscribers that update the thresholds for HSV values
  ros::Subscriber updateLowHSV_sub_;
  ros::Subscriber updateHighHSV_sub_;

#ifdef BALLDETECTOR_DEBUG
  //For debugging
  void debugCallBack(const ball_detector::ballDebug::ConstPtr& debug);
  ros::Subscriber debug_sub_;
  ball_detector::ballDebug debugLevel;

  //Publishers for misc debug images
  image_transport::Publisher debugImg1_pub_,debugImg2_pub_,debugImg3_pub_,debugImg4_pub_;

  //For sending out debug times
  void debugTimeInit();
  void debugTimeStart();
  void debugTimeRecordAndReStart(string description);
  void debugTimeSend();

  ros::Publisher debugTime_pub_;
  ball_detector::debugTimes debugTimes;
  ros::Time debugTimerStart, debugTimerTotalTime;

#endif /** BALLDETECTOR_DEBUG **/

  //Circle x,y,radius
  ros::Publisher circle_pub_;

};

BallDetector::BallDetector()
  :it(nh)
  {
    
    //Get any thresholds from the parameter server
    //centered lens camera
    //int lowh = 125, lows = 0, lowv = 25;
    //int highh = 185, highs = 95, highv = 175;
    //off-center lens camera
    int lowh = 125, lows = 90, lowv = 25;
    int highh = 185, highs = 190, highv = 255;
    nh.param("thresh/low/h",lowh,lowh);
    nh.setParam("thresh/low/h",lowh);
    nh.param("thresh/low/s",lows,lows);
    nh.setParam("thresh/low/s",lows);
    nh.param("thresh/low/v",lowv,lowv);
    nh.setParam("thresh/low/v",lowv);
    nh.param("thresh/high/h",highh,highh);
    nh.setParam("thresh/high/h",highh);
    nh.param("thresh/high/s",highs,highs);
    nh.setParam("thresh/high/s",highs);
    nh.param("thresh/high/v",highv,highv);
    nh.setParam("thresh/high/v",highv);

    lowThresh = cv::Scalar(lowh,lows,lowv,0);
    highThresh = cv::Scalar(highh,highs,highv,0);

    //Subscribe to change messages
    updateLowHSV_sub_ = nh.subscribe<geometry_msgs::Vector3>("thresh/low",1,&BallDetector::updateLowThreshold,this);
    updateHighHSV_sub_ = nh.subscribe<geometry_msgs::Vector3>("thresh/high",1,&BallDetector::updateHighThreshold,this);

    image_sub_ = it.subscribe("image", 1, &BallDetector::imageCb, this);

    
    lowh_pub_ = nh.advertise<std_msgs::Float64>("hsv/center/low/h", 1);
    lows_pub_ = nh.advertise<std_msgs::Float64>("hsv/center/low/s", 1);
    lowv_pub_ = nh.advertise<std_msgs::Float64>("hsv/center/low/v", 1);
    
    meanh_pub_ = nh.advertise<std_msgs::Float64>("hsv/center/mean/h", 1);
    means_pub_ = nh.advertise<std_msgs::Float64>("hsv/center/mean/s", 1);
    meanv_pub_ = nh.advertise<std_msgs::Float64>("hsv/center/mean/v", 1);

    highh_pub_ = nh.advertise<std_msgs::Float64>("hsv/center/high/h", 1);
    highs_pub_ = nh.advertise<std_msgs::Float64>("hsv/center/high/s", 1);
    highv_pub_ = nh.advertise<std_msgs::Float64>("hsv/center/high/v", 1);
    
    circle_pub_ = nh.advertise<ball_detector::ballLocation>("ballLocation", 1);
    
#ifdef BALLDETECTOR_DEBUG
    //Advertise debug images
    debugImg1_pub_ = it.advertise("balldebug/img1",1);
    debugImg2_pub_ = it.advertise("balldebug/img2",1);
    debugImg3_pub_ = it.advertise("balldebug/img3",1);
    debugImg4_pub_ = it.advertise("balldebug/img4",1);

    //For setting the debug level
    debug_sub_ = nh.subscribe<ball_detector::ballDebug>("balldebug/level", 1, &BallDetector::debugCallBack, this);
    //Debug defaults
    debugLevel.sendDebugImages = true;
    //debugLevel.sendDebugImages = false;
    debugLevel.sendDebugTimes = true;

    //Debug time pub
    debugTime_pub_ = nh.advertise<ball_detector::debugTimes>("balldebug/times",1);

#endif /** BALLDETECTOR_DEBUG **/
  }


#ifdef BALLDETECTOR_DEBUG
/**
 * Initialize the debug timer system.  Any timings recorded with
 * debugTimeRecordAndReStart() will be included in any messages that
 * are sent with debugTimeSend() after a call to this function.
 **/
inline void BallDetector::debugTimeInit(){
  debugTimes.labels.clear();
  debugTimes.times.clear();
  //Reserve the first slot for the total time
  debugTimes.labels.push_back("total time");
  debugTimes.times.push_back(-1.0);
  debugTimes.labels.push_back("ravg total time");
  debugTimes.times.push_back(-1.0);
  debugTimerTotalTime = ros::Time::now();
}

/**
 * Start the timer.  Note that you can call this whenever you want and
 * as many times as you want (e.g. to restart the timing)
 **/
inline void BallDetector::debugTimeStart(){
  debugTimerStart = ros::Time::now();
}

/**
 * Record the time it took since the last debugTimeStart() and this
 * function call and record the result in the message that
 * debugTimeSend() sends out.  The description string is also included
 * in the message to provide some readable info on what was timed.
 * Once the time is recorded, the timer is reset.  This enables just
 * calling this function continuously.
 **/
inline void BallDetector::debugTimeRecordAndReStart(string description){
  double time = (ros::Time::now()-debugTimerStart).toSec();
  //add them
  debugTimes.labels.push_back(description);
  debugTimes.times.push_back(time);
  //restart timer
  debugTimeStart();
}

/**
 * Send the recorded debug times out.  Also records the time it took
 * from the first time debugTimeInit() was called. This also reinits
 * the system with debugTimeInit().
 **/
inline void BallDetector::debugTimeSend(){
  double time = (ros::Time::now()-debugTimerTotalTime).toSec();
  static double avgTime = time;
  avgTime = 0.99*avgTime + 0.01*time;
  
  //first slot was reserved in init for the total time
  debugTimes.times.at(0) = time;
  //second is the average time
  debugTimes.times.at(1) = avgTime;

  debugTimes.header.stamp = ros::Time::now();
  debugTime_pub_.publish(debugTimes);

  debugTimeInit();
}
#endif /** BALLDETECTOR_DEBUG **/


#ifdef BALLDETECTOR_DEBUG
void BallDetector::debugCallBack(const ball_detector::ballDebug::ConstPtr& debug){
  debugLevel = *debug;
  ROS_INFO("Debug level changed");
  ROS_INFO("  send debug images %s",debugLevel.sendDebugImages?"true":"false");
  ROS_INFO("  send debug times %s",debugLevel.sendDebugTimes?"true":"false");
}
#endif /** BALLDETECTOR_DEBUG **/

void BallDetector::updateHighThreshold(const geometry_msgs::Vector3::ConstPtr& thresh){
  int h = thresh->x+1, s = thresh->y+1, v = thresh->z+1;
  ROS_INFO("Updated ball high HSV threshold to h,s,v: %d,%d,%d",h,s,v);
  highThresh = cv::Scalar(h,s,v,0);
  nh.setParam("thresh/low/h",h);
  nh.setParam("thresh/low/s",s);
  nh.setParam("thresh/low/v",v);
}

void BallDetector::updateLowThreshold(const geometry_msgs::Vector3::ConstPtr& thresh){
  int h = thresh->x-1, s = thresh->y-1, v = thresh->z-1;
  ROS_INFO("Updated ball low HSV threshold to h,s,v: %d,%d,%d",h,s,v);
  lowThresh = cv::Scalar(h,s,v,0);
  nh.setParam("thresh/high/h",h);
  nh.setParam("thresh/high/s",s);
  nh.setParam("thresh/high/v",v);
}


void BallDetector::imageCb(const sensor_msgs::ImageConstPtr& msg){
  static cv_bridge::CvImageConstPtr cv_const_ptr;

#ifdef BALLDETECTOR_DEBUG
  static cv_bridge::CvImage cv_img;
  cv_img.header.stamp = ros::Time::now();

  if(debugLevel.sendDebugTimes){
    debugTimeInit();
    debugTimeStart();
  }
#endif

  try{
    cv_const_ptr = cv_bridge::toCvShare(msg, enc::RGB8);
  }catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

#ifdef BALLDETECTOR_DEBUG
  if(debugLevel.sendDebugTimes){
    debugTimeRecordAndReStart("cvconvert");
  }
#endif

#ifdef BALLDETECTOR_DEBUG
  //Get a the image just as a Mat
  static cv::Mat colorImg;
  if(debugLevel.sendDebugImages){
    colorImg = cv_const_ptr->image.clone();
    //cv_const_ptr->image.copyTo(colorImg);
  }else{
    colorImg = cv_const_ptr->image;
  }
#else 
  static cv::Mat colorImg;
  colorImg = cv_const_ptr->image;
#endif

  //Create the hsv image
  static cv::Mat hsvImg;
#ifdef BALLDETECTOR_DEBUG
  if(debugLevel.sendDebugTimes){
    debugTimeRecordAndReStart("allocate");
  }
#endif
  
  //colorImg is type CV_8U
  cv::cvtColor(colorImg,hsvImg,CV_RGB2HSV);
  //cv::cvtColor(colorImg,hsvImg,CV_RGB2HLS);
  //cv::cvtColor(colorImg,hsvImg,CV_RGB2Lab); //faster but not on gumstix?

#ifdef BALLDETECTOR_DEBUG
  if(debugLevel.sendDebugTimes){
    debugTimeRecordAndReStart("hsv");
  }
#endif

  //Perform thresholding
  static cv::Mat threshImg;
  cv::inRange(hsvImg,lowThresh,highThresh,threshImg);


#ifdef BALLDETECTOR_DEBUG
  if(debugLevel.sendDebugTimes){
    debugTimeRecordAndReStart("threshold");
  }
#endif

#ifdef BALLDETECTOR_DEBUG
  if(debugLevel.sendDebugImages){
    //Send some images out
    cv_img.image = threshImg;
    cv_img.encoding = enc::MONO8; // TYPE_8UC1;
    debugImg2_pub_.publish(cv_img.toImageMsg());

    cv_img.image = hsvImg;
    cv_img.encoding = enc::RGB8;
    debugImg3_pub_.publish(cv_img.toImageMsg());

  }
#endif /** BALLDETECTOR_DEBUG **/

#ifdef BALLDETECTOR_DEBUG
  if(debugLevel.sendDebugTimes){
    debugTimeStart();
  }
#endif
  //Contours
  static vector<vector<cv::Point> > contours;
  contours.clear();
  static vector<cv::Vec4i> hierarchy;
  hierarchy.clear();

  //Some quick testing indicates CV_RETR_* doesn't make too much
  //difference in performance.  The CV_CHAIN_APPROX_* also doesn't
  //seem to make much difference, but the "NONE" draws the full
  //outline with debug mode (taking more time), but is easier to look
  //at the output.
  cv::findContours( threshImg, contours, hierarchy,
  CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
  //cv::findContours( threshImg, contours, hierarchy,
  //CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );


#ifdef BALLDETECTOR_DEBUG
  if(debugLevel.sendDebugTimes){
    debugTimeRecordAndReStart("findcontours");
  }
#endif

  // iterate through all the top-level contours,
  // draw each connected component with its own random color
  int idxMax = -1;
  unsigned int idxMaxValue = 0;
  int idx = 0;
  if(hierarchy.size() > 0){
    for( ; idx >= 0; idx = hierarchy[idx][0] ){
      //If it is large enough, fit an ellipse to it
      if(contours.at(idx).size() > 15){
        //Slower than the below methods
        //cv::RotatedRect r = cv::fitEllipse(cv::Mat(contours.at(idx)));
        //cv::Rect rect = r.boundingRect();

        //Find the min/max pixel locatiosn of x and y
        int maxX = 0, maxY = 0;
        unsigned int maxXidx = 0, maxYidx = 0;
        //Just needs to be bigger than max image width/height
        int minX = 999999999, minY = 999999999;
        unsigned int minXidx = 0, minYidx = 0;
        for(unsigned int i = 0; i < contours.at(idx).size(); i++){
          cv::Point p = contours.at(idx).at(i);
          if(p.x > maxX){
            maxXidx = i;
            maxX = p.x;
          }
          else if(p.x < minX){
            minXidx = i;
            minX = p.x;
          }
          if(p.y > maxY){
            maxYidx = i;
            maxY = p.y;
          }
          else if(p.y < minY){
            minYidx = i;
            minY = p.y;
          }
        }

        //Keep the ellipse if it is mostly round
          //if((rect.width/(1.0*rect.height) > 0.75) && (rect.height/(1.0*rect.width) > 0.75)){

        if(((1.0*(contours.at(idx).at(maxXidx).x-contours.at(idx).at(minXidx).x)/
             (contours.at(idx).at(maxYidx).y-contours.at(idx).at(minYidx).y))) > 0.75
           &&
           ((1.0*(contours.at(idx).at(maxYidx).y-contours.at(idx).at(minYidx).y)/
             (contours.at(idx).at(maxXidx).x-contours.at(idx).at(minXidx).x)) > 0.75)){
#ifdef BALLDETECTOR_DEBUG
          if(debugLevel.sendDebugImages){
            //cv::ellipse(colorImg,r,cv::Scalar(255,0,0,0),3);

            //Color all the points white (it isn't all points, but a lot on the boarders)
            for(unsigned int i = 0; i < contours.at(idx).size(); i++){
              cv::Point p = contours.at(idx).at(i);
              //Get a pointer to the row, then access the pixel elements
              colorImg.ptr<uchar>(p.y)[3*p.x] = 255;
              colorImg.ptr<uchar>(p.y)[3*p.x+1] = 255;
              colorImg.ptr<uchar>(p.y)[3*p.x+2] = 255;
            }

            //draw max/min points
            /*
            cv::rectangle(colorImg,
                          cv::Point(contours.at(idx).at(maxXidx).x - 1, 
                                    contours.at(idx).at(maxXidx).y- 1),
                          cv::Point(contours.at(idx).at(maxXidx).x + 1, 
                                    contours.at(idx).at(maxXidx).y+ 1),
                          cv::Scalar(255,0,0,0),2);
            cv::rectangle(colorImg,
                          cv::Point(contours.at(idx).at(maxYidx).x - 1, 
                                    contours.at(idx).at(maxYidx).y- 1),
                          cv::Point(contours.at(idx).at(maxYidx).x + 1, 
                                    contours.at(idx).at(maxYidx).y+ 1),
                          cv::Scalar(255,0,0,0),2);
            cv::rectangle(colorImg,
                          cv::Point(contours.at(idx).at(minXidx).x - 1, 
                                    contours.at(idx).at(minXidx).y- 1),
                          cv::Point(contours.at(idx).at(minXidx).x + 1, 
                                    contours.at(idx).at(minXidx).y+ 1),
                          cv::Scalar(255,0,0,0),2);
            cv::rectangle(colorImg,
                          cv::Point(contours.at(idx).at(minYidx).x - 1, 
                              contours.at(idx).at(minYidx).y- 1),
                          cv::Point(contours.at(idx).at(minYidx).x + 1, 
                                    contours.at(idx).at(minYidx).y+ 1),
                          cv::Scalar(255,0,0,0),2);
            */
          }
#endif /** BALLDETECTOR_DEBUG **/
          //If there are more points in it than our previous max, then record it
          //N.B. contours.at(idx).size() isn't all points, rather a
          //representation of them...but it seems to map fairly well.
          if(idxMaxValue < contours.at(idx).size()){
            idxMaxValue = contours.at(idx).size();
            idxMax = idx;
          }
        }
      }
    }  
  }


#ifdef BALLDETECTOR_DEBUG
  if(debugLevel.sendDebugTimes){
    debugTimeRecordAndReStart("iteratecontours");
  }
#endif
  //Make sure we found a good one
  if(idxMax >= 0){

    //Now publish and draw the best
    cv::RotatedRect ellipse = cv::fitEllipse(cv::Mat(contours.at(idxMax)));
    cv::Rect rect = ellipse.boundingRect();
    //Publish the message for the best
    ball_detector::ballLocation circ;
    circ.header.stamp = ros::Time::now();
    //Use the average of the height and width as the radius
    double radius = (rect.height/2.0 + rect.width/2.0)/2.0;
    circ.imageWidth = colorImg.cols;
    circ.imageHeight = colorImg.rows;
    //Make it relative to the center of the image
    circ.x = rect.x + radius - threshImg.cols/2;
    circ.y = -1*(rect.y + radius - threshImg.rows/2);
    circ.radius = radius;
    circle_pub_.publish(circ);
#ifdef BALLDETECTOR_DEBUG
    if(debugLevel.sendDebugImages){
      //Draw the best one bold
      cv::ellipse(colorImg,ellipse,cv::Scalar(0,255,0,0),4);

      //And the center point
      cv::rectangle(colorImg,
                    cv::Point(rect.x + radius-2,rect.y + radius-2),
                    cv::Point(rect.x + radius+2,rect.y + radius+2),
                    cv::Scalar(0,255,0,0),4);

      //Find min/max x,y pixel locations to draw them
      //cout << "---------------------\n";
      int maxX = 0, maxY = 0;
      unsigned int maxXidx = 0, maxYidx = 0;
      //Just needs to be bigger than max image width/height
      int minX = 999999999, minY = 999999999;
      unsigned int minXidx = 0, minYidx = 0;
      for(unsigned int i = 0; i < contours.at(idxMax).size(); i++){
        cv::Point p = contours.at(idxMax).at(i);
        //printf("%d: [%d,%d] ",i,p.x,p.y);
        if(p.x > maxX){
          maxXidx = i;
          maxX = p.x;
        }
        else if(p.x < minX){
          minXidx = i;
          minX = p.x;
        }
        if(p.y > maxY){
          maxYidx = i;
          maxY = p.y;
        }
        else if(p.y < minY){
          minYidx = i;
          minY = p.y;
        }
      }


      /*
      printf("\nminX: %d, %d, %d, %d\n",minX,minXidx,
             contours.at(idxMax).at(minXidx).x,contours.at(idxMax).at(minXidx).y);
      printf("maxX: %d, %d, %d, %d\n",maxX,maxXidx,
             contours.at(idxMax).at(maxXidx).x,contours.at(idxMax).at(maxXidx).y);
      printf("minY: %d, %d, %d, %d\n",minY,minYidx,
             contours.at(idxMax).at(minYidx).x,contours.at(idxMax).at(minYidx).y);
      printf("maxY: %d, %d, %d, %d\n",maxY,maxYidx,
             contours.at(idxMax).at(maxYidx).x,contours.at(idxMax).at(maxYidx).y);
      printf("diffX: %d\n",contours.at(idxMax).at(maxXidx).x-contours.at(idxMax).at(minXidx).x);
      printf("diffY: %d\n",contours.at(idxMax).at(maxYidx).y-contours.at(idxMax).at(minYidx).y);
      printf("ratio: %f %f\n",
             1.0*(contours.at(idxMax).at(maxXidx).x-contours.at(idxMax).at(minXidx).x)/
             (contours.at(idxMax).at(maxYidx).y-contours.at(idxMax).at(minYidx).y),
             1.0*(contours.at(idxMax).at(maxYidx).y-contours.at(idxMax).at(minYidx).y)/
             (contours.at(idxMax).at(maxXidx).x-contours.at(idxMax).at(minXidx).x));
      */

      //draw max/min points
      cv::rectangle(colorImg,
                    cv::Point(contours.at(idxMax).at(maxXidx).x - 1, 
                              contours.at(idxMax).at(maxXidx).y - 1),
                    cv::Point(contours.at(idxMax).at(maxXidx).x + 1, 
                              contours.at(idxMax).at(maxXidx).y + 1),
                    cv::Scalar(0,0,255,0),2);
      cv::rectangle(colorImg,
                    cv::Point(contours.at(idxMax).at(maxYidx).x - 1, 
                              contours.at(idxMax).at(maxYidx).y - 1),
                    cv::Point(contours.at(idxMax).at(maxYidx).x + 1, 
                              contours.at(idxMax).at(maxYidx).y + 1),
                    cv::Scalar(0,0,255,0),2);
      cv::rectangle(colorImg,
                    cv::Point(contours.at(idxMax).at(minXidx).x - 1, 
                              contours.at(idxMax).at(minXidx).y - 1),
                    cv::Point(contours.at(idxMax).at(minXidx).x + 1, 
                              contours.at(idxMax).at(minXidx).y + 1),
                    cv::Scalar(0,0,255,0),2);
      cv::rectangle(colorImg,
                    cv::Point(contours.at(idxMax).at(minYidx).x - 1, 
                              contours.at(idxMax).at(minYidx).y - 1),
                    cv::Point(contours.at(idxMax).at(minYidx).x + 1, 
                              contours.at(idxMax).at(minYidx).y + 1),
                    cv::Scalar(0,0,255,0),2);
    }
#endif /** BALLDETECTOR_DEBUG **/
  }


#ifdef BALLDETECTOR_DEBUG
  if(debugLevel.sendDebugTimes){
    debugTimeRecordAndReStart("publish");
  }
#endif

#ifdef BALLDETECTOR_DEBUG
  if(debugLevel.sendDebugImages){
    cv_img.image = colorImg;
    cv_img.encoding = enc::RGB8;
    debugImg1_pub_.publish(cv_img.toImageMsg());

  }

  if(debugLevel.sendDebugTimes){
    debugTimeSend();
  }
#endif /** BALLDETECTOR_DEBUG **/

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ball_detector");
  BallDetector bd;
  ros::spin();
  return 0;
}
