#ifndef CAPTURE_ROS_H
#define CAPTURE_ROS_H

#include "captureinterface.h"
#include <dirent.h>
#include <string>
#include <list>
#include <algorithm>
#include <fstream>
#include "VarTypes.h"
#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/callback_queue.h>

#ifndef VDATA_NO_QT
  #include <QMutex>
#else
  #include <pthread.h>
#endif

using namespace std;

#ifndef VDATA_NO_QT
  #include <QMutex>
  //if using QT, inherit QObject as a base
class CaptureROS : public QObject, public CaptureInterface
#else
class CaptureROS : public CaptureInterface
#endif
{
#ifndef VDATA_NO_QT
  Q_OBJECT
/*   public slots: */
/*   void changed(VarType * group); */
  protected:
  QMutex mutex;
  public:
#endif

protected:
   //processing variables:
  VarStringEnum * v_colorout;

  //capture variables:
  VarString * v_image_topic;
  VarString * v_camerainfo_topic;
  VarList * capture_settings;
  VarList * conversion_settings;
  fstream file;

  
  unsigned char* frame;
  bool is_capturing;

  double start;
  double end;


  image_transport::ImageTransport * it;
  ros::NodeHandle * nh;
  image_transport::Subscriber sub;
  cv::Mat mat;
  cv::Mat out_img;
  cv::Mat crop;


public:
#ifndef VDATA_NO_QT
  CaptureROS(VarList * _settings, ros::NodeHandle *nh, QObject * parent=0);
  void mvc_connect(VarList * group);
#else
  CaptureROS(VarList * _settings, ros::NodeHandle *nh);
#endif
  ~CaptureROS();
    
  virtual bool startCapture();
  virtual bool stopCapture();
  virtual bool isCapturing() { return is_capturing; };
  
  virtual RawImage getFrame();
  virtual void releaseFrame();
   
  void cleanup();

  virtual string getCaptureMethodName() const;
  virtual bool copyAndConvertFrame(const RawImage & src, RawImage & target);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

};

#endif
