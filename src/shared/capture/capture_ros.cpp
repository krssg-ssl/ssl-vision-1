#include <sys/time.h>
#include <cctype>
#include "capture_ros.h"
#include "image_io.h"
#include "conversions.h"
#include <ctime>
#include <string>
#include <cstring>
#include <cstdio>


#ifndef VDATA_NO_QT
CaptureROS::CaptureROS(VarList * _settings, ros::NodeHandle *nh, QObject * parent) : QObject(parent), CaptureInterface(_settings), nh(nh)
#else
CaptureROS::CaptureROS(VarList * _settings, ros::NodeHandle *nh) : CaptureInterface(_settings), nh(nh)
#endif
{

  settings->addChild(conversion_settings = new VarList("Conversion Settings"));
  settings->addChild(capture_settings = new VarList("Capture Settings"));

   //=======================CONVERSION SETTINGS=======================
  conversion_settings->addChild(v_colorout=new VarStringEnum("convert to mode",Colors::colorFormatToString(COLOR_YUV422_UYVY)));
  v_colorout->addItem(Colors::colorFormatToString(COLOR_RGB8));
  v_colorout->addItem(Colors::colorFormatToString(COLOR_YUV422_UYVY));
    
  //=======================CAPTURE SETTINGS==========================
  capture_settings->addChild(v_image_topic = new VarString("image topic", "/pylon_camera_node/image_raw"));
  capture_settings->addChild(v_camerainfo_topic = new VarString("camera info topic", "/pylon_camera_node/camera_info"));

  is_capturing = false;
  frame = 0;
  it = NULL;
}

CaptureROS::~CaptureROS()
{
}

bool CaptureROS::stopCapture() 
{
  is_capturing = false;
  cleanup();
  return true;
}

void CaptureROS::cleanup()
{
#ifndef VDATA_NO_QT
  mutex.lock();
#endif
  sub.shutdown();
  delete it;
#ifndef VDATA_NO_QT
  mutex.unlock();
#endif
}

void CaptureROS::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
#ifndef VDATA_NO_QT
  mutex.lock();
#endif
  try
  {
    out_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
#ifndef VDATA_NO_QT
    mutex.unlock();
#endif
    // cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
    // cv::waitKey(30);                                                                                                                                               

    // downsample the image
      cv::resize(out_img, crop, cv::Size(840, 630));
    /*if((v_image_topic->getString().compare("/pylon_camera_2_node_2/image_raw"))==0){

      cv::Mat croppedRef(crop,cv::Rect(258,0,840,630));
      croppedRef.copyTo(mat);
    }
    else{
*/
      cv::Mat croppedRef(crop,cv::Rect(0,0,840,630));

      croppedRef.copyTo(mat);
    
    //out_img.copyTo(mat);
    // crop the left half part of the image
    // cout << "cols: " << crop.cols << " rows: " << crop.rows << "\n";
    
    // rect = cv::Rect(0, 0, 720, 540);




    // if (mat.empty()){
    //   cout << "empty\n";
    // }
    // mat = crop(cv::Range(0, 540), cv::Range(0, 360));
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
#ifndef VDATA_NO_QT
    mutex.unlock();
#endif
  }
}

bool CaptureROS::startCapture()
{
#ifndef VDATA_NO_QT
  mutex.lock();
#endif
  it = new image_transport::ImageTransport(*nh);
  sub = it->subscribe(v_image_topic->getString(), 2, &CaptureROS::imageCallback, this);
  is_capturing = true;
#ifndef VDATA_NO_QT
  mutex.unlock();
#endif
  return true;
}

RawImage CaptureROS::getFrame()
{
#ifndef VDATA_NO_QT
   mutex.lock();
#endif
  RawImage result;
  result.setColorFormat(COLOR_RGB8); 
  result.setTime(0.0);
  int width = 640;
  int height = 480;
  if (mat.cols == 0 /* no frame */) {
    // fprintf (stderr, "ROS Error, seems like no image published on topic.\n");
    // is_capturing=false;
    // result.setData(0);
    result.setWidth(width);
    result.setHeight(height);
    frame = new unsigned char[width*height*3];
    result.setData(frame);
  } else {
    width = mat.cols;
    height = mat.rows;
    frame = new unsigned char[width*height*3];
    unsigned char* p = &frame[0];

    for (int i=0; i < width * height; i++)
    {
      int ii = i/width;
      int jj = i%width;

      *p = mat.at<cv::Vec3b>(ii,jj)[2];
      p++;
      *p = mat.at<cv::Vec3b>(ii,jj)[1];
      p++;
      *p = mat.at<cv::Vec3b>(ii,jj)[0];
      p++;
    }

    result.setWidth(width);
    result.setHeight(height);
    result.setData(frame);
    timeval tv;    
    gettimeofday(&tv,0);
    result.setTime((double)tv.tv_sec + tv.tv_usec*(1.0E-6));
  }
#ifndef VDATA_NO_QT
  mutex.unlock();
#endif 
  // file<<"\nsend data:\t"<<std::clock() /(double) CLOCKS_PER_SEC;
  // file<<"\n\n";
  return result;
}

void CaptureROS::releaseFrame() 
{
#ifndef VDATA_NO_QT
  mutex.lock();
#endif
  delete[] frame;
#ifndef VDATA_NO_QT
  mutex.unlock();
#endif
}

string CaptureROS::getCaptureMethodName() const 
{
  return "FromROS";
}


bool CaptureROS::copyAndConvertFrame(const RawImage & src, RawImage & target)
{
#ifndef VDATA_NO_QT
  mutex.lock();
#endif
  ColorFormat output_fmt = Colors::stringToColorFormat(v_colorout->getSelection().c_str());
  ColorFormat src_fmt=src.getColorFormat();
    
  if (target.getData()==0)
    target.allocate(output_fmt, src.getWidth(), src.getHeight());
  else
    target.ensure_allocation(output_fmt, src.getWidth(), src.getHeight());
     
  target.setTime(src.getTime());
     
  if (output_fmt == src_fmt)
  {
    if (src.getData() != 0)
      memcpy(target.getData(),src.getData(),src.getNumBytes());
  }
  else if (src_fmt == COLOR_RGB8 && output_fmt == COLOR_YUV422_UYVY)
  {
    if (src.getData() != 0)
      dc1394_convert_to_YUV422(src.getData(), target.getData(), src.getWidth(), src.getHeight(), 
                               DC1394_BYTE_ORDER_UYVY, DC1394_COLOR_CODING_RGB8, 8);
  }
  else if (src_fmt == COLOR_YUV422_UYVY && output_fmt == COLOR_RGB8)
  {
    if (src.getData() != 0)
      dc1394_convert_to_RGB8(src.getData(),target.getData(), src.getWidth(), src.getHeight(), 
                             DC1394_BYTE_ORDER_UYVY, DC1394_COLOR_CODING_YUV422, 8);
  } 
  else 
  {
    fprintf(stderr,"Cannot copy and convert frame...unknown conversion selected from: %s to %s\n",
            Colors::colorFormatToString(src_fmt).c_str(),
            Colors::colorFormatToString(output_fmt).c_str());
#ifndef VDATA_NO_QT
    mutex.unlock();
#endif
    return false;
  } 
#ifndef VDATA_NO_QT
  mutex.unlock();
#endif
  return true;
}