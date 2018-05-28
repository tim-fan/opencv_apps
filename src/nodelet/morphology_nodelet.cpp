// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-


// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ImgProc/

/**
 * @file Morphology_1.cpp
 * @brief Erosion and Dilation sample code
 * @author OpenCV team
 */
 

#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/MorphologyConfig.h"

namespace opencv_apps {
class MorphologyNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef opencv_apps::MorphologyConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  bool debug_view_;
  ros::Time prev_stamp_;

  int kernel_size_; //size of erosion or dilation kernel

  std::string window_name_;
  static bool need_config_update_;

  void reconfigureCallback(Config &new_config, uint32_t level)
  {
    config_ = new_config;
    kernel_size_ = config_.kernel_size;
  }

  const std::string &frameWithDefault(const std::string &frame, const std::string &image_frame)
  {
    if (frame.empty())
      return image_frame;
    return frame;
  }

  void imageCallbackWithInfo(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    do_work(msg, cam_info->header.frame_id);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    do_work(msg, msg->header.frame_id);
  }

  static void trackbarCallback( int, void* )
  {
    need_config_update_ = true;
  }

  void do_work(const sensor_msgs::ImageConstPtr& msg, const std::string input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Do the work
      cv::Mat src_gray;

      /// Convert it to gray
      if ( frame.channels() > 1 ) {
        cv::cvtColor( frame, src_gray, cv::COLOR_RGB2GRAY );
      } else {
        src_gray = frame;
      }

      /// Create window
      if( debug_view_) {
        cv::namedWindow( window_name_, cv::WINDOW_AUTOSIZE );
      }

      std::string new_window_name;
      cv::Mat img_dst;
      
      int structure_elem = 0;
      int structure_type;  //todo: expose type as dynamic cfg parameter
      if( structure_elem == 0 ){ structure_type = cv::MORPH_RECT; }
      else if( structure_elem == 1 ){ structure_type = cv::MORPH_CROSS; }
      else if( structure_elem == 2) { structure_type = cv::MORPH_ELLIPSE; }  
          
      cv::Mat element = cv::getStructuringElement( structure_type,
                                           cv::Size( 2*kernel_size_ + 1, 2*kernel_size_+1 ),
                                           cv::Point( kernel_size_, kernel_size_ ) );
      switch (config_.operation) {
        case opencv_apps::Morphology_Erode:
          {                                     
            /// Apply the erosion operation
            cv::erode( src_gray, img_dst, element);
            new_window_name = "Erosion Demo";

            break;
          }
        case opencv_apps::Morphology_Dilate:
          {
            /// Apply the dilation operation
            cv::dilate( src_gray, img_dst, element );

            new_window_name = "Dilation Demo";

            break;
          }
        case opencv_apps::Morphology_Open:
          {
            /// Apply the opening operation
            morphologyEx( src_gray, img_dst, cv::MORPH_OPEN, element );
            
            new_window_name = "Opening Demo";

            break;
          }
      }
      
      if( debug_view_) {
        if (need_config_update_) {
          config_.kernel_size = kernel_size_;
          reconfigure_server_->updateConfig(config_);
          need_config_update_ = false;
        }
        if( window_name_ == new_window_name) {
          cv::createTrackbar( "Size:", window_name_, &kernel_size_, 100, trackbarCallback);
        }

        if (window_name_ != new_window_name) {
          cv::destroyWindow(window_name_);
          window_name_ = new_window_name;
        }
        cv::imshow( window_name_, img_dst );
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, img_dst).toImageMsg();
      img_pub_.publish(out_img);
    }
    catch (cv::Exception &e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void subscribe()
  {
    NODELET_DEBUG("Subscribing to image topic.");
    if (config_.use_camera_info)
      cam_sub_ = it_->subscribeCamera("image", 3, &MorphologyNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", 3, &MorphologyNodelet::imageCallback, this);
  }

  void unsubscribe()
  {
    NODELET_DEBUG("Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  virtual void onInit()
  {
    Nodelet::onInit();
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*nh_));

    pnh_->param("debug_view", debug_view_, false);

    if (debug_view_) {
      always_subscribe_ = true;
    }
    prev_stamp_ = ros::Time(0, 0);

    window_name_ = "Morphology Demo";
    kernel_size_ = 3; 

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&MorphologyNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);

    onInitPostProcess();
  }
};
bool MorphologyNodelet::need_config_update_ = false;
} // namespace opencv_apps


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::MorphologyNodelet, nodelet::Nodelet);
