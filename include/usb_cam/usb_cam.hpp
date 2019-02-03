/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Robert Bosch LLC.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Robert Bosch nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/

#ifndef USB_CAM_USB_CAM_HPP
#define USB_CAM_USB_CAM_HPP

#include <internal_pub_sub/internal_pub_sub.hpp>
#include <rclcpp/rclcpp.hpp>
// #include <image_transport/image_transport.h>
// #include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
// #include <std_srvs/srv/Empty.h>
#include <usb_cam/usb_cam_core.hpp>

namespace usb_cam {

class UsbCam : public rclcpp::Node
{
private:
  // TODO(lucasw) put more things into private
  std::shared_ptr<internal_pub_sub::Core> core_;
public:
  UsbCam(std::shared_ptr<internal_pub_sub::Core> core=nullptr);

  UsbCamCore cam_;
  // shared image message
  sensor_msgs::msg::Image::SharedPtr img_;
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  std::shared_ptr<internal_pub_sub::Publisher> image_pub_;

  // parameters
  std::string video_device_name_ = "/dev/video0";
  std::string frame_id_ = "map";

  std::string io_method_name_ = "mmap";
  // these parameters all have to be a combination supported by the device
  // Use
  // v4l2-ctl --device=0 --list-formats-ext
  // to discover them,
  // or guvcview
  std::string pixel_format_name_ = "yuyv";
  int image_width_ = 640;
  int image_height_ = 480;
  int framerate_ = 15;

  // std::string start_service_name_, start_service_name_;
  // TODO(lucasw) use v4l2ucp for these?
  // int exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
  //    white_balance_, gain_;
  // bool autofocus_, autoexposure_, auto_white_balance_;

  std::string camera_name_;
  // std::string camera_info_url_;
  // boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;


  rclcpp::TimerBase::SharedPtr timer_;

#if 0
  ros::ServiceServer service_start_, service_stop_;

  bool service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.start_capturing();
    return true;
  }


  bool service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.stop_capturing();
    return true;
  }
#endif

  bool initted_ = false;
  void init();

  virtual ~UsbCam();

  bool take_and_send_image();

  std::mutex mutex_;
  void update();
};

}

#endif  // USB_CAM_USB_CAM_HPP
