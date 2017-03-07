#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/image_transforms.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>
#include <iostream>

using namespace dlib;
using namespace std;

//static const std::string OPENCV_WINDOW = "Image window";
shape_predictor sp_; // give shape_predictor_68_face_landmarks.dat in command line

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  frontal_face_detector detector_;
  image_window win_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    detector_ = get_frontal_face_detector();
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    try
    {
      cv_image<bgr_pixel> cimg(cv_ptr->image); // dlib wrapper for cv::Mat object
      //pyramid_up(cimg); // upsample
      std::vector<rectangle> faces = detector_(cimg); // detected bounding boxes
      std::vector<full_object_detection> shapes; // pose of each face
      for (unsigned long j = 0; j < faces.size(); ++j)
      {
        // shape predictor generates 68 facial landmarks (iBUG 300-W scheme)
        full_object_detection shape = sp_(cimg, faces[j]);
        shapes.push_back(shape);
      }
      cv_ptr->image = toMat(cimg); // converts dlib image back to cv::Mat
      win_.clear_overlay();
      win_.set_image(cimg);
      // convert into contour overlays for visualizations
      win_.add_overlay(render_face_detections(shapes));
    }
    catch (exception& e)
    {
      ROS_ERROR("\nexception thrown! %s\n", e.what());
      return;
    }

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  if (argc == 1)
  {
    ROS_ERROR("Please enter shape predictor as a command line argument.\n");
    return 0;
  }
  deserialize(argv[1]) >> sp_;
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
