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
#include <dirent.h>

using namespace dlib;
using namespace std;

#define REF_SIZE 10

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
  std::vector<full_object_detection> templates_; // basis set
  
public:
  ImageConverter()
    : it_(nh_), templates_(REF_SIZE)
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

  void initTemplates(char* templatesPath)
  {
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (templatesPath)) != NULL) 
    {
      dlib::array<array2d<bgr_pixel>> face_chips(REF_SIZE);
      /* print all the files and directories within directory */
      while ((ent = readdir (dir)) != NULL) 
      {
        //printf ("%s\n", ent->d_name);
        unsigned long index = ent->d_name[0] - '0'; // index of reference image
        load_image(face_chips[index], ent->d_name);
        std::vector<rectangle> faces = detector_(face_chips[index]);
        // shape predictor generates 68 facial landmarks (iBUG 300-W scheme)
        full_object_detection shape = sp_(face_chips[index], faces[0]);
        templates_[index] = shape;
      }
      closedir (dir);
      // draw each reference image
      for (unsigned long i = 0; i < face_chips.size(); ++i)
      {
        for (unsigned long j = 1; j < templates_[i].num_parts(); ++j)
        {
          dlib::draw_line(face_chips[i], templates_[i].part(j-1), templates_[i].part(j), dlib::bgr_pixel(0,255,0));
        }
      }
      image_window win_faces;
      win_faces.set_image(tile_images(face_chips));
      save_png(tile_images(face_chips), "~/references.png");
    } 
    else 
    {
      ROS_ERROR("could not open directory\n");
    }
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
  if (argc < 3)
  {
    ROS_ERROR("Usage: rosrun face_evaluator face_evaluator_node shape_predictor_68_face_landmarks.dat templatesFolder\n");
    return 0;
  }
  deserialize(argv[1]) >> sp_;
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ic.initTemplates(argv[2]);
  ros::spin();
  return 0;
}
