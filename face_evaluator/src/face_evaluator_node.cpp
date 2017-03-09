//#define DLIB_JPEG_SUPPORT
//#undef DLIB_PNG_SUPPORT
#define DLIB_PNG_SUPPORT
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/image_transforms.h>
//#ifdef DLIB_JPEG_STATIC
//#include <dlib/external/libjpeg/jpeglib.h>
//#else
//#include <jpeglib.h>
//#endif
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>
#include <iostream>
#include <dirent.h>
#include <chrono>
#include <ctime>
#include <cmath>

using namespace dlib;
using namespace std;
//using namespace cv;

#define REF_SIZE 10

static const std::string OPENCV_WINDOW = "Image window";
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
  int countTime;
  
public:
  ImageConverter()
    : it_(nh_)//, templates_(REF_SIZE)
  {
    detector_ = get_frontal_face_detector();
    countTime = 0;
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
      dlib::array<array2d<unsigned char>> face_chips(REF_SIZE);
      /* print all the files and directories within directory */
      while ((ent = readdir (dir)) != NULL) 
      {
        //printf ("%s\n", ent->d_name);
        if(ent->d_name[0] == '.')
        {
          continue;
        }
        unsigned long index = ent->d_name[0] - '0'; // index of reference image
        //load_jpeg(face_chips[index], ent->d_name);
        cv::Mat matImg = cv::imread(ent->d_name, 0);
        cv_image<unsigned char> cimg(matImg);
        assign_image(face_chips[index], cimg);
        pyramid_up(face_chips[index]);
        std::vector<rectangle> faces = detector_(face_chips[index]);
        //cv::imshow(OPENCV_WINDOW, matImg);
        //cv::waitKey(3);
        // shape predictor generates 68 facial landmarks (iBUG 300-W scheme)
        if(faces.size() == 0) {
          ROS_ERROR("no faces detected for %s!!!\n", ent->d_name);
          continue;
        }
        else {
          ROS_INFO("Number of faces detected: %d for %s\n", faces.size(), ent->d_name);
        }
        
        full_object_detection shape = sp_(face_chips[index], faces[0]);
        
        templates_[index] = shape;
      }
      closedir (dir);
      // draw each reference image
      /*for (unsigned long i = 0; i < face_chips.size(); ++i)
      {
        for (unsigned long j = 1; j < templates_[i].num_parts(); ++j)
        {
          dlib::draw_line(face_chips[i], templates_[i].part(j-1), templates_[i].part(j), dlib::bgr_pixel(0,255,0));
        }
      }
      image_window win_faces;
      win_faces.set_image(tile_images(face_chips));
      save_png(tile_images(face_chips), "~/references.png");*/
    } 
    else 
    {
      ROS_ERROR("could not open directory\n");
    }
  }

  int computeExpression(full_object_detection shape)
  {
    //ROS_INFO("type: %s", typeid(shape.part(49).y()).name());
    int difference_ = abs(shape.part(49).y() - shape.part(55).y());
    ROS_INFO("difference: %d", difference_);
    if(difference_ > 15)
    {
      return 2;
    }
    else if(difference_ > 10 && difference_ <= 15)
    {
      return 1;
    }
    return 0;
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
        ROS_INFO("expression %d\n", computeExpression(shape));
        //templates_.push_back(shape);
        //save_png(cimg, to_string(countTime)+"_org");
        // draw overlay
        /*for (unsigned long i = 1; i < shape.num_parts(); ++i)
        {
          dlib::draw_line(cimg, shape.part(i-1), shape.part(i), dlib::bgr_pixel(0,255,0));
        }*/
        //std::ofstream fout(to_string(countTime) + ".dat", std::ios::binary);
        //save_png(cimg, to_string(countTime));
        //countTime++;
        //dlib::serialize(shape, fout);
        //fout.close();
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
    ROS_ERROR("\nUsage: rosrun face_evaluator face_evaluator_node shape_predictor_68_face_landmarks.dat templatesFolder\n");
    return 0;
  }
  deserialize(argv[1]) >> sp_;
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  //ic.initTemplates(argv[2]);
  ros::spin();
  return 0;
}
