//#define DLIB_JPEG_SUPPORT
#define DLIB_PNG_SUPPORT
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
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
#include <cmath>

using namespace dlib;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

#define REF_SIZE 10

//static const std::string OPENCV_WINDOW = "Image window";
shape_predictor sp_; // give shape_predictor_68_face_landmarks.dat in command line
typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;

class ImageConverter
{
  ros::NodeHandle nh_;
  //image_transport::ImageTransport it_;
  //image_transport::Subscriber image_sub_;
  //image_transport::Publisher image_pub_;
  message_filters::Subscriber<Image> *rgb_sub_;
  message_filters::Subscriber<Image> *depth_sub_;
  message_filters::Synchronizer<MySyncPolicy>* my_sync_;
  ros::Publisher expression_pub_;
  frontal_face_detector detector_;
  image_window win_;
  std::vector<full_object_detection> templates_; // basis set
  //int countTime;
  
public:
  ImageConverter()
    //: it_(nh_), templates_(REF_SIZE)
  {
    detector_ = get_frontal_face_detector();
    //countTime = 0;
    // Subscribe to input video feed and publish output video feed
    int queueSize = 10;
    rgb_sub_ = new image_sub_type(nh_, "/camera/rgb/image_raw", queueSize);
    depth_sub_ = new image_sub_type(nh_, "/camera/depth/image_raw", queueSize);
    my_sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(queueSize),  *rgb_sub_, *depth_sub_);
    my_sync_->registerCallback(boost::bind(&ImageConverter::imageCb, this, _1, _2));

    /*image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);*/
    expression_pub_ = nh_.advertise<std_msgs::Int8>("/face_evaluator/expression", 1);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
    delete my_sync_;
    delete rgb_sub_;
    delete depth_sub_;
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
        cv::Mat matImg = cv::imread(ent->d_name, 0); // grayscale
        cv_image<unsigned char> cimg(matImg);
        assign_image(face_chips[index], cimg);
        pyramid_up(face_chips[index]);
        std::vector<rectangle> faces = detector_(face_chips[index]);
        if(faces.size() == 0) {
          ROS_ERROR("no faces detected for %s!!!\n", ent->d_name);
          continue;
        }
        else {
          ROS_INFO("Number of faces detected: %d for %s\n", faces.size(), ent->d_name);
        }
        
        // shape predictor generates 68 facial landmarks (iBUG 300-W scheme)
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

  int computeExpression(full_object_detection shape, cv::Mat depthImg)
  {
    int difference_ = abs(shape.part(49).y() - shape.part(55).y());
    int midLipDist_ = abs(shape.part(62).y() - shape.part(66).y());
    //ROS_INFO("mid lip difference: %d", midLipDist_);
    unsigned short val = depthImg.at<unsigned short>(shape.part(30).y(), shape.part(30).x());
    int depth_ = static_cast<int>(val); // depth in mm
    if(depth_ == 0)
    {
      depth_ = 440; // closest range
    }
    ROS_INFO("depth: %d\n", depth_);
    if(midLipDist_ > 20)
    {
      return 2;
    }
    else if(midLipDist_ > 10 && midLipDist_ <= 20)
    {
      return 1;
    }
    return 0;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::ImageConstPtr& depth_msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr depth_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    try
    {
      cv_image<bgr_pixel> cimg(cv_ptr->image); // dlib wrapper for cv::Mat object
      cv_image<unsigned short> dimg(depth_ptr->image);
      //pyramid_up(cimg); // upsample
      std::vector<rectangle> faces = detector_(cimg); // detected bounding boxes
      std::vector<full_object_detection> shapes; // pose of each face
      for (unsigned long j = 0; j < faces.size(); ++j)
      {
        // shape predictor generates 68 facial landmarks (iBUG 300-W scheme)
        full_object_detection shape = sp_(cimg, faces[j]);
        shapes.push_back(shape);
//        dlib::draw_solid_circle(cimg, shape.part(48), 5, dlib::bgr_pixel(255,0,0)); //outer lip: left corner
//        dlib::draw_solid_circle(cimg, shape.part(49), 5, dlib::bgr_pixel(255,0,0)); //outer lip: upper left
//        dlib::draw_solid_circle(cimg, shape.part(50), 5, dlib::bgr_pixel(255,0,0)); //outer lip: left upper middle
//        dlib::draw_solid_circle(cimg, shape.part(51), 5, dlib::bgr_pixel(255,0,0)); //outer lip: upper middle
//        dlib::draw_solid_circle(cimg, shape.part(52), 5, dlib::bgr_pixel(255,0,0)); //outer lip: right upper middle
//        dlib::draw_solid_circle(cimg, shape.part(53), 5, dlib::bgr_pixel(255,0,0)); //outer lip: upper right
//        dlib::draw_solid_circle(cimg, shape.part(54), 5, dlib::bgr_pixel(255,0,0)); //outer lip: right corner
//        dlib::draw_solid_circle(cimg, shape.part(55), 5, dlib::bgr_pixel(255,0,0)); //outer lip: lower right
//        dlib::draw_solid_circle(cimg, shape.part(56), 5, dlib::bgr_pixel(255,0,0)); //outer lip: right lower middle
//        dlib::draw_solid_circle(cimg, shape.part(57), 5, dlib::bgr_pixel(255,0,0)); //outer lip: lower middle
//        dlib::draw_solid_circle(cimg, shape.part(58), 5, dlib::bgr_pixel(255,0,0)); //outer lip: left lower middle
//        dlib::draw_solid_circle(cimg, shape.part(59), 5, dlib::bgr_pixel(255,0,0)); //outer lip: lower left
//        dlib::draw_solid_circle(cimg, shape.part(60), 5, dlib::bgr_pixel(0,0,255)); //inner lip: left corner
//        dlib::draw_solid_circle(cimg, shape.part(62), 5, dlib::bgr_pixel(0,0,255)); //inner lip: upper middle
//        dlib::draw_solid_circle(cimg, shape.part(66), 5, dlib::bgr_pixel(0,0,255)); //inner lip: lower middle
//        dlib::draw_solid_circle(cimg, shape.part(30), 5, dlib::bgr_pixel(0,0,255)); //nose tip
        std_msgs::Int8 expression_;
        expression_.data = computeExpression(shape, depth_ptr->image);
        ROS_INFO("expression %d\n", expression_.data);
        expression_pub_.publish(expression_);
        //templates_.push_back(shape);
        //save_png(cimg, to_string(countTime)+"_org.png");
        // draw overlay on cimg
        /*for (unsigned long i = 1; i < shape.num_parts(); ++i)
        {
          dlib::draw_line(cimg, shape.part(i-1), shape.part(i), dlib::bgr_pixel(0,255,0));
        }*/
        //std::ostream fout(to_string(countTime) + ".dat", std::ios::binary);
        //save_png(cimg, to_string(countTime)+".png");
        //countTime++;
        //dlib::serialize(shape, fout);
        //fout.close();
      }
      //cv_ptr->image = toMat(cimg); // converts dlib image back to cv::Mat
      win_.clear_overlay();
      win_.set_image(cimg);
      //win_.set_image(dimg);
      // convert into contour overlays for visualizations
      win_.add_overlay(render_face_detections(shapes));
    }
    catch (exception& e)
    {
      ROS_ERROR("\nexception thrown! %s\n", e.what());
      return;
    }

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, depth_ptr->image);
    //cv::waitKey(1);
    
    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    ROS_ERROR("\nUsage: rosrun face_evaluator face_evaluator_node shape_predictor_68_face_landmarks.dat\n");
    return 0;
  }
  deserialize(argv[1]) >> sp_;
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  //ic.initTemplates(argv[2]);
  ros::spin();
  return 0;
}
