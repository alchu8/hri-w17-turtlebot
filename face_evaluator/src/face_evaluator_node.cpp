/******************************************************************************
 * File: face_evaluator_node.cpp
 * Author: Albert Chu
 * Email: alchu8@gmail.com
 * Description: This node detects a face and evaluates an expression score 
 * based on the degree of difference between the current frame and a reference 
 * frame with neutral expression. This node subscribes to RGB and depth from 
 * the camera node; it also subscribes to interaction information from 
 * sound_play node. Finally, it publishes an expression score and saves them
 * in a file. 
 *****************************************************************************/
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
#include <fstream>
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
  message_filters::Subscriber<Image> *rgb_sub_;
  message_filters::Subscriber<Image> *depth_sub_;
  message_filters::Synchronizer<MySyncPolicy>* my_sync_;
  ros::Publisher expression_pub_;
  ros::Subscriber interaction_sub_;
  frontal_face_detector detector_;
  image_window win_;
  std::vector<full_object_detection> templates_; // basis set
  int interaction_; // flag for new user, 1 or 0
  full_object_detection *neutralShape_; // reference shape for each interaction user
  ofstream outFile;
  int interactionId; // counter for out file name
  //int countTime;
  
public:
  ImageConverter()
  {
    detector_ = get_frontal_face_detector();
    //countTime = 0;
    // Subscribe to input video feed and sync RGB and depth
    int queueSize = 10;
    rgb_sub_ = new image_sub_type(nh_, "/camera/rgb/image_raw", queueSize);
    depth_sub_ = new image_sub_type(nh_, "/camera/depth/image_raw", queueSize);
    my_sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(queueSize), *rgb_sub_, *depth_sub_);
    my_sync_->registerCallback(boost::bind(&ImageConverter::imageCb, this, _1, _2));
    // subscribe to detect new user and publish expression score
    expression_pub_ = nh_.advertise<std_msgs::Int8>("/face_evaluator/expression", 1);
    //interaction_sub_ = nh_.subscribe("/new_person", 10, &ImageConverter::interactionCb, this);
    interaction_ = 1; // on startup, user is new
    neutralShape_ = NULL;
    interactionId = 0;

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
    delete my_sync_;
    delete rgb_sub_;
    delete depth_sub_;
  }

  /****************************************************************************
   * Function: initTemplates (NOT USED)
   * Parameters: templatesPath - path to the templates folder containing 
   * references images for basis expressions.
   * Description: Extracts a shape object for each reference face and store 
   * them in templates_ vector. The index of each shape element in the vector 
   * will correspond to the expression score for each respective shape. Thus,
   * this sets up the basis set for which to compare all other faces against.
   * Returns: Nothing.
   ***************************************************************************/
  void initTemplates(char* templatesPath)
  {
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (templatesPath)) != NULL) 
    {
      // an array of original reference images in dlib image format
      dlib::array<array2d<unsigned char>> face_chips(REF_SIZE);
      // print all the files within directory
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
        assign_image(face_chips[index], cimg); // copies image: EXPENSIVE
        pyramid_up(face_chips[index]); // up sample
        std::vector<rectangle> faces = detector_(face_chips[index]);
        if(faces.size() == 0) 
        {
          ROS_ERROR("no faces detected for %s!!!\n", ent->d_name);
          continue;
        }
        else 
        {
          ROS_INFO("Number of faces detected: %d for %s\n", faces.size(), ent->d_name);
        }
        
        // shape predictor generates 68 facial landmarks (iBUG 300-W scheme)
        full_object_detection shape = sp_(face_chips[index], faces[0]);
        templates_[index] = shape;
      }
      closedir (dir);
      // draw each reference image with overlays
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

  /****************************************************************************
   * Function: computeExpression
   * Parameters: shape - 68-landmark shape of the closest face
   * depth_ - depth from camera to the nose tip of the closest face in mm
   * Description: Computes user facial features such as distance between brows
   * and computes thresholds from neutral shape, then classifies the feature
   * divergence into 6 levels of intensity, from disgust to laugh. 
   * Returns: An int score indicating expression, between [0, 5].
   ***************************************************************************/
  int computeExpression(full_object_detection shape, float depth_)
  {
    // user's features
    int midLipDist_ = abs(shape.part(62).y() - shape.part(66).y());
    int innerBrowDist_ = abs(shape.part(21).x() - shape.part(22).x());
    int lipCornerDist_ = abs(shape.part(48).x() - shape.part(54).x());
    depth_ /= 1000; // depth in m
    //ROS_INFO("depth: %.3f\n", depth_);
    // compute thresholds from neutral shape's features
    float midLipDist_thres = 10/depth_;
    float innerBrowDist_thres = abs(neutralShape_->part(21).x() - neutralShape_->part(22).x())/(2*depth_);
    float lipCornerDist_thres = abs(neutralShape_->part(48).x() - neutralShape_->part(54).x())/(2*depth_);
    if(innerBrowDist_ <= innerBrowDist_thres && midLipDist_ > midLipDist_thres)
    {
      return 0; // shock/disgust
    }
    else if(innerBrowDist_ <= innerBrowDist_thres || lipCornerDist_ < lipCornerDist_thres/2)
    {
      return 1; // frown/pouted lips
    }
    else if(midLipDist_ > midLipDist_thres)
    {
      return 5; // laugh
    }
    else if(midLipDist_ > midLipDist_thres/2 && midLipDist_ <= midLipDist_thres)
    {
      return 4; // teeth smile
    }
    else if(lipCornerDist_ > lipCornerDist_thres)
    {
      return 3; // lip smile
    }
    return 2; // neutral
  }

  /****************************************************************************
   * Function: imageCb
   * Parameters: msg - RGB ROS image message
   *             depth_msg - depth ROS image message
   * Description: Synchronized callback function for camera node's RGB and 
   * depth subscribers, since these topics publish at different rates with 
   * different timestamps. All the perception work is done here. Detects a 
   * single closest face, extracts landmark shape from it, and calls 
   * computeExpression() and publishes an expression score during an 
   * interaction. If a new user is detected, indicated by setting the 
   * interaction_ flag to 1, then get a new neutral shape from a detected 
   * face for computing neutral divergence. 
   * Returns: Nothing, but shows a window of the evaluated face with feature
   * overlays. 
   ***************************************************************************/
  void imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::ImageConstPtr& depth_msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr depth_ptr;
    try
    {
      // convert from ROS image messages to OpenCV images of type cv::Mat
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
      //cv_image<unsigned short> dimg(depth_ptr->image);
      std::vector<rectangle> faces = detector_(cimg); // detected bounding boxes
      std::vector<full_object_detection> shapes; // vector of shape objects for each face
      int min_depth_ = INT_MAX; // depth of closest face so far
      for (unsigned long j = 0; j < faces.size(); ++j) // find closest detected face
      {
        // shape predictor generates 68 facial landmarks (iBUG 300-W scheme) for face j
        full_object_detection shape = sp_(cimg, faces[j]);
        unsigned short val = (depth_ptr->image).at<unsigned short>(shape.part(30).y(), shape.part(30).x());
        if(val == 0)
        {
          val = 440; // closest range
        }
        int depth_ = static_cast<int>(val); // depth of shape in mm
        if(depth_ < min_depth_)
        {
          min_depth_ = depth_;
          shapes.clear();
          shapes.push_back(shape); // only consider closest face
        }
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
//        dlib::draw_solid_circle(cimg, shape.part(21), 5, dlib::bgr_pixel(0,0,255)); //left brow: inner
//        dlib::draw_solid_circle(cimg, shape.part(22), 5, dlib::bgr_pixel(0,0,255)); //right brow: inner
      }
      std_msgs::Int8 expression_;
      expression_.data = -1; // default value for publishing
      if(interaction_ == 1) // save new user's neutral face
      {
        ROS_INFO("\nPlease maintain a neutral expression and look at the screen.\n");
        if(!shapes.empty())
        {
          if(neutralShape_ != NULL)
          {
            delete neutralShape_;
            neutralShape_ = NULL;
            outFile.close();
          }
          neutralShape_ = new full_object_detection();
          *neutralShape_ = shapes.at(0);
          interaction_ = 0;
          outFile.open("/home/turtlebot/interactions/interaction"+to_string(interactionId++)+".txt");
        }
      }
      else if(!shapes.empty() && neutralShape_ != NULL) // continually compute score
      {
        expression_.data = computeExpression(shapes[0], min_depth_);
        outFile << to_string(expression_.data) + " ";
        //ROS_INFO("expression %d\n", expression_.data);
      }
      expression_pub_.publish(expression_); // publish a score no matter what
     
      // uncomment below to save images with overlays as png files
      /*save_png(cimg, to_string(countTime)+"_orig.png");
      // draw overlay on cimg
      for (unsigned long i = 1; i < shape.num_parts(); ++i)
      {
        dlib::draw_line(cimg, shape.part(i-1), shape.part(i), dlib::bgr_pixel(0,255,0));
      }
      save_png(cimg, to_string(countTime)+".png");
      countTime++;*/
      //cv_ptr->image = toMat(cimg); // converts dlib image back to cv::Mat
      win_.clear_overlay();
      win_.set_image(cimg);
      //win_.set_image(dimg);
      // convert into contour overlays for visualization
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
  }

  /****************************************************************************
   * Function: interactionCb
   * Parameters: newUserFlag - Int8 ROS message from sound_play node, 0 or 1 
   * indicating if a new user is detected
   * Description: Callback function for interaction subscriber. As soon as a 1
   * is detected, preserve it in interaction_ flag, and we'll set it back to 0
   * ourselves once a neutral shape has been obtained. 
   * Returns: Nothing.
   ***************************************************************************/
  void interactionCb(const std_msgs::Int8& newUserFlag)
  {
    if(newUserFlag.data == 1) // new user detected
    {
      interaction_ = newUserFlag.data; // preserve it so that 0 doesn't overwrite
    }
  }
};

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    ROS_ERROR("\nUsage: rosrun face_evaluator face_evaluator_node /path/shape_predictor_68_face_landmarks.dat\n");
    return 0;
  }
  deserialize(argv[1]) >> sp_; // initialize shape predictor
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  //ic.initTemplates(argv[2]);
  ros::spin();
  return 0;
}
