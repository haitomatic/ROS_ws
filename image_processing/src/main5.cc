// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>
#include <iterator>
#include <set>
#include <cstdio>
#include <iostream>

// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <my_message_and_service/object.h>

#if CV_MAJOR_VERSION < 3

// global
namespace enc = sensor_msgs::image_encodings;
cv::Ptr<cv::linemod::Detector> detector;
my_message_and_service::object object_message;
ros::Publisher pub;
// <-- parameter
int matching_threshold = 80;
bool show_match_result = 1;
int big_num = 5;
// parameter -->
std::vector<std::string> picked;
typedef std::vector<cv::linemod::Match> MatchesVect;
MatchesVect matches2, matches1, object_match_list;
std::vector<std::string> object_list{"house", "holder_3d", "laptop_male_jack", "spring", "keys", "hexa_tool",
                             "cashew_nut", "flexitube", "clock_cap", "cable_shoe"};

// vector finding helper
bool myFindif(std::string object_name, MatchesVect& matches_input, MatchesVect::iterator& it)
{
  auto pred = [object_name](const cv::linemod::Match& item)
  {
    return item.class_id == object_name;
  };
  if((it = std::find_if(matches_input.begin(), matches_input.end(), pred)) != matches_input.end())
    return 1;
  else
    return 0;
}

// calculate cluster location
void clusterLoc(MatchesVect& matches_input, std::vector<std::string>& exclusive_list, int& num, short& x, short& y)
{
  // calculate new centroid based on updated exclusive_list
  for(MatchesVect::iterator it = matches_input.begin(); it != matches_input.end(); ++it)
  {
   //if(std::find(exclusive_list.begin(), exclusive_list.end(), it->class_id) != exclusive_list.end())
     //continue;
   x += it->x;
   y += it->y;
  }
  x /= matches_input.size();
  y /= matches_input.size();

  // check exclusive_list size to return
  //if(num == 2)
    //return;

  // calculate the most isolated contributor
  /*
  int max_dis = 0;
  std::string max_class_id;
  for(MatchesVect::iterator it = matches_input.begin(); it != matches_input.end(); ++it)
  {
   if(std::find(exclusive_list.begin(), exclusive_list.end(), it->class_id) != exclusive_list.end())
     continue;
   int dis = (int)std::sqrt(std::pow(it->x - x, 2) + std::pow(it->y - y, 2));
   if(dis > max_dis)
   {
     max_dis = dis;
     max_class_id = it->class_id;
   }

  }

  // add it to exclusive_list;
  exclusive_list.push_back(max_class_id);
  num++;
  // enter new loop
  clusterLoc(matches_input, exclusive_list, num, x, y);
  */
}

// draw result
void drawResponse(int& x, int& y, const std::vector<cv::linemod::Template>& templates, std::string class_id, cv::Mat& dst)
{
  static const cv::Scalar COLORS[5] = { CV_RGB(0, 0, 255),
                                        CV_RGB(0, 255, 0),
                                        CV_RGB(255, 255, 0),
                                        CV_RGB(255, 140, 0),
                                        CV_RGB(255, 0, 0) };
  cv::Scalar color = COLORS[0];
  cv::Point core; core.x = 0; core.y = 0;
  for(int i = 0; i < (int)templates[0].features.size(); ++i)
  {
    cv::linemod::Feature ft = templates[0].features[i];
    cv::Point pt(ft.x + x, ft.y + y);
    core.x += pt.x;
    core.y += pt.y;
    cv::circle(dst, pt, 2, color);
  }
  core.x = (int)(core.x/templates[0].features.size());
  core.y = (int)(core.y/templates[0].features.size());
  if(class_id == "house")
  {
    core.x -= 30;
    core.y -= 20;
  }
  cv::circle(dst, core, 10, COLORS[3]);
  cv::putText(dst, class_id,  core, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, COLORS[3]);
  for(size_t s = 0 ; s < picked.size() ; s++)
  {
    cv::Point pt(20, 20 +s*15);
    cv::putText(dst, picked.at(s),  pt, cv::FONT_HERSHEY_PLAIN, 1.2, COLORS[2],1);
  }
  x = core.x;
  y = core.y;
}

// ----------callback functions--------------
// picked callback function
void pickedCallback(const std_msgs::StringConstPtr& picked_object_name)
{
  std::string name = picked_object_name->data;
  if(std::find(object_list.begin(), object_list.end(), name) == object_list.end() || picked.size() >= object_list.size())
    return;
  picked.push_back(name);
}

// image callback function
void imageCallback(const sensor_msgs::ImageConstPtr& ori_image)
{
  // get image from camera node and convert it to opencv Mat type
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(ori_image, enc::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    // if there is an error, display it
    ROS_ERROR("image_processing::main.cc::cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat color = cv_ptr->image;
  std::vector<cv::Mat> sources;
  sources.push_back(color);
  cv::Mat display = color.clone();

  // matching step
  MatchesVect matches;
  std::vector<std::string> class_ids;
  std::vector<cv::Mat> quantized_image;
  detector->match(sources,(float)matching_threshold, matches, class_ids, quantized_image);

  if(matches1.empty() || matches2.empty())
  {
    matches2 = matches;
    matches1 = matches;
  }
  else
  {
    matches2 = matches1;
    matches1 = matches;
  }

  // updating object_list
  if(show_match_result){
    printf("------------------------------------------------------------%ld\n", picked.size());
    if(picked.size())
      for(std::string s : picked)
        std::cout<<s<<std::endl;
  }
  if(object_match_list.empty())
    for(std::vector<std::string>::iterator it = object_list.begin(); it != object_list.end(); ++it)
      object_match_list.push_back(cv::linemod::Match(0, 0, 0, *it, 0));
  for(MatchesVect::iterator it = object_match_list.begin(); it != object_match_list.end(); ++it)
  {

    if(std::find(picked.begin(), picked.end(), it->class_id) != picked.end())
    {
      it->similarity = 0;
      continue;
    }
    MatchesVect::iterator place, place1, place2;
    bool ok  = myFindif(it->class_id, matches, place);
    bool ok1 = myFindif(it->class_id, matches1, place1);
    bool ok2 = myFindif(it->class_id, matches2, place2);
    if(ok && ok1 || ok && ok2 || ok1 && ok2)
    {
      if(!ok)
      {
        if(abs(place1->x - place2->x) > 10 || abs(place1->y - place2->y) > 10)
          goto exception;
        it->x = (place1->x + place2->x)/2;
        it->y = (place1->y + place2->y)/2;
        it->similarity = (place1->similarity + place2->similarity)/2;
        it->template_id = place1->template_id;
      }
      else if(!ok1)
      {
        if(abs(place->x - place2->x) > 10 || abs(place->y - place2->y) > 10)
          goto exception;
        it->x = (place->x + place2->x)/2;
        it->y = (place->y + place2->y)/2;
        it->similarity = (place->similarity + place2->similarity)/2;
        it->template_id = place2->template_id;
      }
      else if(!ok2)
      {
        if(abs(place->x - place1->x) > 10 || abs(place->y - place1->y) > 10)
          goto exception;
        it->x = (place->x + place1->x)/2;
        it->y = (place->y + place1->y)/2;
        it->similarity = (place->similarity + place1->similarity)/2;
        it->template_id = place->template_id;
      }
      else
      {
        if(abs(place->x - place1->x) > 10 || abs(place->y - place1->y) > 10)
          goto exception;
        it->x = (place->x + place1->x + place2->x)/3;
        it->y = (place->y + place1->y + place2->y)/3;
        it->similarity = (place->similarity + place1->similarity + place2->similarity)/3;
        it->template_id = place->template_id;
      }
      const std::vector<cv::linemod::Template>& templates = detector->getTemplates(it->class_id, it->template_id);
      drawResponse(it->x, it->y, templates, it->class_id, display);
    }
    else
    {
      exception:
        it->similarity = 1;
    }
    if(show_match_result)
      printf("Similarity: %5.1f%%; x: %3d; y: %3d; class: %s; template: %3d\n",
             it->similarity, it->x, it->y, it->class_id.c_str(), it->template_id);
  }
  cv::imshow("color", display);
  cv::waitKey(1);

  // choosing one object to pick
  if(picked.size() == object_list.size() || matches.size() == 0)
  {
    // no object left
    // ... but double check
    if(matches.size() != 0)
    {
      object_message.object_name = matches.at(0).class_id;
      object_message.x = matches.at(0).x;
      object_message.y = matches.at(0).y;
    }
    else
    {
      object_message.object_name = "end";
      object_message.x = 0;
      object_message.y = 0;
      pub.publish(object_message);
      ros::shutdown();
    }
  }
  else
  {
    // still object to pick
    bool still_big;
    for(std::vector<std::string>::iterator it = object_list.begin(); it != object_list.begin() + big_num; ++it)
      if(still_big = (std::find(picked.begin(), picked.end(), *it) == picked.end()))
        break;
    if(still_big)
    {
      // still at least one of big left
      MatchesVect::iterator bestPtr = std::max_element(object_match_list.begin(), object_match_list.begin() + big_num,
                                         [](const cv::linemod::Match& e1, const cv::linemod::Match& e2){ return e1.similarity < e2.similarity;});
      if(bestPtr->similarity == 1)
      {
        std::vector<std::string> exclusive_list(picked);
        int num = 0;
        object_message.object_name = "push";
        clusterLoc(object_match_list, exclusive_list, num, object_message.x, object_message.y);
        int x = object_message.x, y = object_message.y;
        std::cout<< x << "................." << y <<".........................."<< std::endl;
      }
      else
      {
        object_message.object_name = bestPtr->class_id;
        object_message.x = bestPtr->x;
        object_message.y = bestPtr->y;
      }
    }
    else
    {
      // no big left
      MatchesVect::iterator firstEightiesPtr = object_match_list.end(), firstNinetiesPtr;
      for(firstNinetiesPtr = object_match_list.begin() + big_num; firstNinetiesPtr != object_match_list.end(); ++firstNinetiesPtr)
      {
        if(firstNinetiesPtr->similarity >= 80 && firstEightiesPtr == object_match_list.end())
          firstEightiesPtr = firstNinetiesPtr;
        if(firstNinetiesPtr->similarity >= 90)
          break;
      }
      if(firstNinetiesPtr != object_match_list.end())
      {
        // still nineties case
        object_message.object_name = firstNinetiesPtr->class_id;
        object_message.x = firstNinetiesPtr->x;
        object_message.y = firstNinetiesPtr->y;
      }
      else if(firstEightiesPtr != object_match_list.end())
      {
        // only eighties left
        object_message.object_name = firstEightiesPtr->class_id;
        object_message.x = firstEightiesPtr->x;
        object_message.y = firstEightiesPtr->y;
      }
      else
      {;}
    }
  }
  std::string out = object_message.object_name.c_str();
  std::cout<<"***** "<<out<<" *********"<< std::endl;
  pub.publish(object_message);
}

// function to store detector and template in single XML/YAML file
static cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename)
{
  cv::Ptr<cv::linemod::Detector> detector = new cv::linemod::Detector;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  detector->read(fs.root());

  cv::FileNode fn = fs["classes"];
  for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
    detector->readClass(*i);

  return detector;
}

// MAIN LOOP
int main(int argc, char** argv)
{
 ROS_INFO("node image_processing is starting.");
 ros::init(argc, argv, "image_processing");
 std::string filename;

 if (argc == 1)
 {
   filename = "linemod_templates.yml";
   detector = cv::linemod::getDefaultLINE();
 }
 else
 {
   filename = argv[1];
   detector = readLinemod(filename);
   std::vector<std::string> ids = detector->classIds();
   int num_classes = detector->numClasses();
   printf("Loaded %s with %d classes and %d templates\n",
          argv[1], num_classes, detector->numTemplates());
   if (!ids.empty())
   {
     printf("Class ids:\n");
     std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
   }
 }

 cv::namedWindow("color");
 // register callback
 ros::NodeHandle nh;
 image_transport::ImageTransport it(nh);
 image_transport::Subscriber img_sub = it.subscribe("image_raw", 1, imageCallback);
 ros::Subscriber nor_sub = nh.subscribe("object_just_picked", 1, pickedCallback);
 pub = nh.advertise<my_message_and_service::object>("object_to_pick", 1);
 ros::spin();
 cv::destroyAllWindows();
 return 0;
}

#endif
