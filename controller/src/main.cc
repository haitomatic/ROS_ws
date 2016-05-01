// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <my_message_and_service/object.h>
#include <my_message_and_service/command.h>
#include <my_message_and_service/state.h>

// global
std::map<std::string, int> object_height
{
  {"house", 45},
  {"holder_3d", 25},
  {"laptop_male_jack", 2},
  {"spring", 0},
  {"keys", -5},
  {"hexa_tool", -11},
  {"cashew_nut", -10},
  {"flexitube", -10},
  {"clock_cap", -12},
  {"cable_shoe", -13},
  {"push", 10},
  {"end", 0}
};

struct object_struct
{
  std::string object_name;
  int x;
  int y;
  int z;
} my_object, tmp_object;

struct state_struct
{
  int x;
  int y;
  int z;
  bool suction0;
  bool suction1;
};
state_struct my_state{0,0,0,0,0};
// params
const int fx = 620; // focal length in x direction
const int fy = 620; // focal length in y direction
const int uc = 320; // principal point x coordinate
const int vc = 240; // principal point y coordinate
const int z = 280; // vertical distance to object plane
const double theta = 194; // tilt angle of camera
const int w = 180; // x distance between base and camera
const int l = 230; // y distance between base and camera
const int home_x = 218; // home x coordinate
const int home_y = 0; // home y coordinate
const int home_z = 70; // home z coordinate or home_z + 28 for house
const int house_surplus = 35;
const int bin_x = 241; // bin x coordinate
const int bin_y = -150; // bin y coordinate
const int bin_z = 55; // bin z coordinate or bin_z + 28 for house
const int push_gap = 10;
#define PI 3.14159265

// callback function
void objectCallback(const my_message_and_service::objectConstPtr& object)
{
  my_object.object_name = object->object_name;
  my_object.x = -(object->x - uc)*z/fx + w;
  my_object.y = -(double)((object->y - vc)*z/fy)*cos(theta*PI/180) + (double)z*sin(theta*PI/180) + l;
  my_object.z = object_height[my_object.object_name];
}

void stateCallback(const my_message_and_service::stateConstPtr& state)
{
  my_state.x = state->x;
  my_state.y = state->y;
  my_state.z = state->z;
  my_state.suction0 = my_state.suction1;
  my_state.suction1 = state->suction;

}

void sendCommand(ros::Publisher& pub, int x, int y, int z, bool suction, bool move, int speed, int wait = 0)
{
  my_message_and_service::command my_command;
  my_command.x = x;
  my_command.y = y;
  my_command.z = z;
  my_command.suction = suction;
  my_command.move = move;
  my_command.speed = speed;
  my_command.wait = wait;
  pub.publish(my_command);
}
int main(int argc, char* argv[])
{
  // initialization
  ROS_INFO("node controller is starting.");
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;
  ros::Subscriber sub_object = nh.subscribe("object_to_pick", 1, objectCallback);
  ros::Subscriber sub_state = nh.subscribe("robot_state", 1, stateCallback);
  ros::Publisher pub_picked = nh.advertise<std_msgs::String>("object_just_picked", 1);
  ros::Publisher pub_command = nh.advertise<my_message_and_service::command>("robot_command", 1,1);

  // home
  sendCommand(pub_command, home_x, home_y, home_z, 0, 0, 30,5);
  // variables
  bool done = 1;
  int count = 0;
  std::string just_done;
  while(ros::ok())
  {
    if(done && !my_object.object_name.empty())
    {
      tmp_object = my_object;
      //std::cout<<tmp_object.object_name<<"----------------------------------"<<std::endl;

      if(tmp_object.object_name == "end")
        ros::shutdown();
      else if(tmp_object.object_name == "push" && just_done != "push")
      {
        std_msgs::String msg;
        msg.data = tmp_object.object_name;
        pub_picked.publish(msg);
        sendCommand(pub_command, tmp_object.x, tmp_object.y, home_z, 0, 0, 30);
        usleep(10*1000);
        sendCommand(pub_command, tmp_object.x, tmp_object.y, tmp_object.z, 0, 0, 10);
        usleep(10*1000);
        sendCommand(pub_command, tmp_object.x, tmp_object.y + push_gap, tmp_object.z, 0, 0, 10);
        usleep(10*1000);
        sendCommand(pub_command, tmp_object.x - push_gap, tmp_object.y, tmp_object.z, 0, 0, 10);
        usleep(10*1000);
        sendCommand(pub_command, tmp_object.x, tmp_object.y - push_gap, tmp_object.z, 0, 0, 10);
        usleep(10*1000);
        sendCommand(pub_command, tmp_object.x + push_gap, tmp_object.y, tmp_object.z, 0, 0, 10);
        usleep(10*1000);
        sendCommand(pub_command, tmp_object.x, tmp_object.y + push_gap, tmp_object.z, 0, 0, 10);
        usleep(10*1000);
        sendCommand(pub_command, tmp_object.x, tmp_object.y + push_gap, home_z, 0, 0, 10);
        usleep(10*1000);
        sendCommand(pub_command, home_x, home_y, home_z, 0, 0, 30, 7);
        std::cout << " ----------------------- send 9 commands " << std::endl;
        count = 7;
        done = 0;
      }
      else if(tmp_object.object_name != "push")
      {
        std_msgs::String msg;
        msg.data = tmp_object.object_name;
        pub_picked.publish(msg);
        //send first 2 commands
        sendCommand(pub_command, tmp_object.x, tmp_object.y, tmp_object.object_name == "house"?home_z+house_surplus:home_z, 0, 0, 30);
        sendCommand(pub_command, tmp_object.x, tmp_object.y, tmp_object.z, 1, 0, 10,
                    (tmp_object.object_name == "clock_cap" || tmp_object.object_name == "cable_shoe" || tmp_object.object_name == "hexa_tool")?12:7);
        count += 2;
        done = 0;
      }
    }

    if(!done && my_state.suction0 == 0 && my_state.suction1 == 1 && count == 2)
    {
      // send next 5 commands
      sendCommand(pub_command, tmp_object.x, tmp_object.y, tmp_object.object_name == "house"?home_z+house_surplus:home_z, 1, 0, 5);
      usleep(10*1000);
      sendCommand(pub_command, bin_x, bin_y, tmp_object.object_name == "house"?home_z+house_surplus:home_z, 0, 0, 30);
      usleep(10*1000);
      sendCommand(pub_command, bin_x, bin_y, tmp_object.object_name == "house"?bin_z+house_surplus:bin_z, 0, 0, 10);
      usleep(10*1000);
      sendCommand(pub_command, bin_x, bin_y, tmp_object.object_name == "house"?home_z+house_surplus:home_z, 0, 0, 10);
      usleep(10*1000);
      sendCommand(pub_command, home_x, home_y, home_z, 0, 0, 30, 7);
      count += 5;
      std::cout << " -----------send 5 commands " << std::endl;
    }

    if(!done && abs(my_state.x-home_x) <=1 && abs(my_state.y-home_y) <=1  && abs(my_state.z-home_z) <=1 && my_state.suction1 == 0 && count == 7)
    {
      count = 0;
      done = 1;
      just_done = tmp_object.object_name;
    }
    ros::spinOnce();
  }
  // home
  sendCommand(pub_command, home_x, home_y, home_z, 0, 0, 30);

  return 0;
}
