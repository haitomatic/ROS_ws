/*
A5  CC 6B 71 43  00 00 00 00  40 50 4F 40  00 00 00 00  00 00 00 00  CE 8A FF 41  61 58 30 42  00 00 00 00  00 00 00 00  00 00 00 00  5A

    [X coor]     [Y coor]     [Z coor]     [rot value]  [base ang]   [RA ang]     [FA ang]     [servo ang]  [pump state] [grip state]

*/

#include <ros/ros.h>
#include <my_message_and_service/command.h>
#include <my_message_and_service/state.h>
#include <sstream>
#include <deque>
#include <sys/signal.h>
//c lib
#include <sys/time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>
#include <asm/ioctls.h>
//#include <asm/termios.h>
// custom lib
#include <serial/serial.h>
#define PI 3.14159265
#define hexA5 165
#define hex5A 90

typedef std::deque<my_message_and_service::command> command_list;
command_list my_command_list;
my_message_and_service::state state_msg;
const int x_offset = 15; // end_effector x offset
const int y_offset = 5;  // end_effector y offset
const int home_z = 80;   // home z
const int radi1 = 189;   // inner limit radi
const int radi2 = 262;   // offset radi
const int radi3 = 307;   // outer limit radi


int changeBaudrate(const char* port, int baudrate)
{
  int fd = open(port, O_RDONLY);
  struct termios2 tio;
  ioctl(fd, TCGETS2, &tio);
  tio.c_cflag &= ~CBAUD;
  tio.c_cflag |= BOTHER;
  tio.c_ispeed = baudrate;
  tio.c_ospeed = baudrate;
  int r = ioctl(fd, TCSETS2, &tio);
  close(fd);
  return r;
}


std::string tohex(const std::string& s, bool upper=false)
{
  std::ostringstream ret;

  unsigned int c;
  for (std::string::size_type i = 0; i < s.length(); ++i)
    {
      c = (unsigned int)(unsigned char)s[i];
      ret << std::hex << std::setfill('0') <<
        std::setw(2) << (upper ? std::uppercase : std::nouppercase) << c;
    }
  return ret.str();
}

int hexStringToInt(std::string& hex_string)
{
  unsigned int tmp_i;
  float tmp_f;
  std::stringstream ss;
  // x
  ss << std::hex << hex_string;
  ss >> tmp_i;
  tmp_f = reinterpret_cast<float&>(tmp_i);
  return (int)(tmp_f + 0.5);
}

std::string intToHexString(int input)
{
  std::stringstream ss;
  ss << std::hex << (float)input;
  return ss.str();
}
// get state from serial
bool getState(serial::Serial& my_serial, my_message_and_service::state& my_state)
{
  int byte_no = 0;
  if(byte_no=my_serial.available())
  {
    std::string hex_string = tohex(my_serial.read(byte_no), true);
    //std::cout<< hex_string << std::endl;
    if(byte_no >= 42)
    {
      // filter out a full state
      std::size_t found = hex_string.find("5AA5");
      if(hex_string.size() - found <  87)
        return 0;
      hex_string = hex_string.substr(found+2, 84);
      //std::cout<< hex_string << std::endl;
      // take info from state
      std::string x_string = hex_string.substr(8,2) + hex_string.substr(6,2) + hex_string.substr(4,2) + hex_string.substr(2,2);
      std::string y_string = hex_string.substr(16,2) + hex_string.substr(14,2) + hex_string.substr(12,2) + hex_string.substr(10,2);
      std::string z_string = hex_string.substr(24,2) + hex_string.substr(22,2) + hex_string.substr(20,2) + hex_string.substr(18,2);
      std::string suction_string = hex_string.substr(72,2) + hex_string.substr(70,2) + hex_string.substr(68,2) + hex_string.substr(66,2);
      // x
      my_state.x = hexStringToInt(x_string);
      // y
      my_state.y = hexStringToInt(y_string);
      // z
      my_state.z = hexStringToInt(z_string);
      // suction
      my_state.suction = (bool)hexStringToInt(suction_string);
      std::cout << my_state.x << "  " << my_state.y << "  " << my_state.z << "  " << (int)my_state.suction << std::endl;
      // remap to end effector
      int r = sqrt(pow(my_state.x,2) + pow(my_state.y,2));
      if(r != 0)
      {
      my_state.x = my_state.x + x_offset*my_state.x/r - y_offset*my_state.y/r;
      my_state.y = my_state.y + x_offset*my_state.y/r + y_offset*my_state.x/r;
      }
    }
    return 1;
  }
  return 0;
}

// send command to serial
void sendCommand(serial::Serial& my_serial, std::vector<int>& command_vect)
{
  if(command_vect.size() != 10)
    return;
  std::string command_str(1,(char)hexA5);
  for(std::vector<int>::iterator it = command_vect.begin(); it != command_vect.end(); ++it)
  {
    float f = *it;
    char* c = reinterpret_cast<char*>(&f);
    command_str.append(c, 4);
    //command_str = command_str + *(c+3) + *(c+2) + *(c+1) + *c;
  }
  command_str.push_back((char)hex5A);
  //std::cout << tohex(command_str, 1) << std::endl;
  // send it
  my_serial.write(command_str);
}


// send move command with configured speed
void sendFullMoveCommand(serial::Serial& my_serial, int& count)
{
  std::vector<int> command_vect10;
  switch(count)
  {
  case 0:
    if(my_command_list.empty())
      return;
    // adjust speed first!
    command_vect10 = {10,0,50,
                      my_command_list.front().speed,
                      my_command_list.front().speed,
                      0,0,0,0,0};
    sendCommand(my_serial, command_vect10);
    std::cout << count << std::endl;
    count++;
    break;

  case 1:
    // real move command
    command_vect10 = {3,
                      0,
                      my_command_list.front().x,
                      my_command_list.front().y,
                      my_command_list.front().z,
                      0,
                      (int)my_command_list.front().suction,
                      (int)my_command_list.front().move + 1, // convert to 0 1 2 of dobot protocol
                      0,
                      0
                     };
    sendCommand(my_serial, command_vect10);
    std::cout << count << std::endl;
    count++;
    break;

  case 2:
    command_vect10 = {3,
                      0,
                      my_command_list.front().x,
                      my_command_list.front().y,
                      my_command_list.front().z,
                      0,
                      (int)my_command_list.front().suction,
                      (int)my_command_list.front().move + 1, // convert to 0 1 2 of dobot protocol
                      0,
                      my_command_list.front().wait
                     };
    sendCommand(my_serial, command_vect10);
    std::cout << count << std::endl;
    std::cout << " I sent this: x: " << command_vect10.at(2)
              << "  y: "       << command_vect10.at(3)
              << "  z: "       << command_vect10.at(4)
              << "  suction: " << command_vect10.at(6)
              << "  move: "    << command_vect10.at(7)
              << "  wait: "    << command_vect10.at(9)
              << "  speed: "   << my_command_list.front().speed << std::endl;
    // pop front the command in the list
    my_command_list.pop_front();
    count = 0;
    break;

  default:
    break;
  }
}

// filter out all out of range coordinates
void commandFilter(const my_message_and_service::commandConstPtr& command_ptr, int& x, int& y)
{
  int r = (int)sqrt(pow((double)command_ptr->x, 2) + pow((double)command_ptr->y, 2));
  if(command_ptr->x != 0)
  {
  x = command_ptr->x - x_offset*command_ptr->x/r + y_offset*command_ptr->y/r;
  y = command_ptr->y - x_offset*command_ptr->y/r - y_offset*command_ptr->x/r;
  }
  else
  {
    x = command_ptr->x;
    y = command_ptr->y;
  }
  /*
  int angle = atan2(y,x)*180/PI;
  r = sqrt(pow(x,2) + pow(y,2));
  if(command_ptr->z == home_z)
  {
    if(r > radi3 || r > radi2 && angle > 43 && angle < 63)
    {
      x -= r<radi3?r-radi2:r-radi3;
      y -= r<radi3?r-radi2:r-radi3;
    }
    if(r < radi1)
    {
      x += radi1 - r;
      y += radi1 - r;
    }
  }
  */
}

// callback
void commandCallback(const my_message_and_service::commandConstPtr& command)
{
  int x, y;
  commandFilter(command, x, y);
  my_message_and_service::command tmp_command;
  tmp_command.x = x;
  tmp_command.y = y;
  tmp_command.z = command->z;
  tmp_command.suction = command->suction;
  tmp_command.move = command->move;
  tmp_command.speed = command->speed;
  tmp_command.wait = command->wait;
  my_command_list.push_back(tmp_command);
}


int main(int argc, char* argv[])
{
  ROS_INFO("Node dobot is starting.");
  ros::init(argc, argv, "dobot");
  const std::string port = "/dev/ttyACM0";
  const unsigned long baudrate = 256000;

  // change unix system baudrate
  if(changeBaudrate(port.c_str(), baudrate))
    std::cerr << " error changing baudrate. " << std::endl;
  else
    std::cout << " changed baudrate successfully." << std::endl;

  // setup serial port with port name, baudrate, timeout in milliseconds
  serial::Serial my_serial(port, baudrate, serial::Timeout::simpleTimeout(1000));
  std::cout << "Is the port opened?";
  if(my_serial.isOpen())
    std::cout << "Yes." << std::endl;
  else
    std::cout << "No." << std::endl;

  // create node handle, publisher and subscriber
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<my_message_and_service::state>("robot_state",1);
  ros::Subscriber sub = nh.subscribe("robot_command", 11, commandCallback);

  // main loop
  ros::Rate loop_rate(5);
  timespec start, stop;
  int count = 0;
  while(ros::ok())
  {    
    //clock_gettime(CLOCK_REALTIME, &start);

    if(getState(my_serial, state_msg))
    {
      pub.publish(state_msg);
      sendFullMoveCommand(my_serial,count);
    }
    //std::cout << " ----------------------1------" << std::endl;
    ros::spinOnce();
    //std::cout << " ----------------------2------" << std::endl;
    loop_rate.sleep();
    //clock_gettime(CLOCK_REALTIME, &stop);
    //std::cout<< (stop.tv_nsec - start.tv_nsec)/1000000 << std::endl << std::endl;
  }
  return 0;
}


/*
if(!my_command_list.empty())
{
std::cout << my_command_list.front().x << ' ' << my_command_list.front().y << ' ' << my_command_list.front().z << my_command_list.front().move << std::endl;
my_command_list.pop_front();
}
*/

// initialize dobot
//getState(my_serial, state_msg);
//usleep(500*1000);
/*
std::vector<int> command_vect10;
// 1) set end effector type to suction
command_vect10 = {9,4,0,0,0 ,0,0,0,0,0};
sendCommand(my_serial, command_vect10);
usleep(10*1000);
// 2) teach configuration
command_vect10 = {9,0,200,200,200 ,200,200,200,0,0};
sendCommand(my_serial, command_vect10);
usleep(10*1000);
// 1) playback configuration
command_vect10 = {9,1,200,200,200 ,200,200,400,0,20};
sendCommand(my_serial, command_vect10);
usleep(1000*1000);
// 4) home
getState(my_serial, state_msg);
std::cout << "sending home command" << std::endl;
std::vector<int> command_vect1 = {10,0,50,20,20 ,0,0,0,0,0};
sendCommand(my_serial, command_vect1);
usleep(10*1000);
getState(my_serial, state_msg);
std::vector<int> command_vect2 = {3,0,218,0,70 ,0,0,0,0,0};
sendCommand(my_serial, command_vect2);
usleep(10*1000);
*/


/*
 * #include <ros/ros.h>
#include <my_message_and_service/command.h>
#include <my_message_and_service/state.h>
#include <sstream>
#include <deque>
#include <sys/signal.h>
//c lib
#include <sys/time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>
#include <asm/ioctls.h>
//#include <asm/termios.h>
// custom lib
#include <serial/serial.h>
#define PI 3.14159265
#define hexA5 165
#define hex5A 90

typedef std::deque<my_message_and_service::command> command_list;
command_list my_command_list;
my_message_and_service::state state_msg;
const int x_offset = 7; // end_effector x offset
const int y_offset = 0;  // end_effector y offset
const int home_z = 80;   // home z
const int radi1 = 189;   // inner limit radi
const int radi2 = 262;   // offset radi
const int radi3 = 307;   // outer limit radi


int changeBaudrate(const char* port, int baudrate)
{
  int fd = open(port, O_RDONLY);
  struct termios2 tio;
  ioctl(fd, TCGETS2, &tio);
  tio.c_cflag &= ~CBAUD;
  tio.c_cflag |= BOTHER;
  tio.c_ispeed = baudrate;
  tio.c_ospeed = baudrate;
  int r = ioctl(fd, TCSETS2, &tio);
  close(fd);
  return r;
}


std::string tohex(const std::string& s, bool upper=false)
{
  std::ostringstream ret;

  unsigned int c;
  for (std::string::size_type i = 0; i < s.length(); ++i)
    {
      c = (unsigned int)(unsigned char)s[i];
      ret << std::hex << std::setfill('0') <<
        std::setw(2) << (upper ? std::uppercase : std::nouppercase) << c;
    }
  return ret.str();
}

int hexStringToInt(std::string& hex_string)
{
  unsigned int tmp_i;
  float tmp_f;
  std::stringstream ss;
  // x
  ss << std::hex << hex_string;
  ss >> tmp_i;
  tmp_f = reinterpret_cast<float&>(tmp_i);
  return (int)(tmp_f + 0.5);
}

std::string intToHexString(int input)
{
  std::stringstream ss;
  ss << std::hex << (float)input;
  return ss.str();
}
// get state from serial
bool getState(serial::Serial& my_serial, my_message_and_service::state& my_state)
{
  int byte_no = 0;
  if(byte_no=my_serial.available())
  {
    std::string hex_string = tohex(my_serial.read(byte_no), true);
    //std::cout<< hex_string << std::endl;

    if(byte_no >= 42)
    {
      // filter out a full state
      std::size_t found = hex_string.find("5AA5");
      hex_string = hex_string.substr(found+2, 84);
      //std::cout<< hex_string << std::endl;
      // take info from state
      std::string x_string = hex_string.substr(8,2) + hex_string.substr(6,2) + hex_string.substr(4,2) + hex_string.substr(2,2);
      std::string y_string = hex_string.substr(16,2) + hex_string.substr(14,2) + hex_string.substr(12,2) + hex_string.substr(10,2);
      std::string z_string = hex_string.substr(24,2) + hex_string.substr(22,2) + hex_string.substr(20,2) + hex_string.substr(18,2);
      std::string suction_string = hex_string.substr(72,2) + hex_string.substr(70,2) + hex_string.substr(68,2) + hex_string.substr(66,2);
      // x
      my_state.x = hexStringToInt(x_string);
      // y
      my_state.y = hexStringToInt(y_string);
      // z
      my_state.z = hexStringToInt(z_string);
      // suction
      my_state.suction = (bool)hexStringToInt(suction_string);
      std::cout << my_state.x << "  " << my_state.y << "  " << my_state.z << "  " << (int)my_state.suction << std::endl;
      // remap to end effector
      int r = sqrt(pow(my_state.x,2) + pow(my_state.y,2));
      my_state.x = my_state.x + x_offset*my_state.x/r - y_offset*my_state.y/r;
      my_state.y = my_state.y + x_offset*my_state.y/r + y_offset*my_state.y/r;
    }
    return 1;
  }
  return 0;
}

// send command to serial
void sendCommand(serial::Serial& my_serial, std::vector<int>& command_vect)
{
  if(command_vect.size() != 10)
    return;
  std::string command_str(1,(char)hexA5);
  for(std::vector<int>::iterator it = command_vect.begin(); it != command_vect.end(); ++it)
  {
    float f = *it;
    char* c = reinterpret_cast<char*>(&f);
    command_str.append(c, 4);
    //command_str = command_str + *(c+3) + *(c+2) + *(c+1) + *c;
  }
  command_str.push_back((char)hex5A);
  //std::cout << tohex(command_str, 1) << std::endl;
  // send it
  my_serial.write(command_str);
}

// send move command with configured speed
void sendFullMoveCommand(serial::Serial& my_serial)
{
  if(my_command_list.empty())
    return;
  // adjust speed first!
  std::vector<int> command_vect10 = {10,0,50,
                                     my_command_list.front().speed,
                                     my_command_list.front().speed,
                                     0,0,0,0,0};
  sendCommand(my_serial, command_vect10);
  usleep(10*1000);
  sendCommand(my_serial, command_vect10);
  usleep(10*1000);
  // real move command
  command_vect10 = {3,
                    0,
                    my_command_list.front().x,
                    my_command_list.front().y,
                    my_command_list.front().z,
                    0,
                    (int)my_command_list.front().suction,
                    (int)my_command_list.front().move + 1, // convert to 0 1 2 of dobot protocol
                    0,
                    my_command_list.front().wait
                   };
  sendCommand(my_serial, command_vect10);
  usleep(10*1000);
  sendCommand(my_serial, command_vect10);
  usleep(10*1000);
  sendCommand(my_serial, command_vect10);
  usleep(10*1000);
  sendCommand(my_serial, command_vect10);
  std::cout << " I sent this: " << command_vect10.at(2) << ' '<< command_vect10.at(3) << ' ' << command_vect10.at(4) << ' ' << command_vect10.at(7) << ' ' << my_command_list.front().speed << std::endl;
  // pop front the command in the list
  my_command_list.pop_front();
}

// filter out all out of range coordinates
void commandFilter(const my_message_and_service::commandConstPtr& command_ptr, int& x, int& y)
{
  int r = sqrt(pow(command_ptr->x, 2) + pow(command_ptr->y, 2));
  x = command_ptr->x - x_offset*command_ptr->x/r + y_offset*command_ptr->y/r;
  y = command_ptr->y - x_offset*command_ptr->y/r - y_offset*command_ptr->x/r;
  int angle = atan2(y,x)*180/PI;
  r = sqrt(pow(x,2) + pow(y,2));
  if(command_ptr->z == home_z)
  {
    if(r > radi3 || r > radi2 && angle > 43 && angle < 63)
    {
      x -= r<radi3?r-radi2:r-radi3;
      y -= r<radi3?r-radi2:r-radi3;
    }
    if(r < radi1)
    {
      x += radi1 - r;
      y += radi1 - r;
    }
  }
}

// callback
void commandCallback(const my_message_and_service::commandConstPtr& command)
{
  int x, y;
  commandFilter(command, x, y);
  my_message_and_service::command tmp_command;
  tmp_command.x = x;
  tmp_command.y = y;
  tmp_command.z = command->z;
  tmp_command.suction = command->suction;
  tmp_command.move = command->move;
  tmp_command.speed = command->speed;
  tmp_command.wait = command->wait;
  my_command_list.push_back(tmp_command);
}


int main(int argc, char* argv[])
{
  ROS_INFO("Node dobot is starting.");
  ros::init(argc, argv, "dobot");
  const std::string port = "/dev/ttyACM0";
  const unsigned long baudrate = 256000;

  // change unix system baudrate
  if(changeBaudrate(port.c_str(), baudrate))
    std::cerr << " error changing baudrate. " << std::endl;
  else
    std::cout << " changed baudrate successfully." << std::endl;

  // setup serial port with port name, baudrate, timeout in milliseconds
  serial::Serial my_serial(port, baudrate, serial::Timeout::simpleTimeout(1000));
  std::cout << "Is the port opened?";
  if(my_serial.isOpen())
    std::cout << "Yes." << std::endl;
  else
    std::cout << "No." << std::endl;

  // create node handle, publisher and subscriber
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<my_message_and_service::state>("robot_state",1);
  ros::Subscriber sub = nh.subscribe("robot_command", 7, commandCallback);

  // main loop
  ros::Rate loop_rate(5);
  timespec start, stop;
  while(ros::ok())
  {
    //clock_gettime(CLOCK_REALTIME, &start);

    if(getState(my_serial, state_msg))
    {
      pub.publish(state_msg);
      sendFullMoveCommand(my_serial);
    }

    ros::spinOnce();
    loop_rate.sleep();
    //clock_gettime(CLOCK_REALTIME, &stop);
    //std::cout<< (stop.tv_nsec - start.tv_nsec)/1000000 << std::endl << std::endl;
  }
  return 0;
}
*/

/*
 * // send move command with configured speed
void sendFullMoveCommand(serial::Serial& my_serial)
{
  if(my_command_list.empty())
    return;
  // adjust speed first!
  std::vector<int> command_vect10 = {10,0,50,
                                     my_command_list.front().speed,
                                     my_command_list.front().speed,
                                     0,0,0,0,0};
  sendCommand(my_serial, command_vect10);
  usleep(10*1000);
  sendCommand(my_serial, command_vect10);
  usleep(10*1000);
  // real move command
  command_vect10 = {3,
                    0,
                    my_command_list.front().x,
                    my_command_list.front().y,
                    my_command_list.front().z,
                    0,
                    (int)my_command_list.front().suction,
                    (int)my_command_list.front().move + 1, // convert to 0 1 2 of dobot protocol
                    0,
                    0
                   };
  sendCommand(my_serial, command_vect10);
  usleep(10*1000);
  sendCommand(my_serial, command_vect10);
  usleep(10*1000);
  sendCommand(my_serial, command_vect10);
  std::cout << " I sent this: " << command_vect10.at(2) << ' '<< command_vect10.at(3) << ' ' << command_vect10.at(4) << ' ' << command_vect10.at(7) << ' ' << my_command_list.front().speed << std::endl;
  usleep(10*1000);
  command_vect10 = {3,
                    0,
                    my_command_list.front().x,
                    my_command_list.front().y,
                    my_command_list.front().z,
                    0,
                    (int)my_command_list.front().suction,
                    0,
                    0,
                    my_command_list.front().wait
                   };
  sendCommand(my_serial, command_vect10);

  // pop front the command in the list
  my_command_list.pop_front();
  std::cout << " ----------------------------" << std::endl;
}
*/
