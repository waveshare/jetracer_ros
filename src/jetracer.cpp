#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <dynamic_reconfigure/server.h>
#include "jetracer/jetracerConfig.h"

#include <iostream>
#include <thread>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#define head1 0xAA
#define head2 0x55
#define sendType_velocity    0x11
#define sendType_params      0x12
#define sendType_coefficient 0x13

using namespace std;
using namespace boost::asio;

io_service iosev;              
serial_port sp(iosev);         //Define the serial port for transmission
std::string port_name;
int baud_rate;

bool publish_odom_transform;   
int kp;                        //robot param
int ki;
int kd;
float linear_correction = 1;
int servo_bias = 0;

float a;                        //Coefficient of quartic equation for Steering calibration
float b;
float c;
float d;

double x = 0.0;                //robot Velocity
double y = 0.0;
double yaw = 0.0;
ros::Time cmd_time;

uint8_t checksum(uint8_t* buf, size_t len)
{
  uint8_t sum = 0x00;
  for(int i=0;i<len;i++)
  {
    sum += *(buf + i);
  }
  return sum;
}

/*robot parameter sending function*/
void SetParams(int p,int i, int d, double linear_correction,int servo_bias) {
  static uint8_t tmp[15];
  tmp[0]  = head1;
  tmp[1]  = head2;
  tmp[2]  = 0x0F;
  tmp[3]  = sendType_params;
  tmp[4]  = (p>>8)&0xff;
  tmp[5]  = p&0xff;
  tmp[6]  = (i>>8)&0xff;
  tmp[7]  = i&0xff;
  tmp[8]  = (d>>8)&0xff;
  tmp[9]  = d&0xff;
  tmp[10] = (int16_t)((int16_t)(linear_correction*1000)>>8)&0xff;
  tmp[11] = (int16_t)(linear_correction*1000)&0xff;
  tmp[12] = ((int16_t)((int16_t)servo_bias)>>8)&0xff;
  tmp[13] = ((int16_t)servo_bias)&0xff;
  tmp[14] = checksum(tmp,14);
  write(sp,buffer(tmp,15));
  printf("SetParams\r\n");
  ROS_INFO_STREAM ("set robot param: p="<<p<<" i="<<i<<" d="<<d \
      <<" linear_correction="<<linear_correction<<" servo_bias="<<servo_bias);
}

/*steering Coefficient sending function*/
void SetCoefficient(float a,float b,float c,float d) {
  static uint8_t tmp[21];
  char* p;
  tmp[0]  = head1;
  tmp[1]  = head2;
  tmp[2]  = 0x15;
  tmp[3]  = sendType_coefficient;
  p=(char*)&a;
  tmp[4]  = p[0];
  tmp[5]  = p[1];
  tmp[6]  = p[2];
  tmp[7]  = p[3];
  p=(char*)&b;
  tmp[8]  = p[0];
  tmp[9]  = p[1];
  tmp[10] = p[2];
  tmp[11] = p[3];
  p=(char*)&c;
  tmp[12] = p[0];
  tmp[13] = p[1];
  tmp[14] = p[2];
  tmp[15] = p[3];
  p=(char*)&d;
  tmp[16] = p[0];
  tmp[17] = p[1];
  tmp[18] = p[2];
  tmp[19] = p[3];
  tmp[20] = checksum(tmp,20);
  write(sp,buffer(tmp,21));
  ROS_INFO_STREAM ("set steering coefficient: a="<<a<<" b="<<b<<" c="<<c<<" d="<<d);
}

/*robot speed transmission function*/
void SetVelocity(double x, double y, double yaw)
{
  static uint8_t tmp[11];
  tmp[0] = head1;
  tmp[1] = head2;
  tmp[2] = 0x0b;
  tmp[3] = sendType_velocity;
  tmp[4] = ((int16_t)(x*1000)>>8) & 0xff;
  tmp[5] = ((int16_t)(x*1000)) & 0xff;
  tmp[6] = ((int16_t)(y*1000)>>8) & 0xff;
  tmp[7] = ((int16_t)(y*1000)) & 0xff;
  tmp[8] = ((int16_t)(yaw*1000)>>8) & 0xff;
  tmp[9] = ((int16_t)(yaw*1000)) & 0xff;
  tmp[10] = checksum(tmp,10);
  write(sp,buffer(tmp,11));
}

/* cmd_vel Subscriber callback function*/
void cmd_callback(const geometry_msgs::Twist& msg)
{
  x = msg.linear.x;
  y = msg.linear.x;
  yaw = msg.angular.z;
  cmd_time = ros::Time::now();
}

/*pid dynamic_reconfigure callback function*/
void Config_callback(jetracer::jetracerConfig &config)
{
  kp = config.kp;
  ki = config.ki;
  kd = config.kd;
  servo_bias = config.servo_bias;
  SetParams(kp,ki,kd,linear_correction,servo_bias);
}

//serial port receiving task
void serial_task()
{
  enum frameState
  {
    State_Head1, State_Head2, State_Size, State_Data, State_CheckSum, State_Handle
  };
  
  frameState state = State_Head1;
  
  uint8_t frame_size, frame_sum, frame_type;
  uint8_t data[50];
  
  double  imu_list[9];
  double  odom_list[6];
  ros::Time now_time,last_time;
  
  ros::NodeHandle n;
  
  //Create Publisher message
  sensor_msgs::Imu imu_msgs;
  geometry_msgs::TransformStamped odom_trans;
  geometry_msgs::Quaternion odom_quat;
  nav_msgs::Odometry odom_msgs;
  std_msgs::Int32 lvel_msgs;
  std_msgs::Int32 rvel_msgs;
  std_msgs::Int32 lset_msgs; 
  std_msgs::Int32 rset_msgs;

  //Create Publisher
  tf::TransformBroadcaster odom_broadcaster;
  ros::Publisher imu_pub     = n.advertise<sensor_msgs::Imu>("imu",10);
  ros::Publisher odom_pub    = n.advertise<nav_msgs::Odometry>("odom", 10);
  ros::Publisher lvel_pub    = n.advertise<std_msgs::Int32>("motor/lvel",10);
  ros::Publisher rvel_pub    = n.advertise<std_msgs::Int32>("motor/rvel",10);
  ros::Publisher lset_pub    = n.advertise<std_msgs::Int32>("motor/lset",10);
  ros::Publisher rset_pub    = n.advertise<std_msgs::Int32>("motor/rset",10); 
  
  ROS_INFO("start receive message");
  while(true)
  {
    //State machine
    // [head1 head2 size type data checksum ]
    // [0xAA  0x55  0x2D 0x01 ....  0xXX    ]
    switch (state)
    {
      case State_Head1:             //waiting for frame header 1
          frame_sum = 0x00;
          read(sp, buffer(&data[0], 1));
          state = (data[0] == head1 ? State_Head2 : State_Head1);
          if(state == State_Head1)
          {
              //ROS_INFO_STREAM ("recv head1 error : ->"<<(int)data[0]);
          }
          break;
          
      case State_Head2:             //waiting for frame header 2
          read(sp, buffer(&data[1], 1));
          state = (data[1] == head2 ? State_Size : State_Head1);
          if(state == State_Head1)
          {
              //ROS_INFO_STREAM ("recv head2 error : ->"<<(int)data[1]);
          }
          break;
          
      case State_Size:              //waiting for frame Size
          read(sp, buffer(&data[2], 1));
          frame_size = data[2];
          state = State_Data;
          break;
          
      case State_Data:              //waiting for frame data
          read(sp, buffer(&data[3], frame_size - 4));
          frame_type = data[3];
          state = State_CheckSum;
          break;
          
      case State_CheckSum:         //waiting for frame CheckSum
          read(sp, buffer(&data[frame_size -1], 1));
          frame_sum = checksum(data,frame_size -1);
          state = data[frame_size -1] == frame_sum ? State_Handle : State_Head1;
          if(state == State_Head1)
          {
              //ROS_INFO_STREAM ("check sum error! recv is  : ->"<<(int)data[frame_size -1]<<"  calc is "<<frame_sum);
          }
          break;
          
      case State_Handle:         //processing frame data
          now_time = ros::Time::now();
          
          //gyro
          imu_list[0]=((double)((int16_t)(data[4]*256+data[5]))/32768*2000/180*3.1415);
          imu_list[1]=((double)((int16_t)(data[6]*256+data[7]))/32768*2000/180*3.1415);
          imu_list[2]=((double)((int16_t)(data[8]*256+data[9]))/32768*2000/180*3.1415);
          //Acc 
          imu_list[3]=((double)((int16_t)(data[10]*256+data[11]))/32768*2*9.8);
          imu_list[4]=((double)((int16_t)(data[12]*256+data[13]))/32768*2*9.8);
          imu_list[5]=((double)((int16_t)(data[14]*256+data[15]))/32768*2*9.8);
          //Angle 
          imu_list[6]=((double)((int16_t)(data[16]*256+data[17]))/10.0);
          imu_list[7]=((double)((int16_t)(data[18]*256+data[19]))/10.0);
          imu_list[8]=((double)((int16_t)(data[20]*256+data[21]))/10.0);

          //publish the IMU message
          imu_msgs.header.stamp = ros::Time::now();
          imu_msgs.header.frame_id = "base_imu_link";
          imu_msgs.angular_velocity.x = imu_list[0];
          imu_msgs.angular_velocity.y = imu_list[1];
          imu_msgs.angular_velocity.z = imu_list[2];
          imu_msgs.linear_acceleration.x = imu_list[3];
          imu_msgs.linear_acceleration.y = imu_list[4];
          imu_msgs.linear_acceleration.z = imu_list[5];
          imu_msgs.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,(imu_list[8]/180*3.1415926));
          imu_msgs.orientation_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 0.05};
          imu_msgs.angular_velocity_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e6};
          imu_msgs.linear_acceleration_covariance = {1e-2, 0, 0, 0, 0, 0, 0, 0, 0};
          imu_pub.publish(imu_msgs);
      
          odom_list[0]=((double)((int16_t)(data[22]*256+data[23]))/1000);
          odom_list[1]=((double)((int16_t)(data[24]*256+data[25]))/1000);
          odom_list[2]=((double)((int16_t)(data[26]*256+data[27]))/1000);
          //dx dy dyaw base_frame
          odom_list[3]=((double)((int16_t)(data[28]*256+data[29]))/1000);
          odom_list[4]=((double)((int16_t)(data[30]*256+data[31]))/1000);
          odom_list[5]=((double)((int16_t)(data[32]*256+data[33]))/1000);
      
          //first, we'll publish the transform over tf
          odom_trans.header.stamp = now_time;
          odom_trans.header.frame_id = "odom";
          odom_trans.child_frame_id = "base_footprint";

          odom_trans.transform.translation.x = odom_list[0];
          odom_trans.transform.translation.y = odom_list[1];
          odom_trans.transform.translation.z = 0.0;
          //we'll need a quaternion created from yaw
          odom_quat = tf::createQuaternionMsgFromYaw(odom_list[2]);
          odom_trans.transform.rotation = odom_quat;

          //send the transform
          if(publish_odom_transform)odom_broadcaster.sendTransform(odom_trans);

          //next, we'll publish the odometry message over ROS
          odom_msgs.header.stamp = now_time;
          odom_msgs.header.frame_id = "odom";

          //set the position
          odom_msgs.pose.pose.position.x = odom_list[0];
          odom_msgs.pose.pose.position.y = odom_list[1];
          odom_msgs.pose.pose.position.z = 0.0;
          odom_msgs.pose.pose.orientation = odom_quat;

          //set the velocity
          odom_msgs.child_frame_id = "base_footprint";
          odom_msgs.twist.twist.linear.x = odom_list[3]/((now_time-last_time).toSec());
          odom_msgs.twist.twist.linear.y = odom_list[4]/((now_time-last_time).toSec());
          odom_msgs.twist.twist.angular.z = odom_list[5]/((now_time-last_time).toSec());
          odom_msgs.twist.covariance = { 1e-9, 0, 0, 0, 0, 0, 
                                         0, 1e-3, 1e-9, 0, 0, 0, 
                                         0, 0, 1e6, 0, 0, 0,
                                         0, 0, 0, 1e6, 0, 0, 
                                         0, 0, 0, 0, 1e6, 0, 
                                         0, 0, 0, 0, 0, 0.1 };
          odom_msgs.pose.covariance = { 1e-9, 0, 0, 0, 0, 0, 
                                         0, 1e-3, 1e-9, 0, 0, 0, 
                                         0, 0, 1e6, 0, 0, 0,
                                         0, 0, 0, 1e6, 0, 0, 
                                         0, 0, 0, 0, 1e6, 0, 
                                         0, 0, 0, 0, 0, 1e3 };
          //publish the odom message
          odom_pub.publish(odom_msgs);
          
          //publish the motor message
          lvel_msgs.data = ((int16_t)(data[34]*256+data[35]));
          rvel_msgs.data = ((int16_t)(data[36]*256+data[37]));
          lset_msgs.data = ((int16_t)(data[38]*256+data[39]));
          rset_msgs.data = ((int16_t)(data[40]*256+data[41]));
          lvel_pub.publish(lvel_msgs);
          rvel_pub.publish(rvel_msgs);
          lset_pub.publish(lset_msgs);
          rset_pub.publish(rset_msgs);

          last_time = now_time;

          state = State_Head1;
          break;
      default:
          state = State_Head1;
          break;
    }
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "jetracer");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  /*Get robot parameters from configuration file*/
  pn.param<std::string>("port_name",port_name,std::string("/dev/ttyACM0"));
  pn.param<bool>("publish_odom_transform",publish_odom_transform,true);
  pn.param<float>("linear_correction",linear_correction,1.0);
  pn.param<float>("coefficient_a",a,-0.016073);
  pn.param<float>("coefficient_b",b, 0.176183);
  pn.param<float>("coefficient_c",c,-23.428084);
  pn.param<float>("coefficient_d",d, 1500);

  //set serial port
  boost::system::error_code ec;
  sp.open(port_name,ec);
  sp.set_option(serial_port::baud_rate(115200));   
  sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
  sp.set_option(serial_port::parity(serial_port::parity::none));
  sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
  sp.set_option(serial_port::character_size(8));
  
  //Create Subscriber
  ros::Subscriber cmd_sub = n.subscribe("cmd_vel",10,cmd_callback);
  
  //set pid dynamic_reconfigure
  dynamic_reconfigure::Server<jetracer::jetracerConfig> server;
  dynamic_reconfigure::Server<jetracer::jetracerConfig>::CallbackType f;
  f = boost::bind(&Config_callback, _1);
  server.setCallback(f);

  ros::Duration(0.02).sleep();
  SetCoefficient(a,b,c,d);
  //Create serial port receiving task
  thread serial_thread(boost::bind(serial_task));
  
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  while(ros::ok()){
    //send once velocity to robot base every 0.02
    current_time = ros::Time::now();
    if((current_time - last_time).toSec() > 0.02){           
      last_time = current_time;
      
      if((current_time - cmd_time).toSec() > 1)
      {
        x = 0.0;
        y = 0.0;
        yaw = 0.0;
      }
      SetVelocity(x,y,yaw);
    }
    
    ros::spinOnce();
  }
     
  return 0;
}
