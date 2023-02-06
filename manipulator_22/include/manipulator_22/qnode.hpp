/**
 * @file /include/manipulator_22/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef manipulator_22_QNODE_HPP_
#define manipulator_22_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/


#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/network.h>
#include <QThread>
#include <gazebo/gazebo.hh>

#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <QDebug>
#include <eigen3/Eigen/Dense>
#include "unsupported/Eigen/MatrixFunctions"
#include <time.h>
#include <math.h>
#include "kubo_msgs/RobotState.h"


#if QT_VERSION >= 0x50000
    #include <qmath.h>
#else
    #include <QtCore/qmath.h>
#endif

#define PI 3.14159265358979
#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace manipulator_22 {
using namespace Eigen;
using namespace Qt;
using namespace std;
using namespace gazebo;

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

  bool inited=false;

  float set_joint_angle[6]={0,0,0,0,0,0};
  float get_joint_angle[6]={0,0,0,0,0,0};
  float joint_data[6]={0,0,0,0,0,0};
  float joint_horizon = 0;
  float joint_rad[6]={0,0,0,0,0,0};

  int state = 0;
  int flag = 0;
  bool camera_aspect=false;
  //////////Manipulator_DH_parameter////////////

  float theta1 = 0.0;
  float theta2 = 0.0;
  float theta3 = 0.0;
  float theta4 = 0.0;
  float theta5 = 0.0;
  float theta6 = 0.0;

  int Posture = 0;

  double ik_theta[3] = {0, };

//  // Link ParameteL2r
  float L1 = 47;
  float L2 = 468;
  float L3 = 384;
  float L4 = 0;
  float L5 = 0;
  float L6 = 0;

//  float L2 = 568;
//  float L3 = 317;

  //matlab
//  float L1 = 145;
//  float L2 = 510;
//  float L3 = 315;
//  float L4 = 0;
//  float L5 = 0;
//  float L6 = 0;

  float f_x = 0.0;
  float f_y = 0.0;
  float f_z = 0.0;

//  float past_X = 450.0;
//  float past_Y = 0.0;
//  float past_Z = 500.0;

  float cur_X = 520.0;
  float cur_Y = 0.0;
  float cur_Z = 290.0;

  float last_cur_X;
  float last_cur_Y;
  float last_cur_Z;

  float last_X;
  float last_Y;
  float last_Z;

  float distance_origin;

  int t = 0;
  int i = 0;
  int tt = 0;
  int count = 0;

  int init_cnt_1 = 0;
  int init_cnt_2 = 0;
  int init_cnt_3 = 0;
  int init_cnt_4 = 0;

  int supplies_cnt_1=0;
  int supplies_cnt_2=0;
  int supplies_cnt_3=0;
  int supplies_cnt_4=0;
  int supplies_cnt_5=0;
  int supplies_cnt_6=0;
  int supplies_cnt_7=0;
  int supplies_cnt_8=0;

  float X, Y, Z;
  float inclination_xy;
  float inclination_z;
  float output_Y;
  float output_Z;
  float past_X;
  float past_Z;

  float tar_X = 450;
  float tar_Y = 0;
  float tar_Z = 500;
  float tra_X = 450;
  float tra_Y = 0;
  float tra_Z = 500;

  float c_tar_X = 250;
  float c_tar_Y = 0;
  float c_tar_Z = 600;
  float c_tra_X = 250;
  float c_tra_Y = 0;
  float c_tra_Z = 600;
  float r_tar_X = 250;
  float r_tar_Y = 0;
  float r_tar_Z = 600;

  float past_alpha = 0;
  float past_beta = 0;
  float past_gamma = 0;
  float cur_alpha = 0.0;
  float cur_beta = 0.0;
  float cur_gamma = 0.0;
  float tar_alpha = 0;
  float tar_beta = 0;
  float tar_gamma = 0;
  float tra_alpha = 0;
  float tra_beta = 0;
  float tra_gamma = 0;
  float c_tra_alpha = 0;
  float c_tra_beta = 0;
  float c_tra_gamma = 0;

  float c_tar_alpha = 0;
  float c_tar_beta = 0;
  float c_tar_gamma = 0;

  float r_tar_alpha = 0;
  float r_tar_beta = 0;
  float r_tar_gamma = 0;

  float th[6] = {0.0, 1.5707, -1.5707, 0.0, 0.0, 0.0};
  float th_max[6] = {1.5707, 2.2863, 0, 3.1415, 1.5707, 3.1415};            //  90 133    0  180  90  180
  float th_min[6] = {-1.5707, -1.0471, -3.1415, -3.1415, -1.5707, -3.1415}; // -90 -30 -180 -180 -90 -180
  float past_th3, past_th5;

  //////////Matrix///////////
  MatrixXf Rx( float theta);
  MatrixXf Ry( float theta);
  MatrixXf Rz( float theta);
  MatrixXf Tz( float l);
  MatrixXf Tx( float l);
  MatrixXf Ty( float l);



Q_SIGNALS:

    void rosShutdown();
    void dataChanged();

public:
    float deg2rad(float deg)
    {
        return deg*PI/180;
    }

    float rad2deg(float rad)
    {
        return rad*180/PI;
    }

    void Notice();
    void Option();
    void Singularity_check();
    void Inverse_kinematics();
    void Foward_Kinematics();
    void Mode_Change();
    void Equation_straightline_xy();
    void Equation_straightline_z(float unit);
    void Equation_upline(float unit);
    bool Mode=true;
    bool Init=false;
    bool supplies_1=false;
    bool supplies_2=false;

    int gripper=0;
    void btn_count();
    void Trajectory();
    void Macro();
    int Kinematics(float pX, float pY, float pZ, float rX, float rY, float rZ);
    int check(float x, float  z, float z_coordiante);


private:
	int init_argc;
	char** init_argv;

  // ROS Topic Publishers
  ros::Publisher pub;
  ros::Publisher control_mani_pub;
  ros::Publisher state_pub;
  ros::Publisher manipulator_pub;
  // ROS Topic Subscribers
  ros::Subscriber sub;
  ros::Subscriber sub_joy;
  ros::Subscriber m_init_trigger;
  ros::Subscriber m_init_done;
  ros::Subscriber gripper_trigger;


//  ros::Subscriber sub_mani;

  void gripper_trigger_cb(const std_msgs::Bool::ConstPtr &msg);

  void m_init_trigger_cb(const std_msgs::Bool::ConstPtr &msg);
  void m_init_done_cb(const std_msgs::Bool::ConstPtr &msg);

  void readJointAng(const std_msgs::Float32MultiArrayPtr &msg);
  void writeJointAng();
  void Forward_Kinematics(float th0, float th1, float th2, float th3, float th4, float th5, float x, float y, float z);

  void main();
  void manipulation1();
  void manipulation2();
  void Init_judgement();
  void Supplies_judgement();
  void Mode_judgement();
  void Motion_judgement(int mode);
  int motion_mode = 999;
  void Joystick(const sensor_msgs::Joy::ConstPtr &joy);
  void appendJointState(sensor_msgs::JointState &j, string name, float pos, float vel, float effort);

   std_msgs::Float32MultiArray msg;

private:
  /*joystick data_base*/
  int base_arrowsX, base_arrowsY, base_buttonSq, base_buttonX, base_buttonO, base_buttonTr, base_l1=0, base_r1=0, base_buttonShare, base_buttonOption;
  float base_leftStickY, base_leftStickX, base_rightStickY, base_rightStickX, base_l2=0, base_r2=0;
  int base_buttonTouch=0;


  /*joystick data_mani*/
  float leftStickY=0, leftStickX=0, rightStickY=0, rightStickX=0, l2, r2;
  float pos_X = 890.0, pos_Y = 0.0, pos_Z = 166.0;
  float ori_X = 0.0, ori_Y = 90.0, ori_Z = 0.0;
  float joy_X = 0.0, joy_Y = 0.0, joy_Z = 0.0;

  int arrowsX=0, arrowsY=0, button_Square = 0, button_X=0, button_O=0, button_Triangle = 0,
      button_L1=0, button_R1=0, buttonShare=0, buttonOption=0, button_Share = 0, button_Option=0;
  int button_L2=0, button_R2=0;
  int button_left_right, button_up_down;

  float stick_X_Axis, stick_Y_Axis, stick_Z_Axis;

  clock_t compare_time_ms = 0;

  int press_time = 0;


  float rightStickX_, rightStickY_;

  int buttonModeChange=0;
  int ModeChange=0;
  int button_flag = 0;
  int button_count = 0;

  bool btn_share_once=true;
  bool btn_option_once=true;
  bool btn_init_once = true;
  bool btn_square_once = true;
  bool btn_O_once = true;

   float data_sum=0;

};

}  // namespace manipulator_22

#endif /* manipulator_22_QNODE_HPP_ */
