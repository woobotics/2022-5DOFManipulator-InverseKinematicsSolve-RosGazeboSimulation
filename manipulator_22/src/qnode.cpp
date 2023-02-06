/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "kubo_msgs/RobotState.h"
#include "../include/manipulator_22/qnode.hpp"

#define PI 3.14159265358979
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace manipulator_22 {
using namespace Eigen;
/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{}

QNode::~QNode()
{
    if(ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init()
{
    ros::init(init_argc,init_argv,"manipulator_22");
    if ( ! ros::master::check() )
    {
        return false;
    }
    //ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    //pub = n.advertise<std_msgs::Float32MultiArray>("write_joint_angle" , 1);
    manipulator_pub = n.advertise<kubo_msgs::RobotState>("write_joint_angle" , 1);
    sub = n.subscribe("read_joint_angle",1,&QNode::readJointAng,this);
        sub_joy = n.subscribe("joy_msg_manipulator",1,&QNode::Joystick,this);
//    sub_joy = n.subscribe("/joy",10,&QNode::Joystick,this);

    //    gripper_trigger = n.subscribe("gripper_trigger",1,&QNode::gripper_trigger_cb,this);

    m_init_trigger = n.subscribe("m_init_trigger",1,&QNode::m_init_trigger_cb,this);
    m_init_done = n.subscribe("m_init_done",1,&QNode::m_init_done_cb,this);
    // Add your ros communications here.

    start();
    return true;
}


void QNode::run()
{
    ros::Rate loop_rate(1000);

    while ( ros::ok() )
    {
        ros::spinOnce();

        QNode::main();

        loop_rate.sleep();

    }
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::main()
{
    //  Trajectory();

    //    if(inited)
    //    {
    //        Mode_judgement();
    //        Init_judgement();
    //        Mode_Change();
    //        //Notice();
    //        Foward_Kinematics();

    //        //cout << "Init : " << Init << endl;
    //    }

    //    writeJointAng();
    //    Foward_Kinematics();
    Mode_judgement();
    Init_judgement();
    Supplies_judgement();
    Motion_judgement(motion_mode);
    Mode_Change();
    inclination_xy = f_y/f_x;


    //    inclination_z = -tan(deg2rad(joint_horizon));
    //Notice();
    //    Foward_Kinematics();
    //    cout<<"hori : "<<joint_horizon<<endl;
    //    cout<<"output_Z : "<<output_Z<<endl;
    //    cout<<"inclination_z : "<<inclination_z<<endl;



}

void QNode::Singularity_check()
{

    distance_origin = sqrt(pow(f_x,2) + pow(f_y,2));

    last_X = cur_X;
    last_Y = cur_Y;
    last_Z = cur_Z;

    if(distance_origin < 10)
    {
        cur_X = last_X;
        cur_Y = last_Y;
        cur_Z = last_Z;
    }
}

void QNode::Equation_straightline_xy()
{
    output_Y = inclination_xy * cur_X;

    cur_Y = output_Y;
}

void QNode::Equation_straightline_z(float unit)
{

    if(theta1>=-90 &&theta1<=90)
    {
        cur_Z = -tan(deg2rad(joint_horizon)) * (sqrt(cur_X*cur_X + cur_Y*cur_Y))*abs(cur_X)/cur_X;
    }
    else
    {
        cur_Z = tan(deg2rad(joint_horizon)) * (sqrt(cur_X*cur_X + cur_Y*cur_Y))*abs(cur_X)/cur_X;
    }

    cur_X=cur_X/(sqrt(cur_X*cur_X + cur_Y*cur_Y+cur_Z*cur_Z))*0.2*abs(unit);
    cur_Y=cur_Y/(sqrt(cur_X*cur_X + cur_Y*cur_Y+cur_Z*cur_Z))*0.2*abs(unit);
    cur_Z=cur_Z/(sqrt(cur_X*cur_X + cur_Y*cur_Y+cur_Z*cur_Z))*0.2*abs(unit);


    camera_aspect = true;
}

void QNode::Equation_upline(float unit)
{

    cur_X = cos(deg2rad(theta1))*cur_Z*tan(deg2rad(joint_horizon));
    cur_Y = sin(deg2rad(theta1))*cur_Z*tan(deg2rad(joint_horizon));

    cur_X=cur_X/(sqrt(cur_X*cur_X + cur_Y*cur_Y+cur_Z*cur_Z))*0.2*abs(unit);
    cur_Y=cur_Y/(sqrt(cur_X*cur_X + cur_Y*cur_Y+cur_Z*cur_Z))*0.2*abs(unit);
    cur_Z=cur_Z/(sqrt(cur_X*cur_X + cur_Y*cur_Y+cur_Z*cur_Z))*0.2*abs(unit);

    camera_aspect = true;

}


void QNode::Mode_judgement()
{
    if(Mode)
    {
//         cout << "--------------------  Macro_Mode  --------------------" << endl;
        Macro();

        cur_X = f_x;
        cur_Y = f_y;
        cur_Z = f_z;

    }
    else
    {
//         cout << "--------------------  Kinematics_Mode --------------------" << endl;
        cur_X=0;
        cur_Y=0;
        cur_Z=0;

        manipulation1();
        Inverse_kinematics();
        manipulation2();
        Singularity_check();

//        cout << "cur_X  : " << cur_X << endl;
//        cout << "cur_Y  : " << cur_Y << endl;
//        cout << "cur_Z  : " << cur_Z <<endl;

//        cout << "distance_origin : " << distance_origin << endl;
    }
}

void QNode::readJointAng(const std_msgs::Float32MultiArrayPtr &msg_joint)
{
    writeJointAng();
    theta1 = msg_joint->data[0];
    theta2 = msg_joint->data[1];
    theta3 = msg_joint->data[2];
    theta4 = msg_joint->data[3];
    theta5 = msg_joint->data[4];
    //theta6 = msg_joint->data[5];
    Foward_Kinematics();

    Q_EMIT dataChanged();
}

void QNode::writeJointAng()
{
    kubo_msgs::RobotState state;


    for(int i=0;i<5;i++)
    {
        state.joint[i]= joint_data[i];
    }
    state.gripper = gripper;

    manipulator_pub.publish(state);
}

void QNode::m_init_trigger_cb(const std_msgs::Bool::ConstPtr &msg)
{

    inited = false;

    if(Mode == false)
    {
        Mode = !Mode;
    }

    joint_data[0] = 0;
    joint_data[1] = 0;
    joint_data[2] = 0;
    joint_data[3] = 0;
    joint_data[4] = 0;

    theta1 = 0;
    theta2 = 0;
    theta3 = 0;
    theta4 = 0;
    theta5 = 0;
}

void QNode::m_init_done_cb(const std_msgs::Bool::ConstPtr &msg)
{
    inited = true;
}

void QNode::Supplies_judgement()
{

    if(supplies_1)
    {
        joint_data[0] = theta1;
        joint_data[1] = theta2;
        joint_data[2] = theta3;
        joint_data[3] = theta4;
        joint_data[4] = theta5;


        //ROS_WARN("\n-------------------- Supplies_LOADING -------------------- \n");

        supplies_cnt_1++;
        //       count = t/4;
        supplies_cnt_2 = supplies_cnt_1/60;

        joint_data[2] = theta3 + (50 - theta3)*(0.0002*supplies_cnt_2);


        if(joint_data[2] > 48 && joint_data[2] < 52)
        {

            supplies_cnt_3++;
            supplies_cnt_4 = supplies_cnt_3/60;

            joint_data[1] = theta2 + (-71 - theta2)*(0.00006*supplies_cnt_4);

        }

        if( joint_data[1] < -69 &&  joint_data[1] > -73)
        {

            supplies_cnt_5++;
            supplies_cnt_6 = supplies_cnt_5/60;

            joint_data[0] = theta1 + (-145 - theta1)*(0.00006*supplies_cnt_6);

        }

        if(joint_data[0] > -147 && joint_data[0] < -143)
        {

            supplies_cnt_7++;
            supplies_cnt_8 = supplies_cnt_7/60;


            joint_data[3] = theta4 + (0 - theta4)*(0.0002*supplies_cnt_8);
            joint_data[4] = theta5 + (65 - theta5)*(0.0002*supplies_cnt_8);


        }



        if(joint_data[0] > -147 && joint_data[0] < -143 && joint_data[1] > -73 && joint_data[1] < -69 && joint_data[2] < 52 && joint_data[2] > 48 && abs(joint_data[3]) < 1 && joint_data[4] < 66 && joint_data[4] >62)
        {
            //ROS_INFO("joint hor : %lf", joint_horizon);
            supplies_1 = false;
            Mode = false;
            joint_horizon=85;

            supplies_cnt_1 = 0;
            supplies_cnt_2 = 0;
            supplies_cnt_3 = 0;
            supplies_cnt_4 = 0;
            supplies_cnt_5 = 0;
            supplies_cnt_6 = 0;
            supplies_cnt_7 = 0;
            supplies_cnt_8 = 0;

        }


    }

    if(supplies_2)
    {
        if(Mode == false)
        {
            Mode = !Mode;
            joint_horizon = 0;
        }


    }

}



void QNode::Init_judgement()
{

    if(Init)
    {
        if(Mode == false)
        {
            Mode = !Mode;
            joint_horizon = 0;
        }


        ROS_WARN("\n-------------------- INIT_LOADING -------------------- \n");

        init_cnt_1++;
        //       count = t/4;
        init_cnt_2 = init_cnt_1/60;

        joint_data[0] = theta1 + (0 - theta1)*(0.0003*init_cnt_2);

        if(abs(joint_data[0]) <= 1)
        {
            joint_data[1] = theta2 + (0 - theta2)*(0.00006*init_cnt_2);
        }


        if(abs(joint_data[1]) <= 50)
        {
            init_cnt_3++;
            init_cnt_4 = init_cnt_3/60;


            joint_data[2] = theta3 + (0 - theta3)*(0.0002*init_cnt_4);
            joint_data[3] = theta4 + (0 - theta4)*(0.0002*init_cnt_4);
            joint_data[4] = theta5 + (0 - theta5)*(0.0002*init_cnt_4);

        }


        if(abs(joint_data[0]) <= 0.5 && abs(joint_data[1]) <= 0.5 && abs(joint_data[2]) <= 0.5 && abs(joint_data[3]) <= 0.5 && abs(joint_data[4]) <= 0.5)
        {
            Init = false;

            init_cnt_1 = 0;
            init_cnt_2 = 0;
            init_cnt_3 = 0;
            init_cnt_4 = 0;
        }
    }

}

void QNode::Motion_judgement(int mode)
{

    switch (mode)
    {
    if(mode!=999)
    {
        joint_data[0] = theta1;
        joint_data[1] = theta2;
        joint_data[2] = theta3;
        joint_data[3] = theta4;
        joint_data[4] = theta5;
    }

    case 0:
    {
        if(joint_data[2]<120)
        {
            joint_data[2] = joint_data[2] + 0.02;
        }
        if(joint_data[1]>-69 && joint_data[2]>90)
        {
            joint_data[1] = joint_data[1] - 0.02;
        }
        if(joint_data[4]<47 && joint_data[1]<-22)
        {
            joint_data[4] = joint_data[4] + 0.02;
        }
        if(joint_data[2]>=120 && joint_data[1]<=-69 && joint_data[4]>=47)
        {
            Mode = false;
            motion_mode = 999;
        }
    }break;

    case 1:
    {
        if(joint_data[2]<100)
        {
            joint_data[2] = joint_data[2] + 0.02;
        }
        if(joint_data[1]>-59 && joint_data[2]>40)
        {
            joint_data[1] = joint_data[1] - 0.02;
        }
        if(joint_data[4]<37 && joint_data[1]<-22)
        {
            joint_data[4] = joint_data[4] + 0.02;
        }
        if(joint_data[2]>=100 && joint_data[1]<=-59 && joint_data[4]>=37)
        {
            Mode = false;
            motion_mode = 999;
        }

    }break;

    case 2:
    {

    }break;

    case 3:
    {

    }break;

    default:
        break;
    }

}

void QNode::Mode_Change()
{

    if(button_L2==1&&button_R2==0)
    {
        gripper=-1;
    }
    else if(button_L2==0&&button_R2==1)
    {
        gripper=1;
    }
    else if(button_L2==1&&button_R2==1)
    {
        gripper=2;
    }
    else if(button_L2==0&&button_R2==0)
    {
        gripper=0;
    }

    if(Mode==true && abs(joint_data[0]) <= 10 && abs(joint_data[1]) <= 60 && abs(joint_data[2]) <= 30) //Singularity check
    {

        //ROS_WARN("this is a Singularity section \n");

        if(button_Share == 1)
        {
            //ROS_ERROR("mode change impossible \n");
        }
    }
    else
    {
        //ROS_INFO("mode change possible \n");

        if(button_Share == 1 && btn_share_once==true)
        {
            Mode=!Mode;
            btn_share_once=false;
            if(!Mode) joint_horizon = 0;
        }
        else if(button_Share == 0)
        {
            btn_share_once=true;
        }
    }


    if(button_Option == 1 && btn_option_once == true)
    {
        Init=!Init;
        btn_option_once = false;
    }
    else if(button_Option == 0)
    {
        btn_option_once=true;
    }

    if(button_Square == 1 && btn_square_once == true)
    {
        supplies_1=!supplies_1;
        btn_square_once = false;
    }
    else if(button_Square == 0)
    {
        btn_square_once=true;
    }

    if(button_O == 1 && btn_O_once == true)
    {
        supplies_2=!supplies_2;
        btn_O_once = false;
    }
    else if(button_O == 0)
    {
        btn_O_once=true;
    }

}

void QNode::Notice()
{
    //    if(button_O == 1)
    //    {
    //        cout << "button_O clicked" << endl;
    //    }

    //    if(button_X == 1)
    //    {
    //        cout << "button_X clicked" << endl;
    //    }

    //    if(button_Square == 1)
    //    {
    //        cout << "button_Square clicked" << endl;
    //    }

    //    if(button_Triangle == 1)
    //    {
    //        cout << "button_Triangle clicked" << endl;
    //    }

    //    if(button_Triangle == 1)
    //    {
    //        cout << "button_Triangle clicked" << endl;
    //    }

    //    if(button_L1 == 1)
    //    {
    //        cout << "button_L1 clicked" << endl;
    //    }

    //    if(button_R1 == 1)
    //    {
    //        cout << "button_R1 clicked" << endl;
    //    }

    //    if(button_left_right == 1)
    //    {
    //        cout << "button_Left clicked" << endl;
    //    }

    //    if(button_left_right == -1)
    //    {
    //        cout << "button_Right clicked" << endl;
    //    }

    //    if(button_up_down == 1)
    //    {
    //        cout << "button_Up clicked" << endl;
    //    }

    //    if(button_up_down == -1)
    //    {
    //        cout << "button_Down clicked" << endl;
    //    }

    //    if(button_Share == 1)
    //    {
    //        cout << "button_Share clicked" << endl;
    //    }

    //    if(button_Option == 1)
    //    {
    //        cout << "button_Option clicked" << endl;
    //    }
}

void QNode::Inverse_kinematics()
{
    double r, s, D;

    float last_joint_data0 = joint_data[0];
    r = sqrt(pow(f_x+cur_X,2) + pow(f_y+cur_Y,2));
    s = f_z+cur_Z - L1;
    D = (pow(r,2) + pow(s,2) - pow(L2,2) - pow(L3,2))/(2 * L2 * L3);

    ik_theta[0] = atan2(f_y+cur_Y,f_x+cur_X);
    ik_theta[2] = atan2(-sqrt(1 - pow(D,2)), D);
    ik_theta[1] = atan2(s,r) - atan2(L3 * sin(ik_theta[2]), L2 + L3 * cos(ik_theta[2]));

    ik_theta[0] = ik_theta[0] * 180/PI;
    ik_theta[1] = ik_theta[1] * 180/PI;
    ik_theta[2] = ik_theta[2] * 180/PI;

    joint_data[0] = ik_theta[0];
    joint_data[1] = ik_theta[1] - 180;
    //   joint_data[1] = ik_theta[1];
    joint_data[2] = ik_theta[2] + 180;
    //   joint_data[2] = ik_theta[2];
    cur_X = 0;
    cur_Y = 0;
    cur_Z = 0;

    if(camera_aspect)
    {
        joint_data[0] = last_joint_data0;
    }
    camera_aspect=false;

}

void QNode::manipulation1()
{
    int in_90_deg;
    if(joint_horizon<=90 && joint_horizon>=-90)
    {
        in_90_deg=1;
    }
    else
    {
        in_90_deg=-1;
    }


    if(stick_Y_Axis >0)
    {
        cur_Z += 0.2*in_90_deg;
        Equation_upline(stick_Y_Axis);
    }

    if(stick_Y_Axis <0)
    {
        cur_Z -= 0.2*in_90_deg;
        Equation_upline(stick_Y_Axis);
    }


    if(button_L1 == 1)
    {
        if(theta1>=-90 &&theta1<=90)
        {
            cur_X += 0.2*in_90_deg;
        }
        else
        {
            cur_X -= 0.2*in_90_deg;
        }


        Equation_straightline_xy();
        Equation_straightline_z(0.5);
    }

    if(button_R1 == 1)
    {
        if(theta1>=-90 &&theta1<=90)
        {
            cur_X -= 0.2*in_90_deg;
        }
        else
        {
            cur_X += 0.2*in_90_deg;
        }

        Equation_straightline_xy();
        Equation_straightline_z(0.5);
    }

    if(button_up_down != 0)
    {
        cur_Z += 0.1*button_up_down;
    }



}

void QNode::manipulation2()
{

    if(stick_X_Axis == 1)
    {
        //cur_Y += 0.05;
        joint_data[0] += 0.07;
    }

    if(stick_X_Axis == -1)
    {
        //cur_Y -= 0.05;
        joint_data[0] -= 0.07;
    }



    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if(button_Square == 1)
    {
        if(joint_data[3] <= 182 && joint_data[3] > -180)
        {
            joint_data[3] -= 0.05;
        }
    }

    if(button_O == 1)
    {
        if(joint_data[3] >= -182 && joint_data[3] < 180)
        {
            joint_data[3] += 0.05;
        }
    }


    if(stick_Z_Axis!=0)
    {
        if(joint_data[4] <= 182 && joint_data[4] > -180)
        {
            joint_horizon -= stick_Z_Axis*0.03;
        }
    }


    //horizontal maintenance
    joint_data[4] = (theta3 - abs(theta2)) + joint_horizon;

}


void QNode::Macro()
{
    //joint 5,6 control


    if(abs(theta1) < 0.5 && abs(theta2) < 0.5 && abs(theta3) < 0.5 && abs(theta4) < 0.5 && abs(theta5) < 0.5 )
    {
        //NULL
    }
    else
    {

        if(button_Square == 1)
        {
            if(joint_data[3] <= 182 && joint_data[3] > -180)
            {
                joint_data[3] -= 0.05;
            }

        }

        if(button_O == 1)
        {
            if(joint_data[3] >= -182 && joint_data[3] < 180)
            {
                joint_data[3] += 0.05;
            }
        }

        //        if(button_X == 1)
        //        {
        //            if(joint_data[4] >= -182 && joint_data[4] < 180)
        //            {
        //                joint_data[4] += 0.05;
        //            }
        //        }

        if(button_Triangle == 1)
        {
            motion_mode = 0;
        }
        if(stick_Z_Axis!=0)
        {
            if(joint_data[4] <= 182 && joint_data[4] > -180)
            {
                joint_data[4] -= stick_Z_Axis*0.03;
            }
        }

        //        if(stick_Z_Axis==-1)
        //        {
        //            if(joint_data[4] <= 182 && joint_data[4] > -180)
        //            {
        //                joint_data[4] -= 0.05;
        //            }
        //        }

    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if(button_up_down == 1) // grab_Posture
    {
        t++;
        //     count = t/16;
        count = t/80;

        joint_data[0] = theta1 + (0 - theta1)*(0.00005*count);
        joint_data[1] = theta2 + (-120 - theta2)*(0.00005*count);
        joint_data[2] = theta3 + (60 - theta3)*(0.00005*count);
        joint_data[3] = theta4 + (0 - theta4)*(0.00005*count);
        joint_data[4] = theta5 + (-60 - theta5)*(0.00005*count);

        //cout << "t : " << t << endl;

    }
    else if(button_up_down == -1) // Init_Posture
    {

        t++;
        //       count = t/4;
        count = t/60;

        joint_data[0] = theta1 + (0 - theta1)*(0.00005*count);
        joint_data[1] = theta2 + (0 - theta2)*(0.00005*count);
        joint_data[2] = theta3 + (0 - theta3)*(0.00005*count);
        joint_data[3] = theta4 + (0 - theta4)*(0.00005*count);
        joint_data[4] = theta5 + (0 - theta5)*(0.00005*count);

        //cout << "t : " << t << endl;

    }
    else
    {
        t = 0;
    }

}



void QNode::Foward_Kinematics()
{

    Matrix4f T01;
    Matrix4f T12;
    Matrix4f T23;
    Matrix4f T34;
    Matrix4f T45;
    Matrix4f T56;
    Matrix4f T67;

    Matrix4f T02;
    Matrix4f T03;
    Matrix4f T04;
    Matrix4f T05;
    Matrix4f T06;

    //my code
    T01 = Rz(deg2rad(theta1))*Tz(L1)*Tx(0)*Rx(PI/2);
    T12 = Rz(deg2rad(theta2) - PI)*Tz(0)*Tx(L2)*Rx(0);
    T23 = Rz(deg2rad(theta3) + PI)*Tz(0)*Tx(L3)*Rx(PI/2);

    //T34 = Rz(deg2rad(theta4))*Tz(L3)*Tx(0)*Rx(-PI/2);
    //T45 = Rz(deg2rad(theta5))*Tz(0)*Tx(0)*Rx(PI/2);

    //   T34 = (Rz(theta4)*Tz(L3)*Tx(0)*Rx(-PI/2));
    //   T45 = (Rz(theta5)*Tz(0)*Tx(0)*Rx(PI/2));
    //   T56 = (Rz(theta6)*Tz(L6)*Tx(0)*Rx(0));

    T03 = T01*T12*T23;
    //   T06 = T03*T34*T45*T56*T67;

    // macro mode End effector coordinate
    f_x = T03(0,3);
    f_y = T03(1,3);
    f_z = T03(2,3);


    //   cout << "T12 : \n"<< T12 << "\n" << endl;
    //   cout << "T23 : \n"<< T23 << "\n" << endl;
    //   cout << "T03 : \n"<< T03 << "\n" << endl;


    //    cout << "f_x :" << f_x << endl;
    //    cout << "f_y :" << f_y << endl;
    //    cout << "f_z :" << f_z << "\n" <<endl;



}

void QNode::Trajectory()
{

    //////////////////////// Trajectory base code /////////////////////////////

    tra_X = cur_X + (tar_X - cur_X)*(0.001*t);
    tra_Y = cur_Y + (tar_Y - cur_Y)*(0.001*t);
    tra_Z = cur_Z + (tar_Z - cur_Z)*(0.001*t);

    tra_alpha = cur_alpha + (tar_alpha - cur_alpha)*(0.001*t);
    tra_beta = cur_beta + (tar_beta - cur_beta)*(0.001*t);
    tra_gamma = cur_gamma + (tar_gamma - cur_gamma)*(0.001*t);

    //      if(Kinematics(tra_X, tra_Y, tra_Z, tra_alpha, tra_beta, tra_gamma) == 1)
    //      {
    //        if((abs(tar_X - tra_X) < 0.1) && (abs(tar_Y - tra_Y) < 0.1) && (abs(tar_Z - tra_Z) < 0.1)
    //                 && (abs(tar_alpha - qq5qqqqqqqq
    //        else {
    //            t++;
    //            count = t/4;
    //        }

    //        joint_data[0] = rad2deg(th[0]);
    //        joint_data[1] = rad2deg(th[1]);
    //        joint_data[2] = rad2deg(th[2]);
    //        joint_data[3] = rad2deg(th[3]);
    //        joint_data[4] = rad2deg(th[4]);
    //        joint_data[5] = rad2deg(th[5]);
    //        cur_alpha = tra_alpha;
    //        cur_beta = tra_beta;
    //        cur_gamma = tra_gamma;
    //        cur_X = tra_X;
    //        cur_Y = tra_Y;
    //        cur_Z = tra_Z;
    //    }
    //    else
    //    {
    ////original_code
    //        tar_X = 500;
    //        tar_Y = 0;
    //        tar_Z = 400;

    //        t = 0;
    //    }
    //    //cout << "t : " << t << endl;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

//int QNode::Kinematics(float pX, float pY, float pZ, float rX, float rY, float rZ)
//{
//  float pX_4, pY_4, pZ_4, X_4;
////  float D, joint_err;
//  Matrix3d Rotation_x, Rotation_y, Rotation_z;
//  Matrix3d R0_1, R1_2, R2_3, R3_4, R4_5, R5_6;
//  Matrix3d R0_6, R0_3, R3_6;

//  Matrix4f T01;
//  Matrix4f T12;
//  Matrix4f T23;
//  Matrix4f T34;
//  Matrix4f T45;
//  Matrix4f T56;
//  Matrix4f T67;

//  Matrix4f T02;
//  Matrix4f T03;
//  Matrix4f T04;
//  Matrix4f T05;
//  Matrix4f T06;

//  // deg to rad
//  rX = deg2rad(rX);
//  rY = deg2rad(rY);
//  rZ = deg2rad(rZ);

//  //////////// Inverse Kinematics /////////////////
//  // set Orientation
//  Rotation_x << 1,       0,        0,
//                0, cos(rX), -sin(rX),
//                0, sin(rX),  cos(rX);

//  Rotation_y << cos(rY), 0, sin(rY),
//                      0, 1,       0,
//               -sin(rY), 0, cos(rY);

//  Rotation_z << cos(rZ), -sin(rZ), 0,
//                sin(rZ),  cos(rZ), 0,
//                      0,        0, 1;

//  R0_6 = (Rotation_x*Rotation_y)*Rotation_z;

//  // Solve inverse kinematics equation


////  pX_4 = pX - L4*R0_6(0,2);
////  pY_4 = pY - L4*R0_6(1,2);
////  pZ_4 = pZ - L4*R0_6(2,2);
////  X_4 = sqrt(pow(pX_4,2) + pow(pY_4,2));

////  th[0] = atan2(pY_4, pX_4);

////  D = (pow(pX_4,2) + pow(pY_4,2) + pow((pZ_4-L1),2) - pow(L2,2) - pow(L3,2))/(2*L2*L3);
////  th[2] = atan2(-sqrt(1 - pow(D,2)), D);
////  th[1] = atan2(pZ_4 - L1, sqrt(pow(pX_4,2) + pow(pY_4,2))) - atan2(L3*sin(th[2]), L2 + L3*cos(th[2]));

//  // R0_1 = Rz(th[0])*Rx(PI/2)
//  R0_1 << cos(th[0]), 0, sin(th[0]),
//          sin(th[0]), 0, -cos(th[0]),
//                   0, 1, 0;
//  // R1_2 = Rz(th[1]+PI/2)
//  R1_2 << cos(th[1]+PI/2), -sin(th[1]+PI/2),  0,
//          sin(th[1]+PI/2), cos(th[1]+PI/2), 0,
//          0,  0,  1;

//  // R2_3 = Rz(th[2])*Rx(PI/2)
//  R2_3 << cos(th[2]), 0, sin(th[2]),
//          sin(th[2]), 0, -cos(th[2]),
//                  0, 1, 0;

//  // R3_4 = Rz(th3)*Rx(-PI/2)
//  R3_4 << cos(th[3]), 0, -sin(th[3]),
//         sin(th[3]), 0, cos(th[3]),
//      0, -1, 0;
//  // R4_5 = Rz(th4)*Rx(PI/2)
//  R4_5 << cos(th[4]), 0, sin(th[4]),
//      sin(th[4]), 0, -cos(th[4]),
//      0, 1, 0;
//  // R5_6 = Rx(th5)
//  R5_6 << cos(th[5]), -sin(th[5]), 0,
//      sin(th[5]), cos(th[5]), 0,
//   0, 0, 1;

//  R0_3 = (R0_1*R1_2)*R2_3;
//  R3_6 = R0_3.inverse() * R0_6;

//  th[4] = atan2(sqrt(1-pow(R3_6(2,2),2)),R3_6(2,2));
//  //th[4] = acos(R3_6(2,2));
//  th[3] = atan2(R3_6(1,2), R3_6(0,2));
//  th[5] = atan2(R3_6(2,1), (-R3_6(2,0)));

//   //////////// Forward Kinematics /////////////////
//   T01 = (Rx(0)*Tx(0))*(Rz(th[0])*Tz(L1));
//   T12 = (Rx(PI/2)*Tx(0))*(Rz(th[1])*Tz(0));
//   T23 = (Rx(0)*Tx(L2))*(Rz(th[2] + (PI/2))*Tz(0));
//   T34 = (Rx(PI/2)*Tx(0))*(Rz(th[3])*Tz(L3));
//   T45 = (Rx(-PI/2)*Tx(0))*(Rz(th[4])*Tz(0));
//   T56 = (Rx(PI/2)*Tx(0))*(Rz(th[5])*Tz(0));
//   T67 = (Rx(0)*Tx(0))*(Rz(0)*Tz(L4));

//   T03 = T01*T12*T23;
//   T06 = T03*T34*T45*T56*T67;

//   //cout << "before th3 : " << th[3] << endl;




//   // End effector coordinate
//   f_x = T06(0,3);
//   f_y = T06(1,3);
//   f_z = T06(2,3);
////      cout << "th3 : "<< rad2deg(th[3]) << endl;
////     if(roundf(rad2deg(th[5])) == 0 || roundf(rad2deg(th[5])) == 180 || roundf(rad2deg(th[5])) == -180)
////     {
////         cout << "th[5]" << endl;
////         th[5] = 0.0;
////     }
////     else
////     {
////         th[3] = atan2(R3_6(1,2), R3_6(0,2));
////         th[5] = atan2(R3_6(2,1), (-R3_6(2,0)));
////     }
///////////////////////////////////////////////////////////////////////////////////////////end kinematics
//   // joint limit check
//   for(int i = 0 ; i<6 ; i++)
//   {
//     if(th[i] > th_max[i])
//     {
//       th[i] = th_max[i];
//     }
//     else if(th[i] < th_min[i])
//     {
//       th[i] = th_min[i];
//     }
//   }
//   if(roundf(rad2deg(th[3])) == 180 || roundf(rad2deg(th[3])) == -180)
//   {
//       th[3] = 0.0;
//       th[4] *= -1;
//   }
//   if(roundf(rad2deg(th[5])) > 0)
//   {
//       th[5] -= 3.14159;
//   }
//   else
//   {
//       th[5] += 3.14159;
//   }
//      // cout << "th5 : " << th[5] << endl;

//     if(check(X_4, pZ_4, T03(2,3)) == 1)
//     {
//        // stabe
//     }
//     else if(check(X_4, pZ_4, T03(2,3)) == 2)
//     {
//       // out of workspace
//     }
//     else
//     {
//       //cout << "state = " << state << endl;
//     }

////cout << "th3 " << rad2deg(th[3]) << " th4 " << rad2deg(th[4]) << " th5 " << rad2deg(th[5]) << endl;
//     return check(X_4, pZ_4, T03(2,3));

// if(isnan(th[0]) || isnan(th[1]) || isnan(th[2]) || isnan(th[3]) || isnan(th[4]) || isnan(th[5]))
// {
//   th[0] = 0;
//   th[1] = 0;
//   th[2] = 0;
//   th[3] = 0;
//   th[4] = 0;
//   th[5] = 0;
//   joint_data[0] = rad2deg(th[0]);
//   joint_data[1] = rad2deg(th[1]);
//   joint_data[2] = rad2deg(th[2]);
//   joint_data[3] = rad2deg(th[3]);
//   joint_data[4] = rad2deg(th[4]);
//   joint_data[5] = rad2deg(th[5]);
//   cout << "nan" << endl;
// }

//}


//int QNode::check(float x, float  z, float z_coordiante)
//{
//  if(x > 50) // 1. X > 50
//  {
//      if(z_coordiante > -95) // 2. Z > -95
//      {
//          if((pow(x,2) + pow((z - L1),2)) <= pow((L2 + L3),2)) // 3. R < 890
//          {
//            if((pow(x,2) + pow((z - L1),2)) >= pow(266,2)) // 4. R > 266
//            {
//              state = 1; // go to trajectory
//            }
//            else // 4. R < 266
//            {
//              flag = 12; // too close
//              state = 0;
//              //cout << "R : " << (pow(x,2) + pow((z - L1),2)) << endl;
//            }
//          }
//          else // 3. R > 890
//          {
//            flag = 13; // Too far
//            state = 0;
//            //cout << "R : " << (pow(x,2) + pow((z - L1),2)) << endl;
//          }
//      }
//      else // 2. Z < -95
//      {
//        flag = 11; // To low
//        state = 0;
//        cout << "z : " << z_coordiante << endl;
//      }
//  }
//  else // 1. X < 50
//  {
//    flag = 10; // X limit
//    state = 0;
//    cout << "x : " << x << endl;
//  }
//  return state;
//}


//--------------- Rotation Matrix ----------------//

//input radian!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//

MatrixXf QNode::Rx(float theta)
{
    MatrixXf rx(4,4);
    rx << 1, 0, 0, 0,
            0, cos(theta), -sin(theta), 0,
            0, sin(theta), cos(theta), 0,
            0, 0, 0, 1;
    return rx;
};

MatrixXf QNode::Ry(float theta)
{
    MatrixXf ry(4,4);
    ry << cos(theta), 0, sin(theta), 0,
            0, 1, 0, 0,
            -sin(theta), 0, cos(theta), 0,
            0, 0, 0, 1;
    return ry;
};

MatrixXf QNode::Rz(float theta)
{
    MatrixXf rz(4,4);

    rz << cos(theta), -sin(theta), 0, 0,
            sin(theta), cos(theta), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    return rz;
};

MatrixXf QNode::Tz(float l)
{
    MatrixXf tz(4,4);
    tz << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, l,
            0, 0, 0, 1;
    return tz;
};

MatrixXf QNode::Ty(float l)
{
    MatrixXf ty(4,4);
    ty << 1, 0, 0, 0,
            0, 1, 0, l,
            0, 0, 1, 0,
            0, 0, 0, 1;
    return ty;
};


MatrixXf QNode::Tx(float l)
{
    MatrixXf tx(4,4);
    tx << 1, 0, 0, l,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    return tx;
};

void QNode::Joystick(const sensor_msgs::Joy::ConstPtr &joy)
{

    //  this->joy_X = joy->axes[1]; // get joystick x-axis
    //  this->joy_Y = joy->axes[0]; // get joystick y-axis
    //  this->joy_Z = joy->axes[5]; // get joystick z-axis

    //Wire
    this->button_X = joy->buttons[0];
    this->button_O = joy->buttons[1];
    this->button_Triangle = joy->buttons[2];
    this->button_Square = joy->buttons[3];
    this->button_left_right = joy->axes[6];
    this->button_up_down = joy->axes[7];

    this->button_L1 = joy->buttons[4];
    this->button_R1 = joy->buttons[5];

    this->button_L2 = joy->buttons[6];
    this->button_R2 = joy->buttons[7];


    this->button_Share = joy->buttons[8];
    this->button_Option = joy->buttons[9];


    this->stick_X_Axis = joy->axes[0];
    this->stick_Y_Axis = joy->axes[1];
    this->stick_Z_Axis = joy->axes[4];



    //  if(Ori_button == 1)
    //  {
    //    tar_alpha += joy_X*0.1;
    //    tar_beta += joy_Y*0.1;
    //    tar_gamma += joy_Z*0.1;

    ////    alpha = tra_alpha;
    ////    beta = tra_beta;
    ////    gamma = tra_gamma;

    ////      cout << "tar_alpha : " << tar_alpha << endl;
    ////      cout << "tar_beta : " << tar_beta << endl;
    ////      cout << "tar_gamma : " << tar_gamma << endl;
    //  }
    //  else
    //  {
    //    past_X = tra_X;
    //    past_Y = tra_Y;
    //    past_Z = tra_Z;

    //    tar_X += joy_X*0.1;
    //    tar_Y += joy_Y*0.1;
    //    tar_Z += joy_Z*0.1;
    //  }
    //  if(buttonSq == 1)
    //  {
    //    tar_X = 250;
    //    tar_Y = 0;
    //    tar_Z = 600;
    //    tar_alpha = 0;
    //    tar_beta = 90;
    //    tar_gamma = 0;
    //  }
    //  if(buttonX == 1)
    //  {
    //    tar_X = 690;
    //    tar_Y = 0;
    //    tar_Z = 529;
    //    tar_alpha = 0;
    //    tar_beta = 90;
    //    tar_gamma = 0;
    //  }
    //  if(buttonO == 1)
    //  {
    //    tar_X = 300;
    //    tar_Y = 0;
    //    tar_Z = -100;
    //    tar_alpha = 0;
    //    tar_beta = 180;
    //    tar_gamma = 0;
    //  }
    //  if(buttonTr == 1)
    //  {
    //    tar_X = 300;
    //    tar_Y = 0;
    //    tar_Z = 350;
    //    tar_alpha = 0;
    //    tar_beta = 180;
    //    tar_gamma = 0;
    //  }


}


}  // namespace manipulator_22
