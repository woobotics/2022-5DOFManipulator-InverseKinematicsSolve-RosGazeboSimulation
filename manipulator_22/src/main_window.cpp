/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/manipulator_22/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace manipulator_22 {

using namespace Qt;
using namespace std;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
{
  ui.setupUi(this);
  qnode.init();
  QTimer *timer = new QTimer(this);
  timer->start(1000);
  QObject::connect(&qnode, SIGNAL(dataChanged()), this, SLOT(update_Data()));
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

}

void MainWindow::update_Data(void)
{
  ui.Cur_Theta1->setText(QString::number((float)qnode.theta1));
  ui.Cur_Theta2->setText(QString::number((float)qnode.theta2));
  ui.Cur_Theta3->setText(QString::number((float)qnode.theta3));
  ui.Cur_Theta4->setText(QString::number((float)qnode.theta4));
  ui.Cur_Theta5->setText(QString::number((float)qnode.theta5));
  ui.Cur_Theta6->setText(QString::number((float)qnode.theta6));

  ui.Cur_X->setText(QString::number((float)qnode.cur_X));
  ui.Cur_Y->setText(QString::number((float)qnode.cur_Y));
  ui.Cur_Z->setText(QString::number((float)qnode.cur_Z));
}

MainWindow::~MainWindow() {}


}  // namespace manipulator_22


void manipulator_22::MainWindow::on_btn_init_clicked()
{
  qnode.joint_data[0] = 0;
  qnode.joint_data[1] = 0;
  qnode.joint_data[2] = 0;
  qnode.joint_data[3] = 0;
  qnode.joint_data[4] = 0;
  qnode.joint_data[5] = 0;

}

void manipulator_22::MainWindow::on_btn_grab_clicked()
{
  qnode.joint_data[0] = 0;
  qnode.joint_data[1] = -90;
  qnode.joint_data[2] = 90;
  qnode.joint_data[3] = 0;
  qnode.joint_data[4] = 90;
  qnode.joint_data[5] = 0;

}

void manipulator_22::MainWindow::on_pushButton_clicked()
{
  qnode.joint_data[0] -= 20;
}

void manipulator_22::MainWindow::on_pushButton_2_clicked()
{
  qnode.joint_data[0] += 20;
}

////////////////////Foward Kinematics Manipulation////////////////////

void manipulator_22::MainWindow::on_pushButton_Theta1_plus_clicked()
{

  if(qnode.joint_data[0] >= -180 && qnode.joint_data[0] < 180)
  {
    qnode.joint_data[0] += 20;
  }

}

void manipulator_22::MainWindow::on_pushButton_Theta2_plus_clicked()
{
  if(qnode.joint_data[1] >= -180 && qnode.joint_data[1] < 180)
  {
    qnode.joint_data[1] += 20;
  }

}

void manipulator_22::MainWindow::on_pushButton_Theta3_plus_clicked()
{
  if(qnode.joint_data[2] >= -180 && qnode.joint_data[2] < 180)
  {
    qnode.joint_data[2] += 20;
  }

}

void manipulator_22::MainWindow::on_pushButton_Theta4_plus_clicked()
{
  if(qnode.joint_data[3] >= -180 && qnode.joint_data[3] < 180)
  {
    qnode.joint_data[3] += 20;
  }

}

void manipulator_22::MainWindow::on_pushButton_Theta5_plus_clicked()
{
  if(qnode.joint_data[4] >= -180 && qnode.joint_data[4] < 180)
  {
    qnode.joint_data[4] += 20;
  }

}

void manipulator_22::MainWindow::on_pushButton_Theta6_plus_clicked()
{
  if(qnode.joint_data[5] >= -180 && qnode.joint_data[5] < 180)
  {
    qnode.joint_data[5] += 20;
  }

}

void manipulator_22::MainWindow::on_pushButton_Theta1_minus_clicked()
{

  if(qnode.joint_data[0] > -180 && qnode.joint_data[0] <= 180)
  {
    qnode.joint_data[0] -= 20;
  }
}

void manipulator_22::MainWindow::on_pushButton_Theta2_minus_clicked()
{
  if(qnode.joint_data[1] > -180 && qnode.joint_data[1] <= 180)
  {
    qnode.joint_data[1] -= 20;
  }
}

void manipulator_22::MainWindow::on_pushButton_Theta3_minus_clicked()
{
  if(qnode.joint_data[2] > -180 && qnode.joint_data[2] <= 180)
  {
    qnode.joint_data[2] -= 20;
  }
}

void manipulator_22::MainWindow::on_pushButton_Theta4_minus_clicked()
{
  if(qnode.joint_data[3] > -180 && qnode.joint_data[3] <= 180)
  {
    qnode.joint_data[3] -= 20;
  }
}

void manipulator_22::MainWindow::on_pushButton_Thetar5_minus_clicked()
{
  if(qnode.joint_data[4] > -180 && qnode.joint_data[4] <= 180)
  {
    qnode.joint_data[4] -= 20;
  }
}

void manipulator_22::MainWindow::on_pushButton_Theta6_minus_clicked()
{
  if(qnode.joint_data[5] > -180 && qnode.joint_data[5] <= 180)
  {
    qnode.joint_data[5] -= 20;
  }
}


void manipulator_22::MainWindow::on_pushButton_X_plus_clicked()
{
  qnode.cur_X += 10;
}

void manipulator_22::MainWindow::on_pushButton_Y_plus_clicked()
{
  qnode.cur_Y += 10;
}

void manipulator_22::MainWindow::on_pushButton_Z_plus_clicked()
{
   qnode.cur_Z += 10;
}

void manipulator_22::MainWindow::on_pushButton_X_minus_clicked()
{
   qnode.cur_X -= 10;
}

void manipulator_22::MainWindow::on_pushButton_Y_minus_clicked()
{
   qnode.cur_Y -= 10;
}

void manipulator_22::MainWindow::on_pushButton_Z_minus_clicked()
{
   qnode.cur_Z -= 10;
}


