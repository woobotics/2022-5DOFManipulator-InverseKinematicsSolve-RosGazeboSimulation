/**
 * @file /include/manipulator_22/main_window.hpp
 *
 * @brief Qt based gui for manipulator_22.
 *
 * @date November 2010
 **/
#ifndef manipulator_22_MAIN_WINDOW_H
#define manipulator_22_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace manipulator_22 {
using namespace Qt;
using namespace std;

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
  float p_x, p_y, p_z, r_x, r_y, r_z;
  int timer1 = 0;
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();



public Q_SLOTS:

  void update_Data(void);
  void on_btn_init_clicked();
  void on_btn_grab_clicked();
  void on_pushButton_clicked();
  void on_pushButton_2_clicked();

//Foward Kinematics Manipulation

  void on_pushButton_Theta1_plus_clicked();
  void on_pushButton_Theta2_plus_clicked();
  void on_pushButton_Theta3_plus_clicked();
  void on_pushButton_Theta4_plus_clicked();
  void on_pushButton_Theta5_plus_clicked();
  void on_pushButton_Theta6_plus_clicked();

  void on_pushButton_Theta1_minus_clicked();
  void on_pushButton_Theta2_minus_clicked();
  void on_pushButton_Theta3_minus_clicked();
  void on_pushButton_Theta4_minus_clicked();
  void on_pushButton_Thetar5_minus_clicked();
  void on_pushButton_Theta6_minus_clicked();

//Inverse Kinematics Manipulation

  void on_pushButton_Z_minus_clicked();
  void on_pushButton_Y_minus_clicked();
  void on_pushButton_X_minus_clicked();
  void on_pushButton_Z_plus_clicked();
  void on_pushButton_Y_plus_clicked();
  void on_pushButton_X_plus_clicked();


private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace manipulator_22

#endif // manipulator_22_MAIN_WINDOW_H
