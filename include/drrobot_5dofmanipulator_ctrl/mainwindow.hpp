#ifndef drrobot_5dofmanipulator_ctrl_MAIN_WINDOW_H
#define drrobot_5dofmanipulator_ctrl_MAIN_WINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QtNetwork>
#include <QTimer>
#include <QValidator>
#include "../include/drrobot_5dofmanipulator_ctrl/ui_mainwindow.h"
#include "../include/drrobot_5dofmanipulator_ctrl/qnode.hpp"
namespace drrobot_5dofmanipulator_ctrl {


	class MainWindow : public QMainWindow
	{
		 Q_OBJECT

	public:
		 explicit MainWindow(int argc, char** argv,QWidget *parent = 0);
		 ~MainWindow();
		 void closeEvent(QCloseEvent *event); // Overloaded function

	private:
		 Ui::MainWindow *ui;
		 struct MotorDriverData
		    {

		        double motAmp1;     // motor1 current value
		        double motAmp2;     // motor2 current value
		        int motPower1;      //motor1 output power -1000 ~ +1000
		        int motPower2;      //motor2 output power -1000 ~ +1000
		        int motEncP1;       //motor1 encoder position value
		        int motEncP2;       //motor2 encoder position value
		        int motEncS1;       //motor1 encoder velocity value
		        int motEncS2;       //motor2 encoder velocity value
		        double motTemp1;    //motor1 temperature value
		        double motTemp2;    //motor2 temperature value
		        double drvVoltage;  //driver board voltage, around 12V
		        double batVoltage;  // main power battery voltage
		        double reg5VVoltage;//driver broad 5V
		        double ai3;         // A/D channel 3 raw A/D value, full range is 4095, will translate to motor 1temperature value
		        double ai4;         // A/D channel 4 raw A/D value, full range is 4095, will translate to motor2 temperature value
		        double intTemp;     // on driver baord temperature sensor reading
		        double ch1Temp;     // channel 1 driver chip temperature
		        double ch2Temp;     // channel 2 driver chip temperature
		        int statusFlag ;     // motor driver baord status
		        int mode1;          // channel 1 working mode, 0--open loop, 1 -- close speed control, 2,3 -position control, 4-torgue control
		        int mode2;
		     };
		 struct JointMotorData
		 {
		     int pwmOutput;
		     int encodeSpeed ;
		     int encoderPos;
		     int encoderDir ;
		     double circleCnt;
		     double resolution;
		     bool protect;
		     double angle;
		     int iniPos;
		     int limitPos1;
		     int limitPos2;
		     bool stuckFlag;
		     int cmdDir;
		     int preCmdDir;
		     int manipulatorStuckCnt;
		     int jointCmd;
		     bool jointJoy;
		     bool jointCtrl;
		     double motorTemperature;
		     double currentAmp;
		 };
		    JointMotorData jointMotorData[6];
		    QTcpSocket *tcpRobot1;
		    QTcpSocket *tcpRobot2;
		    QTcpSocket * tcpRobot3;
		    QTcpSocket * tcpRobot4;
		    QTimer pingTimer;
		    QString receivedData1;
		    QString receivedData2;
		    QString receivedData3;
		    void dealWithPackage1( QString received);
		    void dealWithPackage2( QString received);
		    void dealWithPackage3( QString received);

		    MotorDriverData motorData1;
		    MotorDriverData motorData2;
		    MotorDriverData motorData3;
		    double ad2Temperature(int value);
		    int watchDogCnt1;
		    int watchDogCnt2;
		    double trans2Angle(int channel);
		    double manipulatorPos[3];
		    void getPositionXY();
		    double tipAngle;
		    bool armIniFlag;
			 QNode qnode;
	public slots:
		 /******************************************
		 ** Manual connections
		 *******************************************/
		 void cmdSend(int channel,int cmdValue,int motorCtrl);
		 void jointAngleCmdSend(int channel,double angle);

		 void armPosCmdSend(double x, double y, double tipAngle);
		 /******************************************
		 ** GUI Own slot
		 *******************************************/
		 void connectToRobot1();
		 void connectToRobot2();
		 void processRobotData1();
		 void processRobotData2();
		 void processRobotData3();
		 void sendPing();
		 void motor1Up();
		 void motor1Down();
		 void motor1Stop();

		 void motor2Up();
		 void motor2Down();
		 void motor2Stop();

		 void motor3Up();
		 void motor3Down();
		 void motor3Stop();

		 void motor4Up();
		 void motor4Down();
		 void motor4Stop();

		 void motor5Up();
		 void motor5Down();
		 void motor5Stop();

		 void motor6Up();
		 void motor6Down();
		 void motor6Stop();

		 void motorStopAll();
		 void sendQuery1();
		 void sendQuery2();
		 void sendQuery3();
		 void sendQuery4();
		 void j1GoCmd();
		 void j2GoCmd();
		 void rotateGoCmd();
		 void tipGoCmd();
		 void panGoCmd();
		 void armPosGoCmd();
		 void armResetCmd();
		 void armSetIniCmd();

	};
	class IP4Validator : public QValidator
	{
		 public:
		     IP4Validator(QObject *parent=0) : QValidator(parent){}
		     void fixup(QString &input) const {}
		     State validate(QString &input, int &pos) const
		     {
		         if(input.isEmpty())
		             return Acceptable;
		         QStringList slist = input.split(".");
		         int s = slist.size();
		         if(s>4) return Invalid;
		         bool emptyGroup = false;
		         for(int i=0;i<s;i++)
		         {
		             bool ok;
		             if(slist[i].isEmpty())
		             {
		                 emptyGroup = true;
		                 continue;
		             }
		             int val = slist[i].toInt(&ok);
		             if(!ok || val<0 || val>255)
		                 return Invalid;
		         }
		         if(s<4 || emptyGroup)
		             return Intermediate;

		         return Acceptable;
		         }
	};
}
#endif // MAINWINDOW_H
