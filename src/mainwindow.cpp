#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/drrobot_5dofmanipulator_ctrl/mainwindow.hpp"
#include "../include/drrobot_5dofmanipulator_ctrl/ui_mainwindow.h"

#include "qmath.h"
namespace drrobot_5dofmanipulator_ctrl {

using namespace Qt;
#define MOTOR_NUM 6


#define J1_UP_CMD       "!PR 1 30\r"
#define J1_DOWN_CMD     "!PR 1 -30\r"
#define J1_CMD 30
#define J2_UP_CMD       "!PR 2 -75\r"
#define J2_DOWN_CMD     "!PR 2 75\r"
#define J2_CMD 75
#define J3_UP_CMD       "!PR 1 10\r"
#define J3_DOWN_CMD     "!PR 1 -10\r"
#define J3_CMD 30
#define J4_UP_CMD       "!G 2 250\r"
#define J4_DOWN_CMD     "!G 2 -250\r"
#define J4_CMD 250
#define J5_UP_CMD       "!PR 1 30\r"
#define J5_DOWN_CMD     "!PR 1 -30\r"
#define J5_CMD 30
#define J6_UP_CMD       "!PR 2 10\r"
#define J6_DOWN_CMD     "!PR 2 -10\r"
#define J6_CMD 10

const double J1_OFFSET = 0.0;   // 0,
const double J2_OFFSET = 10.0;  //joint 2
const double J6_OFFSET = 0;    //tip
double resTable[25] = {114660,84510,62927,47077,35563,27119,20860,16204,12683,10000,7942,6327,5074,4103,3336,2724,2237,1846,1530,1275,1068,899.3,760.7,645.2,549.4};
double tempTable[25] = { -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 };
double FULLAD = 4095;
double JOINT_CIRCLECNT[6] = {19000,53706.7,3724,756,14275.333,3724};      //1:285 40/12,1:212  40/12, 1:49, 1:27, 1:49 46/12, 1:49  2/1
double JOINT_INI_ANGLE[6] = { M_PI - M_PI * J1_OFFSET / 180, M_PI * J2_OFFSET / 180, 0, 0, 0, M_PI * J6_OFFSET / 180 };
const double ARMLEN1 = 0.455;
const double ARMLEN2 = 0.43;
const double ARMLEN31 = 0.20;
const double ARMLEN32 = 0.28;
MainWindow::MainWindow(int argc, char** argv,QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
	 qnode(argc,argv)
{
    ui->setupUi(this);
    ui->lineEditRobotIP1->setInputMask("000.000.000.000");
    IP4Validator *ip4Validator = new IP4Validator();
    ui->lineEditRobotIP1->setValidator(ip4Validator);

    ui->lineEditRobotIP2->setInputMask("000.000.000.000");
    ui->lineEditRobotIP2->setValidator(ip4Validator);

    connect( ui->pushButtonConnect1, SIGNAL(clicked()),this, SLOT(connectToRobot1()));
    connect( ui->pushButtonConnect2, SIGNAL(clicked()),this, SLOT(connectToRobot2()));
    connect( ui->pushButtonM1Down, SIGNAL(clicked()),this, SLOT(motor1Down()));
    connect( ui->pushButtonM1Up, SIGNAL(clicked()),this, SLOT(motor1Up()));

    connect( ui->pushButtonM2Down, SIGNAL(clicked()),this, SLOT(motor2Down()));
    connect( ui->pushButtonM2Up, SIGNAL(clicked()),this, SLOT(motor2Up()));

    connect( ui->pushButtonM3Down, SIGNAL(clicked()),this, SLOT(motor3Down()));
    connect( ui->pushButtonM3Up, SIGNAL(clicked()),this, SLOT(motor3Up()));

    connect( ui->pushButtonM4Down, SIGNAL(clicked()),this, SLOT(motor4Down()));
    connect( ui->pushButtonM4Up, SIGNAL(clicked()),this, SLOT(motor4Up()));

    connect( ui->pushButtonM5Down, SIGNAL(clicked()),this, SLOT(motor5Down()));
    connect( ui->pushButtonM5Up, SIGNAL(clicked()),this, SLOT(motor5Up()));

    connect( ui->pushButtonM6Down, SIGNAL(clicked()),this, SLOT(motor6Down()));
    connect( ui->pushButtonM6Up, SIGNAL(clicked()),this, SLOT(motor6Up()));

    connect( ui->pushButtonM1Stop, SIGNAL(clicked()),this, SLOT(motor1Stop()));
    connect( ui->pushButtonM2Stop, SIGNAL(clicked()),this, SLOT(motor2Stop()));
    connect( ui->pushButtonM3Stop, SIGNAL(clicked()),this, SLOT(motor3Stop()));
    connect( ui->pushButtonM4Stop, SIGNAL(clicked()),this, SLOT(motor4Stop()));
    connect( ui->pushButtonM5Stop, SIGNAL(clicked()),this, SLOT(motor5Stop()));
    connect( ui->pushButtonM6Stop, SIGNAL(clicked()),this, SLOT(motor6Stop()));
    connect(ui->pushButtonStopAll, SIGNAL(clicked()),this,SLOT(motorStopAll()));

    connect(ui->pushButtonQuery1, SIGNAL(clicked()),this,SLOT(sendQuery1()));
    connect(ui->pushButtonQuery2, SIGNAL(clicked()),this,SLOT(sendQuery2()));
    connect(ui->pushButtonQuery3, SIGNAL(clicked()),this,SLOT(sendQuery3()));
    connect(ui->pushButtonQuery4, SIGNAL(clicked()),this,SLOT(sendQuery4()));
    connect(ui->pushButtonJ1Go, SIGNAL(clicked()),this,SLOT(j1GoCmd()));
    connect(ui->pushButtonJ2Go, SIGNAL(clicked()),this,SLOT(j2GoCmd()));
    connect(ui->pushButtonRotateGo, SIGNAL(clicked()),this,SLOT(rotateGoCmd()));
    connect(ui->pushButtonTipGo, SIGNAL(clicked()),this,SLOT(tipGoCmd()));
    connect(ui->pushButtonPanGo, SIGNAL(clicked()),this,SLOT(panGoCmd()));
    connect(ui->pushButtonArmTargetGo, SIGNAL(clicked()),this,SLOT(armPosGoCmd()));
    connect(ui->pushButtonArmReset, SIGNAL(clicked()),this,SLOT(armResetCmd()));
    connect(ui->pushButtonArmSetIni, SIGNAL(clicked()),this,SLOT(armSetIniCmd()));

    watchDogCnt1 = 2;
    watchDogCnt2 = 2;
    pingTimer.setInterval(500);
    pingTimer.stop();
    tcpRobot1 = NULL;
    tcpRobot2 = NULL;
    tcpRobot3 = NULL;
    tcpRobot4 = NULL;
    ui->pushButtonQuery1->setEnabled(false);
    ui->pushButtonQuery2->setEnabled(false);
    ui->pushButtonQuery3->setEnabled(false);
    ui->pushButtonQuery4->setEnabled(false);
    ui->pushButtonStopAll->setEnabled(false);
    //init joingMotorData
    tipAngle = 0;
    armIniFlag = false;
    for (int i = 0; i < 6; i++)
    {
        jointMotorData[i].circleCnt = JOINT_CIRCLECNT[i];
        jointMotorData[i].resolution = jointMotorData[i].circleCnt / (2 * M_PI);
        jointMotorData[i].iniPos = 0;
        jointMotorData[i].angle = JOINT_INI_ANGLE[i];
        jointMotorData[i].motorTemperature = 0;
        jointMotorData[i].cmdDir = 1;
        jointMotorData[i].encoderDir = 1;
        jointMotorData[i].pwmOutput = 0;
        jointMotorData[i].encodeSpeed = 0;
        jointMotorData[i].protect = false;
        jointMotorData[i].limitPos1 = 0;
        jointMotorData[i].limitPos2 = 0;
        jointMotorData[i].stuckFlag = false;
        jointMotorData[i].preCmdDir = 1;
        jointMotorData[i].manipulatorStuckCnt = 0;
        jointMotorData[i].jointCmd = 0;
        jointMotorData[i].jointJoy = false;
        jointMotorData[i].jointCtrl = false;
        jointMotorData[i].motorTemperature = 0;
        jointMotorData[i].currentAmp = 0;

    }

	//qnode
	if ( !qnode.init() ) {
		QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
	} else {
	
	}
	QObject::connect(&qnode, SIGNAL(cmdUpdated(int,int,int)), this, SLOT(cmdSend(int,int,int)));
   QObject::connect(&qnode, SIGNAL(jointAngleCmdUpdated(int,double)), this, SLOT(jointAngleCmdSend(int,double)));
   QObject::connect(&qnode, SIGNAL(armPosCmdUpdated(double,double,double)), this, SLOT(armPosCmdSend(double,double,double)));
   QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if (tcpRobot1!= NULL){
        tcpRobot1->close();
        delete tcpRobot1;
    }
    if (tcpRobot2 != NULL){
        tcpRobot2->close();
        delete tcpRobot2;
    }
    if (tcpRobot3 != NULL){
        tcpRobot3->close();
        delete tcpRobot3;
    }
    QWidget::closeEvent(event);
}


void MainWindow::connectToRobot1()
{
    bool ok = false;
    QString temp = ui->pushButtonConnect1->text();

    if (temp == tr("Connect"))
    {

        QString addr = ui->lineEditRobotIP1->text().trimmed();
        int port = ui->lineEditRobotPort1_1->text().toInt(&ok,10);
        ui->pushButtonConnect1 ->setText(tr("Disconnect"));
        tcpRobot1 = new QTcpSocket(this);
        tcpRobot1->connectToHost(addr,port);
        connect(tcpRobot1,SIGNAL(readyRead()), this, SLOT(processRobotData1()));
        sendQuery1();

        tcpRobot2 = new QTcpSocket(this);
        port = ui->lineEditRobotPort1_2->text().toInt(&ok,10);
        tcpRobot2->connectToHost(addr,port);
        connect(tcpRobot2,SIGNAL(readyRead()), this, SLOT(processRobotData2()));
        sendQuery2();

        connect(&pingTimer, SIGNAL(timeout()), this, SLOT(sendPing()));
        pingTimer.start();
        watchDogCnt1 = 2;

    }
    else
    {
        pingTimer.stop();
        disconnect(&pingTimer, SIGNAL(timeout()), this, SLOT(sendPing()));
        ui->pushButtonConnect1 ->setText(tr("Connect"));
        tcpRobot1->close();
        tcpRobot2->close();
        watchDogCnt1 = 2;
        ui->groupBoxMotorControl1->setEnabled(false);

        ui->pushButtonQuery1->setEnabled(false);
        ui->pushButtonQuery2->setEnabled(false);

    }
}

void MainWindow::connectToRobot2()
{
    bool ok = false;
    QString temp = ui->pushButtonConnect2->text();

    if (temp == tr("Connect"))
    {

        QString addr = ui->lineEditRobotIP2->text().trimmed();
        int port = ui->lineEditRobotPort2_1->text().toInt(&ok,10);
        ui->pushButtonConnect2 ->setText(tr("Disconnect"));
        tcpRobot3 = new QTcpSocket(this);
        tcpRobot3->connectToHost(addr,port);
        connect(tcpRobot3,SIGNAL(readyRead()), this, SLOT(processRobotData3()));
        sendQuery3();


        watchDogCnt2 = 2;

    }
    else
    {

        ui->pushButtonConnect2 ->setText(tr("Connect"));
        tcpRobot3->close();

        watchDogCnt2 = 2;
        ui->groupBoxMotorControl2->setEnabled(false);
        ui->pushButtonQuery3->setEnabled(false);
        ui->pushButtonQuery4->setEnabled(false);
    }
}

void MainWindow::processRobotData1()
{
    QString received = "";
    qint64 count = 0;
    watchDogCnt1 = 0;
    while( (count = tcpRobot1->bytesAvailable()) > 0)
    {
        received = tcpRobot1->readAll();
        receivedData1.append(received);
    }
    if (receivedData1.endsWith("\r"))
    {
        dealWithPackage1(receivedData1);
        receivedData1 = "";
    }
}

void MainWindow::processRobotData2()
{
    QString received = "";
    qint64 count = 0;
    watchDogCnt1 = 0;
    while( (count = tcpRobot2->bytesAvailable()) > 0)
    {
        received = tcpRobot2->readAll();
        receivedData2.append(received);
    }
    if (receivedData2.endsWith("\r"))
    {
        dealWithPackage2(receivedData2);
        receivedData2 = "";
    }
}

void MainWindow::processRobotData3()
{
    QString received = "";
    qint64 count = 0;
    watchDogCnt2 = 0;
    while( (count = tcpRobot3->bytesAvailable()) > 0)
    {

        received = tcpRobot3->readAll();
        receivedData3.append(received);
    }
    if (receivedData3.endsWith("\r"))
    {
        dealWithPackage3(receivedData3);
        receivedData3 = "";
    }
}

void MainWindow:: dealWithPackage1(QString received)
{
    QString temp;
    QStringList rev = received.split("\r");
    for (int i = 0; i < rev.length(); i++)
    {
        received = rev.at(i);

        if (received.startsWith("A="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData1.motAmp1 = strData.at(0).toDouble()/10;

                    motorData1.motAmp2 = strData.at(1).toDouble()/10;
                    temp.setNum(motorData1.motAmp1,'g',4);
                    ui->lineEditM1Current->setText(temp);
                    temp.setNum(motorData1.motAmp2,'g',4);
                    ui->lineEditM2Current->setText(temp);
                    jointMotorData[0].currentAmp = motorData1.motAmp1;
                    jointMotorData[1].currentAmp = motorData1.motAmp2;

                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("AI="))
        {
            received.remove(0,3);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 3)
                {
                    motorData1.ai3 = strData.at(2).toDouble();
                    motorData1.motTemp1 = ad2Temperature(motorData1.ai3);
                    motorData1.ai4 = strData.at(3).toDouble();

                    motorData1.motTemp2 = ad2Temperature(motorData1.ai4);
                    temp.setNum(motorData1.motTemp1,'g',4);
                    ui->lineEditM1Temp->setText(temp);
                    temp.setNum(motorData1.motTemp2,'g',4);
                    ui->lineEditM2Temp->setText(temp);
                    jointMotorData[0].motorTemperature = motorData1.motTemp1;
                    jointMotorData[1].motorTemperature = motorData1.motAmp2;
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("C="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData1.motEncP1 = strData.at(0).toInt();
                    motorData1.motEncP2 = strData.at(1).toInt();
                    temp.setNum(motorData1.motEncP1);
                    ui->lineEditM1Pos->setText(temp);
                    temp.setNum(motorData1.motEncP2);
                    ui->lineEditM2Pos->setText(temp);
                    jointMotorData[0].encoderPos = motorData1.motEncP1;
                    jointMotorData[1].encoderPos = motorData1.motEncP2;
                    jointMotorData[0].angle = trans2Angle(0);
                    //jointMotorData[1].angle = -(trans2Angle(1) - JOINT_INI_ANGLE[1]) + jointMotorData[0].angle;
                    jointMotorData[1].angle = (trans2Angle(1));
                    temp.setNum(jointMotorData[0].angle * 180/M_PI,'f',1);
                    ui->lineEditJ1Angle->setText(temp);
                    temp.setNum(jointMotorData[1].angle * 180 /M_PI,'f',1);
                    ui->lineEditJ2Angle->setText(temp);
                    //get arm tip end position
                    getPositionXY();
                    temp.setNum(manipulatorPos[0],'f',2);
                    ui->lineEditArmPosX->setText(temp);
                    temp.setNum(manipulatorPos[1],'f',2);
                    ui->lineEditArmPosY->setText(temp);
						//publish all the joint information here
						//publish sensor data here
							 int motorPos[MOTOR_NUM];
							 int motorVel[MOTOR_NUM];
							 int motorPWM[MOTOR_NUM];
							 double motorTemperature[MOTOR_NUM];
							 double jointAngle[MOTOR_NUM];
							for(int i = 0; i < MOTOR_NUM; i++)
							{
 								motorPos[i] = jointMotorData[i].encoderPos;
   							motorVel[i] = jointMotorData[i].encodeSpeed;
								motorPWM[i] = jointMotorData[i].pwmOutput;
    							motorTemperature[i] = jointMotorData[i].motorTemperature;
								jointAngle[i] = jointMotorData[i].angle;
							}
							 qnode.publisher(motorPos,motorVel,motorPWM,motorTemperature,jointAngle,MOTOR_NUM) ;

                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("P="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData1.motPower1 = strData.at(0).toInt();
                    motorData1.motPower2 = strData.at(1).toInt();
                    temp.setNum(motorData1.motPower1);
                    ui->lineEditM1Power->setText(temp);
                    temp.setNum(motorData1.motPower2);
                    ui->lineEditM2Power->setText(temp);
                    jointMotorData[0].pwmOutput = motorData1.motPower1;
                    jointMotorData[1].pwmOutput = motorData1.motPower2;
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("S="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData1.motEncS1 = strData.at(0).toInt();
                    motorData1.motEncS2 = strData.at(1).toInt();
                    temp.setNum(motorData1.motEncS1);
                    ui->lineEditM1Vel->setText(temp);
                    temp.setNum(motorData1.motEncS2);
                    ui->lineEditM2Vel->setText(temp);
                    jointMotorData[0].encodeSpeed = motorData1.motEncS1;
                    jointMotorData[1].encodeSpeed = motorData1.motEncS2;
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("T="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData1.ch1Temp = strData.at(0).toDouble();
                    motorData1.ch2Temp = strData.at(1).toInt();
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("V="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 2)
                {
                    motorData1.drvVoltage = strData.at(0).toDouble()/10;
                    motorData1.batVoltage = strData.at(1).toDouble()/10;
                    motorData1.reg5VVoltage = strData.at(2).toDouble()/1000;
                    temp.setNum(motorData1.batVoltage,'g',4);
                    ui->lineEditBatVol->setText(temp);

                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("MMOD="))
        {
            received.remove(0,5);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData1.mode1 = strData.at(0).toInt();
                    motorData1.mode2 = strData.at(1).toInt();
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("FF="))
        {
            received.remove(0,3);
            received.remove(QChar('\r'),Qt::CaseInsensitive);

            try
            {
                motorData1.statusFlag = received.toInt();
                temp="";
                if((motorData1.statusFlag & 0x01) != 0)
                {
                    temp.append("OverHeat+");
                }
                if((motorData1.statusFlag & 0x02) != 0)
                {
                    temp.append("OverVoltage+");
                }
                if((motorData1.statusFlag & 0x04) != 0)
                {
                    temp.append("UnderVol+");
                }
                if((motorData1.statusFlag & 0x08) != 0)
                {
                    temp.append("Short+");
                }
                if((motorData1.statusFlag & 0x10) != 0)
                {
                    temp.append("ESTOP+");
                }
					 if (motorData1.statusFlag == 0)
                {
                    temp.append("OK");
                }

                ui->lineEditChannel1State->setText(temp);
            }
            catch(...)
            {

            }
        }
    }




}

void MainWindow:: dealWithPackage2(QString received)
{
  QString temp;
    QStringList rev = received.split("\r");
    for (int i = 0; i < rev.length(); i++)
    {
        received = rev.at(i);
        if (received.startsWith("A="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData2.motAmp1 = strData.at(0).toDouble()/10;

                    motorData2.motAmp2 = strData.at(1).toDouble()/10;
                    temp.setNum(motorData2.motAmp1,'g',4);
                    ui->lineEditM3Current->setText(temp);
                    temp.setNum(motorData2.motAmp2,'g',4);
                    ui->lineEditM4Current->setText(temp);
                    jointMotorData[2].currentAmp = motorData2.motAmp1;
                    jointMotorData[3].currentAmp = motorData2.motAmp2;

                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("AI="))
        {
            received.remove(0,3);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 3)
                {
                    motorData2.ai3 = strData.at(2).toDouble();
                    motorData2.motTemp1 = ad2Temperature(motorData2.ai3);
                    motorData2.ai4 = strData.at(3).toDouble();
                    motorData2.motTemp2 = ad2Temperature(motorData2.ai4);
                    temp.setNum(motorData2.motTemp1,'g',4);
                    ui->lineEditM3Temp->setText(temp);
                    temp.setNum(motorData2.motTemp2,'g',4);
                    ui->lineEditM4Temp->setText(temp);
                    jointMotorData[2].motorTemperature = motorData2.motTemp1;
                    jointMotorData[3].motorTemperature = motorData2.motTemp2;
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("C="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData2.motEncP1 = strData.at(0).toInt();
                    motorData2.motEncP2 = strData.at(1).toInt();
                    temp.setNum(motorData2.motEncP1);
                    ui->lineEditM3Pos->setText(temp);
                    temp.setNum(motorData2.motEncP2);
                    ui->lineEditM4Pos->setText(temp);
                    jointMotorData[2].encoderPos = motorData2.motEncP1;
                    jointMotorData[3].encoderPos = motorData2.motEncP2;
                    jointMotorData[2].angle = trans2Angle(2);
                    temp.setNum(jointMotorData[2].angle * 180/M_PI,'f',1);
                    ui->lineEditRotateAngle->setText(temp);
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("P="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData2.motPower1 = strData.at(0).toInt();
                    motorData2.motPower2 = strData.at(1).toInt();
                    temp.setNum(motorData2.motPower1);
                    ui->lineEditM3Power->setText(temp);
                    temp.setNum(motorData2.motPower2);
                    ui->lineEditM4Power->setText(temp);
                    jointMotorData[2].pwmOutput = motorData2.motPower1;
                    jointMotorData[3].pwmOutput = motorData2.motPower2;
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("S="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData2.motEncS1 = strData.at(0).toInt();
                    motorData2.motEncS2 = strData.at(1).toInt();
                    temp.setNum(motorData2.motEncS1);
                    ui->lineEditM3Vel->setText(temp);
                    temp.setNum(motorData2.motEncS2);
                    ui->lineEditM4Vel->setText(temp);
                    jointMotorData[2].encodeSpeed = motorData2.motEncS1;
                    jointMotorData[3].encodeSpeed = motorData2.motEncS2;
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("T="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData2.ch1Temp = strData.at(0).toDouble();
                    motorData2.ch2Temp = strData.at(1).toInt();
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("V="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 2)
                {
                    motorData2.drvVoltage = strData.at(0).toDouble()/10;
                    motorData2.batVoltage = strData.at(1).toDouble()/10;
                    motorData2.reg5VVoltage = strData.at(2).toDouble()/1000;
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("MMOD="))
        {
            received.remove(0,5);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData2.mode1 = strData.at(0).toInt();
                    motorData2.mode2 = strData.at(1).toInt();
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("FF="))
    {
        received.remove(0,3);
        received.remove(QChar('\r'),Qt::CaseInsensitive);

        try
        {
            motorData2.statusFlag = received.toInt();
            temp="";
            if((motorData2.statusFlag & 0x01) != 0)
            {
	            temp.append("OverHeat+");
            }
            if((motorData2.statusFlag & 0x02) != 0)
            {
	            temp.append("OverVoltage+");
            }
            if((motorData2.statusFlag & 0x04) != 0)
            {
	            temp.append("UnderVol+");
            }
            if((motorData2.statusFlag & 0x08) != 0)
            {
	            temp.append("Short+");
            }
            if((motorData2.statusFlag & 0x10) != 0)
            {
	            temp.append("ESTOP+");
            }
				if (motorData1.statusFlag == 0)
            {
                temp.append("OK");
            }

            ui->lineEditChannel2State->setText(temp);
        }
        catch(...)
        {

        }
    }
    }

}

void MainWindow:: dealWithPackage3(QString received)
{
    QString temp;
    QStringList rev = received.split("\r");
    for (int i = 0; i < rev.length(); i++)
    {
        received = rev.at(i);

        if (received.startsWith("A="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData3.motAmp1 = strData.at(0).toDouble()/10;

                    motorData3.motAmp2 = strData.at(1).toDouble()/10;
                    temp.setNum(motorData3.motAmp1,'g',4);
                    ui->lineEditM5Current->setText(temp);
                    temp.setNum(motorData3.motAmp2,'g',4);
                    ui->lineEditM6Current->setText(temp);
                    jointMotorData[4].currentAmp = motorData3.motAmp1;
                    jointMotorData[5].currentAmp = motorData3.motAmp2;
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("AI="))
        {
            received.remove(0,3);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 3)
                {
                    motorData3.ai3 = strData.at(2).toDouble();
                    motorData3.motTemp1 = ad2Temperature(motorData3.ai3);
                    motorData3.ai4 = strData.at(3).toDouble();

                    motorData3.motTemp2 = ad2Temperature(motorData3.ai4);
                    temp.setNum(motorData3.motTemp1,'g',4);
                    ui->lineEditM5Temp->setText(temp);
                    temp.setNum(motorData3.motTemp2,'g',4);
                    ui->lineEditM6Temp->setText(temp);
                    jointMotorData[4].motorTemperature = motorData3.motTemp1;
                    jointMotorData[5].motorTemperature = motorData3.motTemp2;
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("C="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData3.motEncP1 = strData.at(0).toInt();
						  motorData3.motEncP1 = strData.at(0).toInt();
                    motorData3.motEncP2 = strData.at(1).toInt();
                    temp.setNum(motorData3.motEncP1);
                    ui->lineEditM5Pos->setText(temp);
                    temp.setNum(motorData3.motEncP2);
                    ui->lineEditM6Pos->setText(temp);
                    jointMotorData[4].encoderPos = motorData3.motEncP1;
                    jointMotorData[5].encoderPos = motorData3.motEncP2;
                    jointMotorData[4].angle = -trans2Angle(4);
                    jointMotorData[5].angle = -trans2Angle(5);
                    temp.setNum(jointMotorData[4].angle * 180 /M_PI,'f',1);
                    ui->lineEditPanAngle->setText(temp);
                    tipAngle = jointMotorData[5].angle + (jointMotorData[1].angle);
                    temp.setNum(tipAngle * 180 /M_PI,'f',1);
                    ui->lineEditTipAngle->setText(temp);

                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("P="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData3.motPower1 = strData.at(0).toInt();
                    motorData3.motPower2 = strData.at(1).toInt();
                    temp.setNum(motorData3.motPower1);
                    ui->lineEditM5Power->setText(temp);
                    temp.setNum(motorData3.motPower2);
                    ui->lineEditM6Power->setText(temp);
                    jointMotorData[4].pwmOutput = motorData3.motPower1;
                    jointMotorData[5].pwmOutput = motorData3.motPower2;
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("S="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData3.motEncS1 = strData.at(0).toInt();
                    motorData3.motEncS2 = strData.at(1).toInt();
                    temp.setNum(motorData3.motEncS1);
                    ui->lineEditM5Vel->setText(temp);
                    temp.setNum(motorData3.motEncS2);
                    ui->lineEditM6Vel->setText(temp);
                    jointMotorData[4].encodeSpeed = motorData3.motEncS1;
                    jointMotorData[5].encodeSpeed = motorData3.motEncS2;
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("T="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData3.ch1Temp = strData.at(0).toDouble();
                    motorData3.ch2Temp = strData.at(1).toInt();
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("V="))
        {
            received.remove(0,2);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 2)
                {
                    motorData3.drvVoltage = strData.at(0).toDouble()/10;
                    motorData3.batVoltage = strData.at(1).toDouble()/10;
                    motorData3.reg5VVoltage = strData.at(2).toDouble()/1000;
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("MMOD="))
        {
            received.remove(0,5);
            received.remove(QChar('\r'),Qt::CaseInsensitive);
            QStringList strData = received.split(":");
            try
            {
                if (strData.length()> 1)
                {
                    motorData3.mode1 = strData.at(0).toInt();
                    motorData3.mode2 = strData.at(1).toInt();
                }
            }
            catch(...)
            {

            }
        }
        else if (received.startsWith("FF="))
        {
            received.remove(0,3);
            received.remove(QChar('\r'),Qt::CaseInsensitive);

            try
            {
                motorData3.statusFlag = received.toInt();
                temp="";
                if((motorData3.statusFlag & 0x01) != 0)
                {
                    temp.append("OverHeat+");
                }
                if((motorData3.statusFlag & 0x02) != 0)
                {
                    temp.append("OverVoltage+");
                }
                if((motorData3.statusFlag & 0x04) != 0)
                {
                    temp.append("UnderVol+");
                }
                if((motorData3.statusFlag & 0x08) != 0)
                {
                    temp.append("Short+");
                }
                if((motorData3.statusFlag & 0x10) != 0)
                {
                    temp.append("ESTOP+");
                }
					 if (motorData1.statusFlag == 0)
                {
                    temp.append("OK");
                }
                ui->lineEditChannel3State->setText(temp);
            }
            catch(...)
            {

            }
        }
    }




}

double MainWindow::ad2Temperature(int adValue)
{
    //for new temperature sensor
               double tempM = 0;
               double k = (adValue / FULLAD);
               double resValue = 0;
               if (k != 0)
               {
                   resValue = (10000 / k -10000);      //AD value to resistor
               }
               else
               {
                   resValue = resTable[0];
               }


               int index = -1;
               if (resValue >= resTable[0])       //too lower
               {
                   tempM = -20;
               }
               else if (resValue <= resTable[24])
               {
                   tempM = 100;
               }
               else
               {
                   for (int i = 0; i < 24; i++)
                   {
                       if ((resValue <= resTable[i]) && (resValue >= resTable[i + 1]))
                       {
                           index = i;
                           break;
                       }
                   }
                   if (index >= 0)
                   {
                       tempM = tempTable[index] + (resValue - resTable[index]) / (resTable[index + 1] - resTable[index]) * (tempTable[index + 1] - tempTable[index]);
                   }
                   else
                   {
                       tempM = 0;
                   }

               }

               return tempM;
}

void MainWindow::motor1Down()
{
    if (tcpRobot1 != NULL){
        if (tcpRobot1->isWritable()){
            if (armIniFlag){
                if ((jointMotorData[0].encoderPos - J1_CMD) >= jointMotorData[0].limitPos1)
                {
                    tcpRobot1->write(J1_DOWN_CMD);
                }
            }
            else{
                tcpRobot1->write(J1_DOWN_CMD);
            }

        }
    }
}

void MainWindow::motor1Up()
{
    if (tcpRobot1 != NULL){
        if (tcpRobot1->isWritable()){
            if (armIniFlag){
                if ((jointMotorData[0].encoderPos + J1_CMD) <= jointMotorData[0].limitPos2)
                {
                    tcpRobot1->write(J1_UP_CMD);
                }
            }
            else{
                tcpRobot1->write(J1_UP_CMD);
            }

        }
    }
}


void MainWindow::motor2Down()
{
    if (tcpRobot1 != NULL){
        if (tcpRobot1->isWritable()){
            if (armIniFlag){
                if ((jointMotorData[1].encoderPos + J2_CMD) <= jointMotorData[1].limitPos2)
                {
                    tcpRobot1->write(J2_DOWN_CMD);
                }
            }
            else{
                tcpRobot1->write(J2_DOWN_CMD);
            }

        }
    }
}

void MainWindow::motor2Up()
{
    if (tcpRobot1 != NULL){
        if (tcpRobot1->isWritable()){
            if (armIniFlag){
                if ((jointMotorData[1].encoderPos - J2_CMD) >= jointMotorData[1].limitPos1)
                {
                    tcpRobot1->write(J2_UP_CMD);
                }
            }
            else{
                tcpRobot1->write(J2_UP_CMD);
            }
        }
    }
}


void MainWindow::motor3Down()
{
    if (tcpRobot2 != NULL){
        if (tcpRobot2->isWritable()){
            if (armIniFlag){
                if ((jointMotorData[2].encoderPos - J3_CMD) >= jointMotorData[2].limitPos1)
                {
                    tcpRobot2->write(J3_DOWN_CMD);
                }
            }
            else{
                tcpRobot2->write(J3_DOWN_CMD);
            }
        }
    }
}

void MainWindow::motor3Up()
{
    if (tcpRobot2 != NULL){
        if (tcpRobot2->isWritable()){

            if (armIniFlag){
                if ((jointMotorData[2].encoderPos + J3_CMD) <= jointMotorData[2].limitPos2)
                {
                    tcpRobot2->write(J3_UP_CMD);
                }
            }
            else{
                tcpRobot2->write(J3_UP_CMD);
            }
        }

    }
}


void MainWindow::motor4Down()
{
    if (tcpRobot2 != NULL){
        if (tcpRobot2->isWritable()){
            tcpRobot2->write(J4_DOWN_CMD);
        }
    }
}

void MainWindow::motor4Up()
{
    if (tcpRobot2 != NULL){
        if (tcpRobot2->isWritable()){
            tcpRobot2->write(J4_UP_CMD);
        }
    }
}

void MainWindow::motor5Down()
{
    if (tcpRobot3 != NULL){
        if (tcpRobot3->isWritable()){
            if (armIniFlag){
                if ((jointMotorData[4].encoderPos - J5_CMD) >= jointMotorData[4].limitPos1)
                {
                    tcpRobot3->write(J5_DOWN_CMD);
                }
            }
            else{
                tcpRobot3->write(J5_DOWN_CMD);
            }

        }
    }
}

void MainWindow::motor5Up()
{
    if (tcpRobot3 != NULL){
        if (tcpRobot3->isWritable()){
            if (armIniFlag){
                if ((jointMotorData[4].encoderPos + J5_CMD) <= jointMotorData[4].limitPos2)
                {
                    tcpRobot3->write(J5_UP_CMD);
                }
            }
            else{
                tcpRobot3->write(J5_UP_CMD);
            }

        }
    }
}

void MainWindow::motor6Down()
{
    if (tcpRobot3 != NULL){
        if (tcpRobot3->isWritable()){
            if (armIniFlag){
                if ((jointMotorData[5].encoderPos - J6_CMD) >= jointMotorData[5].limitPos1)
                {
                    tcpRobot3->write(J6_DOWN_CMD);
                }
            }
            else{
                tcpRobot3->write(J6_DOWN_CMD);
            }
        }
    }
}

void MainWindow::motor6Up()
{
    if (tcpRobot3 != NULL){
        if (tcpRobot3->isWritable()){
            if (armIniFlag){
                if ((jointMotorData[5].encoderPos + J6_CMD) <= jointMotorData[5].limitPos2)
                {
                    tcpRobot3->write(J6_UP_CMD);
                }
            }
            else{
                tcpRobot3->write(J6_UP_CMD);
            }
            tcpRobot3->write(J6_UP_CMD);
        }
    }
}

void MainWindow::motor1Stop()
{
    if (tcpRobot1 != NULL){
        if (tcpRobot1->isWritable()){
            tcpRobot1->write("!PR 1 0\r");
        }
    }

}

void MainWindow::motor2Stop()
{
    if (tcpRobot1 != NULL){
        if (tcpRobot1->isWritable()){
            tcpRobot1->write("!PR 2 0\r");
        }
    }
}

void MainWindow::motor3Stop()
{
    if (tcpRobot2 != NULL){
        if (tcpRobot2->isWritable()){
            tcpRobot2->write("!PR 1 0\r");
        }
    }
}

void MainWindow::motor4Stop()
{
    if (tcpRobot2 != NULL){
        if (tcpRobot2->isWritable()){
            tcpRobot2->write("!G 2 0\r");
        }
    }

}

void MainWindow::motor5Stop()
{
    if (tcpRobot3 != NULL){
        if (tcpRobot3->isWritable()){
            tcpRobot3->write("!PR 1 0\r");
        }
    }

}

void MainWindow::motor6Stop()
{
    if (tcpRobot3 != NULL){
        if (tcpRobot3->isWritable()){
            tcpRobot3->write("!PR 2 0\r");
        }
    }

}

void MainWindow::motorStopAll()
{
 if (ui->pushButtonStopAll->text() == "ESTOP_ALL")
    {
        if(tcpRobot1 != NULL){
            if (tcpRobot1->isWritable()){
                tcpRobot1->write("!EX\r");
            }
        }

        if(tcpRobot2 != NULL){
            if (tcpRobot2->isWritable()){
                tcpRobot2->write("!EX\r");
            }
        }

        if(tcpRobot3 != NULL){
            if (tcpRobot3->isWritable()){
                tcpRobot3->write("!EX\r");
            }
        }
        ui->pushButtonStopAll->setText("Resume");
    }
    else
    {
         if(tcpRobot1 != NULL){
             if (tcpRobot1->isWritable()){
                 tcpRobot1->write("!MG\r");
             }
         }

         if(tcpRobot2 != NULL){
             if (tcpRobot2->isWritable()){
                 tcpRobot2->write("!MG\r");
             }
         }

         if(tcpRobot3 != NULL){
             if (tcpRobot3->isWritable()){
                 tcpRobot3->write("!MG\r");
             }
         }
        ui->pushButtonStopAll->setText("ESTOP_ALL");
    }
}

void MainWindow::sendQuery1()
{
    if (tcpRobot1 != NULL){
        if (tcpRobot1->isWritable()){
            tcpRobot1->write("# C_?A_?A_?AI_?C_?FF_?P_?S_?T_?V_# 20\r");
        }
    }
}

void MainWindow::sendQuery2()
{
    if (tcpRobot2 != NULL){
        if (tcpRobot2->isWritable()){
            tcpRobot2->write("# C_?A_?A_?AI_?C_?FF_?P_?S_?T_?V_# 20\r");
        }
    }
}

void MainWindow::sendQuery3()
{
    if (tcpRobot3 != NULL){
        if (tcpRobot3->isWritable()){
            tcpRobot3->write("# C_?A_?A_?AI_?C_?FF_?P_?S_?T_?V_# 20\r");
        }
    }
}
void MainWindow::sendQuery4()
{
    if (tcpRobot4 != NULL){
        if (tcpRobot4->isWritable()){
            tcpRobot4->write("# C_?A_?A_?AI_?C_?FF_?P_?S_?T_?V_# 20\r");
        }
    }
}

void MainWindow::jointAngleCmdSend(int channel,double angle)
{
	QString cmdStr = "";
	cmdStr.setNum(angle / M_PI * 180, 'f',2);
	
	if(armIniFlag)
	{
		if (channel == 1)
		{
			ui->lineEditJ1TargetAngle->setText(cmdStr);
			j1GoCmd();
		}
		else if(channel == 2)
		{
			ui->lineEditJ2TargetAngle->setText(cmdStr);
			j2GoCmd();
		}
		else if(channel == 3)
		{
			ui->lineEditRotateTargetAngle->setText(cmdStr);
			rotateGoCmd();
		}
		else if(channel == 4)
		{
			//no position contor??//
		}
		else if(channel == 5)
		{
			ui->lineEditPanTargetAngle->setText(cmdStr);
			panGoCmd();
		}
		else if(channel == 6)
		{
			ui->lineEditTipTargetAngle->setText(cmdStr);
			tipGoCmd();
		}

	}
}

void MainWindow::armPosCmdSend(double x, double y, double tipAngle)
{
	QString cmdStr = "";
	
	if(armIniFlag)
	{
		if ((x == 0) && (y == 0) && (tipAngle == 0))
		{
			//means reset arm
			armResetCmd();
		}
		else
		{
			cmdStr.setNum(x, 'f',2);
			ui->lineEditArmTargetX->setText(cmdStr);
			cmdStr.setNum(y, 'f',2);
			ui->lineEditArmTargetY->setText(cmdStr);
			cmdStr.setNum(tipAngle * M_PI / 180, 'f',2);
			ui->lineEditTipAngle->setText(cmdStr);
			armPosGoCmd();
		}

	}

}

void MainWindow::cmdSend(int channel,int cmdValue,int motorCtrl)
{
    //make sure you well understand the cmdValue and know how the motor move
    QString temp = "";
    temp.setNum(cmdValue);

    if (channel == 0)
    {
        if (motorCtrl == 3)
        {
            temp = "!PR 1 " + temp + "\r";
            tcpRobot1->write(temp.toLatin1().data());
        }

    }
    else if(channel == 1)
    {
        if (motorCtrl == 3)
        {
            temp = "!PR 2 " + temp + "\r";
            tcpRobot1->write(temp.toLatin1().data());

        }
    }
    else if(channel == 2)
    {
        if (motorCtrl == 3)
        {
            temp = "!PR 1 " + temp + "\r";
            tcpRobot2->write(temp.toLatin1().data());
        }
    }
    else if(channel == 3)
    {
        if (motorCtrl == 0)
        {
            temp = "!G 2 " + temp + "\r";
            tcpRobot2->write(temp.toLatin1().data());

        }
    }
}

//////// send out command
void MainWindow::sendPing()
{

        ++watchDogCnt1;
        if (watchDogCnt1 == 1)
        {
                ui->groupBoxMotorControl1->setEnabled(true);
                ui->pushButtonQuery1->setEnabled(true);
                ui->pushButtonQuery2->setEnabled(true);
                ui->pushButtonStopAll->setEnabled(true);
                tcpRobot1->write("~MMOD\r");
                tcpRobot2->write("~MMOD\r");
        }
        else
        {
                if (watchDogCnt1> 10)
                {
                    ui->groupBoxMotorControl1->setEnabled(false);
                    ui->pushButtonQuery1->setEnabled(false);
                    ui->pushButtonQuery2->setEnabled(false);
                    ui->pushButtonStopAll->setEnabled(false);
                    pingTimer.stop();
                    disconnect(&pingTimer, SIGNAL(timeout()), this, SLOT(sendPing()));
                    ui->pushButtonConnect1 ->setText(tr("Connect"));
                    tcpRobot1->close();
                    tcpRobot2->close();
                    watchDogCnt1 = 2;
                }
        }

        ++watchDogCnt2;
        if (watchDogCnt2 == 1)
        {
                ui->groupBoxMotorControl2->setEnabled(true);
                ui->pushButtonQuery3->setEnabled(true);
                tcpRobot3->write("~MMOD\r");
        }
        else
        {
                if (watchDogCnt2> 10)
                {
                    ui->groupBoxMotorControl2->setEnabled(false);
                    ui->pushButtonQuery3->setEnabled(false);
                    ui->pushButtonQuery4->setEnabled(false);

                    ui->pushButtonConnect2 ->setText(tr("Connect"));
                    tcpRobot3->close();

                    watchDogCnt2 = 2;
                }
        }
}


double MainWindow:: trans2Angle(int channel)
{
    double angle = 0;
    //calculate the angle and position
    double delta = jointMotorData[channel].encoderPos - jointMotorData[channel].iniPos;


    delta =  delta / jointMotorData[channel].resolution;
    angle = JOINT_INI_ANGLE[channel ] - delta;
    return angle;
}

void MainWindow:: getPositionXY()
 {
   double x = 0;
   double y = 0;
   double thelta1 = jointMotorData[0].angle;
   double thelta2 = jointMotorData[1].angle;
   double armX1 = ((ARMLEN1 * qCos(thelta1)) );
   double armY1 = ((ARMLEN1 * qSin(thelta1)) );

   double armX2 = armX1 + (ARMLEN2 * qCos(thelta2) );
   double armY2 = armY1 + (ARMLEN2 * qSin(thelta2) );
   x = armX2 + ARMLEN31 * qCos(tipAngle);
   y = armY2 + ARMLEN31 * qSin(tipAngle);
   manipulatorPos[0] = x;
   manipulatorPos[1] = y;
 }

void MainWindow:: j1GoCmd()
{
    double targetAngle = 0;
    double deltaAngle = 0;
    int cmd = 0;
    int targetPos = 0;
    QString cmdStr = "";
    QByteArray cmdByte;
    try
    {
        targetAngle = ui->lineEditJ1TargetAngle->text().toDouble();
        targetAngle = targetAngle /180 * M_PI;
        deltaAngle = targetAngle - jointMotorData[0].angle; //
        cmd = (int)(-deltaAngle * jointMotorData[0].resolution);
        targetPos = cmd + jointMotorData[0].encoderPos;
        if ((targetPos >= jointMotorData[0].limitPos1) && (targetPos <= jointMotorData[0].limitPos2))
        {
            if (!jointMotorData[0].protect)
            {
                if (tcpRobot1 != NULL){
                    if (tcpRobot1->isWritable()){
                        cmdStr = "!PR 1 " + QString::number(cmd) +"\r";
                        cmdByte = cmdStr.toLatin1();

                        tcpRobot1->write(cmdByte.data());
                    }
                }
            }
        }

    }
    catch(...)
    {

    }
}

void MainWindow:: j2GoCmd()
{
    double targetAngle = 0;
    double deltaAngle = 0;
    int cmd = 0;
    int targetPos = 0;
    QString cmdStr = "";
    QByteArray cmdByte;
    try
    {
        targetAngle = ui->lineEditJ2TargetAngle->text().toDouble();
        targetAngle = targetAngle /180 * M_PI;
        deltaAngle = targetAngle - jointMotorData[1].angle;  //[1].
        cmd = -(int)(deltaAngle * jointMotorData[1].resolution);
        targetPos = cmd + jointMotorData[1].encoderPos;
        if ((targetPos >= jointMotorData[1].limitPos1) && (targetPos <= jointMotorData[1].limitPos2))
        {
            if (!jointMotorData[1].protect)
            {
                if (tcpRobot1 != NULL){
                    if (tcpRobot1->isWritable()){
                        cmdStr = "!PR 2 " + QString::number(cmd) +"\r";
                        cmdByte = cmdStr.toLatin1();

                        tcpRobot1->write(cmdByte.data());
                    }
                }
            }
        }

    }
    catch(...)
    {

    }
}

void MainWindow:: rotateGoCmd()
{
    double targetAngle = 0;
    double deltaAngle = 0;
    int cmd = 0;
    int targetPos = 0;
    QString cmdStr = "";
    QByteArray cmdByte;
    try
    {
        targetAngle = ui->lineEditRotateTargetAngle->text().toDouble();
        targetAngle = targetAngle /180 * M_PI;
        deltaAngle = targetAngle - jointMotorData[2].angle;
        cmd = (int)(-deltaAngle * jointMotorData[2].resolution);
        targetPos = cmd + jointMotorData[2].encoderPos;
        if ((targetPos >= jointMotorData[2].limitPos1) && (targetPos <= jointMotorData[2].limitPos2))
        {
            if (!jointMotorData[2].protect)
            {
                if (tcpRobot2 != NULL){
                    if (tcpRobot2->isWritable()){
                        cmdStr = "!PR 1 " + QString::number(cmd) +"\r";
                        cmdByte = cmdStr.toLatin1();

                        tcpRobot2->write(cmdByte.data());
                    }
                }
            }
        }

    }
    catch(...)
    {

    }
}

void MainWindow:: tipGoCmd()
{
    double targetAngle = 0;
    double deltaAngle = 0;
    int cmd = 0;
    int targetPos = 0;
    QString cmdStr = "";
    QByteArray cmdByte;
    try
    {
        targetAngle = ui->lineEditTipTargetAngle->text().toDouble();
        targetAngle = targetAngle /180 * M_PI;
        deltaAngle = targetAngle - tipAngle;
        cmd = (int)(deltaAngle * jointMotorData[5].resolution);
        targetPos = cmd + jointMotorData[5].encoderPos;
        if ((targetPos >= jointMotorData[5].limitPos1) && (targetPos <= jointMotorData[5].limitPos2))
        {
            if (!jointMotorData[5].protect)
            {
                if (tcpRobot3 != NULL){
                    if (tcpRobot3->isWritable()){
                        cmdStr = "!PR 2 " + QString::number(cmd) +"\r";
                        cmdByte = cmdStr.toLatin1();

                        tcpRobot3->write(cmdByte.data());
                    }
                }
            }
        }

    }
    catch(...)
    {

    }

}

void MainWindow:: panGoCmd()
{
    double targetAngle = 0;
    double deltaAngle = 0;
    int cmd = 0;
    int targetPos = 0;
    QString cmdStr = "";
    QByteArray cmdByte;
    try
    {
        targetAngle = ui->lineEditPanTargetAngle->text().toDouble();
        targetAngle = targetAngle /180 * M_PI;
        deltaAngle = targetAngle - jointMotorData[4].angle;
        cmd = (int)(deltaAngle * jointMotorData[4].resolution);
        targetPos = cmd + jointMotorData[4].encoderPos;
        if ((targetPos >= jointMotorData[4].limitPos1) && (targetPos <= jointMotorData[4].limitPos2))
        {
            if (!jointMotorData[4].protect)
            {
                if (tcpRobot3 != NULL){
                    if (tcpRobot3->isWritable()){
                        cmdStr = "!PR 1 " + QString::number(cmd) +"\r";
                        cmdByte = cmdStr.toLatin1();

                        tcpRobot3->write(cmdByte.data());
                    }
                }
            }
        }

    }
    catch(...)
    {

    }
}

void MainWindow:: armPosGoCmd()
{
    QString QStrTemp = "";
    QString cmdStr = "";
    QByteArray cmdByte;
    double tipTargetAngle = 0;
    double tipEndX = 0;
    double tipEndY = 0;
    double x = 0;
    double y = 0;
    try
    {
        tipTargetAngle = ui->lineEditTipAngle->text().toDouble()* M_PI/180;
        tipEndX = ui->lineEditArmTargetX->text().toDouble();
        tipEndY = ui->lineEditArmTargetY->text().toDouble();
        x = tipEndX - ARMLEN31 * qCos(tipTargetAngle);
        y = tipEndY - ARMLEN31 * qSin(tipTargetAngle);
        double l1 = ARMLEN1;
        double l2 = ARMLEN2;
        double s2 = 0;
        double thelta1 = 0;     //joint 1 angle

        double thelta2 = 0;     //joint 2 angle

        double belta = 0;
        double alpha = 0;
        double c2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);

        if (abs(c2) > 1)
        {
            //out of space
            thelta1 = -100;
            thelta2 = -100;

        }
        else
        {
            double r_2 = x*x + y * y;
            double l_sq = l1 * l1 + l2 * l2;
            double term2 = (r_2 -l_sq)/(2 *l1 * l2);
            double term1 = - (qSqrt(1 - term2 * term2));
            thelta2 = qAtan2(term1,term2);
            //option
            //thelta2 = - thelta2;
            double k1 = l1 + l2 * qCos(thelta2);
            double k2 = l2 * qSin(thelta2);
            double r = qSqrt(k1* k1 + k2 * k2);
            double gamma = qAtan2(k2,k1);
            thelta1 = qAtan2(y,x) - gamma;
            thelta2 = thelta1 + thelta2;

        }
        QStrTemp.setNum(thelta1 / M_PI * 180, 'f',2);
        ui->lineEditJ1TargetAngle->setText(QStrTemp);
        QStrTemp.setNum(thelta2 / M_PI * 180, 'f',2);
        ui->lineEditJ2TargetAngle->setText(QStrTemp);

        double tipDeltaAngle = tipTargetAngle - thelta2;
     //   QStrTemp.setNum(tipDeltaAngle / M_PI * 180, 'f',2);
    //    ui->lineEditTipTargetAngle->setText(QStrTemp);


        //translate the joint1,2 target angle to target endoer position
        double deltaAngle = thelta1 - jointMotorData[0].angle; //
        int cmd = (int)(-deltaAngle * jointMotorData[0].resolution);
        int targetPos = cmd + jointMotorData[0].encoderPos;
        if ((targetPos >= jointMotorData[0].limitPos1) && (targetPos <= jointMotorData[0].limitPos2))
        {
            if (!jointMotorData[0].protect)
            {
                if (tcpRobot1 != NULL){
                    if (tcpRobot1->isWritable()){
                        cmdStr = "!PR 1 " + QString::number(cmd) +"\r";
                        cmdByte = cmdStr.toLatin1();

                        tcpRobot1->write(cmdByte.data());
                    }
                }
            }
        }

        deltaAngle = thelta2 - jointMotorData[1].angle;  //[1].
        cmd = -(int)(deltaAngle * jointMotorData[1].resolution);
        targetPos = cmd + jointMotorData[1].encoderPos;
        if ((targetPos >= jointMotorData[1].limitPos1) && (targetPos <= jointMotorData[1].limitPos2))
        {
            if (!jointMotorData[1].protect)
            {
                if (tcpRobot1 != NULL){
                    if (tcpRobot1->isWritable()){
                        cmdStr = "!PR 2 " + QString::number(cmd) +"\r";
                        cmdByte = cmdStr.toLatin1();

                        tcpRobot1->write(cmdByte.data());
                    }
                }
            }
        }
         //tip angle translate to joint 5 angle
        deltaAngle = tipDeltaAngle - JOINT_INI_ANGLE[5];
        cmd = (int)(deltaAngle * jointMotorData[5].resolution);
        targetPos = cmd + jointMotorData[5].iniPos;
        if ((targetPos >= jointMotorData[5].limitPos1) && (targetPos <= jointMotorData[5].limitPos2))
        {
            if (!jointMotorData[5].protect)
            {
                if (tcpRobot3 != NULL){
                    if (tcpRobot3->isWritable()){
                        cmdStr = "!P 2 " + QString::number(targetPos) +"\r";
                        cmdByte = cmdStr.toLatin1();

                        tcpRobot3->write(cmdByte.data());
                    }
                }
            }
        }

        for (int i = 0; i < 2; i++)
        {
            jointMotorData[i].stuckFlag = false;
            jointMotorData[i].protect = false;
            jointMotorData[i].manipulatorStuckCnt = 0;
        }


    }
    catch(...){

    }
}

void MainWindow:: armResetCmd()
{
    //maybe we need a timer to do the reset step
    QString cmdStr = "";
    QByteArray cmdByte;
   for (int i = 0; i < 6; i++)
   {
       jointMotorData[i].protect = false;
       jointMotorData[i].stuckFlag = false;
       jointMotorData[i].manipulatorStuckCnt = 0;
       jointMotorData[i].jointJoy = false;
   }
    if (tcpRobot1 != NULL){
        if (tcpRobot1->isWritable()){
            cmdStr = "!P 1 " + QString::number(jointMotorData[0].iniPos)
                    + "_!P 2 " +QString::number(jointMotorData[1].iniPos) +  "\r";
            cmdByte = cmdStr.toLatin1();

            tcpRobot1->write(cmdByte.data());
        }
    }

    if (tcpRobot2 != NULL){
        if (tcpRobot2->isWritable()){
            cmdStr = "!P 1 " + QString::number(jointMotorData[2].iniPos) + "\r";
            cmdByte = cmdStr.toLatin1();

            tcpRobot2->write(cmdByte.data());
        }
    }
    if (tcpRobot3 != NULL){
        if (tcpRobot3->isWritable()){
            cmdStr = "!P 1 " + QString::number(jointMotorData[4].iniPos)
                    + "_!P 2 " +QString::number(jointMotorData[5].iniPos) +  "\r";
            cmdByte = cmdStr.toLatin1();

            tcpRobot3->write(cmdByte.data());
        }
    }
}

void MainWindow:: armSetIniCmd()
{
    int temp = 0;
    if (ui->pushButtonArmSetIni->text() == "SetIni")
    {
        ui->pushButtonArmSetIni->setText("UnSetIni");
        armIniFlag = true;
        jointMotorData[0].iniPos = jointMotorData[0].encoderPos;
        jointMotorData[1].iniPos = jointMotorData[1].encoderPos;
        jointMotorData[2].iniPos = jointMotorData[2].encoderPos;
        jointMotorData[3].iniPos = jointMotorData[3].encoderPos;
        jointMotorData[4].iniPos = jointMotorData[4].encoderPos;
        jointMotorData[5].iniPos = jointMotorData[5].encoderPos;
        jointMotorData[0].angle = M_PI - M_PI * J1_OFFSET / 180;
        jointMotorData[1].angle = M_PI - M_PI * J2_OFFSET / 180;
        jointMotorData[2].angle = 0;
        jointMotorData[4].angle = 0;
        jointMotorData[5].angle = 0;
         //set some limitation here
        //joint1 half circle
        jointMotorData[0].limitPos1 = jointMotorData[0].iniPos ;
        temp  = (int)(jointMotorData[0].iniPos + jointMotorData [0].circleCnt/2);
        jointMotorData[0].limitPos2 = temp;
        //joint2 almost one circle
        jointMotorData[1].limitPos2 = (int)(jointMotorData[1].iniPos + jointMotorData[1].circleCnt/2);
        temp = jointMotorData[1].iniPos - (int)(jointMotorData[1].circleCnt / 2);
        jointMotorData[1].limitPos1 = temp;

        //joint3 rotate +/- one circle
        int temp = (int)(jointMotorData[2].iniPos + jointMotorData [2].circleCnt) ;
        jointMotorData[2].limitPos2 = temp;
        temp = (int)(jointMotorData[2].iniPos - jointMotorData[2].circleCnt);
        jointMotorData[2].limitPos1 = temp;

        //open/close clipper
     //   temp = jointMotorData[3].iniPos + 5 * jointMotorData[3].circleCnt;
     //   jointMotorData[3].limitPos2 = temp;
     //   temp = jointMotorData[3].iniPos;
     //   jointMotorData[3].limitPos1 = temp;

        //joint5 pan
        jointMotorData[4].limitPos1 = (int)(jointMotorData[4].iniPos - jointMotorData[4].circleCnt / 2);
        jointMotorData[4].limitPos2 = (int)(jointMotorData[4].iniPos + jointMotorData[4].circleCnt / 2);


        //joint6 tilt
        jointMotorData[5].limitPos1 = (int)(jointMotorData[5].iniPos - jointMotorData[5].circleCnt / 2);
        jointMotorData[5].limitPos2 = (int)(jointMotorData[5].iniPos + jointMotorData[5].circleCnt / 2);
        ui->pushButtonArmReset->setEnabled(true);
        ui->pushButtonJ1Go->setEnabled(true);
        ui->pushButtonJ2Go->setEnabled(true);
        ui->pushButtonRotateGo->setEnabled(true);
        ui->pushButtonTipGo->setEnabled(true);
        ui->pushButtonPanGo->setEnabled(true);
        ui->pushButtonArmTargetGo->setEnabled(true);
    }
    else
    {
        ui->pushButtonArmSetIni->setText("SetIni");
        armIniFlag = false;
        ui->pushButtonArmReset->setEnabled(false);
        ui->pushButtonJ1Go->setEnabled(false);
        ui->pushButtonJ2Go->setEnabled(false);
        ui->pushButtonRotateGo->setEnabled(false);
        ui->pushButtonTipGo->setEnabled(false);
        ui->pushButtonPanGo->setEnabled(false);
        ui->pushButtonArmTargetGo->setEnabled(false);
    }



}
} //namespace
