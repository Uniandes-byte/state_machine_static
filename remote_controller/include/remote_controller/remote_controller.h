/*
 * //======================================================================//
 * //  This software is free: you can redistribute it and/or modify        //
 * //  it under the terms of the GNU General Public License Version 3,     //
 * //  as published by the Free Software Foundation.                       //
 * //  This software is distributed in the hope that it will be useful,    //
 * //  but WITHOUT ANY WARRANTY; without even the implied warranty of      //
 * //  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE..  See the      //
 * //  GNU General Public License for more details.                        //
 * //  You should have received a copy of the GNU General Public License   //
 * //  Version 3 in the file COPYING that came with this distribution.     //
 * //  If not, see <http://www.gnu.org/licenses/>                          //
 * //======================================================================//
 * //                                                                      //
 * //      Copyright (c) 2019 SinfonIA Pepper RoboCup Team                 //             
 * //      Sinfonia - Colombia                                             //
 * //      https://sinfoniateam.github.io/sinfonia/index.html              //
 * //                                                                      //
 * //======================================================================//
 */

#ifndef RemoteController_H
#define RemoteController_H

#include <QMainWindow>
#include <QWidget>
#include <QApplication>
#include <QGridLayout>
#include <QLineEdit>
#include <QComboBox>
#include <QPushButton>
#include <QRect>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <string>
#include <QStringList>
#include <QColorDialog>
#include <QLabel>
#include <QTimer>
#include <QDoubleSpinBox>
#include <QKeyEvent>
#include <QVector>
#include <QObject>
#include <QProcess>
#include <QCheckBox>
#include <iostream>
#include <QFile>
#include <QIODevice>
#include <QTextStream>
#include <QTextEdit>
#include <QFile>
#include <QFileDialog>
#include <QPlainTextEdit>
#include <QListWidgetItem>
#include <QPalette>
#include "std_msgs/String.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "remote_controller/camera_thread.h"
#include "remote_controller/pepper_speech.h"
#include "remote_controller/pepper_movement.h"
#include "remote_controller/pepper_animations.h"
#include "remote_controller/controller_thread.h"
#include "remote_controller/rviz_thread.h"
#include "remote_controller/pepper_leds.h"
#include "remote_controller/pepper_activation.h"
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "robot_toolkit_msgs/navigation_tools_srv.h"
#include "robot_toolkit_msgs/navigation_tools_msg.h"

#include "robot_toolkit_msgs/vision_tools_srv.h"
#include "robot_toolkit_msgs/vision_tools_msg.h"

#include "robot_toolkit_msgs/motion_tools_srv.h"

#include "robot_toolkit_msgs/camera_parameters_msg.h"
#include "robot_toolkit_msgs/audio_tools_srv.h"
#include "robot_toolkit_msgs/audio_tools_msg.h"
#include "robot_toolkit_msgs/depth_to_laser_msg.h"

#include "robot_toolkit_msgs/misc_tools_msg.h"
#include "robot_toolkit_msgs/misc_tools_srv.h"

#include "robot_toolkit_msgs/special_settings_msg.h"

// T2S TEXT DEFINITIONS
#define BT_SAY_TXT "Say"
#define BT_SAY0_TXT "Hello"
#define BT_SAY1_TXT "How are you?"
#define BT_SAY2_TXT "My name is Opera"
#define BT_SAY3_TXT "Bye"
#define BT_RVIZ_TXT "Launch RViz"

// ANIMATIONS TEXT DEFINITIONS
#define BT_ANIM0_TXT "Standard Position"
#define BT_ANIM1_TXT "Play Animation"
#define BT_ANIM2_TXT "Navigation Position"

using namespace std;

class MainWindow;
class RemoteController : public QMainWindow
{
  Q_OBJECT

  public:
    RemoteController(int xw, int yw);
    virtual ~RemoteController();
    void setNavigationServiceParameters(QString command, bool tfEnable, float tfFrequency, bool odomEnable, float odomFrequency, bool laserEnable, float laserFrequency, bool cmdEnable, float securityTimer, bool moveBaseEnable, bool goalEnable, bool robotPoseSubscriberEnable, bool pathEnable, float pathFrecuency, bool robotPosePublisherEnable, float robotPosePublisherFrecuency, bool resultEnable, bool depthToLaserEnable, int depthToLaserResolution, float pScanTime, float pRangeMin, float pRangeMax, float pScanHeight, bool freeZone);
    void setVisionServiceParameters(QString cameraName, QString command, int resolution, int frame_rate, int color_space, robot_toolkit_msgs::camera_parameters_msg cameraParameters);
    void setAudioServiceParameters(QString command, int frequency, int channels);
    void setMotionServiceParameters(bool pEnable);
    void setMiscServiceParameters(bool pEnable);

  private:
    void printInfo(QString info);
    void printError(QString error);
    QGridLayout *mainLayout, *optionsLayout, *camerasLayout, *buttonLayout, *cameraSelectionLayout;
    QVBoxLayout *controlLayout, *t2sLayout, *textToSpeechParamLayout;
    QHBoxLayout *inputLayout, *dualShockLayout, *languageLayout;
    
    QPalette p;
    double speed, angle, talkingSpeedRate;
    int lastKeyPressed, controlInterface, jointToMove;
    CameraThread *rosSpinThread;

    QString *_currentAnimation;
    
    PepperMovement *pepMov;
    PepperSpeech   *pepSpeech;
    PepperAnimations *pepAnim;
    PepperLeds *pepLeds;
    PepperActivation *pepActiv;

    QTimer *commandTimer;
    QString speechLanguage;

    QProcess *remoteControllerProcess;
    QLineEdit *movementUpdate;
    
    QPushButton *_sayButton, *_say0, *_say1, *_say2, *_say3;

    // Global layouts
    QHBoxLayout *_lyMain;
    QVBoxLayout *_lyLeft, *_lyRight, *_lyCentral;

    // Logo objects
    QHBoxLayout *_lyLogo;
    QLabel *_lbLabel;

    // LEDs controller
    QVBoxLayout *_lyLeds;
    QLabel *_lbLeds;
    QGridLayout *_lyLedButtons;
    QPushButton *_bLeftEye, *_bRightEye, *_bEyes, *_bEars, *_bResetColors;
    QColorDialog *_leftEye, *_rightEye, *_Eyes, *_Ears;
    QColor *_currentLeftEyeColor, *_currentRightEyeColor, *_currentEyesColor, *_currentEarsColor;

    // Movement objects
    QVBoxLayout *_lyMovement;
    QGridLayout *_lyMovementButtons, *_lyMovementSpeed;
    QLabel *_lbMovementLabel, *_lbMovementSpeed;
    QComboBox *_cbControlSelection;
    QPushButton *_btSynchController, *_btStartJoystickButton;
    QDoubleSpinBox *_sbMovementSpeed;
    QCheckBox *_cBJointAngle;

    // Animations objects
    QVBoxLayout *_lyAnimations;
    QGridLayout *_lyAnimationsButtons, *_lyAnimations2;
    QLabel *_lbAnimations;
    QPushButton *_btAnim0, *_btAnim1, *_btAnim2;
    QComboBox *_cbAnimSelector;
    QTimer *_tmrHeadNavigation;

    // Text to speech objects
    QLabel      *_lbT2S;
    QVBoxLayout *_lyT2S, *_lyT2SParam;
    QHBoxLayout *_lyT2SInput;
    QGridLayout *_lyT2SButtons;
    QComboBox   *_cbLanguage;
    QTextEdit   *_teT2S;
    QCheckBox   *_cbContextual;
    QPushButton *_btSay, *_btSay0, *_btSay1, *_btSay2, *_btSay3;

    // Console objects
    QVBoxLayout     *_lyConsole;
    QLabel          *_lbConsole;
    QPlainTextEdit  *_pteConsole;

    // Global setup objects
    QVBoxLayout *_lyGlobal;
    QGridLayout *_lyEnables;
    QLabel *_lbGlobalSetup;
    QPushButton *_btEnableAll, *_btDisableAll;
    QCheckBox *_cbEnable1, *_cbEnable2, *_cbEnable3, *_cbEnable4, *_cbEnable5, *_cbEnableLeds;

    // Camera objects
    QVBoxLayout *_lyCamera;
    QHBoxLayout *_lyCameraSel;
    QLabel *_lbCamera, *_lbBottom, *_lbTop;
    QCheckBox *_cbBottom, *_cbTop;

    // Rviz objects
    QVBoxLayout *_lyRviz;
    QGridLayout *_lyRvizSettings;
    QLabel *_lbRviz;
    QCheckBox *_cbRvizConfig0, *_cbRvizConfig1, *_cbRvizConfig2, *_cbRvizConfig3, *_cbRvizConfig4, *_cbRvizConfig5;
    QPushButton *_btRvizLaunch;

    // Pepper activation objects
    QVBoxLayout *_lyActivation;
    QLabel *_lbActivation;
    QGridLayout *_lyActivationButtons;
    QPushButton *_btActivate, *_btAutoLife, *_btCollisions;

    // Script objectos
    QVBoxLayout *_lyScript;
    QLabel *_lbScript;
    QPushButton *_btLoad, *_btClear;
    QGridLayout *_lyScriptInfo;
    QLabel *_lbScriptName, *_lbScriptLanguage, *_lbScriptState;
    QGridLayout *_lyScriptButtons;
    QPushButton *_btPrev, *_btPlay, *_btNext;
    QListWidget *_lwScript;
    QString *_fileScriptName;
    QFile *_fileScript;
    QStringList *_scriptStateList, *_scriptAnimationsList, *_scriptTextList;
    QString _scriptLanguage, _talkingSpeed;

    // Deprecated coming soon
    QString textT2S;
    QCheckBox *_bottomCameraCheckBox, *_frontCameraCheckBox;
    QLabel *cameraLabel, *appName, *bottomCameraLabel, *frontCameraLabel, *controlInterfaceName, *t2sLabel, *rvizLabel, *consoleLabel;
    QPixmap *imagePixmap, *logoPixmap;
    RvizThread *myRvizThread;
    ControllerThread *myControllerThread;
    ros::NodeHandle *_nodeHandle;
    ros::Subscriber bottomSub, frontSub;
    ros::ServiceClient navigationClient, visionClient, audioClient, motionClient, miscClient;
    robot_toolkit_msgs::navigation_tools_srv navigationService;
    robot_toolkit_msgs::vision_tools_srv visionService;
    robot_toolkit_msgs::audio_tools_srv audioService;
    robot_toolkit_msgs::motion_tools_srv motionService;
    robot_toolkit_msgs::misc_tools_srv miscService;

  protected:
    void keyPressEvent(QKeyEvent *) override;
    void keyReleaseEvent(QKeyEvent *) override;
    void closeEvent(QCloseEvent *) override;
    
  private slots:
    // Movement slots
    void changedControlInterfaceSlot(int pIndex);
    void changeJointToMoveSlot(int pJoint);
    void synchronizeControllerSlot();
    void readyReadyStandardOutputSlot();
    void processEnded(int exitCode, QProcess::ExitStatus exitStatus);
    void updateControllerCommands(int* controllerAxes);
    void startJoystickSlot();
    void updatedMovementSpeedSlot(double pSpeed);
    
    // RViz slot
    void clickedRvizButton();
    
    // Camera slots
    void bottomCameraSlot(uchar *data, int cols, int rows);
    void frontCameraSlot(uchar *data, int cols, int rows);
    void bottomCameraCheckBoxSlot(int state);
    void frontCameraCheckBoxSlot(int state);
    void textToSpeechSlot();

    // T2S slots
    void languageSelectionSlot(int index);
    void say0TextToSpeechSlot();
    void say1TextToSpeechSlot();
    void say2TextToSpeechSlot();
    void say3TextToSpeechSlot();

    //Leds slots
    void sendLeftEyeColorSlot();
    void selectColorLeftEyeColorSlot(QColor leftEyeColor);
    void sendRightEyeColorSlot();
    void selectColorRightEyeColorSlot(QColor rightEyeColor);
    void sendEyesColorSlot();
    void selectColorEyesColorSlot(QColor EyesColor);
    void sendEarsColorSlot();
    void selectColorEarsColorSlot(QColor EarsColor);
    void resetColors();
    
    // Animation slots
    void anim0Slot();
    void anim1Slot();
    void anim2Slot();
    void changedAnimationSlot(int pAnim);
    void updateHeadMovementNavSlot();

    // Global setup slots
    void enableAllSlot();
    void disableAllSlot();
    void navigationEnableCheckBoxSlot(int pState);
    void animationEnableCheckBoxSlot(int pState);
    void visionEnableCheckBoxSlot(int pState);
    void audioEnableCheckBoxSlot(int pState);
    void ledsEnableCheckBoxSlot(int pState);

    // Activation slots
    void avtivateButtonSlot();
    void autoLifeButtonSlot();
    void collisionButtonSlot();

    // Script slots
    void loadButtonSlot();
    void clearButtonSlot();
    void playButtonSlot();
    void prevButtonSlot();
    void nextButtonSlot();

};
#endif // RemoteController
