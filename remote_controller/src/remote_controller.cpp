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

#include "remote_controller/remote_controller.h"

RemoteController::RemoteController(int xw, int yw)
{
  char **argv;
  int argc = 0; 
  ros::init(argc, argv, "remote_controller");
  
  _nodeHandle = new ros::NodeHandle();
  
  pepMov = new PepperMovement();
  pepSpeech = new PepperSpeech();
  pepAnim = new PepperAnimations();
  pepLeds = new PepperLeds();
  pepActiv = new PepperActivation();
  rosSpinThread = new CameraThread();
  myControllerThread = new ControllerThread();
  remoteControllerProcess = new QProcess();

  // Main layouts initialization
  _lyMain = new QHBoxLayout();
  _lyLeft = new QVBoxLayout();
  _lyCentral = new QVBoxLayout();
  _lyRight = new QVBoxLayout();
  
  //------------------------------------------------------
  // Logo layout objects intialization
  _lyLogo = new QHBoxLayout();
  _lbLabel = new QLabel("");
  //------------------------------------------------------

  //------------------------------------------------------
  // Leds layout
  _lyLeds = new QVBoxLayout();
  _lbLeds = new QLabel("<b>LEDs</b>");
  _lyLedButtons = new QGridLayout();
  _bLeftEye = new QPushButton("Left eye color");
  _leftEye = new QColorDialog();
  _bRightEye = new QPushButton("Right eye color");
  _rightEye = new QColorDialog();
  _bEyes = new QPushButton("Eyes color");
  _Eyes = new QColorDialog();
  _bEars = new QPushButton("Ears color");
  _Ears = new QColorDialog();
  _bResetColors = new QPushButton("Reset Colors");
  _currentLeftEyeColor = new QColor();
  _currentRightEyeColor = new QColor();
  _currentEyesColor = new QColor();
  _currentEarsColor = new QColor();
  
  //------------------------------------------------------
  
  
  //------------------------------------------------------
  // Movement layout objects initialization
  _lyMovement = new QVBoxLayout();
  _lyMovementButtons = new QGridLayout();
  _lbMovementLabel = new QLabel("<b>Movement</b>");
  _cbControlSelection = new QComboBox();
  _cBJointAngle = new QCheckBox("Move Head");
  _btSynchController = new QPushButton("Synchronize DualShock4");
  _btStartJoystickButton = new QPushButton("Start DualShock4");
  _btSynchController = new QPushButton("Synchronize");
  _btStartJoystickButton = new QPushButton("Start");
  _lyMovementSpeed = new QGridLayout();
  _lbMovementSpeed = new QLabel("Pepper's movement speed: ");
  _sbMovementSpeed = new QDoubleSpinBox();
  //-------------------------------------------------------
	
	//-------------------------------------------------------  
  // Animations objects initialization
  _lyAnimations = new QVBoxLayout();
  _lyAnimationsButtons = new QGridLayout();
  _lyAnimations2 = new QGridLayout();
  _lbAnimations = new QLabel("<b>Animations</b>");

  _cbAnimSelector = new QComboBox();
  _currentAnimation = new QString();
  _btAnim0 = new QPushButton(BT_ANIM0_TXT);
  _btAnim1 = new QPushButton(BT_ANIM1_TXT);
  _btAnim2 = new QPushButton(BT_ANIM2_TXT);
  _tmrHeadNavigation = new QTimer();
  //-------------------------------------------------------

  //-------------------------------------------------------  
  // Text to speech objects initialization
  _lyT2S = new QVBoxLayout();
  _lyT2SInput = new QHBoxLayout();
  _lyT2SParam = new QVBoxLayout();
  _lyT2SButtons = new QGridLayout();
  _lbT2S = new QLabel("<b>Text to Speech</b>");
  _cbLanguage = new QComboBox();
  _btSay = new QPushButton(BT_SAY_TXT);
  _btSay0 = new QPushButton(BT_SAY0_TXT);
  _btSay1 = new QPushButton(BT_SAY1_TXT);
  _btSay2 = new QPushButton(BT_SAY2_TXT);
  _btSay3 = new QPushButton(BT_SAY3_TXT);
  _teT2S = new QTextEdit("");
  _cbContextual = new QCheckBox("Contextual");
  //-------------------------------------------------------

  //-------------------------------------------------------
  // Console objects intialization
  _lyConsole = new QVBoxLayout();
  _lbConsole = new QLabel("<b>Console</b>");
  _pteConsole = new QPlainTextEdit();
  //------------------------------------------------------- 

  //-------------------------------------------------------
  // Global setup objects intialization
  _lyGlobal = new QVBoxLayout();
  _lyEnables = new QGridLayout();
  _btEnableAll = new QPushButton("Enable All");
  _btDisableAll = new QPushButton("Disable All");
  _cbEnable1 = new QCheckBox("Enable Movement");
  _cbEnable2 = new QCheckBox("Enable Animation");
  _cbEnable3 = new QCheckBox("Enable Cameras");
  _cbEnable4 = new QCheckBox("Enable Speech");
  _cbEnable5 = new QCheckBox("Enable Mapper");
  _cbEnableLeds = new QCheckBox("Enable Leds");
  _lbGlobalSetup = new QLabel("<b>Global Setup</b> (Enable Topics)");
  //-------------------------------------------------------

  //-------------------------------------------------------
  // Camera objects intialization
  _lyCamera = new QVBoxLayout();
  _lyCameraSel = new QHBoxLayout();
  _lbCamera = new QLabel("<b>Camera<b>");
  _cbBottom= new QCheckBox("Bottom Camera");
  _cbTop= new QCheckBox("Front Camera");
  _lbBottom = new QLabel("");
  _lbTop = new QLabel("");
  //-------------------------------------------------------

  //-------------------------------------------------------
  // Rviz objects initialization
  _lyRviz = new QVBoxLayout();
  _lyRvizSettings = new QGridLayout();
  _lbRviz = new QLabel("<b>RViz</b>");
  commandTimer = new QTimer();
  myRvizThread = new RvizThread();
  
  _cbRvizConfig0 = new QCheckBox("Map");
  _cbRvizConfig1 = new QCheckBox("Laser");
  _cbRvizConfig2 = new QCheckBox("3D Camera Image");
  _cbRvizConfig3 = new QCheckBox("3D Depth Cloud");
  _cbRvizConfig4 = new QCheckBox("tf");
  _cbRvizConfig5 = new QCheckBox("Depth Cloud Laser");
  _btRvizLaunch = new QPushButton(BT_RVIZ_TXT);
  //-------------------------------------------------------

  //-------------------------------------------------------  
  // Global Activation
  _lyActivation = new QVBoxLayout();
  _lbActivation = new QLabel("<b>Global Activation</b>");
  _lyActivationButtons = new QGridLayout();
  _btActivate = new QPushButton("Stand up");
  _btAutoLife = new QPushButton("Turn off awarness");
  _btCollisions = new QPushButton("Turn off security");
  //-------------------------------------------------------  

  //-------------------------------------------------------  
  // Script 
  _lyScript = new QVBoxLayout();
  _lbScript = new QLabel("<b>Script</b>");
  _btLoad = new QPushButton("Load Script File");
  _btClear = new QPushButton("Clear Script");
  _lyScriptInfo = new QGridLayout();
  _lbScriptName = new QLabel("<b>Script Name:<b>");
  _lbScriptLanguage = new QLabel("<b>Script Language:<b>");
  _lbScriptState = new QLabel("<b>Script State:<b>");
  _lyScriptButtons = new QGridLayout();
  _btPrev = new	QPushButton("<<");
  _btPlay = new QPushButton("Play");
  _btNext = new QPushButton(">>");

  _lwScript = new QListWidget();
  _fileScriptName = new QString();
  _fileScript = new QFile();
  _scriptTextList = new QStringList();
  _scriptAnimationsList = new QStringList();

  //-------------------------------------------------------
  // Console
  consoleLabel = new QLabel("<b>Console<b>");
  imagePixmap = new QPixmap();
  logoPixmap = new QPixmap();
  QWidget *window = new QWidget();
  
  //-------------------------------------------------------
  // Logo layout setup
  _lyLogo->addWidget(_lbLabel, 0);

  //-------------------------------------------------------
  // Controller buttons layout setup
  _lyMovementButtons->addWidget(_btSynchController, 0 , 0);
  _lyMovementButtons->addWidget(_btStartJoystickButton, 0 , 1);

  _lyMovementSpeed->addWidget(_lbMovementSpeed, 0, 0);
  _lyMovementSpeed->addWidget(_sbMovementSpeed, 0, 1);

  // Movement layout setup
  _lyMovement->addWidget(_lbMovementLabel);
  _lyMovement->addWidget(_cbControlSelection);
  _lyMovement->addWidget(_cBJointAngle);
  _lyMovement->addLayout(_lyMovementButtons);
  _lyMovement->addLayout(_lyMovementSpeed);

  //-------------------------------------------------------

  //-------------------------------------------------------
  // Animations layout setup
  _lyAnimations2->addWidget(_cbAnimSelector, 0, 0);
  _lyAnimations2->addWidget(_btAnim1, 0, 1);
  
  _lyAnimationsButtons->addWidget(_btAnim0, 0, 0);
  _lyAnimationsButtons->addWidget(_btAnim2, 0, 1);

  _lyAnimations->addWidget(_lbAnimations);
  _lyAnimations->addLayout(_lyAnimations2);
  _lyAnimations->addLayout(_lyAnimationsButtons);
  
  //-------------------------------------------------------
  
  //-------------------------------------------------------
  // Leds layout setup
  _lyLedButtons->addWidget(_bLeftEye, 0, 0);
  _lyLedButtons->addWidget(_bRightEye, 0, 1);
  _lyLedButtons->addWidget(_bEyes, 1, 0);
  _lyLedButtons->addWidget(_bEars, 1, 1);
  
  _lyLeds->addWidget(_lbLeds);
  _lyLeds->addLayout(_lyLedButtons);
  _lyLeds->addWidget(_bResetColors);
  
  //-------------------------------------------------------
  // Say/Contextual layout setup
  _lyT2SParam->addWidget(_btSay);
  _lyT2SParam->addWidget(_cbContextual);

  // Input layout setup
  _lyT2SInput->addWidget(_teT2S);
  _lyT2SInput->addLayout(_lyT2SParam);

  // T2S buttons layout setup
  _lyT2SButtons->addWidget(_btSay0, 0, 0);
  _lyT2SButtons->addWidget(_btSay1, 0, 1);

  // Text to Speech layout setup
  _lyT2S->addWidget(_lbT2S);
  _lyT2S->addWidget(_cbLanguage);
  _lyT2S->addLayout(_lyT2SInput);
  _lyT2S->addLayout(_lyT2SButtons);
  //-------------------------------------------------------

  //-------------------------------------------------------
  // Console layout setup
  //-------------------------------------------------------
  _lyConsole->addWidget(_lbConsole);
  _lyConsole->addWidget(_pteConsole);
  //-------------------------------------------------------

  //-------------------------------------------------------
  // Global setup layout setup
  //-------------------------------------------------------
  _lyEnables->addWidget(_btEnableAll, 0, 0);
  _lyEnables->addWidget(_btDisableAll, 0, 1);
  _lyEnables->addWidget(_cbEnable1, 1, 0);
  _lyEnables->addWidget(_cbEnable2, 1, 1);
  _lyEnables->addWidget(_cbEnable3, 2, 0);
  _lyEnables->addWidget(_cbEnable4, 2, 1);
  _lyEnables->addWidget(_cbEnable5, 3, 0);
  _lyEnables->addWidget(_cbEnableLeds, 3, 1);
  _lyGlobal->addWidget(_lbGlobalSetup);
  _lyGlobal->addLayout(_lyEnables);
  //-------------------------------------------------------

  //-------------------------------------------------------
  // Camera layout setup
  _lyCameraSel->addWidget(_cbTop);
  _lyCameraSel->addWidget(_cbBottom);

  _lyCamera->addWidget(_lbCamera);
  _lyCamera->addLayout(_lyCameraSel);
  _lyCamera->addWidget(_lbTop);
  _lyCamera->addWidget(_lbBottom);
	//-------------------------------------------------------

  //-------------------------------------------------------
  // Rviz layout setup
  _lyRvizSettings->addWidget(_cbRvizConfig2, 1, 1);
  _lyRvizSettings->addWidget(_cbRvizConfig1, 1, 0);
  _lyRvizSettings->addWidget(_cbRvizConfig0, 1, 2);
  _lyRvizSettings->addWidget(_cbRvizConfig3, 2, 1);
  _lyRvizSettings->addWidget(_cbRvizConfig4, 2, 2);
  _lyRvizSettings->addWidget(_cbRvizConfig5, 2, 0);

  _lyRviz->addWidget(_lbRviz);
  _lyRviz->addLayout(_lyRvizSettings);
  _lyRviz->addWidget(_btRvizLaunch);
	//-------------------------------------------------------

	//-------------------------------------------------------
  // Script layout setup
  _lyScriptInfo->addWidget(_lbScriptName, 0, 0);
  _lyScriptInfo->addWidget(_lbScriptLanguage, 1, 0);
  _lyScriptInfo->addWidget(_lbScriptState, 2, 0);

  // Script button layour
  _lyScriptButtons->addWidget(_btPrev, 0, 0);
  _lyScriptButtons->addWidget(_btPlay, 0, 1);
  _lyScriptButtons->addWidget(_btNext, 0, 2);

  //_lyScript->addWidget(_lbScript);
  _lyScript->addWidget(_lbScript);
  _lyScript->addWidget(_btLoad);
  _lyScript->addWidget(_btClear);
  _lyScript->addLayout(_lyScriptInfo);
  _lyScript->addLayout(_lyScriptButtons);
  _lyScript->addWidget(_lwScript);
	//-------------------------------------------------------

  //-------------------------------------------------------
  // Global Activation layout setup
  _lyActivationButtons->addWidget(_btActivate, 0, 0);
  _lyActivationButtons->addWidget(_btAutoLife, 0, 1);
  _lyActivationButtons->addWidget(_btCollisions, 0, 2);

  _lyActivation->addWidget(_lbActivation);
  _lyActivation->addLayout(_lyActivationButtons);
  //-------------------------------------------------------

  //-------------------------------------------------------
  // Main layout setup 
  _lyMain->addLayout(_lyLeft);
  _lyMain->addLayout(_lyCentral);
  _lyMain->addLayout(_lyRight);

  // Left layout setup
  _lyLeft->addLayout(_lyGlobal);
  _lyLeft->addLayout(_lyMovement);
  _lyLeft->addLayout(_lyAnimations);
  _lyLeft->addLayout(_lyLeds);
  _lyLeft->addLayout(_lyT2S);

  // Central layout setup
  _lyCentral->addLayout(_lyLogo);
  _lyCentral->addLayout(_lyActivation);
  _lyCentral->addLayout(_lyScript);
  _lyCentral->addLayout(_lyConsole);

  // Right layout setup
  _lyRight->addLayout(_lyCamera);
  _lyRight->addLayout(_lyRviz);

  //-------------------------------------------------------

  // Main window
  window->setLayout(_lyMain);
  setCentralWidget(window);
  window->setFixedSize(QSize(xw, yw));
  this->setWindowTitle("Remote Controller");
  this->setWindowIcon(QIcon(QString::fromStdString(ros::package::getPath("remote_controller")) + "/resources/SinfonIA.png"));
  
  // Console Init
   p = _pteConsole->palette();
   p.setColor(QPalette::Base, QColor(0, 0, 0));
   p.setColor(QPalette::Text, Qt::white);

  _pteConsole->setPalette(p);
  _pteConsole->setEnabled(false);
  _pteConsole->setMaximumBlockCount(10);
  
  // Movement Init
  _cbControlSelection->addItem("Controller selection");
  _cbControlSelection->addItem("Keyboard");
  _cbControlSelection->addItem("DualShock4");
  _sbMovementSpeed->setRange(0.0,0.5);
  _sbMovementSpeed->setSuffix(" m/s");
  _sbMovementSpeed->setSingleStep(0.05);
  
  // S2T Init
  _cbLanguage->addItem("English");
  _cbLanguage->addItem("Spanish");
  imagePixmap->load("");
  logoPixmap->load(QString::fromStdString(ros::package::getPath("remote_controller")) + "/resources/SinfonIA.png");

  // Custom animations file
  try {
    QFile customAnimationFile(QString::fromStdString(ros::package::getPath("remote_controller")) + "/resources/.sinfoniaanimations");
      QTextStream in(&customAnimationFile);
      while (!in.atEnd()) {
      QString line = in.readLine();
      _cbAnimSelector->addItem(line);
    }
  } catch (int x) {
    printError("Failed to load custom animations file.");
  }

  QFile animationFile(QString::fromStdString(ros::package::getPath("remote_controller")) + "/resources/.animations");

  if (!animationFile.open(QIODevice::ReadOnly | QIODevice::Text))
    printError("Failed to load animations file.");
  else {
    QTextStream in(&animationFile);
    while (!in.atEnd()) {
      QString line = in.readLine();
      _cbAnimSelector->addItem(line);
    }
  }

  // Camera Init
  _lbBottom->setFixedSize(320, 240);
  _lbTop->setFixedSize(320, 240);
  _lbLabel->setScaledContents(true);
  _lbLabel->setFixedSize(0.3*xw, 0.20*yw);
  _lbLabel->setPixmap(*logoPixmap);
  
  // -------- Init Buttons ------
  // Movement Buttons
  _btStartJoystickButton->setEnabled(false);
  _btSynchController->setEnabled(false);
  _cbControlSelection->setEnabled(false);
  // Animation Buttons
  _btAnim0->setEnabled(false);
  _btAnim1->setEnabled(false);
  _btAnim2->setEnabled(false);
  // LEDs Buttons
  _bLeftEye->setEnabled(false);
  _bEyes->setEnabled(false);
  _bEars->setEnabled(false);
  _bRightEye->setEnabled(false);
  _bResetColors->setEnabled(false);
  // Speech Buttons
  _btSay->setEnabled(false);
  _btSay0->setEnabled(false);
  _btSay1->setEnabled(false);
  // Script buttons
  _btPrev->setEnabled(false);
  _btNext->setEnabled(false);
  _btPlay->setEnabled(false);
  
  // ----- Init Variabes ----
  speed = 0.3;
  _sbMovementSpeed->setValue(speed);
  angle = 0.45;
  lastKeyPressed = 0;
  speechLanguage = "English";
  
  // Service lient intialization
  navigationClient = _nodeHandle->serviceClient<robot_toolkit_msgs::navigation_tools_srv>("/robot_toolkit/navigation_tools_srv");
  visionClient = _nodeHandle->serviceClient<robot_toolkit_msgs::vision_tools_srv>("/robot_toolkit/vision_tools_srv");
  audioClient = _nodeHandle->serviceClient<robot_toolkit_msgs::audio_tools_srv>("/robot_toolkit/audio_tools_srv");
  motionClient = _nodeHandle->serviceClient<robot_toolkit_msgs::motion_tools_srv>("/robot_toolkit/motion_tools_srv");
  miscClient = _nodeHandle->serviceClient<robot_toolkit_msgs::misc_tools_srv>("/robot_toolkit/misc_tools_srv");
  
  // Subscribers
  //bottomSub =_nodeHandle->subscribe("/robot_toolkit_node/camera/bottom/image_raw", 10, &CameraThread::cameraBottomCallback, rosSpinThread);
  //frontSub =_nodeHandle->subscribe("/robot_toolkit_node/camera/front/image_raw", 10, &CameraThread::cameraFrontCallback, rosSpinThread);

  // Movement slots
  connect(commandTimer, SIGNAL(timeout()), pepMov, SLOT(sendCommandSlot()));
  connect(_cbControlSelection, SIGNAL(currentIndexChanged(int)), this, SLOT(changedControlInterfaceSlot(int)));
  connect(_cBJointAngle, SIGNAL(stateChanged(int)), this, SLOT(changeJointToMoveSlot(int)));
  connect(_cbLanguage, SIGNAL(currentIndexChanged(int)), this, SLOT(languageSelectionSlot(int)));
  connect(_btSynchController, SIGNAL(clicked()), this, SLOT(synchronizeControllerSlot()));
  connect(_btStartJoystickButton, SIGNAL(clicked()), this, SLOT(startJoystickSlot()));
  connect(remoteControllerProcess, SIGNAL(readyReadStandardOutput()), this, SLOT(readyReadyStandardOutputSlot()));
  connect(remoteControllerProcess, SIGNAL(finished(int, QProcess::ExitStatus)), this, SLOT(processEnded(int, QProcess::ExitStatus)));
  connect(myControllerThread,SIGNAL(updateJoystickAction(int*)), this, SLOT(updateControllerCommands(int*)));
  connect(_sbMovementSpeed, SIGNAL(valueChanged(double)), this, SLOT(updatedMovementSpeedSlot(double)));

  // Leds slots
  connect(_bLeftEye,SIGNAL(clicked()),this,SLOT(sendLeftEyeColorSlot()));
  connect(_leftEye,SIGNAL(colorSelected(QColor)),this, SLOT(selectColorLeftEyeColorSlot(QColor)));
  connect(_bRightEye,SIGNAL(clicked()),this,SLOT(sendRightEyeColorSlot()));
  connect(_rightEye,SIGNAL(colorSelected(QColor)),this, SLOT(selectColorRightEyeColorSlot(QColor)));
  connect(_bEyes,SIGNAL(clicked()),this,SLOT(sendEyesColorSlot()));
  connect(_Eyes,SIGNAL(colorSelected(QColor)),this, SLOT(selectColorEyesColorSlot(QColor)));
  connect(_bEars,SIGNAL(clicked()),this,SLOT(sendEarsColorSlot()));
  connect(_Ears,SIGNAL(colorSelected(QColor)),this, SLOT(selectColorEarsColorSlot(QColor)));
  connect(_bResetColors,SIGNAL(clicked()),this,SLOT(resetColors()));
  
  // Animations slots
  connect(_cbEnable2, SIGNAL(stateChanged(int)), this, SLOT(animationEnableCheckBoxSlot(int)));
  connect(_cbAnimSelector, SIGNAL(currentIndexChanged(int)), this, SLOT(changedAnimationSlot(int)));
  connect(_btAnim0, SIGNAL(clicked()), this, SLOT(anim0Slot()));
  connect(_btAnim1, SIGNAL(clicked()), this, SLOT(anim1Slot()));
  connect(_btAnim2, SIGNAL(clicked()), this, SLOT(anim2Slot()));
  connect(_tmrHeadNavigation, SIGNAL(timeout()), this, SLOT(updateHeadMovementNavSlot()));

  // Text to speech slots
  connect(_btSay, SIGNAL(clicked()), this, SLOT(textToSpeechSlot()));
  connect(_btSay0, SIGNAL(clicked()), this, SLOT(say0TextToSpeechSlot()));
  connect(_btSay1, SIGNAL(clicked()), this, SLOT(say1TextToSpeechSlot()));
  connect(_btSay2, SIGNAL(clicked()), this, SLOT(say2TextToSpeechSlot()));
  connect(_btSay3, SIGNAL(clicked()), this, SLOT(say3TextToSpeechSlot()));

  // Global setup slots
  connect(_btEnableAll, SIGNAL(clicked()), this, SLOT(enableAllSlot()));
  connect(_btDisableAll, SIGNAL(clicked()), this, SLOT(disableAllSlot()));
  connect(_cbEnable1, SIGNAL(stateChanged(int)), this, SLOT(navigationEnableCheckBoxSlot(int)));
  connect(_cbEnable5, SIGNAL(stateChanged(int)), this, SLOT(navigationEnableCheckBoxSlot(int)));
  connect(_cbEnable2, SIGNAL(stateChanged(int)), this, SLOT(animationEnableCheckBoxSlot(int)));
  connect(_cbEnable3, SIGNAL(stateChanged(int)), this, SLOT(visionEnableCheckBoxSlot(int)));
  connect(_cbEnable4, SIGNAL(stateChanged(int)), this, SLOT(audioEnableCheckBoxSlot(int)));
  connect(_cbEnableLeds, SIGNAL(stateChanged(int)), this, SLOT(ledsEnableCheckBoxSlot(int)));
  connect(_btRvizLaunch, SIGNAL(clicked(bool)), this, SLOT(clickedRvizButton()));
  connect(rosSpinThread, SIGNAL(frontCameraSignal(uchar*, int, int)),this,SLOT(frontCameraSlot(uchar*, int, int)));
  connect(rosSpinThread, SIGNAL(bottomCameraSignal(uchar*, int, int)),this,SLOT(bottomCameraSlot(uchar*, int, int)));
  connect(_cbBottom, SIGNAL(stateChanged(int)), this, SLOT(bottomCameraCheckBoxSlot(int)));
  connect(_cbTop, SIGNAL(stateChanged(int)), this, SLOT(frontCameraCheckBoxSlot(int)));

  // Script slots
  connect(_btLoad, SIGNAL(clicked()), this, SLOT(loadButtonSlot()));
  connect(_btClear, SIGNAL(clicked()), this, SLOT(clearButtonSlot()));
  connect(_btPlay, SIGNAL(clicked()), this, SLOT(playButtonSlot()));
  connect(_btPrev, SIGNAL(clicked()), this, SLOT(prevButtonSlot()));
  connect(_btNext, SIGNAL(clicked()), this, SLOT(nextButtonSlot()));

  // Activation slots
  connect(_btActivate, SIGNAL(clicked()), this, SLOT(avtivateButtonSlot()));
  connect(_btAutoLife, SIGNAL(clicked()), this, SLOT(autoLifeButtonSlot()));
  connect(_btCollisions, SIGNAL(clicked()), this, SLOT(collisionButtonSlot()));

  //Enable cmd_vel by default
  _cbEnable1->setCheckState(Qt::Checked);
  _cbEnable5->setCheckState(Qt::Checked);

  // Ros thread
  rosSpinThread->start();
  commandTimer->start(100);
}

RemoteController::~RemoteController()
{
  // Do nothing  
}

void RemoteController::printInfo(QString info)
{
  _pteConsole->appendPlainText("[INFO]: " + info);
}

void RemoteController::printError(QString error)
{
  _pteConsole->appendPlainText("[ERROR]: " + error);
}

void RemoteController::setNavigationServiceParameters(QString command, bool tfEnable, float tfFrequency, bool odomEnable, float odomFrequency, bool laserEnable, float laserFrequency, bool cmdEnable, float securityTimer, bool moveBaseEnable,  bool goalEnable, bool robotPoseSubscriberEnable, bool pathEnable, float pathFrecuency, bool robotPosePublisherEnable, float robotPosePublisherFrecuency, bool resultEnable, bool depthToLaserEnable, int depthToLaserResolution, float pScanTime, float pRangeMin, float pRangeMax, float pScanHeight, bool freeZone)
{
  navigationService.request.data.command = command.toStdString();
  navigationService.request.data.tf_enable = tfEnable;
  navigationService.request.data.tf_frequency = tfFrequency;
  navigationService.request.data.odom_enable = odomEnable;
  navigationService.request.data.odom_frequency = odomFrequency;
  navigationService.request.data.laser_enable = laserEnable;
  navigationService.request.data.laser_frequency = laserFrequency;
  navigationService.request.data.cmd_vel_enable = cmdEnable;
  navigationService.request.data.security_timer = securityTimer;
  navigationService.request.data.move_base_enable = moveBaseEnable;
  navigationService.request.data.goal_enable = goalEnable;
  navigationService.request.data.robot_pose_suscriber_enable = robotPoseSubscriberEnable;
  navigationService.request.data.path_enable = pathEnable;
  navigationService.request.data.path_frequency = pathFrecuency;
  navigationService.request.data.robot_pose_publisher_enable = robotPosePublisherEnable;
  navigationService.request.data.robot_pose_publisher_frequency = robotPosePublisherFrecuency;
  navigationService.request.data.result_enable = resultEnable;
  navigationService.request.data.depth_to_laser_enable = depthToLaserEnable;
  navigationService.request.data.depth_to_laser_parameters.resolution = depthToLaserResolution;
  navigationService.request.data.depth_to_laser_parameters.scan_time = pScanTime;
  navigationService.request.data.depth_to_laser_parameters.range_min = pRangeMin;
  navigationService.request.data.depth_to_laser_parameters.range_max = pRangeMax;
  navigationService.request.data.depth_to_laser_parameters.scan_height = pScanHeight;
  navigationService.request.data.free_zone_enable = freeZone;
}

void RemoteController::setVisionServiceParameters(QString cameraName, QString command, int resolution, int frameRate, int colorSpace, robot_toolkit_msgs::camera_parameters_msg cameraParameters)
{
  visionService.request.data.camera_name = cameraName.toStdString();
  visionService.request.data.command = command.toStdString();
  visionService.request.data.resolution = (uint8_t)resolution;
  visionService.request.data.frame_rate = (uint8_t)frameRate;
  visionService.request.data.color_space = (uint8_t)colorSpace;
  visionService.request.data.camera_parameters = cameraParameters;
}

void RemoteController::setAudioServiceParameters(QString command, int frequency, int channels)
{
  audioService.request.data.command = command.toStdString();
  audioService.request.data.frequency = frequency;
  audioService.request.data.channels = channels;
}


void RemoteController::setMotionServiceParameters(bool pEnable)
{
	if(pEnable){
	      motionService.request.data.command = "enable_all";
	      // It is not neccesary
	      //motionService.request.data.animation = "enable_all";
	      //motionService.request.data.set_angles = "enable_all";
	}
	else{
	  
	      motionService.request.data.command = "disable_all";
	      // It is not neccesary
	      //motionService.request.data.animation = "disable_all";
	      //motionService.request.data.set_angles = "disable_all";
	}
}

void RemoteController::setMiscServiceParameters(bool pEnable)
{
  if(pEnable)
  {
    miscService.request.data.command = "enable_all";
  }
  else{
    miscService.request.data.command = "disable_all";
  }
}


void RemoteController::keyPressEvent(QKeyEvent *event)
{
  if(controlInterface == 1)
  {
    if(event->key() == Qt::Key_W)         //------------ Linear Movement -------------
      pepMov->setSpeed( speed, 0, 0 ); 
    else if(event->key() == Qt::Key_S)
      pepMov->setSpeed( -speed, 0, 0 );
    else if(event->key() == Qt::Key_Q)    
      pepMov->setSpeed( 0, 0, speed );
    else if(event->key() == Qt::Key_E)
      pepMov->setSpeed( 0, 0, -speed ); 
    else if(event->key() == Qt::Key_A)    //--------- Rotational Movement ------------
      pepMov->setSpeed( 0, speed, 0);
    else if(event->key() == Qt::Key_D)
      pepMov->setSpeed( 0, -speed, 0);
    else if(event->key() == Qt::Key_L)    //------------ Head Movement ---------------
      pepMov->sendJointAngle(0,-1.5*angle);
    else if(event->key() == Qt::Key_J)
      pepMov->sendJointAngle(0,1.5*angle);
    else if(event->key() == Qt::Key_K)
      pepMov->sendJointAngle(1.0*angle,0);
    else if(event->key() == Qt::Key_I)
      pepMov->sendJointAngle(-1.0*angle,0);
  }
  // pepMov->setEvent(true);
}

void RemoteController::keyReleaseEvent(QKeyEvent *event)
{
  if (controlInterface == 1)
  {
    pepMov->setSpeed(0, 0, 0);
  }
}

void RemoteController::closeEvent(QCloseEvent* event)
{
  /*robot_toolkit_msgs::camera_parameters_msg camParam;
  setNavigationServiceParameters("disable_all", false, 0.0, false, 0.0, false, 0.0, false, 5.0, true, true, 1, 1.0, 0.45, 10.0, 120);
  if(navigationClient.call(navigationService))
    printInfo("Navigation service client called to disable all with result: " + QString(navigationService.response.result.c_str()));
  else
    printError("Failed to disable navigation service");
  
  setVisionServiceParameters("front_camera", "disable", 0, 0, 0, camParam);
  if(visionClient.call(visionService))
    printInfo("Vision service client called to disable front camera with result: " + QString(visionService.response.result.c_str()));
  else 
    printError("Failed to disable vision service");
  
  setVisionServiceParameters("bottom_camera", "disable", 0, 0, 0, camParam);
  if(visionClient.call(visionService))
    printInfo("Vision service client called to disable bottom camera with result: " + QString(visionService.response.result.c_str()));
  else 
    printError("Failed to disable vision service");
  
  setAudioServiceParameters("disable_tts", 0, 0);
  if(audioClient.call(audioService))
    printInfo("Audio service client called to disable speech with result: " + QString(audioService.response.result.c_str()));
  else 
    printError("Failed to disable speech service"); */
  
  if (remoteControllerProcess->state() == QProcess::Running)
    remoteControllerProcess->kill();
  
  event->accept();
}

void RemoteController::changedControlInterfaceSlot(int pIndex)
{
  controlInterface = pIndex;
  if(controlInterface == 0)
  {
    _btStartJoystickButton->setEnabled(false);
    _btSynchController->setEnabled(false);
  }
  else if(controlInterface == 1)
  {
    printInfo("Press a key");
    _btStartJoystickButton->setEnabled(false);
    _btSynchController->setEnabled(false);
  }
  else if(controlInterface == 2)
  {
    printInfo("Connect PS4 controller");
    _btStartJoystickButton->setEnabled(true);
    _btSynchController->setEnabled(true);
  }
  _cbControlSelection->clearFocus();
}

void RemoteController::changeJointToMoveSlot(int pJoint)
{
  if (_cBJointAngle->isChecked())
  {
    printInfo("Move Opera's head");
    pepMov->activateHeadMove=true;
  }
  else pepMov->activateHeadMove=false;
}

void RemoteController::synchronizeControllerSlot()
{
  if(_btSynchController->text() == "Synchronize")
  {
    remoteControllerProcess->start("ds4drv");
    _btSynchController->setText("Stop");
  }
  else
  {
    remoteControllerProcess->kill();
  }
}

void RemoteController::readyReadyStandardOutputSlot()
{
  QString mOutputString1 = remoteControllerProcess->readAllStandardOutput();
  _pteConsole->appendPlainText(mOutputString1);
}

void RemoteController::processEnded(int, QProcess::ExitStatus)
{
  _btSynchController->setText("Synchronize");
}

void RemoteController::updateControllerCommands(int* controllerAxes)
{
  double velThreshold = 0.3;
  int joystickResolution = 32768;
  int speedController = (int)((-1*controllerAxes[1])/2);
  float v = (float)(speedController)/joystickResolution;
  int steeringController = (int)((-1*controllerAxes[0])/2);
  float w = (float)(steeringController)/joystickResolution;
  int yawController = (int)((-1*controllerAxes[2])/2);    
  int pitchController = (int)((-1*controllerAxes[5])/2);
  float yawAngle = (float)(yawController)/joystickResolution;
  float pitchAngle = (float)(pitchController)/joystickResolution;

  if (controlInterface == 2)
  {
    // Linear motion
    if (v > velThreshold)
      pepMov->setSpeed( 1.0*speed, 0, 0 );
    else if (v < -velThreshold)
      pepMov->setSpeed( -1.0*speed, 0, 0 );
    // Angular motion
    if (w < -velThreshold)
      pepMov->setSpeed( 0, 0, -1.0*speed );
    else if (w > velThreshold)
      pepMov->setSpeed( 0, 0, 1.0*speed );
    // No base motion
    if (((v < velThreshold) && (v > -velThreshold)) && ((w < velThreshold) && (w > -velThreshold)))
      pepMov->setSpeed(0, 0, 0);
    // Yaw movement
    if(yawAngle < -velThreshold)
      pepMov->sendJointAngle(0,-1.0*angle);
    else if(yawAngle > velThreshold)
      pepMov->sendJointAngle(0,1.0*angle);
    // Pitch movement
    if(pitchAngle < -velThreshold)
      pepMov->sendJointAngle(1.0*angle,0);
    if(pitchAngle > velThreshold)
      pepMov->sendJointAngle(-1.0*angle,0);
    // No head motion
    if (((yawAngle < velThreshold) && (yawAngle > -velThreshold)) && ((pitchAngle < velThreshold) && (pitchAngle > -velThreshold)))
      pepMov->sendJointAngle(0, 0);
  }
}

void RemoteController::startJoystickSlot()
{    
  if(_btStartJoystickButton->text() == "Start")
  {
    myControllerThread->start();
    _btStartJoystickButton->setText("Stop");
    printInfo("Joystick started");
  }
  else if (_btStartJoystickButton->text() == "Stop")
  {
    myControllerThread->setStarted(false);
    myControllerThread->exit();
    _btStartJoystickButton->setText("Start");
    printInfo("Joystick stopped");
  }
}

void RemoteController::updatedMovementSpeedSlot(double pSpeed){
  speed = pSpeed;
}

//-----------------------------------------------
// RViz Implementation
//-----------------------------------------------
void RemoteController::clickedRvizButton()
{
  QString archive = "";
  bool map_value = _cbRvizConfig0->isChecked();
  bool laser_value = _cbRvizConfig1->isChecked();
  bool camera_image_value = _cbRvizConfig2->isChecked();
  bool camera_depth_cloud_value = _cbRvizConfig3->isChecked();
  bool tf_value = _cbRvizConfig4->isChecked();
  bool depth_laser = _cbRvizConfig5->isChecked();
  
  if (map_value)
  {
    tf_value = true;
    laser_value = true;
  }
  
  //Enable Navegation topics
  setNavigationServiceParameters("custom", tf_value, 50.0, map_value, 50.0, laser_value, 10.0, true, 5.0, true, false, false, false, 0.0, false, 0.0, false, depth_laser, 1, 1.0, 0.45, 10.0, 120, false);
  if(navigationClient.call(navigationService))
    printInfo("Navigation service client called with result: " + QString(navigationService.response.result.c_str()));
  else 
    printError("Failed to call navigation service");
  
  //Enable Camera topics
  if (camera_image_value)
  {
    //Enable front camera
    robot_toolkit_msgs::camera_parameters_msg camParam;  
    setVisionServiceParameters("front_camera", "enable", 0, 0, 0, camParam);
    if(visionClient.call(visionService))
        printInfo("Vision service client called to enable front camera with result: " + QString(visionService.response.result.c_str()));
    else 
        printError("Failed to call vision service");
    
    //Enable bottom camera
    setVisionServiceParameters("bottom_camera", "enable", 0, 0, 0, camParam);
    if(visionClient.call(visionService))
        printInfo("Vision service client called to enable bottom camera with result: " + QString(visionService.response.result.c_str()));
    else 
        printError("Failed to call vision service");
  }
  
  if (laser_value && map_value)
    archive = "laser_map.rviz";  
  if (laser_value && !map_value)
      archive = "laser_base.rviz";
  if (!laser_value)
  {
    if (camera_depth_cloud_value && camera_image_value && map_value && tf_value)
      archive = "depth_camera_tf_map.rviz";
    else if (camera_depth_cloud_value && camera_image_value && !map_value && tf_value)
      archive = "depth_camera_tf_base.rviz";
    else if (camera_depth_cloud_value && camera_image_value && map_value && !tf_value)
      archive = "depth_camera_map.rviz";
    else if (camera_depth_cloud_value && camera_image_value && !map_value && !tf_value)
      archive = "depth_camera_base.rviz";
    else if (camera_depth_cloud_value && !camera_image_value && map_value && !tf_value)
      archive = "depth_map.rviz";
    else if (camera_depth_cloud_value && !camera_image_value && !map_value && !tf_value)
      archive = "depth_base.rviz";
    else if (camera_depth_cloud_value && !camera_image_value && map_value && tf_value)
      archive = "depth_tf_map.rviz";
    else if (camera_depth_cloud_value && !camera_image_value && !map_value && !tf_value)
      archive = "depth_tf_base.rviz";
    else if (!camera_depth_cloud_value && !camera_image_value && map_value && tf_value)
      archive = "tf_map.rviz";
    else if (!camera_depth_cloud_value && !camera_image_value && !map_value && tf_value)
      archive = "tf_base.rviz";
    else if (!camera_depth_cloud_value && camera_image_value && !map_value && !tf_value)
      archive = "camera_base.rviz";
    else if (!camera_depth_cloud_value && camera_image_value && map_value && !tf_value)
      archive = "camera_map.rviz";
    else if (!camera_depth_cloud_value && camera_image_value && !map_value && tf_value)
      archive = "camera_tf_base.rviz";
    else if (!camera_depth_cloud_value && camera_image_value && map_value && tf_value)
      archive = "camera_tf_map.rviz";
    else if (!camera_depth_cloud_value && !camera_image_value && map_value && !tf_value)
      archive = "initial_map.rviz";
    else if (camera_depth_cloud_value && !camera_image_value && !map_value && !tf_value)
      archive = "initial_base.rviz";
    else if (camera_depth_cloud_value && !camera_image_value && !map_value && tf_value)
      archive = "depth_tf_base.rviz";
  }
  myRvizThread->setArchive(archive);
  myRvizThread->start();
}

//-----------------------------------------------
// Camera Implementation
//-----------------------------------------------
void RemoteController::frontCameraSlot(uchar *data, int cols, int rows)
{
	_lbTop->setPixmap(QPixmap::fromImage(QImage(data,cols,rows,QImage::Format_RGB888)).scaled(_lbTop->width(), _lbTop->height(), Qt::KeepAspectRatio));
}

void RemoteController::bottomCameraSlot(uchar *data, int cols, int rows)
{
	_lbBottom->setPixmap(QPixmap::fromImage(QImage(data,cols,rows,QImage::Format_RGB888)).scaled(_lbBottom->width(), _lbBottom->height(), Qt::KeepAspectRatio));	
} 

void RemoteController::bottomCameraCheckBoxSlot(int state)
{
  if (_cbBottom->isChecked())
  {
    rosSpinThread->setEmitBottomCamera(true);
  }
  else 
  {
    rosSpinThread->setEmitBottomCamera(false);
    _lbBottom->setPixmap(*imagePixmap);
    _lbBottom->setScaledContents(true);
    _lbBottom->setAlignment(Qt::AlignCenter);
  } 
}

void RemoteController::frontCameraCheckBoxSlot(int state)
{
  if (_cbTop->isChecked())
  {
    rosSpinThread->setEmitFrontCamera(true);
  }
  else 
  {
    rosSpinThread->setEmitFrontCamera(false);
    _lbTop->setPixmap(*imagePixmap);
    _lbTop->setScaledContents(true);
    _lbTop->setAlignment(Qt::AlignCenter);
  } 
}

//-----------------------------------------------
// Texto to Speech Implementation
//-----------------------------------------------
void RemoteController::textToSpeechSlot()
{   
  textT2S = _teT2S->toPlainText();
  pepSpeech->sendSpeechText(textT2S, speechLanguage, _cbContextual->isChecked() );
}

void RemoteController::languageSelectionSlot(int index)
{
  if (index == 0)
    speechLanguage = "English";
  else if (index == 1)
    speechLanguage = "Spanish";
}

void RemoteController::say0TextToSpeechSlot()
{
  if (speechLanguage == "English")
    pepSpeech->sendSpeechText("Hello", speechLanguage, _cbContextual->isChecked() );
  else if (speechLanguage == "Spanish")
    pepSpeech->sendSpeechText("Hola", speechLanguage, _cbContextual->isChecked() );
}

void RemoteController::say1TextToSpeechSlot()
{
  if (speechLanguage == "English")
    pepSpeech->sendSpeechText("How are you?", speechLanguage, _cbContextual->isChecked() );
  else if (speechLanguage == "Spanish")
    pepSpeech->sendSpeechText("¿Cómo estás?", speechLanguage, _cbContextual->isChecked() );
}

void RemoteController::say2TextToSpeechSlot()
{
  if (speechLanguage == "English")
    pepSpeech->sendSpeechText("My name is Opera", speechLanguage, _cbContextual->isChecked() );
  else if (speechLanguage == "Spanish")
    pepSpeech->sendSpeechText("Me llamo Ópera", speechLanguage, _cbContextual->isChecked() );
}

void RemoteController::say3TextToSpeechSlot()
{
  if (speechLanguage == "English")
    pepSpeech->sendSpeechText("Bye", speechLanguage, _cbContextual->isChecked() );
  else if (speechLanguage == "Spanish")
    pepSpeech->sendSpeechText("Chao", speechLanguage, _cbContextual->isChecked() );
}

//-----------------------------------------------
// Leds slot implementation
//-----------------------------------------------

void RemoteController::sendLeftEyeColorSlot()
{
    _leftEye->open();
}

void RemoteController::selectColorLeftEyeColorSlot(QColor leftEyeColor)
{
  pepLeds->sendLeds("LeftFaceLeds",leftEyeColor.red(),leftEyeColor.blue(),leftEyeColor.green());
  QPalette leftEyePal = _bLeftEye->palette();
  leftEyePal.setColor(QPalette::Button,QColor(leftEyeColor));
  _bLeftEye->setAutoFillBackground(true);
  _bLeftEye->setPalette(leftEyePal);
  _bLeftEye->update();
  *_currentLeftEyeColor = leftEyeColor;
}


void RemoteController::sendRightEyeColorSlot()
{
    _rightEye->open();
    
}

void RemoteController::selectColorRightEyeColorSlot(QColor rightEyeColor)
{
  pepLeds->sendLeds("RightFaceLeds",rightEyeColor.red(),rightEyeColor.blue(),rightEyeColor.green());
  QPalette rightEyePal = _bRightEye->palette();
  rightEyePal.setColor(QPalette::Button,QColor(rightEyeColor));
  _bRightEye->setAutoFillBackground(true);
  _bRightEye->setPalette(rightEyePal);
  _bRightEye->update();
  *_currentRightEyeColor = rightEyeColor;
}


void RemoteController::sendEyesColorSlot()
{
    _Eyes->open();
}

void RemoteController::selectColorEyesColorSlot(QColor EyesColor)
{
  pepLeds->sendLeds("FaceLeds",EyesColor.red(),EyesColor.blue(),EyesColor.green());
  QPalette eyesPal = _bEyes->palette();
  eyesPal.setColor(QPalette::Button,QColor(EyesColor));
  _bEyes->setAutoFillBackground(true);
  _bEyes->setPalette(eyesPal);
  _bEyes->update();
  *_currentEyesColor = EyesColor;
}


void RemoteController::sendEarsColorSlot()
{
    _Ears->open();

}

void RemoteController::selectColorEarsColorSlot(QColor EarsColor)
{
  pepLeds->sendLeds("EarLeds",EarsColor.red(),EarsColor.blue(),EarsColor.green());
  QPalette EarsPal = _bEars->palette();
  EarsPal.setColor(QPalette::Button,QColor(EarsColor));
  _bEars->setAutoFillBackground(true);
  _bEars->setPalette(EarsPal);
  _bEars->update();
  *_currentEarsColor = EarsColor;
}

void RemoteController::resetColors()
{
  QPalette leftEyePal = _bLeftEye->palette();
  leftEyePal.setColor(QPalette::Button,QColor(Qt::white));
  _bLeftEye->setAutoFillBackground(true);
  _bLeftEye->setPalette(leftEyePal);
  _bLeftEye->update();
  
  QPalette rightEyePal = _bRightEye->palette();
  rightEyePal.setColor(QPalette::Button,QColor(Qt::white));
  _bRightEye->setAutoFillBackground(true);
  _bRightEye->setPalette(rightEyePal);
  _bRightEye->update();
  
  QPalette EyesPal = _bEyes->palette();
  EyesPal.setColor(QPalette::Button,QColor(Qt::white));
  _bEyes->setAutoFillBackground(true);
  _bEyes->setPalette(EyesPal);
  _bEyes->update();
  
  QPalette EarPal = _bEars->palette();
  EarPal.setColor(QPalette::Button,QColor(Qt::blue));
  _bEars->setAutoFillBackground(true);
  _bEars->setPalette(EarPal);
  _bEars->update();
  
  pepLeds->sendLeds("FaceLeds",255, 255, 255);
  pepLeds->sendLeds("EarLeds", 0, 255, 0);
}

//-----------------------------------------------
// Animation slot implementation
//-----------------------------------------------
void RemoteController::changedAnimationSlot(int pAnim){
  *_currentAnimation = _cbAnimSelector->itemText(pAnim);
}

void RemoteController::anim0Slot()
{
	pepAnim->sendAnimation("animations", "Gestures/Maybe_1");
}

void RemoteController::anim1Slot()
{
 QStringList animList = _currentAnimation->split('/');
  // Costum animation
  QString animPrefix = animList.at(0);
  if (animPrefix == "SinfonIA")
  {
   QString customAnimation = "";
   // Custom Animation
   for (int i=1;i<animList.size();++i)
   {
     customAnimation = customAnimation + "/" + animList.at(i);
   }
   pepAnim->sendAnimation("animations_sinfonia", customAnimation);
   printInfo(customAnimation);
   }
   else {
   //Standard Animation
   pepAnim->sendAnimation("animations", *_currentAnimation); 
  }
}

void RemoteController::anim2Slot()
{
 if (_btAnim2->text()=="Navigation Position")
 {
   _btAnim2->setText("Stop Navigation");
   pepAnim->sendAnimation("animations", "Gestures/Maybe_1");
   _cBJointAngle->setCheckState(Qt::Checked);
   pepMov->sendJointAngle(angle,0);
   _sbMovementSpeed->setValue(0.15);
   _tmrHeadNavigation->start(10000);
 }
 else
 {
   _btAnim2->setText("Navigation Position");
   _tmrHeadNavigation->stop();
   pepAnim->sendAnimation("animations", "Gestures/Maybe_1");
 }
}

void RemoteController::updateHeadMovementNavSlot()
{
  pepMov->sendJointAngle(angle,0);
}


//-----------------------------------------------
// Global setup slot implementation
//-----------------------------------------------
void RemoteController::enableAllSlot(){
    printInfo("Asking robot_toolking to enable all topics...");
		_cbEnable1->setCheckState(Qt::Checked);
		_cbEnable2->setCheckState(Qt::Checked);
		_cbEnable3->setCheckState(Qt::Checked);
		_cbEnable4->setCheckState(Qt::Checked);
		_cbEnable5->setCheckState(Qt::Checked);
		_cbEnableLeds->setCheckState(Qt::Checked);
}

void RemoteController::disableAllSlot(){
    printInfo("Asking robot_toolking to disable all topics...");
		_cbEnable1->setCheckState(Qt::Unchecked);
		_cbEnable2->setCheckState(Qt::Unchecked);
		_cbEnable3->setCheckState(Qt::Unchecked);
		_cbEnable4->setCheckState(Qt::Unchecked);
		_cbEnable5->setCheckState(Qt::Unchecked);
		_cbEnableLeds->setCheckState(Qt::Unchecked);
}

void RemoteController::animationEnableCheckBoxSlot(int pState){
    // Create publisher
    pepAnim->createPubAnimations();
    _btAnim0->setEnabled(_cbEnable2->isChecked());
    _btAnim1->setEnabled(_cbEnable2->isChecked());
    _btAnim2->setEnabled(_cbEnable2->isChecked());

    if(_cbEnable2->isChecked())
      {
// Enable motion topics
      setMotionServiceParameters(true);
      if(motionClient.call(motionService))
      {
	printInfo("Motion service client called to enable motion with result: " + QString(motionService.response.result.c_str()));
      } 
      else printError("Failed to enable motion service");
      } 
    else 
      {
		// Disable 
	setMotionServiceParameters(false);
	if(motionClient.call(motionService))
	{
	  printInfo("Motion service client called to disable motion with result: " + QString(motionService.response.result.c_str()));
	} 
	else 
	  printError("Failed to disable motion service");
	}
}


void RemoteController::navigationEnableCheckBoxSlot(int pState){
  // Create publisher
  pepMov->createMovPub();
  _cbControlSelection->setEnabled(_cbEnable1->isChecked());
	bool movementUp = _cbEnable1->isChecked();
	bool mapUp = _cbEnable5->isChecked();
   //Enable Navegation topics
  if (mapUp){
    setNavigationServiceParameters("custom", true, 50.0, true, 50.0, true, 10.0, true, 5.0, true, false, false, false, 0.0, false, 0.0, false, false, 1, 1.0, 0.45, 10.0, 120, false);;
  } else{
  	setNavigationServiceParameters("disable_all", false, 50.0, true, 50.0, true, 10.0, true, 5.0, true, false, false, false, 0.0, false, 0.0, false, true, 1, 1.0, 0.45, 10.0, 120, false);
  }
  if(navigationClient.call(navigationService))
    printInfo("Navigation service client called with result: " + QString(navigationService.response.result.c_str()));
  else 
    printError("Failed to call navigation service");
}

void RemoteController::visionEnableCheckBoxSlot(int pState){
  // Subscribe to camera topics
  bottomSub =_nodeHandle->subscribe("/robot_toolkit_node/camera/bottom/image_raw", 10, &CameraThread::cameraBottomCallback, rosSpinThread);
  frontSub =_nodeHandle->subscribe("/robot_toolkit_node/camera/front/image_raw", 10, &CameraThread::cameraFrontCallback, rosSpinThread);
	
  robot_toolkit_msgs::camera_parameters_msg camParam;
    camParam.compress = true;
    camParam.compression_factor = 30;
    camParam.brightness = 0;
    camParam.contrast = 32;
    camParam.saturation = 64;
    camParam.hue = 0;
    camParam.horizontal_flip = 0;
    camParam.vertical_flip = 0;
    camParam.auto_exposition = 1;
    camParam.auto_white_balance = 1;
    camParam.auto_gain = 1;
    camParam.reset_camera_registers = 0;
    camParam.auto_focus = 1;

	if(_cbEnable3->isChecked()){
  //Enable front camera
  setVisionServiceParameters("front_camera", "enable", 0, 0, 0, camParam);
  if(visionClient.call(visionService)){
      printInfo("Vision service client called to enable front camera with result: " + QString(visionService.response.result.c_str()));
      // Enable compression image
      setVisionServiceParameters("front_camera", "set_parameters", 0, 0, 0, camParam);
      if(visionClient.call(visionService))
        printInfo("Parameters changed to set image compression with a factor of " + QString::number(camParam.compression_factor));
      else
      	printError("Failed to set image compression");
  } else 
      printError("Failed to call vision service");
  //Enable bottom camera
  setVisionServiceParameters("bottom_camera", "enable", 0, 0, 0, camParam);
  if(visionClient.call(visionService)){
      printInfo("Vision service client called to enable bottom camera with result: " + QString(visionService.response.result.c_str()));
      setVisionServiceParameters("bottom_camera", "set_parameters", 0, 0, 0, camParam);
      if(visionClient.call(visionService))
        printInfo("Parameters changed to set image compression with a factor of " + QString::number(camParam.compression_factor));
      else
      	printError("Failed to set image compression");
  } else 
      printError("Failed to call vision service");
	} else{
	// Disable front camera
	setVisionServiceParameters("front_camera", "disable", 0, 0, 0, camParam);
  if(visionClient.call(visionService))
    printInfo("Vision service client called to disable front camera with result: " + QString(visionService.response.result.c_str()));
  else 
    printError("Failed to disable vision service");
  // Disable bottom camera
  setVisionServiceParameters("bottom_camera", "disable", 0, 0, 0, camParam);
  if(visionClient.call(visionService))
    printInfo("Vision service client called to disable bottom camera with result: " + QString(visionService.response.result.c_str()));
  else 
    printError("Failed to disable vision service");
	}
}

void RemoteController::ledsEnableCheckBoxSlot(int pState)
{
  // Create publisher
  pepLeds->createLedPub();
  _bLeftEye->setEnabled(_cbEnableLeds->isChecked());
  _bEyes->setEnabled(_cbEnableLeds->isChecked());
  _bEars->setEnabled(_cbEnableLeds->isChecked());
  _bRightEye->setEnabled(_cbEnableLeds->isChecked());
  _bResetColors->setEnabled(_cbEnableLeds->isChecked());
  if(_cbEnableLeds->isChecked())
  {
    setMiscServiceParameters(true);
    if(miscClient.call(miscService))
      {
	printInfo("Misc service client called to enable motion with result: " + QString(miscService.response.result.c_str()));
      } 
      else printError("Failed to enable misc service");
      } 
  else 
  {
    setMiscServiceParameters(false);
	if(miscClient.call(miscService))
	{
	  printInfo("Misc service client called to disable motion with result: " + QString(miscService.response.result.c_str()));
	} 
	else 
	  printError("Failed to disable misc service");
	}

}

void RemoteController::audioEnableCheckBoxSlot(int pState){
  // Create publisher
  pepSpeech->createSpeechPub();
  // Speech Buttons
  _btSay->setEnabled(_cbEnable4->isChecked());
  _btSay0->setEnabled(_cbEnable4->isChecked());
  _btSay1->setEnabled(_cbEnable4->isChecked());
	if(_cbEnable4->isChecked()){
	// Enable Speech
  setAudioServiceParameters("enable_tts", 48000, 0);
  if(audioClient.call(audioService))
     printInfo("Audio service client called to enable speech with result: " + QString(audioService.response.result.c_str()));
  else 
    printError("Failed to call speech service");
	} else{
	// Disable Speech
	setAudioServiceParameters("disable_tts", 0, 0);
  if(audioClient.call(audioService))
    printInfo("Audio service client called to disable speech with result: " + QString(audioService.response.result.c_str()));
  else 
    printError("Failed to disable speech service");

	}
}

//-----------------------------------------------
// Pepper Activation slot implementation
//-----------------------------------------------
void RemoteController::avtivateButtonSlot(){
  if (_btActivate->text()=="Stand up"){
    pepActiv->sendActivation("rest", false);
    _btActivate->setText("Rest");
  } else{
    pepActiv->sendActivation("rest", true);
    _btActivate->setText("Stand up");
  }
}

void RemoteController::autoLifeButtonSlot(){
  if (_btAutoLife->text()== "Turn off awareness"){
  pepActiv->sendActivation("awareness", false);
  _btAutoLife->setText("Turn on awareness");
  } else{
    pepActiv->sendActivation("awareness", true);
    _btAutoLife->setText("Turn off awareness");
  }
}

void RemoteController::collisionButtonSlot(){
  if (_btCollisions->text()== "Turn off security"){
  pepActiv->sendActivation("external_collision_protection_enabled", false);
  _btCollisions->setText("Turn on security");
  } else{
    pepActiv->sendActivation("external_collision_protection_enabled", true);
    _btCollisions->setText("Turn off security");
  }

}



//-----------------------------------------------
// Script slot implementation
//-----------------------------------------------
  // Load a saved script
  void RemoteController::loadButtonSlot(){
    // Open Dialog to chose file
  	*_fileScriptName = QFileDialog::getOpenFileName(this,
    tr("Load Pepper's Script"), (QString::fromStdString(ros::package::getPath("remote_controller")) + "/resources"), tr("All files (*)"));
    if (_fileScriptName->isEmpty())
        printError("File is Empty");
    else {
        _fileScript->setFileName(*_fileScriptName);
    }
    // Update file name in the config box
    QStringList nameDir = _fileScriptName->split('/');
    _lbScriptName->setText("<b>Script Name: </b> " + nameDir.at(nameDir.size()-1));
    // Open file
    if (!_fileScript->open(QIODevice::ReadOnly | QIODevice::Text)){
      printError("Script file cannot be access");
      return;
      }
    else
    {
    	// SCRIPT CONFIG
    	int lineNum = 0;
    	int stateNum = 0;
    	QTextStream in(_fileScript);
    	// Reading line by line
    	while (!in.atEnd()) {
        QString line = in.readLine();
        // Verify correct cofing
        if ((lineNum == 0) && (line != "<config>")){
          printError("La configuración del Script no es correcta");
          // Close file
          _fileScript->close();
          return;
        }
        if ((lineNum == 2) && (line != "</config>")){
          printError("La configuración del Script no es correcta");
          // Close file
          _fileScript->close();
          return;
        }
        if (lineNum < 3){
          // Verify Language
        	QStringList linePieces = line.split("=");
        	// Set script language
        	if (linePieces.at(0) == "language"){
        		_scriptLanguage = linePieces.at(1);
        		_lbScriptLanguage->setText("<b>Script Language: </b> " + _scriptLanguage);
        		if(_scriptLanguage == "English"){
        			_cbLanguage->setCurrentIndex(0);
        		} else if(_scriptLanguage == "Spanish"){
        			_cbLanguage->setCurrentIndex(1);
        		} else{
        			printError("Selected language is not available!");
        		}
          }
        }
        // SCRIPT STATES
        else {
        	if (line != ""){
            stateNum++;
        	// Proccess each state (line)
        	QStringList linePieces = line.split(",");
        	_lwScript->addItem(QString::number(stateNum) + ". " + linePieces.at(0));
        	_scriptAnimationsList->append(linePieces.at(1));
        	QString sayText = "";
        	for(int i=2; i<linePieces.length(); i++){
        		if (i == 2)
        		  sayText = sayText + linePieces.at(i);
        		else
        		  sayText = sayText + "," +linePieces.at(i);
        	    }
        	_scriptTextList->append(sayText);
            }
        }
        lineNum ++;
    	}
    	// Set total states
    	_lbScriptState->setText("<b>Script State: </b> 1 / " + QString::number(stateNum));
    	_lwScript->setCurrentRow(0);
  	}
  	// Enable needed topics
  	_cbEnable2->setCheckState(Qt::Checked);
  	_cbEnable4->setCheckState(Qt::Checked);
    // Enable buttons
    _btPrev->setEnabled(true);
    _btNext->setEnabled(true);
    _btPlay->setEnabled(true);

    // Disable load button
    _btLoad->setEnabled(false);

    printInfo("Script file " + nameDir.at(nameDir.length()-1) + " has been loaded");
    // Close file
    _fileScript->close();
  }

  void RemoteController::clearButtonSlot(){
    // Clear Lists
    int sizeList = _scriptTextList->length();
    int i = 0;
    while( i < sizeList ){
    _lwScript->takeItem(0);
    _scriptAnimationsList->removeAt(0);
    _scriptTextList->removeAt(0);
    i++;
    }
    // Empty file
    *_fileScriptName = "";
    // Clear Script Config
    _lbScriptName->setText("<b>Script Name:<b>");
    _lbScriptLanguage->setText("<b>Script Language:<b>");
    _lbScriptState->setText("<b>Script State:<b>");
    // Disable buttons
    _btPrev->setEnabled(false);
    _btNext->setEnabled(false);
    _btPlay->setEnabled(false);
    // Enable load button
    _btLoad->setEnabled(true);
    
    printInfo("Script has been cleared");
  }
  
  void RemoteController::playButtonSlot(){
  	int currentState = _lwScript->currentRow();
  	printInfo("Playing State " + QString::number(currentState+1) + "...");
	  if (_scriptAnimationsList->at(currentState) == ""){
	    printInfo(_scriptTextList->at(currentState));
	    pepSpeech->sendSpeechText(_scriptTextList->at(currentState), speechLanguage, true);
	    } 
	  // There is an animation
	  else{
	    QString currentAnim = _scriptAnimationsList->at(currentState);
	    QStringList customAnimList =  currentAnim.split("/");
	    // Costum animation
	      QString animPrefix = customAnimList.at(0);
	      if (animPrefix == "SinfonIA")
	      {
		QString customAnimation = "";
		// Custom Animation
		for (int i=1;i<customAnimList.size();++i)
		{
		  customAnimation = customAnimation + "/" + customAnimList.at(i);
		}
		pepAnim->sendAnimation("animations_sinfonia", customAnimation);
		printInfo(customAnimation);
	      }
	      else {
		//Standard Animation
		pepAnim->sendAnimation("animations", _scriptAnimationsList->at(currentState)); 
	      }
	      // Text to Speech
	      printInfo(_scriptTextList->at(currentState));
	      pepSpeech->sendSpeechText(_scriptTextList->at(currentState), speechLanguage, false);
	  }
  	}

  void RemoteController::prevButtonSlot(){
  	int currentState = _lwScript->currentRow();
  	int listSize = _scriptTextList->length();
  	if (currentState != 0) {
  		// Update current State
  	  _lwScript->setCurrentRow(currentState - 1);
  	  currentState = _lwScript->currentRow();
  	  _lbScriptState->setText("<b>Script State: </b> " + QString::number(currentState+1) + " / " + QString::number(listSize));
  	  printInfo("Playing State " + QString::number(currentState+1) + "...");
  	  // Verify if there is no animation
  	  if (_scriptAnimationsList->at(currentState) == ""){
  	    printInfo(_scriptTextList->at(currentState));
  	    pepSpeech->sendSpeechText(_scriptTextList->at(currentState), speechLanguage, true);
  	    } 
  	  // There is animation
  	  else{
  	    QString currentAnim = _scriptAnimationsList->at(currentState);
	    QStringList customAnimList =  currentAnim.split("/");
	    // Costum animation
	      QString animPrefix = customAnimList.at(0);
	      if (animPrefix == "SinfonIA")
	      {
		QString customAnimation = "";
		// Custom Animation
		for (int i=1;i<customAnimList.size();i++)
		{
		  customAnimation = customAnimation + "/" + customAnimList.at(i);
		}
		pepAnim->sendAnimation("animations_sinfonia", customAnimation);
		printInfo(customAnimation);
	      }
	      else {
		//Standard Animation
		pepAnim->sendAnimation("animations", _scriptAnimationsList->at(currentState)); 
	      }
	      // Text to Speech
	      printInfo(_scriptTextList->at(currentState));
	      pepSpeech->sendSpeechText(_scriptTextList->at(currentState), speechLanguage, false);
  	    }
  	  } 
  	  else
  		  printError("Already in the first state");
  	}

  void RemoteController::nextButtonSlot(){
  	int currentState = _lwScript->currentRow();
  	int listSize = _scriptTextList->length();
  	if (currentState != listSize-1) {
  		// Update current State
  	  _lwScript->setCurrentRow(currentState + 1);
  	  currentState = _lwScript->currentRow();
  	  _lbScriptState->setText("<b>Script State: </b> " + QString::number(currentState+1) + " / " + QString::number(listSize));
  	  printInfo("Playing State " + QString::number(currentState+1) + "...");
      // Verify if there is no animation
  	  if (_scriptAnimationsList->at(currentState) == ""){
  	    printInfo(_scriptTextList->at(currentState));
  	    pepSpeech->sendSpeechText(_scriptTextList->at(currentState), speechLanguage, true);
  	    } 
  	  // There is animation
  	  else{
  	    QString currentAnim = _scriptAnimationsList->at(currentState);
	    QStringList customAnimList =  currentAnim.split("/");
	    // Costum animation
	      QString animPrefix = customAnimList.at(0);
	      if (animPrefix == "SinfonIA")
	      {
		QString customAnimation = "";
		// Custom Animation
		for (int i=1;i<customAnimList.size();i++)
		{
		  customAnimation = customAnimation + "/" + customAnimList.at(i);
		}
		pepAnim->sendAnimation("animations_sinfonia", customAnimation);
		printInfo(customAnimation);
	      }
	      else {
		//Standard Animation
		pepAnim->sendAnimation("animations", _scriptAnimationsList->at(currentState)); 
	      }
	      // Text to Speech
	      printInfo(_scriptTextList->at(currentState));
	      pepSpeech->sendSpeechText(_scriptTextList->at(currentState), speechLanguage, false);
  	    }
  	  } 
  	  else
  		  printError("Already in the last state");
  	}

