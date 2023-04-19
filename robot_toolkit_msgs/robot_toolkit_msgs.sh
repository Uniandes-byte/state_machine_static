#!/bin/sh

if [ "$1" = "-h" ] || [ "$1" = "--help" ]
	then :
		echo "Arguments:"
		echo "  -m, --mode=MODE        Defines the execution mode"
		echo "                         --------------------------------------------------"
		echo "                         --------------------------------------------------"
		echo "                         MODE = compile, COMPILE, c, C"
		echo "                         Compiles the robot_toolkit files inside the VM"
		echo "                         REQUIRES: --vnip and --vnpass"
		echo "                         --------------------------------------------------"
		echo "                         MODE = install, INSTALL, i, I"
		echo "                         Compiles the robot_toolkit files inside the VM and"
		echo "                         installs the resulting binary files inside the robot"
		echo "                         REQUIRES: --vnip, --robotip, --vnpass and --robotpass"
		echo "                         --------------------------------------------------"
		echo "                         MODE = clean_vn, CLEAN_VN, cv, CV"
		echo "                         Ereases the compilation and binary files inside the"
		echo "                         virtual nao"
		echo "                         REQUIRES: --vnip and --vnpass"
		echo "                         --------------------------------------------------"
		echo "                         MODE = clean_robot, CLEAN_ROBOT, cr, CR"
		echo "                         Ereases the binary files inside the robot"
		echo "                         REQUIRES: --robotip and --robotpass"
		echo "                         --------------------------------------------------"
		echo "                         MODE = clean_all, CLEAN_ALL, ca, CA"
		echo "                         Ereases the compilation and binary files inside the"
		echo "                         virtual nao and also the binary files inside"
		echo "                         the robot"
		echo "                         REQUIRES: --robotip and --robotpass"
		echo "                         --------------------------------------------------"
		echo "  -v, --vnip=IP          IP = Virtual Nao IP"
		echo "  -r, --robotip=IP       IP = Robot IP"
		echo "  -n, --vnpass=PASSWORD  PASSWORD = Virtual Nao password"
		echo "  -p, --robotpass=IP     PASSWORD = Robot password"



	exit
fi

for i in "$@"
do
case $i in
    -m=*|--mode=*)
    MODE="${i#*=}"
    shift # past argument=value
    ;;
    -r=*|--robotip=*)
    ROBOTIP="${i#*=}"
    shift # past argument=value
    ;;
    -v=*|--vnip=*)
    VNIP="${i#*=}"
    shift # past argument=value
    ;;
    -n=*|--vnpass=*)
    VNPASS="${i#*=}"
    shift # past argument=value
    ;;
    -p=*|--robotpass=*)
    ROBOTPASS="${i#*=}"
    shift # past argument=value
    ;;
    --default)
    DEFAULT=YES
    shift # past argument with no value
    ;;
    *)
          # unknown option
    ;;
esac
done

if [ -z ${MODE} ]
	then echo "Not enough arguments"
	exit
fi
if [ ${MODE} = "compile" ] || [ ${MODE} = "COMPILE" ] || [ ${MODE} = "c" ] || [ ${MODE} = "C" ]
	then :
	if [ -z ${VNIP} ]
		then echo "Not enough arguments, virtual nao IP missing"
		exit
	fi
	if [ -z ${VNPASS} ]
		then echo "Not enough arguments, virtual nao PASSWORD missing"
		exit
	fi
	MODE="compile"
fi
if [ ${MODE} = "install" ] || [ ${MODE} = "INSTALL" ] || [ ${MODE} = "i" ] || [ ${MODE} = "I" ]
	then :
	if [ -z ${VNIP} ]
		then echo "Not enough arguments, virtual nao IP missing"
		exit
	fi
	if [ -z ${ROBOTIP} ]
		then echo "Not enough arguments, robot IP missing"
		exit
	fi
	if [ -z ${VNPASS} ]
		then echo "Not enough arguments, virtual nao PASSWORD missing"
		exit
	fi
	if [ -z ${ROBOTPASS} ]
		then echo "Not enough arguments, robot PASSWORD missing"
		exit
	fi
	MODE="install"
fi
if [ ${MODE} = "clean_vn" ] || [ ${MODE} = "CLEAN_VN" ] || [ ${MODE} = "cv" ] || [ ${MODE} = "CV" ]
	then :
	if [ -z ${VNIP} ]
		then echo "Not enough arguments, virtual nao IP missing"
		exit
	fi
	if [ -z ${VNPASS} ]
		then echo "Not enough arguments, virtual nao PASSWORD missing"
		exit
	fi
	MODE="clean_vn"
fi
if [ ${MODE} = "clean_robot" ] || [ ${MODE} = "CLEAN_ROBOT" ] || [ ${MODE} = "cr" ] || [ ${MODE} = "CR" ]
	then :
	if [ -z ${ROBOTIP} ]
		then echo "Not enough arguments"
		exit
	fi
	if [ -z ${ROBOTPASS} ]
		then echo "Not enough arguments, robot PASSWORD missing"
		exit
	fi
	MODE="clean_robot"
fi
if [ ${MODE} = "clean_all" ] || [ ${MODE} = "CLEAN_ALL" ] || [ ${MODE} = "ca" ] || [ ${MODE} = "CA" ]
	then :
	if [ -z ${VNIP} ]
		then echo "Not enough arguments, virtual nao IP missing"
		exit
	fi
	if [ -z ${ROBOTIP} ]
		then echo "Not enough arguments, robot IP missing"
		exit
	fi
	if [ -z ${VNPASS} ]
		then echo "Not enough arguments, virtual nao PASSWORD missing"
		exit
	fi
	if [ -z ${ROBOTPASS} ]
		then echo "Not enough arguments, robot PASSWORD missing"
		exit
	fi
	MODE="clean_all"
fi
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'

clean_vn()
{
	echo "${YELLOW}Cleaning virtual nao... ${NC}"
	COMMAND="sudo rm -rf /home/nao/.temp/ && sudo rm -rf /home/nao/build_isolated/robot_toolkit_msgs && sudo rm -rf /home/nao/devel_isolated/robot_toolkit_msgs && sudo rm -rf /home/nao/ros_toolchain_install/lib/robot_toolkit_msgs && sudo rm -rf /home/nao/ros_toolchain_install/lib/pkgconfig/robot_toolkit_msgs.pc && sudo rm -rf /home/nao/ros_toolchain_install/share/robot_toolkit_msgs && sudo rm -rf /home/nao/src/robot_toolkit_msgs && sudo rm -rf /home/nao/ros_toolchain_install/include/robot_toolkit_msgs/ && sudo rm -rf /home/nao/ros_toolchain_install/lib/python2.7/site-packages/robot_toolkit_msgs"
	sshpass -p ${VNPASS} ssh nao@${VNIP} ${COMMAND}
	echo "Executing: ${COMMAND}"
	echo "${GREEN}virtual nao cleaned successfully ${NC}"
}

clean_robot()
{
	echo "${YELLOW}Cleaning robot... ${NC}"	
	COMMAND="rm -rf /home/nao/ros/lib/robot_toolkit_msgs && rm -rf /home/nao/ros/lib/pkgconfig/robot_toolkit_msgs.pc && rm -rf /home/nao/ros/share/robot_toolkit_msgs && rm -rf /home/nao/src/robot_toolkit_msgs && rm -rf /home/nao/ros/include/robot_toolkit_msgs/ && rm -rf /home/nao/ros/lib/python2.7/site-packages/robot_toolkit_msgs"
	sshpass -p ${ROBOTPASS} ssh nao@${ROBOTIP} ${COMMAND}
	echo "Executing: ${COMMAND}"
	echo "${GREEN}robot cleaned successfully ${NC}"
}


if [ ${MODE} = "compile" ] || [ ${MODE} = "install" ] || [ ${MODE} = "all" ]
	then :
		echo "${YELLOW}Starting compilation ... ${NC}"
		OUTPUT="$(sshpass -p ${VNPASS} ssh nao@${VNIP} 'pwd')"
		echo ${OUTPUT}
		if [ -z ${OUTPUT} ]
			then : 
				echo "${RED}ERROR: Compilation failed, please check the password  and IP ${NC}"
				exit
		fi
		clean_vn
		sshpass -p ${VNPASS} scp -r ../robot_toolkit_msgs nao@${VNIP}:/home/nao/src/
		sshpass -p ${VNPASS} ssh nao@${VNIP} 'cd /home/nao && sudo rm -rf .temp/ && sudo src/catkin/bin/catkin_make_isolated --pkg robot_toolkit_msgs --install -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_INSTALL_PREFIX=/home/nao/ros_toolchain_install -j4 && sudo mkdir -p /home/nao/.temp/share/robot_toolkit_msgs/srv && sudo mkdir -p /home/nao/.temp/share/robot_toolkit_msgs/cmake && sudo mkdir -p /home/nao/.temp/share/common-lisp/ros/robot_toolkit_msgs/srv && sudo mkdir -p /home/nao/.temp/include/robot_toolkit_msgs/ && sudo mkdir -p /home/nao/.temp/lib/python2.7/site-packages/robot_toolkit_msgs && sudo mkdir -p /home/nao/.temp/lib/pkgconfig/ && sudo cp /home/nao/ros_toolchain_install/_setup_util.py /home/nao/.temp && sudo cp /home/nao/ros_toolchain_install/env.sh /home/nao/.temp && sudo cp /home/nao/ros_toolchain_install/setup.bash /home/nao/.temp && sudo cp /home/nao/ros_toolchain_install/setup.sh /home/nao/.temp && sudo cp /home/nao/ros_toolchain_install/setup.zsh /home/nao/.temp && sudo cp /home/nao/ros_toolchain_install/.rosinstall /home/nao/.temp && sudo cp -r /home/nao/ros_toolchain_install/share/robot_toolkit_msgs/cmake/* /home/nao/.temp/share/robot_toolkit_msgs/cmake/ && sudo cp -r /home/nao/ros_toolchain_install/share/common-lisp/ros/robot_toolkit_msgs/srv/* /home/nao/.temp/share/common-lisp/ros/robot_toolkit_msgs/srv && sudo cp -r /home/nao/ros_toolchain_install/share/robot_toolkit_msgs/package.xml /home/nao/.temp/share/robot_toolkit_msgs/package.xml && sudo cp -r /home/nao/ros_toolchain_install/include/robot_toolkit_msgs/* /home/nao/.temp/include/robot_toolkit_msgs/ && sudo cp -r /home/nao/ros_toolchain_install/lib/python2.7/site-packages/robot_toolkit_msgs/* /home/nao/.temp/lib/python2.7/site-packages/robot_toolkit_msgs'
		echo "${GREEN}Compilation finished${NC}"
fi
if [ ${MODE} = "install" ] || [ ${MODE} = "all" ]
	then :
		echo "${YELLOW}Starting installation ... ${NC}"
		OUTPUT="$(sshpass -p ${ROBOTPASS} ssh nao@${ROBOTIP} 'pwd')"
		if [ -z ${OUTPUT} ]
			then : 
				echo "${RED}ERROR: Installation failed, please check the password and IP${NC}"
				exit	
		fi
		clean_robot
		sshpass -p ${VNPASS} scp -r nao@${VNIP}:/home/nao/.temp .temp
		sshpass -p ${ROBOTPASS} scp -r .temp/* nao@${ROBOTIP}:/home/nao/ros
		rm -r .temp
		echo "${GREEN}Binaries installed successfully${NC}"
fi
if [ ${MODE} = "run" ] || [ ${MODE} = "all" ]
	then :
		echo "${YELLOW}Trying to run on robot ... ${NC}"
		OUTPUT="$(sshpass -p ${ROBOTPASS} ssh nao@${ROBOTIP} 'pwd')"
		if [ -z ${OUTPUT} ]
			then : 
				echo "${RED}ERROR: Could not run robot_toolkit, please check the password and IP${NC}"
				exit	
		fi
		kill_process
		echo "${GREEN}Launching robot_toolkit"
		COMMAND="source /home/nao/ros/setup.bash && export ROS_MASTER_URI=http://localhost:11311 && export ROS_HOSTNAME=$ROBOTIP && export ROS_IP=$ROBOTIP && roslaunch robot_toolkit robot_toolkit.launch"
		sshpass -p ${ROBOTPASS} ssh nao@${ROBOTIP} ${COMMAND}

		exit
fi
if [ ${MODE} = "kill" ]
	then :
		echo "${YELLOW}Trying to kill robot_toolkit ... ${NC}"
		OUTPUT="$(sshpass -p ${ROBOTPASS} ssh nao@${ROBOTIP} 'pwd')"
		if [ -z ${OUTPUT} ]
			then : 
				echo "${RED}ERROR: Could not kill robot_toolkit, please check the password and IP${NC}"
				exit	
		fi
		kill_process
fi
if [ ${MODE} = "clean_vn" ] || [ ${MODE} = "clean_all" ]
	then :
		clean_vn
fi
if [ ${MODE} = "clean_robot" ] || [ ${MODE} = "clean_all" ]
	then :
		clean_robot
fi


