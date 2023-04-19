#!/usr/bin/env python3
import rospy
import rospkg 
import actionlib
import numpy as np
import networkx as nx
import random

from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped, Twist
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid, Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from navigation_msgs.srv import set_current_place_srv, go_to_relative_point_srv, go_to_place_srv, start_random_navigation_srv, robot_stop_srv, add_place_srv, go_to_place_srvRequest, robot_stop_srvRequest, follow_you_srv, spin_srv, spin_srvRequest, add_place_srvRequest, go_to_defined_angle_srv, get_absolute_position_srvResponse, get_absolute_position_srv, get_route_guidance_srv,set_current_place_srvRequest,correct_position_srv
from navigation_msgs.msg import simple_feedback_msg
from robot_toolkit_msgs.msg import touch_msg, navigation_tools_msg, depth_to_laser_msg, motion_tools_msg, special_settings_msg
from robot_toolkit_msgs.srv import navigation_tools_srv, motion_tools_srv
import ConsoleFormatter
import NavigationGraph

class NavigationUtilities:

    # -----------------------------------------------------------------------------------------------------------------------
    # -----------------------------------------------------INIT--------------------------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------------

    def __init__(self):
        # Create robot_toolkit_service
        print(consoleFormatter.format("Waiting for navigation_tools service", "WARNING"))
        rospy.wait_for_service('/robot_toolkit/navigation_tools_srv')
        print(consoleFormatter.format("navigation_tools service connected!", "OKGREEN"))
        self.navigationToolsServiceClient = rospy.ServiceProxy('/robot_toolkit/navigation_tools_srv', navigation_tools_srv)

        print(consoleFormatter.format("Waiting for motion_tools service", "WARNING"))
        rospy.wait_for_service('/robot_toolkit/motion_tools_srv')
        print(consoleFormatter.format("motion_tools service connected!", "OKGREEN"))
        self.motionToolsServiceClient = rospy.ServiceProxy('/robot_toolkit/motion_tools_srv', motion_tools_srv)

        # Motion robot_toolkit_service - Request
        motion_request=motion_tools_msg()
        motion_request.command = 'enable_all'
        self.motionToolsServiceClient.call(motion_request)
        # Navigation robot_toolkit_service - Request
        navigation_request = navigation_tools_msg()
        # Enable all
        #navigation_request.command = "enable_all"
        # Custom - Same params as remote controller
        navigation_request.command = "custom"
        navigation_request.tf_enable = True
        navigation_request.tf_frequency = 50.0 
        navigation_request.odom_enable = True
        navigation_request.odom_frequency = 50.0
        navigation_request.laser_enable = True
        navigation_request.laser_frequency = 10.0
        navigation_request.cmd_vel_enable = True
        navigation_request.security_timer = 5.0
        navigation_request.move_base_enable = True
        navigation_request.goal_enable = False
        navigation_request.robot_pose_suscriber_enable = False
        navigation_request.path_enable = False
        navigation_request.path_frequency = 0.0
        navigation_request.robot_pose_publisher_enable = False
        navigation_request.robot_pose_publisher_frequency = 0.0
        navigation_request.result_enable = False
        navigation_request.depth_to_laser_enable = False
        navigation_request.depth_to_laser_parameters = depth_to_laser_msg()
        navigation_request.depth_to_laser_parameters.resolution = 1
        navigation_request.depth_to_laser_parameters.scan_time = 1.0
        navigation_request.depth_to_laser_parameters.range_min = 0.45
        navigation_request.depth_to_laser_parameters.range_max = 10.0
        navigation_request.depth_to_laser_parameters.scan_height = 120
        navigation_request.free_zone_enable = False
        # Request navigation service
        self.navigationToolsServiceClient(navigation_request)
        print(consoleFormatter.format('Navigation service was executed successfully', 'OKGREEN'))

        # Create an action client called move_base with action definition file "MoveBaseAction"
        self.actionClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        print(consoleFormatter.format("Waiting for move_base action server", "WARNING"))
        self.actionClient.wait_for_server()
        print(consoleFormatter.format("Move_base action server connected!", "OKGREEN"))

        # Service Clients
        print(consoleFormatter.format("Waiting for clear_costmap service", "WARNING"))
        rospy.wait_for_service('/move_base/clear_costmaps')
        print(consoleFormatter.format("Clear_costmap service connected!", "OKGREEN"))
        self.clearCostmapServiceClient = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        print(consoleFormatter.format("Waiting for get_static_map service", "WARNING"))
        rospy.wait_for_service('/static_map')
        print(consoleFormatter.format("Get_static_map service connected!", "OKGREEN"))
        self.staticMapServiceClient = rospy.ServiceProxy('/static_map', GetMap)

        # Service Servers
        self.setCurrentPlaceServer = rospy.Service('navigation_utilities/set_current_place_srv', set_current_place_srv, self.callback_set_current_place_srv)
        print(consoleFormatter.format('Set_pose_rv on!', 'OKGREEN'))    

        self.goToRelativePointServer = rospy.Service('navigation_utilities/go_to_relative_point_srv', go_to_relative_point_srv, self.callback_go_to_relative_point_srv)
        print(consoleFormatter.format('Go_to_relative_point_srv on!', 'OKGREEN'))

        self.goToPlaceServer = rospy.Service('navigation_utilities/go_to_place_srv', go_to_place_srv, self.callback_go_to_place_srv)
        print(consoleFormatter.format('Go_to_place_srv on!', "OKGREEN"))

        self.startRandomNavigationServer = rospy.Service('navigation_utilities/start_random_navigation_srv', start_random_navigation_srv, self.callback_start_random_navigation_srv)
        print(consoleFormatter.format('Start_random_navigation_srv on!', 'OKGREEN'))

        self.addPlaceServer = rospy.Service('navigation_utilities/add_place_srv', add_place_srv, self.callback_add_place_srv)
        print(consoleFormatter.format('Add_place_srv on!', 'OKGREEN'))

        self.followYouServer = rospy.Service('navigation_utilities/follow_you_srv', follow_you_srv, self.callback_follow_you_srv)
        print(consoleFormatter.format('Follow_you_srv on!', 'OKGREEN'))

        self.robotStopSever = rospy.Service('navigation_utilities/robot_stop_srv', robot_stop_srv, self.callback_robot_stop_srv)
        print(consoleFormatter.format("Robot_stop_srv on!", "OKGREEN"))

        self.SpinServer = rospy.Service('navigation_utilities/spin_srv', spin_srv, self.callback_spin_srv)
        print(consoleFormatter.format('Spin_srv on!', 'OKGREEN'))

        self.goToDefinedAngleServer = rospy.Service('navigation_utilities/go_to_defined_angle_srv', go_to_defined_angle_srv, self.callback_go_to_defined_angle_srv)
        print(consoleFormatter.format('Go_to_defined_angle_srv on!', 'OKGREEN'))

        self.getAbsolutePositionServer = rospy.Service('navigation_utilities/get_absolute_position_srv', get_absolute_position_srv, self.callback_get_absolute_position_srv)
        print(consoleFormatter.format('Get_absolute_position_srv on!', 'OKGREEN'))

        self.getRouteGuidanceServer = rospy.Service('navigation_utilities/get_route_guidance_srv', get_route_guidance_srv, self.callback_get_route_guidance_srv)
        print(consoleFormatter.format('Get_route_guidance_srv on!', 'OKGREEN'))

        self.correctPositionServer = rospy.Service('navigation_utilities/correctPosition_srv', correct_position_srv, self.callback_correct_position_srv)
        print(consoleFormatter.format('Correct_position_srv on!', 'OKGREEN'))


        # Publishers
        self.simpleFeedbackPublisher = rospy.Publisher('/navigation_utilities/simple_feedback', simple_feedback_msg, queue_size=10)
        print(consoleFormatter.format("Simple_feedback topic is up!","OKGREEN"))

        self.completeFeedbackPublisher = rospy.Publisher('/navigation_utilities/complete_feedback', MoveBaseFeedback, queue_size=100)
        print(consoleFormatter.format("Complete_feedback topic is up!","OKGREEN"))

        self.posePublisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

        self.cmd_velPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.special_settingsPublisher = rospy.Publisher('/special_settings', special_settings_msg, queue_size=10)
        special_settings_request =  special_settings_msg()
        special_settings_request.command = 'rest'
        special_settings_request.state = False
        self.special_settingsPublisher.publish(special_settings_request)

        # Subscribers
        self.currentPoseSubscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback_amcl_pose_subscriber)

        self.costMapSubscriber = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.callback_costmap_subscriber)

        self.currentPoseOdomSuscriber = rospy.Subscriber('/odom', Odometry, self.callback_odom_subscriber)

        self.headSensorSubscriber = rospy.Subscriber('/touch', touch_msg, self.callback_head_sensor_subscriber)

        
        # Constants
        self.FRAME_ID_MAP = 'map'
        self.FRAME_ID_BASE_LINK = 'base_link'

        self.PATH_NAVIGATION_UTLITIES = rospkg.RosPack().get_path('navigation_utilities')

        self.PATH_PLACES = self.PATH_NAVIGATION_UTLITIES+'/resources/places.txt'
        self.PATH_EDGES = self.PATH_NAVIGATION_UTLITIES+'/resources/edges.txt'

        self.ANGULAR_THRESHOLD = 2 #Degrees
        self.MAX_TRIES = 3

        # Graph
        self.graph = NavigationGraph.NavigationGraph(self.PATH_PLACES, self.PATH_EDGES).graph

        self.STATIC_MAP_INFO = self.loadStaticMapInfo()
        self.KNOWN_PLACES = self.loadKnownPlaces()

        # Attributes
        self.navigation_status = 0
        self.numero_intentos = 0
        self.poses_sequence = list()
        self.goal_index = 0
        self.sensor_pressed = False
        self.placeFollowYou = ""
        self.random_navigation = False
        self.navigation_graph = None
        self.robot_stopped = False
        self.follow_active = False
        self.isTouched = False
        self.sensorMiddle = False
        self.sensorRear = False
        self.sensorFront = False
        self.destino_final= ""

    # -----------------------------------------------------------------------------------------------------------------------
    # ----------------------------------------------------SERVICES CALLBACKS-------------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------------
    
    def callback_set_current_place_srv(self, req):
        """
        Callback for set_current_place_srv service: Sets the pose of the robot to the coordinates of the place specified in
        the request message, then it clears the global costmap.

        Args:
            req: set_current_place_srv request message. {name(str)}

        Returns:
            Returns 'approved' if the request message has a known place, 'not-approved' otherwise.
        """
        print(consoleFormatter.format("\nRequested set pose service", "WARNING"))
        if req.name.lower() in list(self.graph.nodes()):

            poseWithCovarianceStampedMsg = PoseWithCovarianceStamped()
            poseWithCovarianceStampedMsg.header.frame_id = self.FRAME_ID_MAP
            poseWithCovarianceStampedMsg.header.stamp = rospy.Time.now()
            poseWithCovarianceStampedMsg.pose.pose = self.createPosesSequence([req.name.lower()], True)[0]
            print(consoleFormatter.format("Setting pose to "+req.name.lower(), "OKBLUE"))
            self.posePublisher.publish(poseWithCovarianceStampedMsg)
            print(consoleFormatter.format("Pose was set successfully", "OKGREEN"))
            print(consoleFormatter.format('Clearing global costmap_2D', 'OKBLUE'))
            rospy.sleep(1.5)
            self.clearCostmapServiceClient()
            print(consoleFormatter.format('Global costmap_2D was cleared successfully', 'OKGREEN'))
            return 'approved'
        else:
            print(consoleFormatter.format("Set current place service not-approved: Place '"+req.name.lower()+"' is not known.", "FAIL"))
            return 'not-approved'
    
    def callback_go_to_relative_point_srv(self, req):
        """
        Callback for go_to_relative_point_srv service: Sends the robot to the coordinates (in meters and relative to the robot)
        specified in the request message.

        Args:
            req: go_to_relative_point request message. {x(float), y(float), theta(float)}

        Returns:
            Returns 'approved' if the request message has a reachable point, 'not-approved' otherwise.
        """
        print(consoleFormatter.format('\nRequested go to relative point service', 'WARNING'))
        self.goal_index = 0
        self.navigation_graph = 0
        xGoal = self.currentPositionAmcl.position.x + req.x * np.cos(self.yawAmcl) + req.y * np.cos(self.yawAmcl + np.radians(90))
        yGoal = self.currentPositionAmcl.position.y + req.x * np.sin(self.yawAmcl) + req.y * np.sin(self.yawAmcl + np.radians(90))
        thetaGoal = np.degrees(self.yawAmcl) + req.theta
        map_x, map_y = self.worldToMap(xGoal, yGoal,  self.STATIC_MAP_INFO['map_origin_x'], self.STATIC_MAP_INFO['map_origin_y'], self.STATIC_MAP_INFO['map_resolution'])
        id_x = map_x + map_y * self.STATIC_MAP_INFO['map_size_x']
        if self.STATIC_MAP_INFO['map_data'][id_x] != 0:
            print(consoleFormatter.format('Go to relative point service not-approved: Goal point is out of the boundary of the known map', 'FAIL'))
            return 'not-approved'
        elif self.costMap[id_x] >= 85:
            print(consoleFormatter.format("Go to relative point service not-approved: Goal is not reachable, there could be an obstacle too close to the target point", 'FAIL'))
            return 'not-approved'
        else:
            print(consoleFormatter.format('Navigating to x: '+str(req.x)+' y: '+str(req.y)+' theta: '+str(req.theta)+' relative to the current robot position', 'HEADER'))
            self.graph.add_node('temp', x=xGoal, y=yGoal, theta=thetaGoal)
            self.poses_sequence = self.createPosesSequence(['temp'], False)
            self.route = ['temp']
            self.sendGoal(self.FRAME_ID_MAP)
            self.graph.remove_node('temp')
            return 'approved'

    def callback_go_to_place_srv(self, req):
        """
        Callback for go_to_place_srv service: If graph is 0, send the objective directly to the action client.
        If, on the contrary, graph is 1, it maps the place of origin and finds the shortest route between it 
        and the target place and sends each place of this route to the action client.

        Args:
            req: go_to_place_srv request message. {name(str), graph(int)}

        Returns:
            Returns 'approved' if the request message has a known place and graph is 0 or 1, 'not-approved' otherwise.
        """
        print(consoleFormatter.format("\nRequested go to place service", "WARNING"))
        self.robot_stopped = False
        self.goal_index = 0
        self.navigation_graph = req.graph
        if req.name.lower() in list(self.graph.nodes()):
            self.destino_final=req.name.lower()
            if req.graph == 0 or req.graph == 1:
                if req.graph == 0:
                    print(consoleFormatter.format("Navigating without graph to "+req.name.lower(), "HEADER"))
                    self.poses_sequence = self.createPosesSequence([req.name.lower()], False)
                    self.route = [req.name.lower()]
                elif req.graph == 1:
                    print(consoleFormatter.format("Navigating with graph to "+req.name.lower(), "HEADER"))
                    sourceNode = self.mapSourceNode()
                    if sourceNode==req.name.lower():
                        reqAddPlace=add_place_srvRequest()
                        reqAddPlace.name="temporal"
                        reqAddPlace.persist=0
                        reqAddPlace.edges=[req.name.lower()]
                        self.callback_add_place_srv(reqAddPlace)  
                        sourceNode=reqAddPlace.name
                    self.route = nx.shortest_path(self.graph, source=sourceNode, target=req.name.lower())[1:]
                    self.poses_sequence = self.createPosesSequence(self.route, False)
                    print('\033[95m'+"Route to follow: ")
                    print(*self.route, sep='->')
                    print('\033[0m')
                self.sendGoal(self.FRAME_ID_MAP)
                return 'approved'
            else:
                print(consoleFormatter.format("Go to place service not-approved: Graph must be 0 for not using graph or 1 for using.", "FAIL"))
                return 'not-approved'
        else:
            print(consoleFormatter.format("Go to place service not-approved: Place '"+req.name.lower()+"' is not known.", "FAIL"))
            return 'not-approved'

    def callback_start_random_navigation_srv(self, req):
        """
        Callback for start_random_navigation_srv service: Starts a random navigation in the static map loaded. It constantly chooses
        a random goal (but reachable) and send it to the move_action client until robot_stop_srv service is called.

        Args:
            req: start_random_navigation_srv request message. {Empty}

        Returns:
            Returns 'approved' if the random navigation was started successfully.
        """
        print(consoleFormatter.format('\nRequested start random navigation service', 'WARNING'))
        self.random_navigation = True
        self.generateRandomTarget()
        self.sendGoal(self.FRAME_ID_MAP)
        print(consoleFormatter.format('Random navigation started successfully', 'OKGREEN'))
        return 'approved'

    def callback_add_place_srv(self, req):
        """
        Callback for add_place_srv service: Add a place in a .txt file. It specifies the name of the place and the coordinates. 
        If persist is a 1, the place will be saved for future occasions. If it is a 0, it will remember the location until 
        navigation_utilities is disabled. Regardless of whether it is persisted or not, the location will be added to NavigationGraph.

        Args:
            req: add_place_srv request message. {name(str),persist(int),edges(str[])}

        Returns:
            Returns 'approved' if the place was saved successfully.
        """
        print(consoleFormatter.format('\nRequested add place service', 'WARNING'))
        fileP = open(self.PATH_PLACES,"a")
        fileE = open(self.PATH_EDGES,"a")
        xGoal = round(self.currentPositionAmcl.position.x,2)
        yGoal = round(self.currentPositionAmcl.position.y,2)
        thetaGoal = round(np.degrees(self.yawAmcl),2)
        self.graph.add_node(req.name.strip().lower(), x=float(xGoal), y=float(yGoal), theta=float(thetaGoal))
        print(consoleFormatter.format('La cantidad de elementos en la lista es de: ' + str(len(req.edges)), 'OKGREEN'))
        for edge_name in req.edges:
            self.graph.add_edge(req.name.strip().lower(), edge_name.strip().lower())
        if req.persist == 1:
            fileP.write(req.name + "," + str(xGoal) + "," + str(yGoal) + "," + str(thetaGoal)+ "\n")
            fileP.close()
            for edge_name in req.edges:
                fileE.write(req.name.strip().lower() + "," + edge_name.strip().lower()+ "\n")
            fileE.close()   
        print(consoleFormatter.format('Place saved successfully', 'OKGREEN'))
        return 'approved' 


    def callback_follow_you_srv(self, req):
        """
        Callback for follow_you_srv service: Moves  look_for_object_srvRequest, get_labels_srv, get_labels_srvRequest, save_image_srv, save_image_srvRequest, get_person_description_srvRequest, get_person_description_srv
lace(str)}

        Returns:
            Returns 'approved' if the place was reached successfully.
        """
        self.placeFollowYou=req.place
        print(consoleFormatter.format('\nRequested follow_you service', 'WARNING'))
        if self.isTouched:
            self.follow_active=True
            reqGoToPlace=go_to_place_srvRequest()
            reqGoToPlace.name = self.placeFollowYou
            reqGoToPlace.graph = self.navigation_graph
            self.callback_go_to_place_srv(reqGoToPlace)
    
        else:
            self.follow_active=False
            print(consoleFormatter.format('Follow_you service not-approved', 'FAIL'))
            return 'not_approved'
        print(consoleFormatter.format('Follow_you service started successfully', 'OKGREEN'))
        return 'approved' 


    def callback_robot_stop_srv(self, req):
        """
        Callback for robot_stop_srv service: Tries to cancel active goals. If random navigation is active it is disabled and 'temp'
        node is removed.

        Args:
            req: robot_stop_srv request message. {Empty}. 

        Returns:
            Returns 'approved' if the robot was stopped successfully and 'not-approved' otherwise.
        """
        print(consoleFormatter.format("\nRequested robot stop service", "WARNING"))
        self.robot_stopped = True
        try:
            self.actionClient.cancel_all_goals()
            if self.random_navigation == True:
                self.graph.remove_node('temp')
                self.random_navigation = False
                print(consoleFormatter.format('Random navigation was stopped', 'WARNING'))
            print(consoleFormatter.format('Robot was stopped successfully', 'OKGREEN'))
            return 'approved'
        except:
            print(consoleFormatter.format("Robot stop service not-approved: No goals are currently active. Cannot stop robot", "FAIL"))
            return 'not-approved'


    def callback_spin_srv(self, req):
        """
        Callback for spin_srv service: Makes the robot turn depending on the angle (in degrees) that has been entered, 
        the path will be the shortest to reach the objective.


        Args:
            req: spin_srv request message. {degrees(float64)}. 

        Returns:
            Returns 'approved' if the robot has turned successfully and 'not-approved' otherwise.
        """
        print(consoleFormatter.format("\nRequested spin service", "WARNING"))
        self.robot_stopped=False
        if req.degrees>360 or req.degrees<-360:
            print(consoleFormatter.format("Invalid arguments, Robot could not turn", "FAIL"))
            return 'not-approved'
        actual_angle=self.correctAngle(np.degrees(self.yawOdom))
        thetaGoal=actual_angle+req.degrees    
        thetaGoal=self.correctAngle(thetaGoal)
        if 0<req.degrees<=180 or -360<=req.degrees<=-180:
            z=1.25
        elif 180<req.degrees<=360 or -180<req.degrees<0:
            z=-1.25
        TwistMsg=Twist()    
        print(consoleFormatter.format('Robot is turning!', 'HEADER'))
        duration = rospy.Duration(40)
        beginTime = rospy.Time.now()
        endTime = beginTime + duration
        while abs(actual_angle - thetaGoal) >= self.ANGULAR_THRESHOLD and not self.robot_stopped:
            actual_angle=self.correctAngle(np.degrees(self.yawOdom))
            radiansToSpin = self.calculateMinimumDifferenceBetweenTwoAngles(np.radians(actual_angle), np.radians(thetaGoal))
            TwistMsg.angular.z = z*(1/(1+np.exp((-5*abs(radiansToSpin))/(4*np.pi)))-0.35)
            self.cmd_velPublisher.publish(TwistMsg)
            if rospy.Time.now()>endTime:                
                TwistMsg.angular.z=0
                self.cmd_velPublisher.publish(TwistMsg)
                print(consoleFormatter.format('Robot has aborted the spin_srv: RUN_TIME_ERROR', 'FAIL'))

                return "not-approved"
            rospy.Rate(10).sleep()
        TwistMsg.angular.z=0
        duration = rospy.Duration(2)
        beginTime = rospy.Time.now()
        endTime = beginTime + duration       
        while rospy.Time.now() < endTime:
            self.cmd_velPublisher.publish(TwistMsg)
            rospy.Rate(15).sleep()          
        print(consoleFormatter.format('Robot has turned successfully', 'OKGREEN'))
        return 'approved'            

    def callback_go_to_defined_angle_srv(self, req):
        """
        Callback for go_to_defined_angle_srv service: Makes the robot turn 
        so that it is in a defined orientation relative to the map.


        Args:
            req: go_to_defined_angle_srv request message. {degrees(float64)}. 

        Returns:
            Returns 'approved' if the robot has turned successfully and 'not-approved' otherwise.
        """
        print(consoleFormatter.format("\nRequested do to defined angle service", "WARNING"))
        actual_angle=self.correctAngle(np.degrees(self.yawAmcl))
        thetaGoal=req.degrees-actual_angle
        reqSpinSrv=spin_srvRequest()
        reqSpinSrv.degrees=thetaGoal
        self.callback_spin_srv(reqSpinSrv)
        print(consoleFormatter.format("\nThe robot has reached the angle of "+str(req.degrees)+" degrees", "OKBLUE"))
        return "approved"
    
    def callback_get_absolute_position_srv(self,req): 
        """
        Callback for get_absolute_position_srv service:Returns the main information of the robot's location relative to the map.
        This is made up of coordinates on the x,y axis and the orientation of the robot. .


        Args:
            req: get_absolute_position_srv request message. {Empty}. 

        Returns:
            res: get_absolute_position_srv response message. {x(float64),y(float64),theta(float64)}.
        """
        print(consoleFormatter.format("\nRequested get absolute position service", "WARNING"))
        res = get_absolute_position_srvResponse()
        res.x = round(self.currentPositionAmcl.position.x,2)
        res.y = round(self.currentPositionAmcl.position.y,2)
        res.theta = self.correctAngle(round(np.degrees(self.yawAmcl),2))
        print(consoleFormatter.format("\nThe coordinates in x are: "+str(res.x)+"\nThe coordinates in y are: "+str(res.y)+"\nThe robot orientation is: "+str(res.theta)+" degrees", "OKBLUE"))
        return res

    def callback_get_route_guidance_srv(self, req):
        """
        Callback for get_absolute_position_srv service:Returns the necessary indications for the robot to reach
        the place that is entered as parameter.


        Args:
            req: get_route_guidance_srv request message. {place(string)}. 

        Returns:
            res: get_route_guidance_srv response message. {instructions(string)}.
        """
        print(consoleFormatter.format("\nRequested get rout guidance service", "WARNING"))
        instructions = ""
        sourceNode = self.mapSourceNode()
        route = nx.shortest_path(self.graph, source=sourceNode, target=req.place.lower())[1:]
        previousAngleBetween = self.correctAngle(np.degrees(self.yawAmcl))
        previousPlace = [self.currentPositionAmcl.position.x, self.currentPositionAmcl.position.y]
        print(previousPlace)
        angles = []
        for place in route:
            actualNode = self.graph.nodes[place]
            actualAngleBetween = self.calculateDesiredOrientation(previousPlace[0], previousPlace[1], actualNode["x"], actualNode["y"]) 
            angles.append(np.degrees(self.calculateMinimumDifferenceBetweenTwoAngles(previousAngleBetween, actualAngleBetween)))
            previousAngleBetween = actualAngleBetween
            previousPlace = [actualNode["x"], actualNode["y"]]
        i_angle = 0
        metros = 0
        prev_x = self.currentPositionAmcl.position.x
        prev_y = self.currentPositionAmcl.position.y
        while i_angle<len(angles):
            if abs(angles[i_angle]) <= 15:
                instructions+=" Go straight"
            else:
                direccion = "left"
                if angles[i_angle] < 0:
                    direccion = "right"
                if metros > 0:
                    if i_angle == 0:
                        instructions+=" Go straight "
                    instructions+=str(round(metros, 1))+ " meters \n"
                    if route[i_angle-1] in self.KNOWN_PLACES:
                        instructions+="until you find the " + str(route[i_angle-1])+" \n"
                if abs(angles[i_angle])>15 and abs(angles[i_angle])<=65:
                    instructions+="turn slightly to the "+direccion+" and then go straight \n"
                elif abs(angles[i_angle])>65 and abs(angles[i_angle])<110:
                    instructions+="turn to the "+direccion+" and then go straight \n"
                    
            while i_angle+1<len(angles) and abs(angles[i_angle+1])<20:
                    i_angle+=1
            metros = self.calculateEuclideanDistance(prev_x, prev_y, self.graph.nodes[route[i_angle]]["x"], self.graph.nodes[route[i_angle]]["y"])
            if i_angle == len(angles)-1:
                instructions+=str(round(metros,1))+ " meters \n"
                if route[i_angle] in self.KNOWN_PLACES:
                    instructions+="until "+ route[i_angle]+" \n"
            prev_x = self.graph.nodes[route[i_angle]]["x"]
            prev_y = self.graph.nodes[route[i_angle]]["y"]
            i_angle+=1
        print(consoleFormatter.format(instructions, "HEADER"))
        return instructions

    def callback_correct_position_srv(self, req):
        xGoal = round(self.currentPositionAmcl.position.x,2)
        yGoal = round(self.currentPositionAmcl.position.y,2)
        actualTheta=round(np.degrees(self.yawAmcl),2)
        thetaGoal = self.correctAngle(actualTheta+req.degrees)
        self.graph.add_node("tempcorrectposition", x=float(xGoal), y=float(yGoal), theta=float(thetaGoal))
        reqsetCurrentPlace=set_current_place_srvRequest()
        reqsetCurrentPlace.name="tempcorrectposition"
        self.callback_set_current_place_srv(reqsetCurrentPlace)
        rospy.sleep(3)
        self.graph.remove_node('tempcorrectposition')
        return "approved"

                

    # -----------------------------------------------------------------------------------------------------------------------
    # ----------------------------------------------------AUXILIAR CALLBACKS-------------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------------


    def callback_active(self):
        """
        Callback for active send_goal action client: Prints the information of the goal being processed.
        """
        self.navigation_status = 1
        print(consoleFormatter.format("Goal '"+self.route[self.goal_index]+"' is now being processed by the Navigation Stack Action Server...", "OKBLUE"))

        

    def callback_feedback(self, feedback):
        """
        Callback for feedback received by the action client: When feedback is received the simpleFeedbackPublisher and the
        completeFeedbackPublisher publish.

        Args:
            feedback (MoveBaseFeedback): Feedback received by the move_base action.
        """
        self.completeFeedbackPublisher.publish(feedback)
        self.simpleFeedbackPublisher.publish(self.navigation_status)
        if self.follow_active:
            if not self.isTouched:
                reqRobotStop=robot_stop_srvRequest()
                self.callback_robot_stop_srv(reqRobotStop)
                duracion = rospy.Duration(5)
                beginTime = rospy.Time.now()
                endTime = beginTime + duracion
                print(consoleFormatter.format("Please touch my head", "WARNING"))
                while self.isTouched==False and rospy.Time.now() < endTime:
                    rospy.sleep(0.5)
                if self.isTouched:
                    reqGoToPlace=go_to_place_srvRequest()
                    reqGoToPlace.name = self.placeFollowYou
                    reqGoToPlace.graph = self.navigation_graph
                    self.callback_go_to_place_srv(reqGoToPlace)
                else:
                    self.follow_active=False
                    print(consoleFormatter.format("You did not touch my head, so I will not continue", "FAIL"))


    def callback_done(self, status, result):
        """
        Callback for status change of the action client: Prints information related to the status on the goal sent to the
        move_base action. Also if the goal was reached checks if all the goals in poses_sequence were sent and if not it 
        send the remaining goals. If random navigation is active it generates another random target and send it like goal
        to the move_base client.

        Args:
            status (int): Status information on the goal sent to the move_base action.
        """
        self.goal_index += 1
        if status == 2:
            print(consoleFormatter.format("Goal '"+self.route[self.goal_index-1]+"' received a cancel request after it started executing, completed execution!","WARNING"))
            self.navigation_status = 4
            self.publish_simple_feedback(3)
            self.navigation_status = 0
            self.publish_simple_feedback(3)

        if status == 3:
            print(consoleFormatter.format("Goal '"+self.route[self.goal_index-1]+"' reached\n", "OKGREEN"))
            if self.route[self.goal_index-1]=='temporal':
                self.graph.remove_node('temporal')
            self.numero_intentos=0
            if self.goal_index < len(self.poses_sequence):
                self.sendGoal(self.FRAME_ID_MAP)
            elif self.random_navigation == True:
                self.generateRandomTarget()
                self.sendGoal(self.FRAME_ID_MAP)
            else:
                if self.navigation_graph == 1:
                    spinRequest = spin_srvRequest()
                    yaw2 = np.radians(self.graph.nodes[self.route[self.goal_index-1]]['theta'])
                    spinRequest.degrees = np.degrees(self.calculateMinimumDifferenceBetweenTwoAngles(self.yawAmcl, yaw2))
                    self.callback_spin_srv(spinRequest)
                print(consoleFormatter.format("--- Final goal reached! ---", "OKGREEN"))
                self.navigation_status = 2
                self.publish_simple_feedback(3)
                self.navigation_status = 0
                self.publish_simple_feedback(3)
                return

        if status == 4:
            if self.numero_intentos < self.MAX_TRIES:
                self.numero_intentos += 1
                print(consoleFormatter.format('The robot tried to abort. This is the attempt '+ str(self.numero_intentos), 'FAIL'))
                rospy.sleep(5)
                print(consoleFormatter.format('Clearing global costmap_2D', 'OKBLUE'))
                rospy.sleep(1.5)
                self.clearCostmapServiceClient()
                print(consoleFormatter.format('Global costmap_2D was cleared successfully', 'OKGREEN'))
                reqGoToPlace=go_to_place_srvRequest()
                reqGoToPlace.name = self.destino_final
                reqGoToPlace.graph = self.navigation_graph
                self.callback_go_to_place_srv(reqGoToPlace)
            else:
                self.numero_intentos = 0 
                print(consoleFormatter.format("Goal pose '"+self.route[self.goal_index-1]+"' was aborted by the Action Server", "FAIL"))
                self.navigation_status = 3
                self.publish_simple_feedback(3)
                self.navigation_status = 0
                self.publish_simple_feedback(3)
            return

        if status == 5:
            print(consoleFormatter.format("Goal pose '"+self.route[self.goal_index-1]+"' has been rejected by the Action Server", 'FAIL'))
            self.navigation_status = 3
            self.publish_simple_feedback(3)
            self.navigation_status = 0
            self.publish_simple_feedback(3)
            return

        if status == 8:
            print(consoleFormatter.format("Goal pose '"+self.route[self.goal_index-1]+"' received a cancel request before it started executing, successfully cancelled!", "WARNING"))
            self.navigation_status = 4
            self.publish_simple_feedback(3)
            self.navigation_status = 0
            self.publish_simple_feedback(3)

    # -----------------------------------------------------------------------------------------------------------------------
    # ----------------------------------------------------SUBSCRIBERS CALLBACKS----------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------------

    def callback_amcl_pose_subscriber(self, msg:PoseWithCovarianceStamped):
        """
        Callback for /amcl_pose subscriber: It changes the currentPositionAmcl attribute with the information of the
        input message.

        Args:
            msg (PoseWithCovarianceStamped): Robot's estimated pose in the map, with covariance.
        """
        self.currentPositionAmcl = msg.pose.pose
        self.yawAmcl = euler_from_quaternion([self.currentPositionAmcl.orientation.x, self.currentPositionAmcl.orientation.y, self.currentPositionAmcl.orientation.z, self.currentPositionAmcl.orientation.w])[2]

    def callback_odom_subscriber(self,msg:Odometry):
        """
        Callback for /odom subscriber: It changes the currentPositionOdom attribute with the information of the
        input message.

        Args:
            msg (Odometry): Robot's estimated pose by the odom.
        """
        self.currentPositionOdom = msg.pose.pose
        self.yawOdom = euler_from_quaternion([self.currentPositionOdom.orientation.x, self.currentPositionOdom.orientation.y, self.currentPositionOdom.orientation.z, self.currentPositionOdom.orientation.w])[2]

    def callback_costmap_subscriber(self, msg:OccupancyGrid):
        """
        Callback for /move_base/global_costmap/costmap subscriber: It changes the costMap attribute with the information of the
        input message.

        Args:
            msg (OccupancyGrid): 2-D Grid map, in which each cell represents the probability of occupancy.
        """
        self.costMap = msg.data
    
    def callback_head_sensor_subscriber(self, msg:touch_msg):
        """
        Callback for /touch subscriber: It changes the isTouched attribute with the information of the
        input message.

        Args:
            msg (touch.msg): {name, state} The value of 'name' represents the name of the sensor in the head and the 'state' is a boolean 
            indicating whether it is pressed or not.
        """
        if msg.name == "head_rear":
            self.sensorRear = msg.state
        elif msg.state == "head_middle":
            self.sensorMiddle = msg.state
        elif msg.state == "head_front":
            self.sensorFront = msg.state
        if self.sensorFront or self.sensorMiddle or self.sensorRear:
            self.isTouched=True
        else:
            self.isTouched=False

    # -------------------------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------AUXILIAR FUNCTIONS-------------------------------------------------
    # -------------------------------------------------------------------------------------------------------------------------

    def mapSourceNode(self):
        """
        Given the current position of the robot, it estimates which is the node of the graph (place) where it is currently
        and where the path should begin.
        
        Returns:
            Returns the name of the current node (place) estimated. 
        """
        distances = dict()
        for place in list(self.graph.nodes()):
            distances[place] = self.calculateEuclideanDistance(self.currentPositionAmcl.position.x, self.currentPositionAmcl.position.y, self.graph.nodes[place]["x"], self.graph.nodes[place]["y"])
        return min(distances, key=distances.get)

    def createPosesSequence(self, places, isSetCurrentPlace):
        """
        Given a list of known places it creates a set of poses corresponding to the position and orientation of each
        place.

        Args:
            places (list): List of places to create the set of poses.
            isSetCurrentPlace (bool): Boolean that indicates if function is called by set_current_place service
        
        Returns:
            Returns a list of poses corresponding to each place in the input list.
        """
        points = list()
        quaternions = list()
        poses_sequence = list()
        for place in range(len(places)):
            infoPlace = self.graph.nodes[places[place]]
            points.append([infoPlace['x'], infoPlace['y'], 0])
            theta = None
            if (self.navigation_graph != 1 or isSetCurrentPlace):
                theta = np.radians(infoPlace['theta'])
            else:
                if place == 0:
                    theta = self.calculateDesiredOrientation(self.currentPositionAmcl.position.x, self.currentPositionAmcl.position.y, infoPlace['x'], infoPlace['y'])
                else:
                    infoPreviousPlace = self.graph.nodes[places[place-1]]
                    theta = self.calculateDesiredOrientation(infoPreviousPlace['x'], infoPreviousPlace['y'], infoPlace['x'], infoPlace['y'])
            quaternions.append(Quaternion(*(quaternion_from_euler(0, 0, theta, axes='sxyz'))))
        for point in range(len(points)):
            poses_sequence.append(Pose(Point(*points[point]), quaternions[point]))
        return poses_sequence

    def sendGoal(self, frame_id):
        """
        Sends the goal in the position goal_index of the poses_sequence list attribute to the move_base action client with
        the frame_id specified by parameter.

        Args:
            frame_id (str): Coordinate frame of the goal (e.g., map, base_link, base_footprint)
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.poses_sequence[self.goal_index]
        if self.navigation_graph == 1:
            spinRequest = spin_srvRequest()
            (roll, pitch, actualYaw) = euler_from_quaternion([self.poses_sequence[self.goal_index].orientation.x, self.poses_sequence[self.goal_index].orientation.y, self.poses_sequence[self.goal_index].orientation.z, self.poses_sequence[self.goal_index].orientation.w])
            if self.calculateEuclideanDistance(self.currentPositionAmcl.position.x, self.currentPositionAmcl.position.y, self.graph.nodes[self.route[self.goal_index]]['x'], self.graph.nodes[self.route[self.goal_index]]['y'])<0.15:
                spinRequest.degrees = 0
            else:
                spinRequest.degrees = np.degrees(self.calculateMinimumDifferenceBetweenTwoAngles(self.yawAmcl, actualYaw))
            self.callback_spin_srv(spinRequest)
        if not self.robot_stopped:
            print(consoleFormatter.format("Sending goal: '"+self.route[self.goal_index]+"' to the Navigation Stack Action Server", "WARNING"))
            self.actionClient.send_goal(goal, self.callback_done, self.callback_active, self.callback_feedback)
        
    def publish_simple_feedback(self, time):
        """
        SimpleFeedback publishes the navigation_status for the time (in seconds) specified by parameter.

        Args:
            time (int): Time for wich it must be published in seconds. 
        """
        duration = rospy.Duration(time)
        beginTime = rospy.Time.now()
        endTime = beginTime + duration
        while rospy.Time.now() < endTime:
            self.simpleFeedbackPublisher.publish(self.navigation_status)
            rospy.sleep(0.5)

    def mapToWorld(self, map_x, map_y, map_origin_x, map_origin_y, map_resolution):
        """
        Converts the coordinates of a grid point to the world coordinates (map frame).

        Args:
            map_x (int): x_coordinate of the grid point to convert.
            map_y (int): y_coordinate of the grid point to convert.
            map_origin_x (float): x_coordinate of the map's origin.
            map_origin_y (float): y_coordinate of the map's origin.
            map_resolution (float): resolution of the map.
        
        Returns:
            Returns the converted world coordinates.
        """
        world_x = map_origin_x + (map_x + 0.5) * map_resolution
        world_y = map_origin_y + (map_y + 0.5) * map_resolution
        return world_x, world_y

    def worldToMap(self, world_x, world_y, map_origin_x, map_origin_y, map_resolution):
        """
        Converts the coordinates of a world point to the grid coordinates.

        Args:
            world_x (int): x_coordinate of the world point to convert.
            world_y (int): y_coordinate of the world point to convert.
            map_origin_x (float): x_coordinate of the map's origin.
            map_origin_y (float): y_coordinate of the map's origin.
            map_resolution (float): resolution of the map.
        
        Returns:
            Returns the converted world coordinates.
        """
        map_x = round((world_x - map_origin_x)/map_resolution)
        map_y = round((world_y - map_origin_y)/map_resolution)
        return map_x, map_y

    def generateRandomTarget(self):
        """
        Given the static map data it chooses a random reachable target and put it in the poses_sequence list and in the route list
        to be send it to the move_base action client through the send_goal method. If a random target was created before, its removed
        and a new random target is created.
        """
        id_x = None
        self.goal_index = 0
        while(id_x is None or self.STATIC_MAP_INFO['map_data'][id_x]!=0 or self.costMap[id_x]>85):
            map_x = random.randint(5, self.STATIC_MAP_INFO['map_size_x']-5)   
            map_y = random.randint(5, self.STATIC_MAP_INFO['map_size_y']-5)
            world_x, world_y = self.mapToWorld(map_x, map_y, self.STATIC_MAP_INFO['map_origin_x'], self.STATIC_MAP_INFO['map_origin_y'], self.STATIC_MAP_INFO['map_resolution'])
            id_x = map_x + map_y * self.STATIC_MAP_INFO['map_size_x']
        if 'temp' in list(self.graph.nodes()):
            self.graph.remove_node('temp')
        self.graph.add_node('temp', x=world_x, y=world_y, theta=90)
        self.poses_sequence = self.createPosesSequence(['temp'], False)
        self.route = ['temp']
        
    def loadStaticMapInfo(self):
        """
        Sends a request to the static_map service provided by the map_server node and extracts the data from the response to a
        dictionary which is returned.
        
        Returns:
            Returns a dictionary with the map data.
        """
        print(consoleFormatter.format('Loading static mapa data', 'OKBLUE'))
        mapInfo=dict()
        res = self.staticMapServiceClient.call()
        mapInfo['map_origin_x'] = res.map.info.origin.position.x
        mapInfo['map_origin_y'] = res.map.info.origin.position.y
        mapInfo['map_resolution'] = res.map.info.resolution
        mapInfo['map_size_x']= res.map.info.width
        mapInfo['map_size_y'] = res.map.info.height
        mapInfo['map_data'] = res.map.data
        print(consoleFormatter.format('Static map data loaded successfully', 'OKGREEN'))
        return mapInfo

    def loadKnownPlaces(self):
        print(consoleFormatter.format('Loading known places data', 'OKBLUE'))
        knownPlaces = [place for place, attributes in self.graph.nodes(data=True) if attributes['known']==True]
        print(knownPlaces)
        print(consoleFormatter.format('Known places loaded successfully', 'OKGREEN'))
        return knownPlaces
    
    def correctAngle(self, angle):
        """
        Receives an angle (in degrees) outside the range 0 to 360 and converts it to its complementary angle that is in this range.
        
        Args:
        angle (float): angle you want to pass to the desired range.

        Returns:
            Returns the angle corrected.
        """
        if angle>360:
            return angle-360
        if angle<0:
            return angle+360
        else:
            return angle

    def calculateDesiredOrientation(self, xOrigin, yOrigin, xGoal, yGoal):
        """
        Receives two points on the map (one start and one end) and calculates the direction (an angle)
        of the resulting vector by joining these.

        Args:
            xOrigin (float): x_coordinate of the origin point on the map.
            yOrigin (float): y_coordinate of the origin point on the map.
            xGoal (float): x_coordinate of the goal point on the map.
            yGoal (float): y_coordinate of the origin point on the map.

        Returns:
            Returns the final direction (an angle) of the vector between the two points.
        """
        return np.arctan2(yOrigin-yGoal, xOrigin-xGoal)+(np.pi)

    def calculateMinimumDifferenceBetweenTwoAngles(self, thetaOrigin, thetaGoal):
        """
        Receives two angles (in radians) and calculate the minimum difference between them (an angle).

        Args:
            thetaOrigin (float): starting position angle.
            thetaGoal (float): final postion angle.

        Returns:
            Returns an angle hat represents the minimum difference.
        """

        return np.arctan2(np.sin(thetaGoal - thetaOrigin), np.cos(thetaGoal - thetaOrigin))

    def calculateEuclideanDistance(self, xPoint1, yPoint1, xPoint2, yPoint2):
        """
        Given the coordinates of 2 points it calculates the euclidean distance between those points.

        Args:
            xPoint1 (float): x coordinate of the first point.
            yPoint1 (float): y coordinate of the first point.
            xPoint2 (float): x coordinate of the second point.
            yPoint2 (float): y coordinate of the second point
        
        Returns:
            Returns the distance between the points.
        """
        return np.linalg.norm(np.array([xPoint1, yPoint1])-np.array([xPoint2, yPoint2]))
       
    
# -----------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------MAIN-----------------------------------------------------------
# -----------------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    consoleFormatter=ConsoleFormatter.ConsoleFormatter()
    rospy.init_node('navigation_utilities')
    navigationUtilities=NavigationUtilities()
    if(navigationUtilities.graph is not None):
        print(consoleFormatter.format(" \n----------------------------------------------------------", "OKGREEN"))  
        print(consoleFormatter.format(" --- navigation_utilities node successfully initialized --- ", "OKGREEN"))
        print(consoleFormatter.format(" ----------------------------------------------------------\n", "OKGREEN")) 
        rospy.spin()
    else:
        print(consoleFormatter.format(" \n------------------------------------------------", "FAIL"))  
        print(consoleFormatter.format(" --- navigation_utilities was not initialized ---", "FAIL"))
        print(consoleFormatter.format(" ------------------------------------------------\n", "FAIL"))  