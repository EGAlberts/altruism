#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node

from altruism_msgs.action import Identify
from std_msgs.msg import Float64
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from nav2_msgs.action import NavigateToPose

from darknet_ros_msgs.action import CheckForObjects
from copy import deepcopy
from itertools import cycle
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math


PICTURE_RT_PARAM = "pic_rate"
GOAL_OBJ_NAME_PARAM = "goal_obj"
DET_THRESH_PARAM = "det_threshold"
#GPT
def explore_group(row, col, group, visited, grid):
    rows, columns = grid.shape
    if row < 0 or row >= rows or col < 0 or col >= columns:
        return
    if grid[row, col] != 100  or visited[row, col]:
        return
    
    visited[row, col] = True
    group.append([row, col])
    
    for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        explore_group(row + dr, col + dc, group, visited, grid)
#GPT
def find_groups(grid):
    rows, columns = grid.shape
    visited = np.zeros((rows, columns), dtype=bool)
    groups = []
    
    for row in range(rows):
        for col in range(columns):
            if grid[row, col] == 100 and not visited[row, col]:
                new_group = []
                explore_group(row, col, new_group, visited, grid)
                groups.append(new_group)
    
    # Find the largest connected group
    map_border = max(groups, key=len) #this is the edge of the room and we aren't interested in it
    
    # Remove the largest group from the list
    groups.remove(map_border)
    

    return groups

#code also appears in the other action_server, make general..
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def cycle_toandfro(iterable):
    # cycle_toandfro('ABCD') --> A B C D C B A B C D C B A
    while True:
        saved = []
        for element in iterable:
            yield element
            saved.append(element)
        saved.reverse()
        saved.pop(0)
        saved.pop()
        while saved:
            yield saved.pop(0)

              
class IdentifyActionServer(Node):

    def __init__(self):
        super().__init__('identify_action_server')
        self._action_server = ActionServer(
            self,
            Identify,
            'identify',
            self.execute_callback,
            callback_group=MutuallyExclusiveCallbackGroup())
        
        self.declare_parameter(GOAL_OBJ_NAME_PARAM, "fire hydrant")
        self.declare_parameter(PICTURE_RT_PARAM, 1)
        self.declare_parameter(DET_THRESH_PARAM,14)
        

        
        self.check_obj_acclient = ActionClient(self, CheckForObjects, 'checkForObjectsActionName', callback_group = MutuallyExclusiveCallbackGroup())
        print(type(self.check_obj_acclient))
        

        self.subscription = self.create_subscription(Image,'/camera/image_raw',self.tb_image_cb,10, callback_group = MutuallyExclusiveCallbackGroup())
        
        self.tb_image = None
        self.map = None
        self.counter = 0
        self.prev_done = True
        self.ID_fdback_msg = Identify.Feedback()
        self.ID_fdback_msg.obj_idd.object_names = []
        self.ID_fdback_msg.obj_idd.probabilities = []
        self.ID_fdback_msg.obj_idd.object_detected = False
        self.initial_time = time.time()
        self.times_detected = 0
        #bool object_detected
        #string object_name
        #float32 probability
        self.get_logger().info('ID Action server created...')

        self.picture_rate = self.get_parameter(PICTURE_RT_PARAM).get_parameter_value().integer_value
        self.ID_fdback_msg.picture_rate = self.picture_rate

        self.detection_threshold = self.get_parameter(DET_THRESH_PARAM).get_parameter_value().integer_value #parameterize
        self.goal_object = self.get_parameter(GOAL_OBJ_NAME_PARAM).get_parameter_value().string_value

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)
        self.initial_pose = PoseWithCovarianceStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.get_clock().now().to_msg()
        
        self.initial_pose.pose.pose.position.x = -2.0
        self.initial_pose.pose.pose.position.y = -0.5

        self.previous_pose = self.initial_pose.pose

        q = quaternion_from_euler(0, 0, 0)
        self.initial_pose.pose.pose.orientation.x = q[0]
        self.initial_pose.pose.pose.orientation.y = q[1]
        self.initial_pose.pose.pose.orientation.z = q[2]
        self.initial_pose.pose.pose.orientation.w = q[3]
        




    def tb_image_cb(self,msg):
        self.tb_image = msg
        self.counter+=1
        if(self.counter % 1000 == 0): self.get_logger().info('1000th Image Msg Received!!')
        if self.counter > 200000: self.counter = 0


    #code also appears in the other action_server, make general..
    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        if pose is None:
            return False
        
        # #Intermediate Pose to help navigation -- perhaps should use recursion..
        # inter_pose = PoseStamped()
        # inter_pose.header.frame_id = 'map'

        # inter_pose.pose.orientation = pose.pose.orientation

        # inter_pose.pose.position.x = (self.previous_pose.pose.position.x + pose.pose.position.x)/2
        # inter_pose.pose.position.y = (self.previous_pose.pose.position.y + pose.pose.position.y)/2

        # inter_pose.header.stamp = self.get_clock().now().to_msg()

        # self.get_logger().debug("Waiting for 'NavigateToPose' action server")
        # while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
        #     self.get_logger().info("'NavigateToPose' action server not available, waiting...")


        # goal_msg = NavigateToPose.Goal()
        # goal_msg.pose = inter_pose
        # goal_msg.behavior_tree = behavior_tree

        # send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
        #                                                            self._feedbackCallback)
        # rclpy.spin_until_future_complete(self, send_goal_future)
        
        # self.goal_handle = send_goal_future.result()

        # if not self.goal_handle.accepted:
        #     self.get_logger().error('Goal to ' + str(pose.pose.position.x) + ' ' +
        #                str(pose.pose.position.y) + ' was rejected!')
        #     return False

        # self.result_future = self.goal_handle.get_result_async()
        # rclpy.spin_until_future_complete(self, self.result_future,timeout_sec=20)

        ###

        pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.get_logger().info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future,timeout_sec=None)
        
        #self.get_logger().info('NavToPose result error code: ' + str(self.result_future.result().result.error_code)) this should work in a future release of nav2

        self.previous_pose = pose    

        return True
    

    def execute_callback(self, goal_handle):

        self.get_logger().info('inital pose...')

        self.initial_pose_pub.publish(self.initial_pose) #this needs to be provided for when you use a map and not slam.

        self.get_logger().info('Executing goal...')

        self.get_logger().info('Its me, hi, Im a ID action server')
        self.map = goal_handle.request.map

        map_rows = [self.map.data[x:x+self.map.info.width] for x in range(0, len(self.map.data), self.map.info.width)]

        #ASSERT len(chunks) == height

        map_as_array = np.array(map_rows)
        # Crucially the notion of 0,0 are upside down, 
        map_as_array = np.flipud(map_as_array)
        #So I make it so indexing matches up and down of robot.

        # for row in map_as_array:
        #     self.get_logger().info(str(row))



        obstacles = find_groups(map_as_array) #this contains the rows, col of each obstacles
        # Altough 0,0 may match here, the x and y are flipped, because if I do obstacles[0] I get the row while in x,y the x coord is the column.
        #the row, col needs to become x,y. 
        
        # the matrix' 0,0 is the top left while the 0,0 (x,y) for the robot and map is bottom left.
        #



        origin_x, origin_y = [self.map.info.origin.position.x, self.map.info.origin.position.y]
        for obstacle in obstacles: #convert from cells (grid representation) to actual meters
            for pt in obstacle:
                #self.get_logger().info('origin x : {1} resolution: {0} pt[0]: {2} '.format(origin_x, self.map.info.resolution, pt[0]))
                #self.get_logger().info('origin y : {1} resolution: {0} pt[1]: {2} '.format(origin_y, self.map.info.resolution, pt[1]))
                
                #0 is the rows, 1 is the columns, which is reverse of x,y
                pt[1] = origin_x + ( (pt[1]*self.map.info.resolution) + (self.map.info.resolution/2))
                pt[0] = origin_y + ( (pt[0]*self.map.info.resolution) + (self.map.info.resolution/2))

                # self.get_logger().info('pt[0]: {0} pt[1]: {1}  resolution {2}'.format(pt[0], pt[1], self.map.info.resolution))

        points_to_visit = []

        for obstacle in obstacles:
            all_the_ys = [pt[0] for pt in obstacle] 
            all_the_xs = [pt[1] for pt in obstacle]
            #Once again as the pt's are in row,col this is the reverse of x,y so we handle them in reverse.
            horizontal_middle = (max(all_the_ys) + min(all_the_ys)) / 2
            vertical_bottom = min(all_the_xs)
            points_to_visit.append([vertical_bottom - 0.4, -horizontal_middle + 0.15, math.atan2(-horizontal_middle - (-horizontal_middle + 0.15), vertical_bottom - (vertical_bottom - 0.4))]) #the 0.4 is so the robot is a bit below and doesn't collide with the obstacle


        route_poses = []

        pose = PoseStamped()
        pose.header.frame_id = 'map'

        for pt in points_to_visit:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            q = quaternion_from_euler(0, 0, pt[2])
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            route_poses.append(deepcopy(pose))
        
        self.get_logger().info("the poses:")
        self.get_logger().info(str(route_poses))
        self.get_logger().info("\n\n")



        
        reordered_visiting = []

        reordered_visiting.append(min(route_poses,key=lambda rt_pose: math.dist([-2.0, -0.5],[rt_pose.pose.position.x,rt_pose.pose.position.y])))
        route_poses.remove(reordered_visiting[-1])


        while route_poses != []:
            last_entry = reordered_visiting[-1]
            reordered_visiting.append(min(route_poses,key=lambda rt_pose: math.dist([last_entry.pose.position.x, last_entry.pose.position.y],[rt_pose.pose.position.x,rt_pose.pose.position.y])))
            route_poses.remove(reordered_visiting[-1]) #could be a one-liner if you use pop and index..

        self.obstacle_visiting_loop = cycle_toandfro(reordered_visiting)
            



        #find the average y and the lowest x, send the robot to a bit below that each time and after you reach it you identify it?? but this defeats our adaptation :(
        self.picture_taken = False
        while self.times_detected < self.detection_threshold:

            self.goToPose(next(self.obstacle_visiting_loop, None))
            #time.sleep(self.picture_rate)        
            self.picture_rate = self.get_parameter(PICTURE_RT_PARAM).get_parameter_value().integer_value
            self.get_logger().info("Picture Rate: " + str(self.picture_rate))
            
            for i in range(self.picture_rate):
                #take a picture

                #take new
                self.prev_done = False
                self.send_goal()
                
                self.ID_fdback_msg.obj_idd.stamp = self.get_clock().now().to_msg()
                goal_handle.publish_feedback(self.ID_fdback_msg)
                




            
        goal_handle.succeed()
        res = Identify.Result()
        res.time_elapsed = time.time() - self.initial_time
        res.picture_rate = self.picture_rate
        res.detection_threshold = self.detection_threshold
        return res

    def visit_obstacle(self, obstacle):
        pass

    def send_goal(self):
        #ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 1.9, y: 0.7, z: 0.0}, orientation: {w: 1.0}}}"

        self.get_logger().info("Sending the goal")

        """Send a `CheckForObjects` action request."""
        self.get_logger().debug("Waiting for 'CheckForObjects' action server")
        while not self.check_obj_acclient.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'CheckForObjects' action server was not available, will try again")
            

        goal_msg = CheckForObjects.Goal()
        goal_msg.id = 1
        while self.tb_image is None:
            self.get_logger().info("No image received yet to send, waiting...")
        #self.get_logger().info("Uhh:" + str(type(self.tb_image)))

        goal_msg.image = self.tb_image

        send_goal_future = self.check_obj_acclient.send_goal_async(goal_msg, self._feedbackCallback)
        
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future,timeout_sec=20)
        self.ID_fdback_msg.obj_idd.object_names = []
        self.ID_fdback_msg.obj_idd.probabilities = []
        self.ID_fdback_msg.obj_idd.object_detected = False
        result = get_result_future.result().result
        if(len(result.bounding_boxes.bounding_boxes) > 0):
            self.ID_fdback_msg.obj_idd.object_detected = True
            if(self.goal_object in [box.class_id for box in result.bounding_boxes.bounding_boxes]):
                self.times_detected += 1
            for box_index, box in enumerate(result.bounding_boxes.bounding_boxes):
                self.ID_fdback_msg.obj_idd.object_names.append(box.class_id)
                self.ID_fdback_msg.obj_idd.probabilities.append(box.probability)
                self.get_logger().info('Box: {2} Result: {0} Probability: {1}'.format(box.class_id, str(box.probability), box_index))
                self.get_logger().info('Times Detected: {0}'.format(self.times_detected))
        else:
                self.get_logger().info('Nothing detected')
                self.get_logger().info('Times {1} Detected: {0}'.format(self.times_detected, self.goal_object))


        self.prev_done = True


        return True


    def _feedbackCallback(self, msg):
        self.get_logger().debug('Received action feedback message')
        self.feedback = msg.feedback
        return
    
def main(args=None):
    rclpy.init(args=args)

    id_action_server = IdentifyActionServer()
    
    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(id_action_server)
    mt_executor.spin()

    id_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()