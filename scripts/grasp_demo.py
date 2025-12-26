#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))
    
def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class KukaKR3Demo(object):
    def __init__(self):
        super(KukaKR3Demo, self).__init__()

        # Инициализация MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('kuka_kr3_demo', anonymous=True)

        # Создание объектов для управления роботом и gripper'ом
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "kuka_arm"  # Название группы MoveIt для манипулятора
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # Для визуализации в RViz
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20)

        # Информация для отладки
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

    def go_to_joint_state(self):
        # Перемещение в заданное состояние суставов
        print("Moving to joint state...")
        
        joint_goal = self.move_group.get_current_joint_values()
        joint_states=[0.11735195700515266, 0.10799937003659998, 0.42989330267591797, 0.010644582782874501, 1.3320427916985453, 0.12155879292882688]
        joint_goal[0] = joint_states[0] #0  # [rad] для joint1-0.1706654003334318, -0.12007862416483839, 0.9573334456915754, 0.014354816797277792, 0.7437530350873063, 0.0063603156053195775
        joint_goal[1] = joint_states[1] #-pi/4  # [rad] для joint2
        joint_goal[2] = joint_states[2] #0  # [rad] для joint3
        joint_goal[3] = joint_states[3] #-pi/2  # [rad] для joint4
        joint_goal[4] = joint_states[4]  # [rad] для joint5
        joint_goal[5] = joint_states[5] #pi/3  # [rad] для joint6

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()  # Остановка всех оставшихся движений

    def go_to_pose_goal(self, pose_goal):
        
        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        self.move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
    
    def go_to_home_pose(self):
        print("Moving to 'home' pose...")
        self.move_group.set_named_target("home")
        self.move_group.go(wait=True)
        self.move_group.stop()
        print("Reached 'home' pose")
            
    def open_gripper(self):
        # Открытие gripper'а
        print("Opening gripper...")
        
        # Здесь должен быть код для управления вашим конкретным gripper'ом
        # Например, если используется стандартный ROS-интерфейс:
        try:
            gripper_group = moveit_commander.MoveGroupCommander("robotiq_gripper")
            gripper_joint_goal = gripper_group.get_current_joint_values()
            gripper_joint_goal[0] = 0.0  # Открытое положение
            gripper_group.go(gripper_joint_goal, wait=True)
            gripper_group.stop()
        except:
            print("No gripper group found or error in gripper control")

    def close_gripper(self):
        # Закрытие gripper'а
        print("Closing gripper...")
        
        # Аналогично open_gripper, но с другими значениями суставов
        try:
            gripper_group = moveit_commander.MoveGroupCommander("robotiq_gripper")
            gripper_joint_goal = gripper_group.get_current_joint_values()
            gripper_joint_goal[0] = 0.3  # Закрытое положение (значение может отличаться)
            gripper_group.go(gripper_joint_goal, wait=True)
            gripper_group.stop()
        except:
            print("No gripper group found or error in gripper control")

def main():
    try:
        print("Starting KUKA KR3 demo in Gazebo")
        kuka_demo = KukaKR3Demo()
        
        # Пример последовательности действий
        kuka_demo.go_to_home_pose()
        kuka_demo.open_gripper()
        kuka_demo.go_to_joint_state()
        # rospy.sleep(1)
        
        # Захват объекта
        kuka_demo.close_gripper()
        
        cur_pose=kuka_demo.move_group.get_current_pose().pose
        cur_pose.position.z+=0.1

        kuka_demo.go_to_pose_goal(cur_pose)
               
        # Возврат в исходное положение
        kuka_demo.go_to_home_pose()

        cur_pose=kuka_demo.move_group.get_current_pose().pose
        cur_pose.position.y-=0.2
        kuka_demo.go_to_pose_goal(cur_pose)

        cur_pose=kuka_demo.move_group.get_current_pose().pose
        cur_pose.position.z-=0.04
        kuka_demo.go_to_pose_goal(cur_pose)
        
        kuka_demo.open_gripper()
        
        cur_pose=kuka_demo.move_group.get_current_pose().pose
        cur_pose.position.z+=0.4
        kuka_demo.go_to_pose_goal(cur_pose)

        kuka_demo.go_to_home_pose()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()