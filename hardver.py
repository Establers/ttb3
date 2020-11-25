#!/usr/bin/env python
# license removed for brevity
__author__ = 'fiorellasibona'
import rospy
import math # 보는방향을 설정 할 때 pi를 사용하기 위해서 쓰는 듯 함
#
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus  # 코드상에서 status 라고 쓰고 있음
# 각 번호마다 어떤 상황인지 나와있는 사이트
# http://docs.ros.org/en/api/actionlib_msgs/html/msg/GoalStatus.html

from geometry_msgs.msg import Pose, Point, Quaternion # pose안에 point, quaternion 이 있는데?..
from tf.transformations import quaternion_from_euler # 보는 방향 계산할 때..?


class MoveBaseSeq(): # 클래스 MoveBaseSeq()
    def __init__(self):
        rospy.init_node('move_base_sequence')
        points_seq = rospy.get_param('move_base_seq/p_seq')
        # points_seq = [x1,y1,z1, x2,y2,z2, ...xn,yn,zn] .launch 파일에서 가지고 오는 것

        # Only yaw angle required (no ratotions around x and y axes) in deg:
        yaweulerangles_seq = rospy.get_param('move_base_seq/yea_seq')
        # yaweulerangles_seq = [x1,y1,z1] .launch 파일에서 가지고 오는 것

        #List of goal quaternions: # quaternions는 보는 방향과 관련된 값으로 추측
        quat_seq = list()
        # 비어 있는 리스트 quat_seq 만드는 것

        #List of goal poses:
        self.pose_seq = list()  # 비어 있는 리스트 pose_seq 만드는 것
        self.goal_cnt = 0       # goal_cnt 라는 변수 선언
        for yawangle in yaweulerangles_seq:
            #Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
            # 앞에서 launch 파일에서 가져온 값을 하나하나 for 문로 계산
            # quat_seq에 append로 리스트 맨 뒤에 추가
        n = 3 # 이 함수는 기본으로 세개의 좌표로 순서대로 가는 것으로 설정되어 있어서 3인듯 함

        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        # 리스트안에 리스트를 넣기위해 이런 연산을 함
        # 결과 값 points : [[x1,y1,z1], [x2,y2,z2], [xn,yn,zn]]

        for point in points:
            #Exploit n variable to cycle in quat_seq
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3])) # *은 리스트를 Unpacking 할 때 쓰임
            n += 1 # 해석하기 어렵다 pose_seq 리스트에 추가하는 것 같다
        #

        #Create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")

        wait = self.client.wait_for_server(rospy.Duration(5.0))
        # 5초안에 연결되면 wait = True
        if not wait: # 5초안에 연결이 안된다면
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            # 종료하고 끝냄
            return

        # 연결이 정상일 때
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()
        # movebase_client 라는 함수를 실행한다.

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")
        # 실행될 때 이제 실행되고 있다고 출력함

    def feedback_cb(self, feedback): # feedback 콜백 함수가 실행이 되는데 feedback 파라미터는 뭔지 잘 모르겠음
        #To print current pose at each feedback:
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    # send_goal에 done_cb 콜백 함수가 실행이 되게 되어있는데 성공적으로 갔으면 status = 3 이고
    # 또 다시 좌표값을 찍고 send_goal을 실행해 계속적으로 돌게 함
    # send_goal 실행 전에 if 로 goal_cnt와 len(pose_seq) 비교로 3번만 돌게
    def done_cb(self, status, result): # status는 goalstatus를 의미함
        self.goal_cnt += 1
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2: # 실행하고 있을 때 취소요청을 받을 때
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3: # goal이 성공적으로 수행 되었을 때
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4: # 액션 서버에 의해 중지 되었을 때 (의도치 않게)
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5: # 액션 서버에 의해 중지 되었을 때 (의도)
            # The goal was rejected by the action server without being processed,
            # because the goal was unattainable or invalid (Terminal State)
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8: # 취소요청을 받았고 성공적으로 취소 되었을 때..?
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        # https://docs.ros.org/en/diamondback/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html#a6e13a58b401b78d4d1423d072494f99d
        # 파라미터 순서(self, goal, done_cb, active_cb, feedback_cb)
        #
        #done_cb 	Callback that gets called on transitions to Done. The callback should take two parameters: the terminal state (as an integer from actionlib_msgs/GoalStatus) and the result.
        # 시행되었을 때 실행되는 콜백 함수 2개의 파라미터 지금은 status랑 묶어서 여러가지 에러사항에 대처하는 코드로 작성해놓은 듯 함
        #active_cb 	실행할때 파라미터 없는 콜백 함수
        #feedback_cb 	Callback that gets called whenever feedback for this goal is received. Takes one parameter: the feedback.
        #feedback이라는 파라미터를 가지고 있고 goal을 받았을 때 실행되는 콜백 함수?

        rospy.spin()
        # node가 종료되지 않게...??? 잘 뭔지 모르곘다. 볼 때 마다 의문

if __name__ == '__main__': # 메인일 때
    try:
        MoveBaseSeq() # 이런 클래스를 실행 함
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
