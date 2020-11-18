#!/usr/bin/env python  # 이게 있어야 python 파일임

import rospy # ros에서 python coding하기위해 반드시 불러와야됨

# Brings in the SimpleActionClient
import actionlib # Client가 보낸 요청된 목표를 실행해주는 도구 제공하는 라이브러리 (조금 더 알아보기...)
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # move_base_msgs.msg에 있는 메시지 타입 중 MoveBaseAction, MoveBaseGoal를 불러옴.
# MoveBaseAction은 .action을 가져오는 것이다. action은 서버와 클라이언트간에 통신에 필요한 것들
# 여기 3줄은 compact message definition
# move_base_msgs/MoveBaseActionGoal action_goal
# move_base_msgs/MoveBaseActionResult action_result
# move_base_msgs/MoveBaseActionFeedback action_feedback
# 밑엔 raw
# MoveBaseActionGoal action_goal
# MoveBaseActionResult action_result
# MoveBaseActionFeedback action_feedback
# http://docs.ros.org/en/diamondback/api/move_base_msgs/html/msg/MoveBaseAction.html
# Goal, Result, Feedback 모두 정의가 되어있는 것을 확인할 수 있다.

# MoveBaseGoal은
# geometry_msgs/PoseStamped target_pose

# geometry_msgs/PoseStamped target_pose
#     Header header
#         uint32 seq
#         time stamp
#         string frame_id
#     geometry_msgs/Pose pose
#         geometry_msgs/Point position
#             float64 x
#             float64 y
#             float64 z
#         geometry_msgs/Quaternion orientation
#             float64 x
#             float64 y
#             float64 z
#             float64 w
# 이런식으로 되어있고 어디로 보낼지에 대한 좌표값이 들어갈 곳이다.

def movebase_client(): # movebase_client 함수 정의 부분
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html
    # pointop에서 쓰던 subscriber나 publisher와 비슷하게
    # Action에서는 Client와 Server와 통신으로 구성되는데
    # 보내는쪽이 Client 받는 쪽이 Server 인 것 같다?
    # SimpleActionClient는 그전에 Subcriber로 snowboy를 선언해서 msg를 받았던 것 처럼
    # 이것도 마찬가지로 move_base라는 이름으로 action 파일은 아까 import한 MoveBaseAction로 파라미터를 설정하여
    # client라는 Instance를 만들었다.
    # SimpleActionClient(ns='move_base', ActionSpec=MoveBaseAction)
    #
    #ns	        The namespace in which to access the action. For example, the "goal" topic should occur under ns/goal
    #ActionSpec	The *Action message type. The SimpleActionClient will grab the other message types from this type.

    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()
    # https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html#afd4d2c147e2ddc59af9c6570b72151fc
    # 아래는 함수가 정의된 부분이다.
    # def wait_for_server(self, timeout=rospy.Duration()):
    # return self.action_client.wait_for_server(timeout)
    # 설명 : Blocks until the action server connects to this client.
    # 리턴값은 서버가 설정된시간(wait_for_server의 파라미터, 없으면 무한대 시간)안에 연결되면 True, timeout 일 때 False

    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    # goal이라는 MovebaseGoal 인스턴스를 생성한다.
    # 왜 MoveBaseGoal?
    # 아마 여기에 메세지가 담길만한 정보가 있기때문? 좌표값이 지정될 곳이다.
    # https://answers.ros.org/question/34684/header-frame_id/
    # frame_id와 base_link에 대한 설명

    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # Move 0.5 meters forward along the x axis of the "map" coordinate frame
    # 아래는 설명 (https://edu.gaitech.hk/ria_e100/map-navigation.html)
    # goal.target_pose.header.frame_id = "map" specifies the reference frame for that location.
    # In this example, it is specified as the map frame, which simply means that
    # the coordinates will be considered in the global reference frame related to the map itself.
    # In other words, it is the absolute position on the map.
    # In case where the reference frame is set with respect to the robot, namely goal.target_pose.header.frame_id = "base_link" (like in this tutorial), the coordinate will have a completely other meaning.
    # In fact, in this case the coordinate will represent the (x,y) coordinate with respect to robot base frame attached to the robot,
    # so it is a relative position rather than a absolute position as in the case of using the map frame.

    # 이사이트에 좌표를 설정해놓고 키 입력으로 갈 곳을 정하는 비슷한 코드가 있다
    # https://edu.gaitech.hk/ria_e100/map-navigation.html
    # easy 버전과 매우 비슷함
    # 이걸로 스노우보이와 연동을 하는게 어떤지!?
    # hard version와 easy version에서의 큰 차이는 send_goal에서의 차이가 있다
    # send_goal 파라미터인 client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
    # done_cb, active_cb, feedback_cb를 모두 사용한다.
    # 전부다 콜백함수가 들어갈 자리이다. (active와 feedback은 특별하게 하는 건 없지만 상태를 출력하는 것이고
    # done_cb는 대충보면 다음에 이동할 좌표를 설정해놓았는 것을 보면 연달아서 함수를 실행할 때 쓰는 듯 하다

    # position은 갈 목표의 x y 좌표가 들어갈 곳이다.
    goal.target_pose.pose.position.x = 0.5
    # y 좌표는 goal.target_pose.pose.position.y = 0.5 이런식으로 넣으면 된다.

    # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0
    # orientation 은 회전에 관련된 것 같은데
    # x y z w 로 회전을 하는 것 같은데... 잘 모르겠다... 쿼터니언?..

    # Sends the goal to the action server.
    client.send_goal(goal)
    # 앞에서 설정한 goal location을 Move_base 액션 서버에 보내는 것이다.

    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
    # 그리고 이것의 실행을 기다리는 코드이다.
    # 이것의 결과로 도착을 못했는지 했는지 알 수 있게 된다.
    # 파라미터
    # timeout : Max time to block before returning. A zero timeout is interpreted as an infinite timeout.
    # 리턴값 : True if the goal finished. False if the goal didn't finish within the allocated timeout

    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        # try ~ except에 쓰이는 logerr 인 것 같은데.. 왜 그냥 쌩뚱맞게 들어가있는지 모르겠다.
        # 아무튼 안될 때 저런 문구를 출력하는 것.. 의미 없다.
        rospy.signal_shutdown("Action server not available!")
        # 수동으로 노드를 종료시키는 것이다.
        # 종료시킬때 Action server not available! 라는 텍스트를 남기고 꺼진다.
        # Initiate node shutdown. reason is a human-readable string that documents why a node is being shutdown.
        # http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown

    else:
        # Result of executing the action
        return client.get_result()
        # 이 함수의 최종 리턴값이다.. 정상적으로 실행되면 True가 리턴이 되는 듯하다.

    # If the python node is executed as main process (sourced directly)


if __name__ == '__main__':
    try:
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        # 이러한 이름의 노드를 초기화 (필수)
        result = movebase_client()
        # 함수를 실행하는데 리턴값을 result에 넣는것 (if not wait: else: retrun client.get_result())
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
        # 이게 에러가 날 경우인데 왜 정상적인것처럼 써놓았는지 모르겠다.
        # 다른 예제에는
        # except rospy.ROSInterruptException:
        # print("program interrupted before completion", file=sys.stderr)
        # 이런식으로 되어있다. 아무튼 에러가 날경우 저걸 출력하는 코드라는 것을 알 수 있다.