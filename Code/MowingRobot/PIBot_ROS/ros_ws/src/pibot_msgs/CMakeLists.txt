cmake_minimum_required(VERSION 2.8.3)
project(pibot_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(
    DIRECTORY msg
    FILES
        Action.msg
        RawImu.msg
        PointArray.msg
        Pose.msg
        RobotState.msg
        NaviStatus.msg 
        RelocateResult.msg
)

add_service_files(
    DIRECTORY srv
    FILES
        ManualLocation.srv
        MappingParams.srv
        ActionParams.srv
        SetMapParams.srv
        NaviParams.srv
        Relocation.srv
        SetPoseParams.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
