Hi

* Erenus done - with rockin and origin
* Bipin done - with rockin and origin
* 
command to move the arm from terminal

    rostopic pub -1 arm_1/arm_controller/position_command brics_actuator/JointPositions '{positions: [ {joint_uri: arm_joint_1, unit: rad, value: 2.93812}, {joint_uri: arm_joint_2, unit: rad, value: 1.04351}, {joint_uri: arm_joint_3, unit: rad, value: -2.44154}, {joint_uri: arm_joint_4, unit: rad, value: 1.83102}, {joint_uri: arm_joint_5, unit: rad, value: 2.96447} ]}'
