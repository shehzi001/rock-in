#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

//Function prototypes
Eigen::VectorXd getEefPose(Eigen::VectorXd jointState);
Eigen::MatrixXd getJacobian(Eigen::VectorXd jointState);
double randomInRange( double _min, double _max );
void jacobianTest(void);

/* \brief Pointer to group for planning */
robot_state::JointModelGroup* joint_model_group;
/* \brief Pointer to current state of robot */
robot_state::RobotStatePtr kinematic_state;

robot_model::RobotModelPtr kinematic_model;

int main(int argc, char **argv)
{
     ros::init (argc, argv, "Jacobian_Test");
     ros::AsyncSpinner spinner(1);
     spinner.start();

     /* Load the robot model */
     robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

     /* Get a shared pointer to the model */
     kinematic_model = robot_model_loader.getModel();

     /* Get and print the name of the coordinate frame in which the transforms for this model are computed*/
     ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

     /* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */
     kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));

     /* Set all joints in this state to their default values */
     kinematic_state->setToDefaultValues();

     joint_model_group = kinematic_model->getJointModelGroup("arm_1");    

     jacobianTest();

     std::cout<<"Test complete!"<<std::endl;

     return 0;
}

/* \brief Return end effector pose as 6x1 vector (x,y,z,r,p,y) */
Eigen::VectorXd getEefPose(Eigen::VectorXd jointState)
{
	kinematic_state->setJointGroupPositions(joint_model_group, jointState);

	const Eigen::Affine3d &end_effector_state = kinematic_state->
	getGlobalLinkTransform(kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()));
	Eigen::Quaterniond quaternion (end_effector_state.rotation());
	Eigen::VectorXd eulerAngles = end_effector_state.rotation().eulerAngles(0,1,2);
	Eigen::VectorXd eefPose(7);	
	eefPose << end_effector_state.translation(), quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z();
	Eigen::VectorXd returnVec(4);
	returnVec << eefPose.tail(4);
	return returnVec;
}

/* \brief Return Jacobian as 7xn matrix */
Eigen::MatrixXd getJacobian(Eigen::VectorXd jointState)
{
	kinematic_state->setJointGroupPositions(joint_model_group, jointState);
	Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
	Eigen::MatrixXd jacobian;
	kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                     reference_point_position, jacobian, true /*Quaternion */);
  
  ROS_INFO_STREAM("jacobian determinent:\n " << jacobian);
	return jacobian.block(3,0,3,5);
}

double randomInRange( double _min, double _max ) {
	return _min + ((_max - _min) * ((double)rand() / ((double)RAND_MAX + 1)));
}

/* \brief Jacobian test to see if it working at all */
void jacobianTest(void)
{
	Eigen::VectorXd jointStateHome(5);
	for (int i = 0 ; i < 5; i++)
	    jointStateHome(i) = .6;

	kinematic_state->setJointGroupPositions(joint_model_group, jointStateHome);
	Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
	Eigen::MatrixXd jacobian;
	kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
															 reference_point_position,jacobian, true /*Quaternion */);

	std::cout<<"Jacobian test: "<< std::endl;
	std::cout<<jacobian<<std::endl;

	Eigen::VectorXd initialJoints(5);
	Eigen::VectorXd finalJoints(5);
	
	for (unsigned int i = 0; i < 50; ++i)
	{
		initialJoints << randomInRange(-1,1), randomInRange(-1,1), randomInRange(-1,1), randomInRange(-1,1),randomInRange(-1,1);
		
		Eigen::VectorXd initialPose = getEefPose(initialJoints);
    
    finalJoints << randomInRange(-1,1), randomInRange(-1,1), randomInRange(-1,1), randomInRange(-1,1),randomInRange(-1,1);
                                 
  	Eigen::VectorXd finalPose = getEefPose(finalJoints);
  	
		Eigen::MatrixXd jacobian = getJacobian(initialJoints);
		
		Eigen::MatrixXd jacPseudoInv = jacobian.transpose() * ((jacobian * jacobian.transpose()).inverse());
		
		Eigen::VectorXd newJoints = initialJoints + jacPseudoInv * (finalPose-initialPose)*0.02;
		
		Eigen::VectorXd newPose = getEefPose(newJoints);
		
		double dotProduct = (finalPose-initialPose).normalized().dot((newPose-initialPose).normalized());
		
		std::cout<<"Manipulator test dot product: "<<dotProduct<<std::endl;
			//std::cout<<"Initial dist: "<<(finalPose-initialPose).norm()<<" Final dist: "<<(finalPose-newPose).norm()<<std::endl;
	}
}
