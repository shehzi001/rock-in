#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <Eigen/Geometry>
#include <Eigen/LU>

#include <kdl/frames_io.hpp>

using namespace std;
using namespace Eigen;

typedef Matrix<double, 6, 5> Matrix6x5;
typedef Matrix<double, 6, 6> Matrix6x6;

//double min_jl[5] = {0.0,0.0,-5.18363,0.0,0.0};
//double max_jl[5] = {5.89921,2.70526,0.0,3.57792,5.84685};

const double min_jl[5] = { 0.0100692, 0.0100692, -5.02655, 0.0221239, 0.110619 };
const double max_jl[5] = { 5.84014, 2.61799, -0.015708, 3.4292, 5.64159};
double pre_grasp[5] = {2.93836, 2.020597, -1.88253, 3.36243, 3.01283};
double candle[5] = {2.9496, 1.13446, -2.54818, 1.78896, 2.93075};

//double singular_config[5] = {1.28494,0.055097,-0.21541,1.15916,0.780551};

std::vector<double> joint_values;
std::vector<double> pre_grasping ;
Eigen::MatrixXd jacobian;
Eigen::MatrixXd jacPseudoInv;

void computePivot();
void computerQR();

/* \brief Pointer to group for planning */
robot_state::JointModelGroup* joint_model_group;
/* \brief Pointer to current state of robot */
robot_state::RobotStatePtr kinematic_state;

robot_model::RobotModelPtr kinematic_model;

bool checkJointLimits(Eigen::VectorXd joint_values)
{
	for(int i = 0;i < 5 ; i++)
	{
		if(!(joint_values[i] > min_jl[i] && joint_values[i] < max_jl[i]))
		{
			if(joint_values[i] < min_jl[i])
				joint_values[i] = min_jl[i];
			else if(joint_values[i] > max_jl[i])
				joint_values[i] = max_jl[i];
			return false;
		}
	}
	return true;
}

void moveit_ik(const Eigen::Affine3d end_effector_state)
{
	bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);
  
  if(found_ik)
	{
		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		
		for(std::size_t i=0; i < 5; ++i)
		{
			std::cout << "[" << i << "," << joint_values[i] << "],";
		}
	}
	else
	{
		ROS_INFO("Did not find IK solution.\n");
	}
}

double randomInRange( double _min, double _max ) 
{
	return _min + ((_max - _min) * ((double)rand() / ((double)RAND_MAX + 1)));
}

void computePivot()
{
	//Eigen::FullPivLU<Eigen::Matrix<double, 6, 5>> lu(jacobian);
	
	cout << "\n======================Pivot======================"<<endl;
	Matrix6x5 m = jacobian.block(0,0,6,5);
	
	cout << "Here is the matrix m:" << endl << m << endl;
	
	Eigen::FullPivLU<Matrix6x5> lu(m);
	
	//cout << "Here is, up to permutations, its LU decomposition matrix:"
	//<< endl << lu.matrixLU() << endl;
	
	cout << "Here is the L part:" << endl;
	
	Matrix6x6 l = Matrix6x6::Identity();
	
	l.block<6,5>(0,0).triangularView<StrictlyLower>() = lu.matrixLU();
	
	cout << l << endl;
	
	cout << "Here is the U part:" << endl;
	
	Matrix<double, 6, 5> u = lu.matrixLU().triangularView<Upper>();
	
	cout << u << endl;
	
	//cout << "Let us now reconstruct the original matrix m:" << endl;
	//cout << lu.permutationP().inverse() * l * u * lu.permutationQ().inverse() << endl;
	
	std::cout << "jacPseudoInv:\n " << jacPseudoInv << std::endl;
	
	std::cout << "rank:\n " << lu.rank() << std::endl;
	
	cout << "=========================Pivot end=========================="<<endl;
}

void computerQR()
{
	cout << "\n======================QR======================"<<endl;
	
	MatrixXd thinQ(MatrixXd::Identity(6,5)), Q , R;
	
	//A = jacobian.block(0,0,6,5);
	
	HouseholderQR<MatrixXd> qr(jacobian);
	
	Q = qr.householderQ();
	
	thinQ = qr.householderQ() * thinQ;
	
	R = qr.matrixQR().triangularView<Upper>();
	
	std::cout << "The complete unitary matrix Q is:\n" << Q << "\n\n";
	
	std::cout << "The R is:\n" << R << "\n\n";
	
	cout << "=========================QR end=========================="<<endl;
}

void get_joint_values()
{
	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	std::cout << "current joints:";
	for(int i = 0;i < 5;i++)
	   std::cout << joint_values[i] << " ";
	   std::cout << std::endl;
}
Eigen::VectorXd moveit_fk(Eigen::VectorXd jointState)
{
	kinematic_state->setJointGroupPositions(joint_model_group, jointState);
	
	const Eigen::Affine3d &end_effector_state = kinematic_state->
	getGlobalLinkTransform(kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()));
	
	//moveit_ik(end_effector_state);
	
	//Eigen::Quaterniond quaternion (end_effector_state.rotation());
	Eigen::VectorXd eulerAngles = end_effector_state.rotation().eulerAngles(0,1,2);
	
	Eigen::VectorXd eefPose(6);
	
	//eefPose << end_effector_state.translation(), quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z();
	
	eefPose << end_effector_state.translation(), eulerAngles;
	
	//ROS_INFO_STREAM("end_eff:\n " << eefPose);
	
	return eefPose;
}

void moveit_move_arm(std::vector<double> joint_vals)
{
	move_group_interface::MoveGroup group("arm_1");
	
	group.setJointValueTarget(joint_vals);
	
	moveit::planning_interface::MoveGroup::Plan my_plan;
	
	bool success = group.plan(my_plan);
	
	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	
	group.move();
	
	sleep(5);
}

/*moveit_jacobian_inv:Compute jacobian's Pseudo inverse */

Eigen::MatrixXd moveit_jacobian_inv(Eigen::VectorXd jointState)
{
	kinematic_state->setJointGroupPositions(joint_model_group, jointState);
	
	Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
	
	kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
																 reference_point_position,jacobian);
	
	Eigen::MatrixXd matrix_temp = jacobian * jacobian.transpose();
	
	if(matrix_temp.determinant() != 0)
	{
		jacPseudoInv = jacobian.transpose() * ((matrix_temp).inverse());
		//std::cout << "\n===============================" << endl;
		//std::cout << "jacobian:\n " << jacobian << std::endl;
		//std::cout << "jacPseudoInv:\n " << jacPseudoInv << std::endl;
		//std::cout << "\n===============================" << endl;
		//computePivot();
		//computerQR();
  }
	return jacPseudoInv;
}

void moveitIKPseudoInverseSolver(void)
{
	Eigen::VectorXd current_joint_state(5);
	
	Eigen::VectorXd goal_joint_state(5);
	
	std::vector<double> arm_joint_angles(5);
	
	Eigen::VectorXd new_joint_angles(5);
	
	Eigen::VectorXd error(6);
	
	for(int i = 0;i < 5;i++)
	{
	 current_joint_state[i] = pre_grasp[i];
	 goal_joint_state[i] = candle[i];
	 arm_joint_angles[i] = pre_grasp[i];
	}
	//moveit_fk(jointState);
	
	
	/* moving arm to initial position */
	moveit_move_arm(arm_joint_angles);
	
	//get_joint_values();
	
	Eigen::VectorXd initial_pose = moveit_fk(current_joint_state);   //initial Pose;
	
	Eigen::VectorXd goal_pose = moveit_fk(goal_joint_state); //final Pose;
	
	//change final pose 
	 //goal_pose[0] = -0.1; 
	//goal_pose[1] = 0.02;
	goal_pose[2] -= 0.05;
	
	
	std::cout<<"Generating plan to goal pose:\n" << goal_pose.transpose() << std::endl;

	Eigen::VectorXd deltaX = goal_pose - initial_pose;
	
	unsigned int iter = 0;
	
	while(iter < 100)
	{
		//goal_pose[2] -= 0.01; 
		
		 new_joint_angles = current_joint_state + moveit_jacobian_inv(current_joint_state) * (deltaX);
		
		//Eigen::VectorXd newPose = moveit_fk(new_joint_angles);
		double error  = ((Eigen::Matrix<double,6, 6>::Identity() - jacobian * jacPseudoInv) * (deltaX)).norm();
		
		//std::cout<<"\n\nStart arm configuration:"<< current_joint_state.transpose() << std::endl;
		
		//std::cout<<"New arm configuration:"<< new_joint_angles.transpose()<< std::endl;
		
		
		
		if(error <= 0.2)
		{
			//std::cout << "\nerror:" <<  error << std::endl;
			
			//moveit_move_arm(arm_joint_angles);
			
			//initial_pose = goal_pose;
			
			current_joint_state = new_joint_angles;
			
			deltaX = goal_pose - moveit_fk(current_joint_state);
			
			//sleep(5);
			
			iter++;
		}
		else
			deltaX = 0.5 * deltaX;
		
		if(error < 0.001)
		{
			//if(checkJointLimits(new_joint_angles))
			{
				for(int i=0;i < 5 ;i++)
				{
				 arm_joint_angles[i] = new_joint_angles[i]; 
				}
				ROS_INFO_STREAM("Solution found with intermediate Points : " <<  iter );
				ROS_INFO_STREAM("Moving arm to goal Pose.");
				moveit_move_arm(arm_joint_angles);
			}
			//else
			//	ROS_INFO_STREAM("No solution found Joint Limit Violation.");
			 break;
		}
		
		if(iter == 99)
		   std::cout << "\nNo solution found with error difference:" <<  error << std::endl;
	}
}

int main(int argc, char *argv[])
{
     ros::init (argc, argv, "kinematic_model_tutorial");
     ros::NodeHandle nh;
     ros::AsyncSpinner spinner(1);
     spinner.start();

     /* Load the robot model */
     robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

     /* Get a shared pointer to the model */
     kinematic_model = robot_model_loader.getModel();

     /* Get and print the name of the coordinate frame in 
      * which the transforms for this model are computed*/
     ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

     /* Create a kinematic state - this represents the configuration for 
      * the robot represented by kinematic_model */
     kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));

     /* Set all joints in this state to their default values */
     kinematic_state->setToDefaultValues();

     joint_model_group = kinematic_model->getJointModelGroup("arm_1");    

     moveitIKPseudoInverseSolver();

     ROS_INFO_STREAM("Test complete.");
     
     return -1;
}
