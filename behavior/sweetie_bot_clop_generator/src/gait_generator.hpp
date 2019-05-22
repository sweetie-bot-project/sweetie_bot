#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>

#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>

#include <sweetie_bot_clop_generator/MoveBaseAction.h>
#include <sweetie_bot_control_msgs/FollowStepSequenceAction.h>

namespace sweetie_bot {

class ClopGenerator 
{
	protected:
		typedef sweetie_bot_clop_generator::EndEffectorGoal EndEffectorGoal;

		typedef sweetie_bot_clop_generator::MoveBaseAction MoveBaseAction;
		typedef sweetie_bot_clop_generator::MoveBaseGoal MoveBaseGoal;
		typedef sweetie_bot_clop_generator::MoveBaseResult MoveBaseResult;
		typedef sweetie_bot_clop_generator::MoveBaseGoalConstPtr MoveBaseGoalConstPtr;
		
		// typedef actionlib::ServerGoalHandle< MoveBaseAction > MoveBaseGoalHandle;

		typedef sweetie_bot_control_msgs::FollowStepSequenceAction FollowStepSequenceAction;
		typedef sweetie_bot_control_msgs::FollowStepSequenceGoal FollowStepSequenceGoal;
		typedef sweetie_bot_control_msgs::FollowStepSequenceResult FollowStepSequenceResult;

	
	protected:
		struct EndEffectorInfo {
			int towr_index; /**< Index of chain in towr library. */
			std::string frame_id; /**< End effector frame. */
		};

	protected:
		// NODE INTERFACE
		// NodeHandler
		ros::NodeHandle node_handler;
		// publisers
		// subscribers
		// tf
		tf2_ros::Buffer tf_buffer;
		std::unique_ptr<tf2_ros::TransformListener> tf_listener;
		// action client
		std::unique_ptr< actionlib::SimpleActionClient<FollowStepSequenceAction> > execute_step_sequence_ac;
		// action server
		std::unique_ptr< actionlib::SimpleActionServer<MoveBaseAction> > move_base_as;

		// PARAMETERS
		double period;
		double contact_height_tolerance; /**< Is used during contact detection and initial pose check [m] */
		std::string towr_parameters_ns;

		// BUFFERS

		// COMPONENT STATE
		std::string base_frame_id;
		std::map<std::string, EndEffectorInfo> end_effector_index;
		towr::NlpFormulation formulation;
		ifopt::IpoptSolver::Ptr solver;
		ifopt::Problem nlp;
		towr::SplineHolder solution;

	protected:
		void storeSolutionInStepSequenceGoalMsg(FollowStepSequenceGoal& msg);

	public:
		ClopGenerator(const std::string& name);

		bool configure();
		bool configureSolver();
		bool configureRobotModel();

		KDL::Frame convertTFToPathTF(const KDL::Frame& T);
		bool checkEERangeConditions(const towr::BaseState& base_pose, const towr::NlpFormulation::EEPos& ee_pose);

		void setInitialStateFromNominal(double ground_z);
		bool setInitialStateFromTF();

		void setGoalPoseFromMsg(const MoveBaseGoal& msg);
		void setGaitFromGoalMsg(const MoveBaseGoal& msg);
		void setFreeMovementsPhasesFromGoalMsg(towr::Parameters& param, const MoveBaseGoal& msg);

		void abortGoal(const std::string& where, int error_code, const std::string& error_string);
		void succeedGoal(int error_code, const std::string& error_string);

		void callbackExecuteMoveBase(const MoveBaseGoalConstPtr& msg);
};

} // namespace sweetie_bot

