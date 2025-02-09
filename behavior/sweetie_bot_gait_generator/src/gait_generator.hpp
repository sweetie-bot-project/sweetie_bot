#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>

#include <towr/nlp_formulation_base.h>
#include <ifopt/ipopt_solver.h>

#include <std_srvs/SetBool.h>
#include <sweetie_bot_gait_generator/MoveBaseAction.h>
#include <sweetie_bot_gait_generator/SaveTrajectory.h>
#include <sweetie_bot_gait_generator/DisplayMarkers.h>
#include <sweetie_bot_control_msgs/FollowStepSequenceAction.h>

namespace sweetie_bot {

class ClopGenerator 
{
	protected:
		// MoveBase action server typdefs
		typedef sweetie_bot_gait_generator::EndEffectorGoal EndEffectorGoal;

		typedef sweetie_bot_gait_generator::MoveBaseAction MoveBaseAction;
		typedef sweetie_bot_gait_generator::MoveBaseGoal MoveBaseGoal;
		typedef sweetie_bot_gait_generator::MoveBaseResult MoveBaseResult;
		typedef sweetie_bot_gait_generator::MoveBaseGoalConstPtr MoveBaseGoalConstPtr;

		// FollowStepSequence action server typdefs
		typedef sweetie_bot_control_msgs::FollowStepSequenceAction FollowStepSequenceAction;
		typedef sweetie_bot_control_msgs::FollowStepSequenceGoal FollowStepSequenceGoal;
		typedef sweetie_bot_control_msgs::FollowStepSequenceResult FollowStepSequenceResult;

		// SaveTrajectory service
		typedef sweetie_bot_gait_generator::SaveTrajectory::Response SaveTrajectoryResponse;
		typedef sweetie_bot_gait_generator::SaveTrajectory::Request SaveTrajectoryRequest;

		// Markers
		typedef sweetie_bot_gait_generator::DisplayMarkers::Response DisplayMarkersResponse;
		typedef sweetie_bot_gait_generator::DisplayMarkers::Request DisplayMarkersRequest;
		typedef std_srvs::SetBool::Response SetBoolResponse;
		typedef std_srvs::SetBool::Request SetBoolRequest;
	
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
		ros::Publisher markers_pub;
		// subscribers
		// service servers
		ros::ServiceServer save_trajectory_service;
		ros::ServiceServer display_trajectory_markers_serv;
		ros::ServiceServer display_limits_markers_serv;
		// tf
		tf2_ros::Buffer tf_buffer;
		std::unique_ptr<tf2_ros::TransformListener> tf_listener;
		// action client
		std::unique_ptr< actionlib::SimpleActionClient<FollowStepSequenceAction> > execute_step_sequence_ac;
		// action server
		std::unique_ptr< actionlib::SimpleActionServer<MoveBaseAction> > move_base_as;

		// PARAMETERS
		double period;  /**< ExecuteStepSequenceGoal period parameter. Must be compatible with period of ExecuteStepSequence controller. */
		double contact_height_tolerance; /**< Is used during contact detection and initial pose check [m] */
		std::string towr_parameters_ns; /**< Namespace where gait generator parameters tree is located. */
		std::string world_frame; /**< Unmoving coordinate frame respect to which base movements are is published */
		std::string planning_frame; /**< Planning frame in which trajectory planning is performed. */
		std::string storage_ns; /**< Default namespace for saving StepSequenceGoal messages. */

		// BUFFERS

		// COMPONENT STATE
		// persisent state
		std::string base_frame_id;
		std::map<std::string, EndEffectorInfo> end_effector_index;
		std::vector<KDL::Vector> end_effector_contact_point;
		Eigen::Vector3d com_B; // Robot body CoM position in base link frame.
		// NLP formulation and its solution
		std::unique_ptr<towr::NlpFormulationBase> formulation;
		ifopt::IpoptSolver::Ptr solver;
		ifopt::Problem nlp;
		towr::SplineHolder solution;
		// information about last planning request
		bool last_request_successed;
		MoveBaseGoal last_request_goal;
		KDL::Frame last_request_wTp;

	protected:
		void storeSolutionInStepSequenceGoalMsg(FollowStepSequenceGoal& msg, const KDL::Frame& wTp);

	public:
		ClopGenerator(const std::string& name);

		bool configure();
		bool configureSolver();
		bool configureRobotModel();

		KDL::Vector getContactPointFromRobotModel(const std::string& contact);
		KDL::Frame convertTFToPathTF(const KDL::Frame& T);
		bool checkEERangeConditions(const towr::BaseState& base_pose, const towr::NlpFormulationBase::EEPos& ee_pose);

		void setInitialStateFromNominal(double ground_z);
		void setInitialStateFromTF();

		void setGoalPoseFromMsg(const MoveBaseGoal& msg);
		void setGaitFromGoalMsg(const MoveBaseGoal& msg);
		void setFreeMovementsPhasesFromGoalMsg(towr::Parameters& param, const MoveBaseGoal& msg);

		void abortGoal(const std::string& where, int error_code, const std::string& error_string);
		void succeedGoal(int error_code, const std::string& error_string);

		bool performMotionPlanning();
		void callbackExecuteMoveBase(const MoveBaseGoalConstPtr& msg);
		bool callbackSaveTrajectory(SaveTrajectoryRequest& req, SaveTrajectoryResponse& resp);
		bool callbackDisplayMarker(DisplayMarkersRequest& req, DisplayMarkersResponse& resp);
		bool callbackDisplayEELimitsMarker(SetBoolRequest& req, SetBoolResponse& resp);
};

} // namespace sweetie_bot

