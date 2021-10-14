#include "gait_generator.hpp"

#include <algorithm>
#include <xmlrpcpp/XmlRpcException.h>

#include <coin/IpReturnCodes.hpp>

#include <towr/models/endeffector_mappings.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/single_rigid_body_with_point_ee_dynamics.h>
#include <towr/models/general_kinematic_model.h>
#include <towr/initialization/quadruped_gait_generator.h>
#include <towr/terrain/height_map.h>

#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <tf_conversions/tf_kdl.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <towr/nlp_formulation.h>
#include <towr/nlp_formulation_planar.h>
#include <towr/nlp_formulation_zmp_planar.h>

#include "towr_trajectory_visualizer.hpp"
#include "rigid_body_inertia_calculator.hpp"

using XmlRpc::XmlRpcValue;
using XmlRpc::XmlRpcException;
using Eigen::Vector3d;
using namespace towr;

std::ostream& operator<<(std::ostream& out, const std::vector<int>& v) {
	for(auto it = v.begin(); it != v.end(); it++) out << *it << " ";
	return out;
}
std::ostream& operator<<(std::ostream& out, const std::vector<unsigned int>& v) {
	for(auto it = v.begin(); it != v.end(); it++) out << *it << " ";
	return out;
}
std::ostream& operator<<(std::ostream& out, const std::vector<double>& v) {
	for(auto it = v.begin(); it != v.end(); it++) out << *it << " ";
	return out;
}

std::ostream& operator<<(std::ostream& out, const Eigen::AlignedBox3d& box) {
	out << "min: (" << box.min().transpose() << "), max: (" << box.max().transpose() << ")";
	return out;
}

std::ostream& operator<<(std::ostream& out, const towr::Sphere3d& sphere) {
	out << "center: (" << sphere.center().transpose() << "), r: " << sphere.radius();
	return out;
}

namespace sweetie_bot {

inline Eigen::Vector3d KDLToEigen(const KDL::Vector& v) {
	return Eigen::Vector3d(v.x(), v.y(), v.z());
}

inline KDL::Vector EigenToKDL(const Eigen::Vector3d& v) {
	return KDL::Vector(v.x(), v.y(), v.z());
}

static void DebugPrintFormulation(const NlpFormulationBase& formulation)
{
	int n_ee = formulation.model_.kinematic_model_->GetNumberOfEndeffectors();

	ROS_INFO_STREAM("KINEMATIC_MODEL");
	for(int ee = 0; ee < n_ee; ee++) {
		auto nominal_stance_B = formulation.model_.kinematic_model_->GetNominalStanceInBase(ee);
		ROS_INFO_STREAM("EE (" << ee << ") nominal_pose = (" << nominal_stance_B.transpose() << ")");
		ROS_INFO_STREAM("EE (" << ee << ") bounding_box = (" << formulation.model_.kinematic_model_->GetBoundingBox(ee) << ")");
		ROS_INFO_STREAM("EE (" << ee << ") bounding_sphere = (" << formulation.model_.kinematic_model_->GetBoundingSphere(ee) << ")");
	}

	ROS_INFO_STREAM("DYNAMIC_MODEL");
	ROS_INFO_STREAM("mass = " << formulation.model_.dynamic_model_->m() << " g = " << formulation.model_.dynamic_model_->g() << " n_ee = " << formulation.model_.dynamic_model_->GetEECount());

	ROS_INFO_STREAM("INITIAL_STATE");
	ROS_INFO_STREAM("Initial BASE pose: p = (" << formulation.initial_base_.lin.at(kPos).transpose() << "), RPY = (" << formulation.initial_base_.ang.at(kPos).transpose() << ")");
	ROS_INFO_STREAM("Initial BASE pose: p = (" << formulation.initial_base_.lin.at(kVel).transpose() << "), RPY = (" << formulation.initial_base_.ang.at(kVel).transpose() << ")");
	for(int ee = 0; ee < formulation.initial_ee_W_.size(); ee++) {
		ROS_INFO_STREAM("Initial EE (" << ee << ") pose: p = (" << formulation.initial_ee_W_[ee].transpose() << ")");
	}

	ROS_INFO_STREAM("FINAL_STATE");
	ROS_INFO_STREAM("Goal BASE pose: p = (" << formulation.final_base_.lin.at(kPos).transpose() << "), RPY = (" << formulation.final_base_.ang.at(kPos).transpose() << ")");
	ROS_INFO_STREAM("Goal BASE pose: p = (" << formulation.final_base_.lin.at(kVel).transpose() << "), RPY = (" << formulation.final_base_.ang.at(kVel).transpose() << ")");
	ROS_INFO_STREAM("Goal BASE bounds: lin_pos (" << formulation.params_.bounds_final_lin_pos_ << "), lin_vel (" << formulation.params_.bounds_final_lin_vel_ << "),  ang_pos (" << formulation.params_.bounds_final_ang_pos_ << "), ang_vel (" << formulation.params_.bounds_final_ang_pos_ << ")");
	for(int ee = 0; ee < formulation.final_ee_W_.size(); ee++) {
		ROS_INFO_STREAM("Goal EE (" << ee << ") pose: p = (" << formulation.final_ee_W_[ee].transpose() << "), bounds: pos = ("	<< formulation.params_.ee_bounds_final_lin_pos_[ee] << "), vel = (" << formulation.params_.ee_bounds_final_lin_vel_[ee] << ")");
	}

	ROS_INFO_STREAM("GAIT PHASES");
	ROS_INFO_STREAM("BASE phases: (" << Eigen::Map<const Eigen::VectorXd>(formulation.params_.base_phase_durations_.data(), formulation.params_.base_phase_durations_.size()).transpose() << ")");
	n_ee = std::min(formulation.params_.ee_phase_durations_.size(), formulation.params_.ee_in_contact_at_start_.size());
	for(int ee = 0; ee < n_ee; ee++) {
		ROS_INFO_STREAM("EE (" << ee << ") phases: contact at start " << formulation.params_.ee_in_contact_at_start_[ee] << " phases (" 
				<< Eigen::Map<const Eigen::VectorXd>(formulation.params_.ee_phase_durations_[ee].data(), formulation.params_.ee_phase_durations_[ee].size()).transpose() << ")");
	}

	ROS_INFO_STREAM("GAIT PARAMETERS");
	ROS_INFO_STREAM("duration_base_polynomial: " << formulation.params_.duration_base_polynomial_);
	ROS_INFO_STREAM("dt_constraint_base_motion: " << formulation.params_.dt_constraint_base_motion_);
	ROS_INFO_STREAM("dt_constraint_range_of_motion: " << formulation.params_.dt_constraint_range_of_motion_);
	ROS_INFO_STREAM("dt_constraint_dynamic: " << formulation.params_.dt_constraint_dynamic_);
	ROS_INFO_STREAM("bound_phase_duration_min: " << formulation.params_.bound_phase_duration_.first);
	ROS_INFO_STREAM("bound_phase_duration_max: " << formulation.params_.bound_phase_duration_.second);
	ROS_INFO_STREAM("ee_polynomials_per_swing_phase: " << formulation.params_.ee_polynomials_per_swing_phase_);
	ROS_INFO_STREAM("force_limit_in_normal_direction: " << formulation.params_.force_limit_in_normal_direction_);
	ROS_INFO_STREAM("min_swing_height: " << formulation.params_.min_swing_height_);
	ROS_INFO_STREAM("stability_margin: " << formulation.params_.stability_margin_);

	ROS_INFO_STREAM("COSTS");
	for(auto& cost_pair : formulation.params_.costs_) {
		ROS_INFO_STREAM("Cost Term (" << cost_pair.first << ") weight: " << cost_pair.second);
	}
}


ClopGenerator::ClopGenerator(const std::string& name)
{
	// publish topics
	//xpp_trajectory_pub = node_handler.advertise<RobotStateCartesianTrajectory>("xpp_trajectory", 1);
	//xpp_robot_pram_pub = node_handler.advertise<RobotParameters>("xpp_robot_parameters", 1);	
	// subscribe topics
	// advertise servises
	ros::NodeHandle node_handler_private("~");
	save_trajectory_service = node_handler_private.advertiseService("save_trajectory", &ClopGenerator::callbackSaveTrajectory, this);
	// tf listener
	tf_listener.reset( new tf2_ros::TransformListener(tf_buffer) );
	// action server
	// TODO implement goal rejection
	move_base_as.reset( new actionlib::SimpleActionServer<MoveBaseAction>(node_handler, name, boost::bind(&ClopGenerator::callbackExecuteMoveBase, this, _1), false) );
	//move_base_as.reset( new actionlib::ActionServer<MoveBaseAction>(node_handler, name) );
	//move_base_as.registerGoalCallback( boost::bind(&ClopGenerator::MoveBaseGoalCallback, this, _1) );
	// action client
	execute_step_sequence_ac.reset( new actionlib::SimpleActionClient<FollowStepSequenceAction>(node_handler, "step_sequence", false) );

	// perform configuration
	if (!configure()) {
		ROS_ERROR("Initialization failed.");
		ros::shutdown();
	}

	move_base_as->start();
}

bool ClopGenerator::configure()
{
	// get towr parameters location
	if (!ros::param::get("~towr_parameters_namespace", towr_parameters_ns)) {
		towr_parameters_ns = "~";
	}
	else {
		// add trailing '/' if necessary
		if (towr_parameters_ns.back() != '/' && towr_parameters_ns.back() != '~') towr_parameters_ns.append(1, '/');
	}

	// configure persistent variables
	if (!ros::param::get(towr_parameters_ns + "period", period)) {
		period = 0.056;
	}
	if (!ros::param::get(towr_parameters_ns + "contact_height_tolerance", contact_height_tolerance)) {
		contact_height_tolerance = 0.005;
	}
	if (!ros::param::get(towr_parameters_ns + "world_frame", world_frame)) {
		world_frame = "odom_combined";
	}
	if (!ros::param::get(towr_parameters_ns + "planning_frame", planning_frame)) {
		planning_frame = "base_link_path";
	}
	if (!ros::param::get("~storage_namespace", storage_ns)) {
		storage_ns = "";
	}

	// init this->solver
	if (!configureSolver()) {
		return false;
	}
	// init this->formulation.model_
	if (!configureRobotModel()) {
		return false;
	}
	// perform formulation initializaion
	// initial_ee_W_ and initial_base_
	setInitialStateFromNominal(0.0); 
	// terrain_
	formulation->terrain_ = towr::HeightMap::MakeTerrain(towr::HeightMap::FlatID);

	// reset trajectory cache
	last_request_successed = false;

	ROS_INFO("ClopGenerator configured!");
	return true;
};


bool ClopGenerator::configureSolver()
{
	solver.reset( new ifopt::IpoptSolver() );

	try {
		//parse solver parameters 
		XmlRpcValue solver_params;
		ros::param::get(towr_parameters_ns + "ipopt_solver_parameters", solver_params);
		if (solver_params.getType() != XmlRpcValue::TypeStruct) throw std::string("'ipopt_solver_parameters' parameter must be structure");

		// now process individual parameters
		for(auto param_it = solver_params.begin(); param_it != solver_params.end(); param_it++) {
			switch (param_it->second.getType()) {
				case XmlRpcValue::TypeInt:
					solver->SetOption(param_it->first, static_cast<int>(param_it->second));
					ROS_INFO_STREAM("Solver parameter '" <<  param_it->first << "': " << static_cast<int>(param_it->second));
					break;
				case XmlRpcValue::TypeDouble:
					solver->SetOption(param_it->first, static_cast<double>(param_it->second));
					ROS_INFO_STREAM("Solver parameter '" <<  param_it->first << "': " << static_cast<double>(param_it->second));
					break;
				case XmlRpcValue::TypeString:
					solver->SetOption(param_it->first, static_cast<std::string>(param_it->second));
					ROS_INFO_STREAM("Solver parameter '" <<  param_it->first << "': " << static_cast<std::string>(param_it->second));
					break;
				default:
					throw std::string("solver parameter must be int, double or string: " + param_it->first);
			}
		}
	}
	catch (std::string& e) {
		ROS_ERROR_STREAM("'ipopt_solver_parameteres' parameter parse error: " << e);
		return false;
	}
	catch (XmlRpc::XmlRpcException& e) {
		ROS_ERROR_STREAM("'ipopt_solver_parameteres' parameter parse error: " << e.getMessage());
		return false;
	};
	return true;
}

bool ClopGenerator::configureRobotModel() 
{
	const std::map<const std::string, const int> end_effector_mapper = { {"LF", towr::LF}, {"RF", towr::RF}, {"LH", towr::LH}, {"RH", towr::RH} };

	// clear robot model
	this->base_frame_id = "";
	this->end_effector_index.clear();
	this->end_effector_contact_point.assign(end_effector_mapper.size(), KDL::Vector::Zero());
	this->formulation.reset();

	std::shared_ptr<towr::GeneralKinematicModel> kinematic_model = std::make_shared<towr::GeneralKinematicModel>(4);
	std::shared_ptr<towr::DynamicModel> dynamic_model;

	// get urdf model
	std::string urdf_model;
	if (!ros::param::get("robot_description", urdf_model)) {
		ROS_ERROR("Parameter 'robot_description' is empty.");
		return false;
	}

	// init dynamic quantities calculator
	RigidBodyInertiaCalculator kdl_inertia_calculator(urdf_model);
	KDL::RigidBodyInertia base_inetria_tensor;
	std::vector<double> ee_masses(end_effector_mapper.size(), 0.0);

	// parse towr model
	try {
		XmlRpcValue towr_model;
		ros::param::get(towr_parameters_ns + "towr_model", towr_model);
		if (towr_model.getType() != XmlRpcValue::TypeStruct) throw std::string("'towr_model' parameter must be structure");
		// get formulation type
		{
			XmlRpcValue& nlp_type_param = towr_model["nlp_type"];
			if (nlp_type_param.getType() != XmlRpcValue::TypeString) throw std::string("towr_model must contain 'nlp_type' string"); 
			if (static_cast<std::string>(nlp_type_param) == "6d") {
				this->formulation.reset(new towr::NlpFormulation());
				this->formulation->params_.constraints_ = { Parameters::Dynamic, Parameters::Force, Parameters::EndeffectorRom, Parameters::Terrain, Parameters::Swing, Parameters::BaseAcc };
			}
			else if (static_cast<std::string>(nlp_type_param) == "planar") {
				this->formulation.reset(new towr::NlpFormulationPlanar());
				this->formulation->params_.constraints_ = { Parameters::Dynamic, Parameters::Force, Parameters::EndeffectorRom, Parameters::Swing, Parameters::BaseAcc };
			}
			else if (static_cast<std::string>(nlp_type_param) == "zmp") {
				this->formulation.reset(new towr::NlpFormulationZMPPlanar());
				this->formulation->params_.constraints_ = { Parameters::Dynamic, Parameters::EndeffectorRom, Parameters::Swing, Parameters::BaseAcc };
			}
			else if (static_cast<std::string>(nlp_type_param) == "zmp_phase") {
				this->formulation.reset(new towr::NlpFormulationZMPPlanar());
				this->formulation->params_.constraints_ = { Parameters::DynamicPhase, Parameters::EndeffectorRom, Parameters::Swing, Parameters::BaseAcc };
			}
			else {
				throw std::string("'nlp_type' must be '6d', 'planar', 'zmp', 'zmp_phase'"); 
			}
		}
		// process base
		{
			XmlRpcValue& base_param = towr_model["base"];
			if (base_param.getType() != XmlRpcValue::TypeStruct) throw std::string("towr_model must contain 'base' subtree"); 
			// get frame
			XmlRpcValue& frame_id = base_param["frame_id"];
			if (frame_id.getType() != XmlRpcValue::TypeString) throw std::string("base description must contain 'frame_id' string");
			this->base_frame_id = static_cast<std::string>(frame_id);
			// get links list for dynamic model calculation
			XmlRpcValue& base_links_param = base_param["links"];
			if (base_links_param.valid()) {
				std::vector<std::string> base_links;
				// links array supplied, get it as vector of strings to calculate base inertia and mass
				if (base_links_param.getType() != XmlRpcValue::TypeArray || base_links_param.size() < 1 || base_links_param[0].getType() != XmlRpcValue::TypeString) {
					throw std::string("base description must contain non-empty array of strings 'links' ");
				}
				base_links.reserve(base_links_param.size());
				for (int i = 0; i < base_links_param.size(); i++) base_links.push_back( static_cast<std::string>(base_links_param[i]) );
				// calculate base inertia tensor
				base_inetria_tensor = kdl_inertia_calculator.getInertia(base_frame_id, base_links);
			}
			else {
				// links array is not supplied, assume that base is massive
				base_inetria_tensor = kdl_inertia_calculator.getInertiaTotal();
			}
		}
		 
		// process end effectors
		for(auto ee_it = end_effector_mapper.begin(); ee_it != end_effector_mapper.end(); ee_it++) {
			XmlRpcValue& leg_param = towr_model[ee_it->first];
			if (leg_param.getType() != XmlRpcValue::TypeStruct) throw std::string(ee_it->first + ": 'towr_model' parameter must contain '" + ee_it->first + "' subtree");

			// Setup end effector index
			EndEffectorInfo ee_info;
			ee_info.towr_index = ee_it->second;
			// get name
			XmlRpcValue& name = leg_param["name"];
			if (name.getType() != XmlRpcValue::TypeString) throw std::string(ee_it->first + ": end effector description must contain 'name' string");
			// get frame
			XmlRpcValue& frame_id = leg_param["frame_id"];
			if (frame_id.getType() != XmlRpcValue::TypeString) throw std::string(ee_it->first + " end effector description must contain 'frame_id' string");
			ee_info.frame_id = static_cast<std::string>(frame_id);
			// add end effctor to index
			end_effector_index[name] = ee_info;

			// Configure kinematic model
			// get nominal_stance
			XmlRpcValue& nominal_stance = leg_param["nominal_stance"];
			if (nominal_stance.getType() != XmlRpcValue::TypeArray || nominal_stance.size() != 3 || nominal_stance[0].getType() != XmlRpcValue::TypeDouble) {
				throw std::string(ee_it->first + " end effector description must contain 'nominal_stance' double[3]");
			}
			towr::KinematicModel::Vector3d nominal_stance_p(nominal_stance[0], nominal_stance[1], nominal_stance[2]);
			//get bounding box
			XmlRpcValue& bounding_box = leg_param["bounding_box"];
			if (bounding_box.getType() != XmlRpcValue::TypeArray || bounding_box.size() != 6 || bounding_box[0].getType() != XmlRpcValue::TypeDouble) {
				throw std::string(ee_it->first + " end effector description must contain 'bounding_box' double[6]");
			}
			Eigen::Vector3d bounding_box_p1(bounding_box[0], bounding_box[1], bounding_box[2]);
			Eigen::Vector3d bounding_box_p2(bounding_box[3], bounding_box[4], bounding_box[5]);
			if (Eigen::AlignedBox3d(bounding_box_p1, bounding_box_p2).isEmpty()) {
				throw std::string(ee_it->first + " end effector bounding box is empty");
			}
			// get bounding sphere
			XmlRpcValue& bounding_sphere = leg_param["bounding_sphere"];
			if (bounding_sphere.getType() != XmlRpcValue::TypeArray || bounding_sphere.size() != 4 || bounding_sphere[0].getType() != XmlRpcValue::TypeDouble) {
				throw std::string(ee_it->first + " end effector description must contain 'bounding_sphere' double[4]");
			}
			Eigen::Vector3d center(bounding_sphere[0], bounding_sphere[1], bounding_sphere[2]);
			double radius = bounding_sphere[3];

			// configure end effector: note that TOWR plans trajectory for CoM so kinematics model should be "shifted".
			Eigen::Vector3d cog(base_inetria_tensor.getCOG().data);
			kinematic_model->configureEndEffector(ee_info.towr_index, nominal_stance_p - cog, Eigen::AlignedBox3d(bounding_box_p1 - cog, bounding_box_p2 - cog), towr::Sphere3d(center - cog, radius) );

			// Get information about contact
			XmlRpcValue& contact_point = leg_param["contact_point"];
			if (contact_point.getType() != XmlRpcValue::TypeArray || contact_point.size() != 3 || contact_point[0].getType() != XmlRpcValue::TypeDouble) {
				throw std::string(ee_it->first + " end effector description must contain 'contact_point' double[3]");
			}
			// get contact point coordinates
			end_effector_contact_point.at(ee_info.towr_index) = KDL::Vector(contact_point[0], contact_point[1], contact_point[2]);

			// get links list (if supplied)
			XmlRpcValue& leg_links_param = leg_param["links"];
			if (leg_links_param.valid()) {
				std::vector<std::string> leg_links;
				// links array supplied, get it as vector of strings to calculate base inertia and mass
				if (leg_links_param.getType() != XmlRpcValue::TypeArray || leg_links_param.size() < 1 || leg_links_param[0].getType() != XmlRpcValue::TypeString) {
					throw std::string("end effector 'links' must be non-empty array of strings ");
				}
				leg_links.reserve(leg_links_param.size());
				for (int i = 0; i < leg_links_param.size(); i++) leg_links.push_back( static_cast<std::string>(leg_links_param[i]) );
				// calculate leg inertia tensor
				ee_masses.at(ee_info.towr_index) = kdl_inertia_calculator.getInertia(base_frame_id, leg_links).getMass();
			}
			
			ROS_INFO_STREAM("GeneralKinematicModel: " << ee_it->first << " (" << ee_info.frame_id << ", EE " << ee_info.towr_index << "): nominal_stance (" << nominal_stance_p.transpose() << 
					"), bounding_box (" << Eigen::AlignedBox3d(bounding_box_p1, bounding_box_p2) << "), bounding_sphere (" << towr::Sphere3d(center, radius) << ")" );
		}
		// this is impossible!
		if (!kinematic_model->isConfigured()) throw std::string("not all end effectors present");
	}
	catch (std::string& e) {
		ROS_ERROR_STREAM("'towr_model' parameter parse error: " << e);
		return false;
	}
	catch (std::invalid_argument& e) {
		ROS_ERROR_STREAM("'towr_model' parameter parse error: " << e.what());
		return false;
	}
	catch (XmlRpc::XmlRpcException& e) {
		ROS_ERROR_STREAM("'towr_model' parameter parse error: " << e.getMessage());
		return false;
	};

	try {
		// init dynamic model
		double M = base_inetria_tensor.getMass();
		Eigen::Matrix3d Icog(base_inetria_tensor.RefPoint( base_inetria_tensor.getCOG() ).getRotationalInertia().data);
		Eigen::Vector3d cog(base_inetria_tensor.getCOG().data);

		if (std::any_of(ee_masses.begin(), ee_masses.end(), [](double m) { return m != 0.0; } )) {
			dynamic_model.reset( new towr::SingleRigidBodyWithPointEEDynamics(M, Icog , ee_masses) );
		}
		else {
			dynamic_model.reset( new towr::SingleRigidBodyDynamics(M, Icog , ee_masses.size()) );
		}

		ROS_INFO_STREAM("SingleRigidBodyDynamics: mass = " << M << " CoM = (" << cog.transpose() << "), I = " << std::endl << Icog  << std::endl << "ee_masses = " << ee_masses << std::endl);
	}
	catch (std::invalid_argument& e) {
		ROS_ERROR_STREAM("'towr_model' processing error: " << e.what());
		return false;
	}

	// save models
	this->formulation->model_.kinematic_model_ = kinematic_model;
	this->formulation->model_.dynamic_model_ = dynamic_model;
	this->com_B = Eigen::Vector3d(base_inetria_tensor.getCOG().data);

	return true;
}

void ClopGenerator::setInitialStateFromNominal(double ground_z)
{
	int n_ee = end_effector_index.size();
	// get nominal pose in base
	formulation->initial_ee_W_.resize(n_ee);
	for(int ee = 0; ee < n_ee; ee++) formulation->initial_ee_W_[ee] =  formulation->model_.kinematic_model_->GetNominalStanceInBase(ee) + com_B;
	// get base heigh
	double base_height = - formulation->initial_ee_W_.front().z();
	// set base posiition
	formulation->initial_base_.lin.at(kPos) = Eigen::Vector3d(0.0f, 0.0f, base_height + ground_z) + com_B;
	formulation->initial_base_.ang.at(kPos).setZero();
	// set base velocity
	formulation->initial_base_.lin.at(kVel).setZero();
	formulation->initial_base_.ang.at(kVel).setZero();
	// set end effector height
	std::for_each(formulation->initial_ee_W_.begin(), formulation->initial_ee_W_.end(), [&](Vector3d& p){ p.z() = ground_z; });

	ROS_DEBUG_STREAM("Initial BASE pose from nominal: p = (" << formulation->initial_base_.lin.at(kPos).transpose() << "), RPY = (" << formulation->initial_base_.ang.at(kPos).transpose() << ")");
	for(auto it = formulation->initial_ee_W_.begin(); it != formulation->initial_ee_W_.end(); it++) {
		ROS_DEBUG_STREAM("Initial EE (" << it - formulation->initial_ee_W_.begin() << ") pose from nominal: (" << it->transpose() << ")");
	}
}
	
void ClopGenerator::setInitialStateFromTF()
{
	//try {
		// get base position
		{
			geometry_msgs::TransformStamped T;
			Vector3d tmp;

			T = tf_buffer.lookupTransform(planning_frame, base_frame_id, ros::Time(0));
			// set orientation
			KDL::Rotation rot;
			tf::quaternionMsgToKDL(T.transform.rotation, rot);
			rot.GetRPY(tmp.x(), tmp.y(), tmp.z());
			/*Eigen::Quaternion quat;
			tf::quaternionMsgToEigen(T.transform.orientation, quat);
			tmp = quat.toRotationMatrix().eulerAngles(0,1,2);*/
			/* tf::Quaternion quat;
		    tf::quaternionMsgToTF(msg, quat);
			tf::Matrix3x3(quat).getRPY(tmp.x(), tmp.y(), tmp.z()); */
			formulation->initial_base_.ang.at(towr::kPos) = tmp;
			// set position 
			tf::vectorMsgToEigen(T.transform.translation, tmp);
			Eigen::Map< Eigen::Matrix<double,3,3,Eigen::RowMajor> > rot_eigen(rot.data);
			formulation->initial_base_.lin.at(kPos) = tmp + rot_eigen * com_B;
			// set velocities to zero
			formulation->initial_base_.lin.at(towr::kVel).setZero();
			formulation->initial_base_.ang.at(towr::kVel).setZero();

			ROS_DEBUG_STREAM("Initial BASE pose from TF: p = (" << formulation->initial_base_.lin.at(kPos).transpose() << "), RPY = (" << formulation->initial_base_.ang.at(kPos).transpose() << ")");
		}
		// get end effector position
		// NOTE: ignore orientation
		formulation->initial_ee_W_.resize(end_effector_index.size());
		for(const auto& ee_info : end_effector_index) {
				// get transform
				geometry_msgs::TransformStamped hoof_tf;
				hoof_tf = tf_buffer.lookupTransform(planning_frame, ee_info.second.frame_id, ros::Time(0));
				// get ee position
				KDL::Frame T;
				tf::transformMsgToKDL(hoof_tf.transform, T);
				tf::vectorKDLToEigen(T * end_effector_contact_point[ee_info.second.towr_index], formulation->initial_ee_W_[ee_info.second.towr_index]);

				ROS_DEBUG_STREAM("Initial EE '" << ee_info.first << "' (" << ee_info.second.towr_index << ") pose from TF: (" << formulation->initial_ee_W_[ee_info.second.towr_index].transpose() << ")");
		}
	/*}
	catch (tf2::TransformException &ex) {
		ROS_ERROR("get robot initial state tf::lookupTransform error: %s", ex.what());
		return false;
	}*/
}

bool ClopGenerator::checkEERangeConditions(const towr::BaseState& base_pose, const towr::NlpFormulationBase::EEPos& ee_pose) 
{
	// extract base transformation in form (R,p)
	Vector3d rpy = base_pose.ang.at(towr::kPos);
	KDL::Rotation rot_kdl( KDL::Rotation::RPY(rpy.x(), rpy.y(), rpy.z()) );
	Eigen::Map< Eigen::Matrix<double,3,3,Eigen::RowMajor> > R(rot_kdl.data);
	Vector3d p = base_pose.lin.at(kPos);
	// check bounding box
	for(int i = 0; i < ee_pose.size(); i++) {
		const Vector3d& ee_pos_W = ee_pose[i];
		Vector3d ee_pos_B = R.transpose() * (ee_pos_W - p);

		if ( ! formulation->model_.kinematic_model_->GetBoundingBox(i).contains(ee_pos_B) ) { 
			ROS_ERROR_STREAM("Check pose: EE " << i << " (" << ee_pos_B.transpose() <<  ") outside bounding box (" << formulation->model_.kinematic_model_->GetBoundingBox(i) << ")");
			return false;
		}
		if ( ! formulation->model_.kinematic_model_->GetBoundingSphere(i).contains(ee_pos_B) ) { 
			ROS_ERROR_STREAM("Check pose: EE " << i << " (" << ee_pos_B.transpose() <<  ") outside bounding box (" << formulation->model_.kinematic_model_->GetBoundingSphere(i) << ")");
			return false;
		}
	}

	return true;
}

KDL::Frame ClopGenerator::convertTFToPathTF(const KDL::Frame& T) 
{
	KDL::Frame TP;
	// calculate rotation
	Eigen::Quaterniond q;
	T.M.GetQuaternion(q.x(), q.y(), q.z(), q.w());
	q.normalize();
	TP.M = KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w());
	// calculate position
	TP.p = KDL::Vector(T.p.x(), T.p.y(), formulation->terrain_->GetHeight(T.p.x(), T.p.y()));
	return TP;
}

void ClopGenerator::setGoalPoseFromMsg(const MoveBaseGoal& msg) 
{
	geometry_msgs::PoseStamped goal_pose;
	{
		geometry_msgs::PoseStamped goal_pose_origin;
		goal_pose_origin.pose = msg.base_goal;
		goal_pose_origin.header = msg.header;

		geometry_msgs::TransformStamped T;
		T = tf_buffer.lookupTransform(planning_frame, msg.header.frame_id, ros::Time(0));
		tf2::doTransform(goal_pose_origin, goal_pose, T);
	}

	Eigen::Vector3d tmp;
	KDL::Frame initial_pose, final_pose;
	KDL::Vector com = EigenToKDL( com_B );

	// get initial base pose
	tf::vectorEigenToKDL(formulation->initial_base_.lin.at(kPos), initial_pose.p);
	tmp = formulation->initial_base_.ang.at(kPos);
	initial_pose.M = KDL::Rotation::RPY(tmp.x(), tmp.y(), tmp.z());
	initial_pose.p -= initial_pose.M * com;

	// set final orientation
	tf::quaternionMsgToKDL(goal_pose.pose.orientation, final_pose.M);
	final_pose.M.GetRPY(tmp.x(), tmp.y(), tmp.z());
	formulation->final_base_.ang.at(kPos) = tmp;
	// set final position
	tf::pointMsgToEigen(goal_pose.pose.position, tmp);
	formulation->final_base_.lin.at(kPos) = tmp + KDLToEigen(final_pose.M * com);
	tf::vectorEigenToKDL(tmp, final_pose.p);
	// set speed to zero
	formulation->final_base_.lin.at(kVel).setZero();
	formulation->final_base_.ang.at(kVel).setZero();
	// set final bounds
	formulation->params_.bounds_final_lin_pos_.clear();
	for(unsigned int dim = X; dim <= Z; dim++) {
		if (! (msg.base_goal_bounds & (1u << dim))) formulation->params_.bounds_final_lin_pos_.push_back(dim);
	}
	formulation->params_.bounds_final_ang_pos_.clear();
	for(unsigned int dim = X; dim <= Z; dim++) {
		if (! (msg.base_goal_bounds & (8u << dim))) formulation->params_.bounds_final_ang_pos_.push_back(dim);
	}
	// speed
	formulation->params_.bounds_final_lin_vel_ = {X,Y,Z};
	formulation->params_.bounds_final_ang_vel_ = {X,Y,Z};

	ROS_DEBUG_STREAM("Goal BASE pose from msg: p = (" << formulation->final_base_.lin.at(kPos).transpose() << "), RPY = (" << formulation->final_base_.ang.at(kPos).transpose() << ")");

	// Process end effectors
	int n_ee = end_effector_index.size();
	// init EE final poses with default (nominal) position
	formulation->final_ee_W_.clear();
	for(int ee = 0; ee < n_ee; ee++) {
		KDL::Vector ee_nominal_pos_B;
		tf::vectorEigenToKDL(formulation->model_.kinematic_model_->GetNominalStanceInBase(ee), ee_nominal_pos_B);
		ee_nominal_pos_B += com;
		formulation->final_ee_W_.emplace_back();
		tf::vectorKDLToEigen(final_pose * ee_nominal_pos_B, formulation->final_ee_W_.back());
	}
	// remove all final bounds for end effectors
	formulation->params_.ee_bounds_final_lin_pos_.assign(n_ee, towr::DimSet()); // by default no restriction on EE positions
	formulation->params_.ee_bounds_final_lin_vel_.assign(n_ee, {X,Y,Z}); // effectively set velocity to zero

	// now reassign EE poses according to EndEffectorGoal structure
	for (const EndEffectorGoal& ee_goal : msg.ee_goal) {
		// find EE in index
		auto it = end_effector_index.find(ee_goal.name);
		if (it == end_effector_index.end()) throw std::invalid_argument("Unknown end effector: " + ee_goal.name);
		int towr_index = it->second.towr_index;

		// add EE final bound
		for(unsigned int dim = X; dim <= Z; dim++) {
			if (! (ee_goal.position_bounds & (1u << dim))) formulation->params_.ee_bounds_final_lin_pos_[towr_index].push_back(dim); // zero means that coresponding dimension is fixed
		}

		// set final pose for end effector
		KDL::Vector ee_pos;
		tf::pointMsgToKDL(ee_goal.position, ee_pos);
		switch (ee_goal.frame_type) {
			case EndEffectorGoal::BASE_FINAL:
				tf::vectorKDLToEigen( final_pose * ee_pos, formulation->final_ee_W_[towr_index]);
				break;
			case EndEffectorGoal::PATH_FINAL:
				tf::vectorKDLToEigen( convertTFToPathTF(final_pose)  * ee_pos, formulation->final_ee_W_[towr_index]);
				break;
			case EndEffectorGoal::BASE_INITIAL:
				tf::vectorKDLToEigen( initial_pose * ee_pos, formulation->final_ee_W_[towr_index]);
				break;
			case EndEffectorGoal::PATH_INITIAL:
				tf::vectorKDLToEigen( convertTFToPathTF(initial_pose) * ee_pos, formulation->final_ee_W_[towr_index]);
				break;
			case EndEffectorGoal::NOMINAL_POSE:
				break;
			default:
				throw std::invalid_argument("Unknown end effector frame type: " + std::to_string(ee_goal.frame_type));
		}

		ROS_DEBUG_STREAM("Goal EE '" << it->first << "' (" << towr_index << ") pose final: (" << formulation->final_ee_W_[towr_index].transpose() << ")");
	}

	// check if final pose is correct
	if (!checkEERangeConditions(formulation->final_base_, formulation->final_ee_W_)) {
		throw std::invalid_argument("Infeasible final pose.");
	}
}

void getTowrParametersFromRos(towr::Parameters& params, const std::string& ns, double time_scale = 1.0)
{
	bool do_scale;
	if (ros::param::getCached(ns + "/scale_per_step", do_scale)) {
		if (!do_scale) time_scale = 1.0;
	}
	// get cached parameters: double
	double dvalue;
	if (ros::param::getCached(ns + "/duration_base_polynomial", dvalue)) {
		params.duration_base_polynomial_ = dvalue*time_scale;
	}
	if (ros::param::getCached(ns + "/dt_constraint_base_motion", dvalue)) {
		params.dt_constraint_base_motion_ = dvalue*time_scale;
	}
	if (ros::param::getCached(ns + "/dt_constraint_range_of_motion", dvalue)) {
		params.dt_constraint_range_of_motion_ = dvalue*time_scale;
	}
	if (ros::param::getCached(ns + "/dt_constraint_dynamic", dvalue)) {
		params.dt_constraint_dynamic_ = dvalue*time_scale;
	}
	if (ros::param::getCached(ns + "/bound_phase_duration_min", dvalue)) {
		params.bound_phase_duration_.first = dvalue*time_scale;
	}
	if (ros::param::getCached(ns + "/bound_phase_duration_max", dvalue)) {
		params.bound_phase_duration_.second = dvalue*time_scale;
	}
	if (params.bound_phase_duration_.first < 0.0 || params.bound_phase_duration_.first > params.bound_phase_duration_.second) {
		throw std::invalid_argument("getTowrParametersFromRos: `bound_phase_duration_` interval is incorrect");
	}
	if (ros::param::getCached(ns + "/force_limit_in_normal_direction", dvalue)) {
		params.force_limit_in_normal_direction_ = dvalue;
	}
	if (ros::param::getCached(ns + "/stability_margin", dvalue)) {
		params.stability_margin_ = dvalue;
	}
	if (ros::param::getCached(ns + "/min_swing_height", dvalue)) {
		params.min_swing_height_ = dvalue;
	}
	// get cached parameters: int
	int ivalue;
	if (ros::param::getCached(ns + "/ee_polynomials_per_swing_phase", ivalue)) {
		params.ee_polynomials_per_swing_phase_ = ivalue;
	}
	if (ros::param::getCached(ns + "/force_polynomials_per_stance_phase", ivalue)) {
		params.force_polynomials_per_stance_phase_ = ivalue;
	}
	// get cost settings
	if (ros::param::getCached(ns + "/costs/base_lin_motion", dvalue)) {
		if (dvalue > 0.0) params.costs_.push_back( std::make_pair(towr::Parameters::BaseLinAccCostID, dvalue) );
	}
	if (ros::param::getCached(ns + "/costs/base_ang_motion", dvalue)) {
		if (dvalue > 0.0) params.costs_.push_back( std::make_pair(towr::Parameters::BaseAngAccCostID, dvalue) );
	}
	if (ros::param::getCached(ns + "/costs/ee_motion", dvalue)) {
		if (dvalue > 0.0) params.costs_.push_back( std::make_pair(towr::Parameters::EEAccCostID, dvalue) );
	}
}

static bool compare_tail(const std::string& str, const std::string& tail) 
{
	if (str.size() < tail.size()) return false;
	int pos = str.size() - tail.size();
	return str.compare(pos, std::string::npos, tail) == 0;
}

static void SetGaitGeneratorCombo(towr::GaitGenerator& gait_gen, std::string gait_type, int n_steps)
{
	// create necessary movment combo
	std::vector<towr::GaitGenerator::Gaits> combo;
	bool fliplr_flag = false;

	if (gait_type == "stand") {
		// NOTE: ignore n_steps
		combo.push_back(towr::GaitGenerator::Stand);
	}
	else {
		// check provided message: number of steps must be positive
		if (n_steps <= 0) throw std::invalid_argument("Number of steps must be positive");

		// check if left and right legs should be swapped
		const std::string flip_postfix = "_right";
		if (compare_tail(gait_type, flip_postfix)) {
			gait_type.erase(gait_type.end() - flip_postfix.size(), gait_type.end());
			fliplr_flag = true;
		}

		// generate combo with prescribed number of steps
		combo.push_back(towr::GaitGenerator::Stand);
		if (gait_type == "walk") {
			combo.insert(combo.end(), n_steps, towr::GaitGenerator::Walk1);
		}
		else if (gait_type == "walk_overlap") {
			combo.insert(combo.end(), n_steps-1, towr::GaitGenerator::Walk2);
			combo.push_back(towr::GaitGenerator::Walk2E);
		}
		else if (gait_type == "trot") {
			combo.insert(combo.end(), n_steps, towr::GaitGenerator::Run1);
		}
		else if (gait_type == "trot_fly") {
			combo.insert(combo.end(), n_steps-1, towr::GaitGenerator::Run2);
			combo.push_back(towr::GaitGenerator::Run2E);
		}
		else if (gait_type == "pace") {
			combo.insert(combo.end(), n_steps-1, towr::GaitGenerator::Run3);
			combo.push_back(towr::GaitGenerator::Run3E);
		}
		else if (gait_type == "bound") {
			combo.insert(combo.end(), n_steps-1, towr::GaitGenerator::Hop1);
			combo.push_back(towr::GaitGenerator::Hop1E);
		}
		else if (gait_type == "gallop") {
			combo.insert(combo.end(), n_steps-1, towr::GaitGenerator::Hop3);
			combo.push_back(towr::GaitGenerator::Hop3E);
		}
		else {
			throw std::invalid_argument("Unknown gait type: " + gait_type + (fliplr_flag ? flip_postfix : "") );
		}
		combo.push_back(towr::GaitGenerator::Stand);
	}
	// assign combo to gait generator
	gait_gen.SetGaits(combo);
	if (fliplr_flag) gait_gen.FlipLeftRight();
}


void ClopGenerator::setFreeMovementsPhasesFromGoalMsg(towr::Parameters& params, const MoveBaseGoal& msg)
{
	// free mode: assign phase manually, according to robot state and goal specification
	const double move_phase_duration = 0.3;
	const double stand_phase_duration = 0.1;
	double total_unscaled_duration = 0.0;
	bool all_ee_in_contact;
	int n_ee = end_effector_index.size();
	std::vector<bool> ee_is_in_contact_at_end(n_ee, true);

	// clear phase durations
	params.ee_in_contact_at_start_.clear();
	params.ee_phase_durations_.resize(n_ee);
	for(int ee = 0; ee < n_ee; ee++) params.ee_phase_durations_[ee].clear();

	// guess contact state
	// TODO use contact_state
	all_ee_in_contact = true; // for initial state
	for(int ee = 0; ee < n_ee; ee++) {
		Vector3d ee_pos_W = formulation->initial_ee_W_[ee];
		double height = ee_pos_W.z() - formulation->terrain_->GetHeight(ee_pos_W.x(), ee_pos_W.y());
		if ( height < contact_height_tolerance ) { 
			params.ee_in_contact_at_start_.push_back(true);
		}
		else {
			params.ee_in_contact_at_start_.push_back(false);
			all_ee_in_contact = false;
		}
	}
	// if leg is in air then place it on the ground
	if (!all_ee_in_contact) {
		// add two phases for legs in air and one phase for support legs
		// so robot can place all legs on ground and then adjust it center of mass
		for(int ee = 0; ee < n_ee; ee++) {
			if (!params.ee_in_contact_at_start_[ee]) {
				// place leg on ground
				params.ee_phase_durations_[ee].push_back(move_phase_duration); 
				params.ee_phase_durations_[ee].push_back(stand_phase_duration);
			}
			else params.ee_phase_durations_[ee].push_back(move_phase_duration + stand_phase_duration);
		}
		total_unscaled_duration += move_phase_duration + stand_phase_duration;
	}
	else {
		// starting phase when all legs are on ground
		for(int ee = 0; ee < n_ee; ee++) params.ee_phase_durations_[ee].push_back(stand_phase_duration);
		total_unscaled_duration += stand_phase_duration;
	}
	// Now move each leg in order defined in GoalMsg
	all_ee_in_contact = true; // for final state
	for(const EndEffectorGoal& ee_goal : msg.ee_goal) {
		auto it = end_effector_index.find(ee_goal.name);
		if (it != end_effector_index.end()) {
			// save contact state
			ee_is_in_contact_at_end[it->second.towr_index] = ee_goal.contact;
			// move only legs that are in contact at start and at the end of the motion
			if (ee_goal.contact && params.ee_in_contact_at_start_[it->second.towr_index]) {
				for(int ee = 0; ee < n_ee; ee++) {
					if (ee == it->second.towr_index) {
						params.ee_phase_durations_[ee].push_back(move_phase_duration);
						params.ee_phase_durations_[ee].push_back(stand_phase_duration);
					}
					else params.ee_phase_durations_[ee].back() += move_phase_duration + stand_phase_duration;
				}
				total_unscaled_duration += move_phase_duration + stand_phase_duration;
			}
			else {
				all_ee_in_contact = false;
			}
		}
	}
	// Final stage: rise legs that should be free at the end
	if (!all_ee_in_contact) {
		for(int ee = 0; ee < n_ee; ee++) {
			if (!ee_is_in_contact_at_end[ee]) {
				params.ee_phase_durations_[ee].push_back(move_phase_duration);
			}
			else params.ee_phase_durations_[ee].back() += move_phase_duration;
		}
		total_unscaled_duration += move_phase_duration;
	}
	// Now normalize movement duration
	for (int ee = 0; ee < n_ee; ee++) {
		std::transform(params.ee_phase_durations_[ee].begin(), params.ee_phase_durations_[ee].end(), params.ee_phase_durations_[ee].begin(), 
				[&msg, total_unscaled_duration](double phase_duration) { 
					return phase_duration * msg.duration/total_unscaled_duration; 
				});
	}
}

void ClopGenerator::setGaitFromGoalMsg(const MoveBaseGoal& msg)
{
	// check provided message: duration must be positive
	if (msg.duration <= 0.0) throw std::invalid_argument("MoveBaseGoal: step sequence duration must be positive.");

	// formulation parameters: default vaules
	towr::Parameters params;
	int n_ee = end_effector_index.size();

	// now generate gait 
	if (msg.gait_type == "free") {
		// generate gait phases according to goal message
		setFreeMovementsPhasesFromGoalMsg(params, msg);
		getTowrParametersFromRos(params, towr_parameters_ns + "towr_parameters", msg.duration / 1.0);
	}
	else {
		// create gait genarator and assign combo
		std::shared_ptr<towr::GaitGenerator> gait_gen = GaitGenerator::MakeGaitGenerator(n_ee);
		SetGaitGeneratorCombo(*gait_gen, msg.gait_type, msg.n_steps);

		// load parameters from namespace "~towr_paramters" with step-based timescale
		getTowrParametersFromRos(params, towr_parameters_ns + "towr_parameters", msg.duration / gait_gen->GetUnscaledTotalDuration());

		// now construct add phases
		for (int ee = 0; ee < n_ee; ++ee) {
			params.ee_phase_durations_.push_back(gait_gen->GetPhaseDurations(msg.duration, ee));
			params.ee_in_contact_at_start_.push_back(gait_gen->IsInContactAtStart(ee));
		}
		// add base phases
		params.base_phase_durations_ = gait_gen->GetBasePhaseDurations(msg.duration);
	}
	// debug output
	for (int ee = 0; ee < n_ee; ++ee) {
		ROS_DEBUG_STREAM("EE (" << ee << ") phases: contact at start " << params.ee_in_contact_at_start_[ee] << " phases (" 
				<< Eigen::Map<Eigen::VectorXd>(params.ee_phase_durations_[ee].data(), params.ee_phase_durations_[ee].size()).transpose() << ")");
	}

	// check contact conditions for initial pose and final pose
	for(int ee = 0; ee < n_ee; ee++) {
		// check contact conditions for initial pose
		Vector3d ee_pos_W = formulation->initial_ee_W_[ee];
		double height = ee_pos_W.z() - formulation->terrain_->GetHeight(ee_pos_W.x(), ee_pos_W.y());
		if (params.ee_in_contact_at_start_[ee] && height > contact_height_tolerance || height < -contact_height_tolerance) {
			ROS_ERROR_STREAM("Check intital pose: EE " << ee << " terrain constraint is violated, height = " << height);
			throw std::invalid_argument("Terrain constraints violated for initial pose.");
		}
		// fix height 
		if (params.ee_in_contact_at_start_[ee]) formulation->initial_ee_W_[ee].z() -= height;
		// check contact conditions for final pose
		ee_pos_W = formulation->final_ee_W_[ee];
		// check if is in contact contact at end
		bool contact_at_end = params.ee_in_contact_at_start_[ee];
		if (params.ee_phase_durations_[ee].size() % 2 == 0) contact_at_end = !contact_at_end;
		// check if Z-ccordinate is fixed 
		towr::DimSet& ee_bounds = formulation->params_.ee_bounds_final_lin_pos_[ee];
		auto it = std::find(ee_bounds.begin(), ee_bounds.end(), Z);
		// final height if leg is free and Z coordinate is fixed
		if (!contact_at_end && it != ee_bounds.end()) {
			if (ee_pos_W.z() < formulation->terrain_->GetHeight(ee_pos_W.x(), ee_pos_W.y())) {
				ROS_ERROR_STREAM("Check pose: EE " << ee << " terrain constraint is violated, height < 0");
				throw std::invalid_argument("Terrain constraints violated for final pose.");
			}
		}
	}

	// Additional settings
	// TODO get constraints from paramteres
	params.constraints_ = formulation->params_.constraints_;
	
	// set final EE velocity to zero
	// TODO modify towr to divide actual parameter and final pose specification
	// TODO now simply copy bounds from formulation.params_ to params
	// base final bounds
	params.bounds_final_lin_pos_ = formulation->params_.bounds_final_lin_pos_;
	params.bounds_final_lin_vel_ = formulation->params_.bounds_final_lin_vel_;
	params.bounds_final_ang_pos_ = formulation->params_.bounds_final_ang_pos_;
	params.bounds_final_ang_vel_ = formulation->params_.bounds_final_ang_vel_;
	// end effectors final bounds
	params.ee_bounds_final_lin_pos_ = formulation->params_.ee_bounds_final_lin_pos_;
	params.ee_bounds_final_lin_vel_ = formulation->params_.ee_bounds_final_lin_vel_;

	// increases optimization time, but sometimes helps find a solution for more difficult terrain.
	bool optimize_phase_durations = false;
	ros::param::getCached(towr_parameters_ns + "towr_parameters/optimize_phase_durations", optimize_phase_durations);
	if (optimize_phase_durations) params.OptimizePhaseDurations();

	formulation->params_ = params;
}


void ClopGenerator::abortGoal(const std::string& where, int error_code, const std::string& error_string) 
{
	MoveBaseResult result;
	result.error_code = error_code;
	result.error_string = error_string;
	move_base_as->setAborted(result, error_string);
	ROS_ERROR_STREAM("Abort MoveBase goal: " << where << ": "<< error_string);
}

void ClopGenerator::succeedGoal(int error_code, const std::string& error_string) 
{
	MoveBaseResult result;
	result.error_code = error_code;
	result.error_string = error_string;
	move_base_as->setSucceeded(result, error_string);
	ROS_INFO_STREAM("MoveBase goal achived: "<< error_string);
}

bool ClopGenerator::performMotionPlanning() 
{
	DebugPrintFormulation(*formulation);

	// no we have correct formulataion so solve NL problem
	nlp = ifopt::Problem();
	for (auto c : formulation->GetVariableSets(solution)) nlp.AddVariableSet(c);
	for (auto c : formulation->GetConstraints(solution)) nlp.AddConstraintSet(c);
	for (auto c : formulation->GetCosts(solution)) nlp.AddCostSet(c);

	bool success = solver->Solve(nlp);

	if (!success) {
		// inspect return code more closely
		int status = solver->GetIpoptExitStatus();
		std::cout << "IOPT EXIT STATUS " << status << std::endl;

		// check if feasible solution but not optimal solution is found
		if (nlp.HasCostTerms()) {
			if ( status == Ipopt::Maximum_Iterations_Exceeded || status == Ipopt::Maximum_CpuTime_Exceeded) {
				Eigen::VectorXd constraints_values = nlp.GetConstraintsValues();
				ifopt::Problem::VecBound bounds = nlp.GetBoundsOnConstraints();

				success = true;
				for(int k = 0; k < bounds.size(); k++) {
					if (! bounds[k].contains( constraints_values[k], 0.001 ) ) {
						success = false;
						break;
					}
				}

				if (success) ROS_INFO("nlp: Feasible posible non-optimal solution is found");
			}
		}

		// check for alternative success codes
		if ( status == Ipopt::Solved_To_Acceptable_Level || status == Ipopt::Feasible_Point_Found ) {
			success = true;
			ROS_INFO("nlp: Feasible solution is found");
		}
	}

	return success;
}

void ClopGenerator::callbackExecuteMoveBase(const MoveBaseGoalConstPtr& msg) 
{
	ROS_INFO("New MoveBase goal received");

	// setup planning frame
	KDL::Frame wTp; // transform between planning and world frame
	geometry_msgs::TransformStamped wTFp;
	wTFp = tf_buffer.lookupTransform(world_frame, planning_frame, ros::Time(0));
	tf::transformMsgToKDL(wTFp.transform, wTp);

	// PLAN MOTION
	if (msg->execute_only) {
		// execute the last succesfully planned motion
		if (!last_request_successed) {
			abortGoal("execute preplanned motion ", MoveBaseResult::SOLUTION_NOT_FOUND, "valid solution is not available");
			return;
		}
	}
	else {
		// process goal message
		try {
			// initial pose 
			setInitialStateFromTF();
			// assign goal
			setGoalPoseFromMsg(*msg);
			// check pose
			if (!checkEERangeConditions(formulation->initial_base_, formulation->initial_ee_W_)) {
				abortGoal("message processing", MoveBaseResult::INVALID_INITIAL_POSE, "invalid initial pose");
				return;
			}
			// set gait
			setGaitFromGoalMsg(*msg);
		}
		catch (tf2::TransformException& e) {
			abortGoal("message processing", MoveBaseResult::INTERNAL_ERROR, e.what());
			return;
		}
		catch (std::invalid_argument& e) {
			abortGoal("message processing", MoveBaseResult::INVALID_GOAL, e.what());
			return;
		}

		// perform planning
		last_request_goal = *msg;
		last_request_successed = performMotionPlanning();
	
		if (!last_request_successed) {
			// planning failed
			nlp.PrintCurrent(); // print additional information if solver has not successed.
			abortGoal("nlp optimization", MoveBaseResult::SOLUTION_NOT_FOUND, "Ipopt exit status: " + std::to_string(solver->GetIpoptExitStatus()));
			return;
		}
	}

	// xpp visualization: always perform visualization
	if (msg->visualize_only) {
		TowrSolutionVisualizer visualizer(period);
		visualizer.PlayTrajectory(*formulation, solution, 1.0);
	}

	if (msg->visualize_only) {
		succeedGoal(MoveBaseResult::SUCCESS, "visualized");
		return;
	}

	// EXECUTE TRAJECTORY
	
	// construct FollowStepSequenceGoal message
	FollowStepSequenceGoal steps_msg;
	steps_msg.header.stamp = ros::Time::now();
	steps_msg.header.frame_id = world_frame;
	steps_msg.append = false;
	steps_msg.position_tolerance = msg->position_tolerance;
	steps_msg.orientation_tolerance = msg->orientation_tolerance;
	storeSolutionInStepSequenceGoalMsg(steps_msg, wTp);
	ROS_DEBUG_STREAM("FollowStepSequenceGoal message" << std::endl << steps_msg);
	// check if action server is available
	if (!execute_step_sequence_ac->isServerConnected()) {
		if (!execute_step_sequence_ac->waitForServer(ros::Duration(1, 0))) {
			ROS_ERROR("FollowStepSequence action server is unavailible!");
			abortGoal("execution", MoveBaseResult::INTERNAL_ERROR, "controller is unavailable");
			return;
		}
	}
	// execute step seuence
	ROS_INFO("Send goal to FollowStepSequence action server.");
	actionlib::SimpleClientGoalState state = execute_step_sequence_ac->sendGoalAndWait(steps_msg, ros::Duration(steps_msg.time_from_start.back() * 1.1)); // 10% additional time TODO: timeout processing

	ROS_INFO_STREAM("FollowStepSequence execution: " << state.toString() << " (" << state.getText() << ")");
	if (state.isDone()) {
		FollowStepSequenceResult result = *execute_step_sequence_ac->getResult();
		ROS_INFO_STREAM("FollowStepSequence result: " << state.toString() << " (" << state.getText() << ")");
	}
	if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
		succeedGoal(MoveBaseResult::SUCCESS, "executed");
	}
	else {
		abortGoal("execution", MoveBaseResult::EXECUTION_FAILED, "execution failed");
	}
};

template<class MsgType> static bool saveMessage(ros::NodeHandle& node_handler, const MsgType& msg, const std::string& param_name) {
	uint32_t serial_size = ros::serialization::serializationLength(msg);
	boost::shared_array<unsigned char> buffer(new unsigned char[serial_size]);
	ros::serialization::OStream ostream(buffer.get(), serial_size);
	ros::serialization::serialize(ostream, msg);
	XmlRpc::XmlRpcValue param((unsigned char*)&buffer[0], serial_size);
	// write parameter to Parameter server
	try {
		node_handler.setParam(param_name, param);
	}
	catch (ros::InvalidNameException& e) {
		ROS_ERROR_STREAM("SaveTrajectoryService: ros graph name is not valid '" << param_name << "'");
		return false;
	}
	return true;
}

bool ClopGenerator::callbackSaveTrajectory(SaveTrajectoryRequest& req, SaveTrajectoryResponse& resp)
{
	if (!last_request_successed) {
		ROS_ERROR_STREAM("SaveTrajectoryService: last planning request has failed, no valid message to save.");
		return false;
	}
	// derive parameters absolute path
	std::string step_sequence_param_name, move_base_param_name; 
	if (req.name.empty()) {
		ROS_ERROR_STREAM("SaveTrajectoryService: parameter name must be nonempty.");
		return false;
	}
	if (req.name[0] != '/') {
		// use relative path inside storage namespace
		step_sequence_param_name = storage_ns + "/step_sequence/" + req.name; // default namespace for non-absolute path
		move_base_param_name = storage_ns + "/move_base/" + req.name; // default namespace for non-absolute path
	}
	else {
		// add _request prefix to parameter name
		step_sequence_param_name = req.name; 
		move_base_param_name = req.name + "_request";
	}

	// create FollowStepSequence goal in planning frame
	FollowStepSequenceGoal steps_msg;
	steps_msg.header.stamp = ros::Time::now();
	steps_msg.header.frame_id = planning_frame;
	steps_msg.append = false;
	steps_msg.position_tolerance = last_request_goal.position_tolerance;
	steps_msg.orientation_tolerance = last_request_goal.orientation_tolerance;
	storeSolutionInStepSequenceGoalMsg(steps_msg, KDL::Frame::Identity());

	ROS_INFO_STREAM("Saving messages: FollowStepSequenceGoal to " << step_sequence_param_name << " and MoveBaseGoal to " << move_base_param_name);
	bool success = saveMessage(node_handler, last_request_goal, move_base_param_name) && saveMessage(node_handler, steps_msg, step_sequence_param_name);
	return true;
}

void ClopGenerator::storeSolutionInStepSequenceGoalMsg(FollowStepSequenceGoal& msg, const KDL::Frame& wTp)
{
	int n_ee = end_effector_index.size();
	double t_total = solution.base_linear_->GetTotalTime();
	int n_samples = std::ceil(t_total / period) + 1; // we need addition point to include t_total
	towr::EulerConverter base_angular(solution.base_angular_);
	KDL::Vector com = EigenToKDL( com_B );

	// clear buffers, allocate memory 
	msg.time_from_start.clear(); 
	msg.time_from_start.reserve(n_samples);
	msg.base_motion.points.clear(); 
	msg.base_motion.points.reserve(n_samples);
	msg.ee_motion.resize(n_ee);
	for(int ee = 0; ee < n_ee; ee++) {
		msg.ee_motion[ee].points.clear();
		msg.ee_motion[ee].points.reserve(n_samples);
	}
	// assign persistent fields
	msg.base_motion.name = base_frame_id;
	for(auto it = end_effector_index.begin(); it != end_effector_index.end(); it++) msg.ee_motion[it->second.towr_index].name = it->first;

	// now fill trajectories
	for(int k = 0; k < n_samples; k++) {
		double t = std::min(period * k, t_total);
		// assign time vector
		msg.time_from_start.push_back(period * k);

		// BASE_LINK state
		msg.base_motion.points.emplace_back();
		sweetie_bot_control_msgs::RigidBodyTrajectoryPoint& point = msg.base_motion.points.back();
		KDL::Frame base_frame;
		// angular position, velocity and acceleration
		Eigen::Quaterniond base_quat = base_angular.GetQuaternionBaseToWorld(t);
		tf::quaternionEigenToKDL(base_quat, base_frame.M);
		tf::vectorEigenToKDL(base_angular.GetAngularVelocityInWorld(t), point.twist.rot);
		tf::vectorEigenToKDL(base_angular.GetAngularAccelerationInWorld(t), point.accel.rot);
		// linear position, velocity and acceleration
		towr::State lin_state =	solution.base_linear_->GetPoint(t);
		tf::vectorEigenToKDL(lin_state.at(kPos), base_frame.p);
		tf::vectorEigenToKDL(lin_state.at(kVel), point.twist.vel);
		tf::vectorEigenToKDL(lin_state.at(kAcc), point.accel.vel);
		// return to base frame from CoM frame
		base_frame.p -= base_frame.M * com;
		// now point contain pose twist, change reference point to produce screw twists
		point.twist = point.twist.RefPoint(-base_frame.p);
		point.accel = point.accel.RefPoint(-base_frame.p); //TODO check this conversion!
		// for base contact is always false
		point.contact = false;

		// END EFFECTOR state
		// we assume that end effector has no angular speed and have the same orientation as base_link in XY world plane.
		// TODO end effector should be oriented along terrain normal?
	
		// calculate end effector orientation planning frame
		Eigen::Quaterniond ee_quat(base_quat.w(), 0.0, 0.0, base_quat.z()); // w, x, y, x
		ee_quat.normalize();
		KDL::Rotation ee_R;
		tf::quaternionEigenToKDL(ee_quat, ee_R);

		for(int ee = 0; ee < n_ee; ee++) {
			msg.ee_motion[ee].points.emplace_back();
			sweetie_bot_control_msgs::RigidBodyTrajectoryPoint& point = msg.ee_motion[ee].points.back();
			// get linear state
			towr::State lin_state =	solution.ee_motion_.at(ee)->GetPoint(t);
			KDL::Vector p;

			tf::vectorEigenToKDL(lin_state.at(kPos), p);
			// position of EE frame
			p = p - ee_R * end_effector_contact_point[ee];
			tf::pointKDLToMsg(base_frame.Inverse(p), point.pose.position); // in base_link frame
			tf::quaternionEigenToMsg(base_quat.conjugate() * ee_quat, point.pose.orientation); // in base_link frame
			// velocity of EE frame
			tf::vectorEigenToKDL(lin_state.at(kVel), point.twist.vel);
			point.twist.rot = KDL::Vector::Zero();
			point.twist = base_frame.Inverse(point.twist - msg.base_motion.points[k].twist); // in base_link frame 
			// acceleration
			tf::vectorEigenToKDL(lin_state.at(kAcc), point.accel.vel);
			point.accel.rot = KDL::Vector::Zero();
			point.accel = base_frame.Inverse(point.accel - msg.base_motion.points[k].accel); // in base_link frame
			// contact
			point.contact = solution.phase_durations_.at(ee)->IsContactPhase(t);
		}

		// convert base pose from planning frame to world frame
		base_frame = wTp * base_frame;	
		tf::poseKDLToMsg(base_frame, point.pose);
		point.twist = wTp * point.twist;
		point.accel = wTp * point.accel;
	}
}


} // namespace sweetie_bot

int main(int argc, char **argv)
{
	ros::init(argc, argv, "clop_generator");

	sweetie_bot::ClopGenerator generator("clop_generator");

	ros::spin();

	return 0;
}
