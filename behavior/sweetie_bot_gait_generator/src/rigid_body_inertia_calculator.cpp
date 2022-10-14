#include "rigid_body_inertia_calculator.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <orocos/sweetie_bot_orocos_misc/joint_state_check.hpp>


RigidBodyInertiaCalculator::RigidBodyInertiaCalculator(const std::string& urdf_model)
{
	// load urdf model 
	if (!kdl_parser::treeFromString(urdf_model, tree)) {
		throw std::invalid_argument("Failed to construct kdl tree from urdf model.");
	}
	// create solver
	fk_solver.reset( new KDL::TreeFkSolverPos_recursive( tree ) );
	// create joint index: add only segments with joints
	for(const auto& element : tree.getSegments()) {
		const KDL::Joint& joint = GetTreeElementSegment(element.second).getJoint();
		if (joint.getType() != KDL::Joint::None) {
			joint_index[joint.getName()] = GetTreeElementQNr(element.second);
		}
	}
	// create buffer for current pose
	jnt_array.resize(tree.getNrOfJoints());
	jnt_array.data.setZero();
}

void RigidBodyInertiaCalculator::setPose(const sensor_msgs::JointState& joints) 
{
	if (!sweetie_bot::isValidJointStateNamePos(joints)) {
		throw std::invalid_argument("Invalid JointState message.");
	}
	// save pose in jnt_array, ignore unknown joints
	for(int i = 0; i < joints.name.size(); i++) {
		int QNr = joint_index[joints.name[i]];
		jnt_array(QNr) = joints.position[i];
	}
}


KDL::RigidBodyInertia RigidBodyInertiaCalculator::getInertiaTotal() 
{
	KDL::RigidBodyInertia I_total;
	KDL::Frame frame;

	for(const auto& element : tree.getSegments()) {
		const KDL::Segment& segment  = GetTreeElementSegment(element.second);
		// forward kinematic
		// TODO cache optimization: exclude sovler altogether
		int retval = fk_solver->JntToCart(jnt_array, frame, segment.getName());
		if (retval < 0){
			throw std::invalid_argument("Unable calculate pose for Segment " + segment.getName());
		}
		// get inertia
		I_total = I_total + frame * segment.getInertia();
	}
	return I_total;
}


KDL::RigidBodyInertia RigidBodyInertiaCalculator::getInertia(const std::string& frame_name, const std::vector<std::string>& segments)
{
	KDL::RigidBodyInertia I_total;
	KDL::Frame frame;
	// calculate inertia in root frame
	for(const std::string& segment_name : segments) {
		// forward kinematic
		// TODO cache optimization: exclude sovler altogether
		int retval = fk_solver->JntToCart(jnt_array, frame, segment_name);
		if (retval < 0){
			throw std::invalid_argument("Unable calculate pose for Segment " + segment_name);
		}
		// get inertia
		auto element_it = tree.getSegments().find(segment_name); // we know that segemnt exists
		I_total = I_total + frame * GetTreeElementSegment(element_it->second).getInertia();
	}
	// convert to frame
	int retval = fk_solver->JntToCart(jnt_array, frame, frame_name);
	if (retval < 0){
		throw std::invalid_argument("Unable calculate pose for Segment " + frame_name);
	}
	return frame.Inverse()*I_total;
}

