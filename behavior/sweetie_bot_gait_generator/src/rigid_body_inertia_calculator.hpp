#ifndef  RIGID_BODY_INERTIA_CALCULATOR_HPP
#define  RIGID_BODY_INERTIA_CALCULATOR_HPP

#include <kdl/tree.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/treefksolver.hpp>

#include <sensor_msgs/JointState.h>

class RigidBodyInertiaCalculator 
{
	protected:
		typedef std::map<std::string, int> JointIndex;
	protected:
		KDL::Tree tree;
		KDL::JntArray jnt_array;
		JointIndex joint_index;
		std::unique_ptr<KDL::TreeFkSolverPos> fk_solver;

	public:
		RigidBodyInertiaCalculator(const std::string& urdf_model);

		void setPose(const sensor_msgs::JointState& pose);
		void setPoseZero() { jnt_array.data.setZero(); };
		KDL::RigidBodyInertia getInertiaTotal();
		KDL::RigidBodyInertia getInertia(const std::string& frame, const std::vector<std::string>& segments);
};


#endif  /*RIGID_BODY_INERTIA_CALCULATOR_HPP*/
