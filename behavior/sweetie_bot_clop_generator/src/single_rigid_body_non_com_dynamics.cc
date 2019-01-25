#include "single_rigid_body_non_com_dynamics.h"
#include <towr/variables/cartesian_dimensions.h>

namespace towr {

using SingleRigidBodyNonCoMDynamics::Jac;
using SingleRigidBodyNonCoMDynamics::Vector3d;

// builds a cross product matrix out of "in", so in x v = X(in)*v
static Jac Cross(const Eigen::Vector3d& in)
{
	Jac out(3,3);

	out.coeffRef(0,1) = -in(2); out.coeffRef(0,2) =  in(1);
	out.coeffRef(1,0) =  in(2); out.coeffRef(1,2) = -in(0);
	out.coeffRef(2,0) = -in(1); out.coeffRef(2,1) =  in(0);

	return out;
}

SingleRigidBodyNonCoMDynamics::SingleRigidBodyDynamics (double mass, Vector3d com_point_b, const Eigen::Matrix3d& inertia_b, int ee_count) :
	SingleRigidBodyDynamics(mass, inertia_b, ee_count), com_pos_b_(com_point_b)
{ }

void SingleRigidBodyNonCoMDynamics::SetCurrent (const ComPos& base_W, const Vector3d base_acc_W,
                          const Matrix3d& w_R_b, const AngVel& omega_W, const Vector3d& omega_dot_W,
                          const EELoad& force_W, const EEPos& pos_W)
{
  Vector3d vec_b_com_w = w_R_b*com_pos_b_;
  com_pos_   = base_W + vec_b_com_w;
  com_acc_   = base_acc_W + omega_dot_W.cross(vec_b_com_w) + omega_dot_W.cross(omega_dot_W.cross(vec_b_com_w));

  w_R_b_     = w_R_b;
  omega_     = omega_W;
  omega_dot_ = omega_dot_W;

  ee_force_  = force_W;
  ee_pos_    = pos_W;
}


Jac SingleRigidBodyNonCoMDynamics::GetJacobianWrtBaseAng (const EulerConverter& base_euler, double t) const
{
	Vector3d vec_b_com_w = w_R_b*com_pos_b_;
	// derivative of p + R*p_c
	Jac jac_com_pos = base_euler.DerivOfRotVecMult(t, com_pos_b_, false);
	// derivative of a_w + dw x p_com + w x (w x p_com), p_com = R*p_com_b
	// derivative of second term: - p_com x d_dn dw + dw x (d_dn R * p_com_b)
    Jac jac_ang_acc = base_euler.GetDerivOfAngAccWrtEulerNodes(t);
	Jac jac_com_acc = - Cross(vec_b_com_w)*jac_ang_acc + Cross(omega_dot_)*jac_com_pos;
	// derivative of thrid term: w x (- p_com x d_dn w + w x d_dn R * p_com_b) - ( w x p_com ) x d_dn w
	Jac jac_ang_vel = base_euler.GetDerivOfAngVelWrtEulerNodes(t);
	jac_com_acc += Cross(omega_)*(- Cross(vec_b_com_w)*jac_ang_vel + Cross(omega_)*jac_com_pos);
	jac_com_acc -= Cross(omega_.cross(vec_b_com_w))*jac_ang_vel;

	// calculate jacobian
	return SingleRigidBodyDynamics::GetJacobianWrtBaseAng(base_euler, t) + GetJacobianWrtBaseLin(jac_com_pos, jac_com_acc);
}

} /* namespace towr */
