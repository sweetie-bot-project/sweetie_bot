#include "single_rigid_body_with_point_ee_dynamics.h"

#include <towr/variables/cartesian_dimensions.h>
#include <iostream>

namespace towr {


// builds a cross product matrix out of "in", so in x v = X(in)*v
static SingleRigidBodyWithPointEEDynamics::Jac Cross(const Eigen::Vector3d& in)
{
  SingleRigidBodyWithPointEEDynamics::Jac out(3,3);

  out.coeffRef(0,1) = -in(2); out.coeffRef(0,2) =  in(1);
  out.coeffRef(1,0) =  in(2); out.coeffRef(1,2) = -in(0);
  out.coeffRef(2,0) = -in(1); out.coeffRef(2,1) =  in(0);

  return out;
}

SingleRigidBodyWithPointEEDynamics::SingleRigidBodyWithPointEEDynamics (double mass, const Eigen::Matrix3d& inertia_b, const std::vector<double>& _ee_masses)
    : DynamicModel(mass, _ee_masses.size())
{
  I_b = inertia_b.sparseView();
  ee_masses = _ee_masses;
}

SingleRigidBodyWithPointEEDynamics::BaseAcc 
SingleRigidBodyWithPointEEDynamics::GetDynamicViolation () const
{
  // https://en.wikipedia.org/wiki/Newton%E2%80%93Euler_equations

  Vector3d f_sum, tau_sum;

  // gravity force for base
  f_sum = Vector3d(0.0, 0.0, -m()*g());
  tau_sum.setZero();

  for (int ee=0; ee<ee_pos_.size(); ++ee) {
    double m = ee_masses.at(ee);
	Vector3d r = ee_pos_.at(ee) - com_pos_;
	// contact reaction forces
    Vector3d f = ee_force_.at(ee);
    f_sum   += f;
    tau_sum += r.cross(f);
	// inertia forces of end effectors
	Vector3d a = ee_acc_.at(ee); 
	f_sum -= m * a;
	tau_sum -= m * r.cross(a);
	// gravity forces for end effectors
	f_sum += m * Vector3d(0.0, 0.0, -m*g());
	tau_sum += r.cross(Vector3d(0.0, 0.0, -m*g()));
  }

  // express inertia matrix in world frame based on current body orientation
  Jac I_w = w_R_b_.sparseView() * I_b * w_R_b_.transpose().sparseView();

  BaseAcc acc;
  acc.segment(AX, k3D) = I_w*omega_dot_
                         + Cross(omega_)*(I_w*omega_)
                         - tau_sum;
  acc.segment(LX, k3D) = m()*com_acc_
                         - f_sum;
  return acc;
}

SingleRigidBodyWithPointEEDynamics::Jac
SingleRigidBodyWithPointEEDynamics::GetJacobianWrtBaseLin (const Jac& jac_pos_base_lin,
                                        const Jac& jac_acc_base_lin) const
{
  int n_ee = GetEECount();
  int n = jac_pos_base_lin.cols();
  // build the com jacobian int n = jac_pos_base_lin.cols();

  Jac jac_tau_sum(k3D, n);
  for (int ee = 0; ee < n_ee; ee++) {
    double m = ee_masses[ee];
    Jac jac_tau = Cross( ee_force_[ee] + Vector3d(0.0, 0.0, -m*g()) - m*ee_acc_[ee] )*jac_pos_base_lin;
    jac_tau_sum += jac_tau;
  }

  Jac jac(k6D, n);
  jac.middleRows(AX, k3D) = -jac_tau_sum;
  jac.middleRows(LX, k3D) = m()*jac_acc_base_lin;

  return jac;
}

SingleRigidBodyWithPointEEDynamics::Jac
SingleRigidBodyWithPointEEDynamics::GetJacobianWrtBaseAng (const EulerConverter& base_euler,
                                        double t) const
{
  Jac I_w = w_R_b_.sparseView() * I_b * w_R_b_.transpose().sparseView();

  // Derivative of R*I_b*R^T * wd
  // 1st term of product rule (derivative of R)
  Vector3d v11 = I_b*w_R_b_.transpose()*omega_dot_;
  Jac jac11 = base_euler.DerivOfRotVecMult(t, v11, false);

  // 2nd term of product rule (derivative of R^T)
  Jac jac12 = w_R_b_.sparseView()*I_b*base_euler.DerivOfRotVecMult(t, omega_dot_, true);

  // 3rd term of product rule (derivative of wd)
  Jac jac_ang_acc = base_euler.GetDerivOfAngAccWrtEulerNodes(t);
  Jac jac13 = I_w * jac_ang_acc;
  Jac jac1 = jac11 + jac12 + jac13;


  // Derivative of w x Iw
  // w x d_dn(R*I_b*R^T*w) -(I*w x d_dnw)
  // right derivative same as above, just with velocity instead acceleration
  Vector3d v21 = I_b*w_R_b_.transpose()*omega_;
  Jac jac21 = base_euler.DerivOfRotVecMult(t, v21, false);

  // 2nd term of product rule (derivative of R^T)
  Jac jac22 = w_R_b_.sparseView()*I_b*base_euler.DerivOfRotVecMult(t, omega_, true);

  // 3rd term of product rule (derivative of omega)
  Jac jac_ang_vel = base_euler.GetDerivOfAngVelWrtEulerNodes(t);
  Jac jac23 = I_w * jac_ang_vel;

  Jac jac2 = Cross(omega_)*(jac21+jac22+jac23) - Cross(I_w*omega_)*jac_ang_vel;


  // Combine the two to get sensitivity to I_w*w + w x (I_w*w)
  int n = jac_ang_vel.cols();
  Jac jac(k6D, n);
  jac.middleRows(AX, k3D) = jac1 + jac2;

  return jac;
}

SingleRigidBodyWithPointEEDynamics::Jac
SingleRigidBodyWithPointEEDynamics::GetJacobianWrtForce (const Jac& jac_force, EE ee) const
{
  Vector3d r = ee_pos_.at(ee) - com_pos_;
  Jac jac_tau = Cross(r)*jac_force;

  int n = jac_force.cols();
  Jac jac(k6D, n);
  jac.middleRows(AX, k3D) = -jac_tau;
  jac.middleRows(LX, k3D) = -jac_force;

  return jac;
}

SingleRigidBodyWithPointEEDynamics::Jac
SingleRigidBodyWithPointEEDynamics::GetJacobianWrtEEPos (const Jac& jac_ee_pos, const Jac& jac_ee_acc, EE ee) const
{
  double m = ee_masses.at(ee);
  Vector3d f = ee_force_.at(ee);
  Vector3d a = ee_acc_.at(ee);
  Vector3d r = ee_pos_.at(ee) - com_pos_;

  // contact forces + inertia forces  + gravity force
  Jac jac_tau = Cross( -(f + Vector3d(0.0,0.0,-m*g()) - m*a) )*jac_ee_pos + Cross(-m*r)*jac_ee_acc;

  Jac jac(k6D, jac_tau.cols());
  jac.middleRows(AX, k3D) = -jac_tau;
  jac.middleRows(LX, k3D) = m*jac_ee_acc;;

  return jac;
}

} /* namespace towr */
