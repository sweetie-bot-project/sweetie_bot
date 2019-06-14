#ifndef TOWR_MODELS_SINGLE_RIBID_BODY_WITH_POINT_EE_DYNAMICS_MODEL_H_
#define TOWR_MODELS_SINGLE_RIBID_BODY_WITH_POINT_EE_DYNAMICS_MODEL_H_

#include <towr/models/dynamic_model.h>

namespace towr {

/**
 * @brief Dynamics model relating forces to base accelerations.
 *
 * This class implements a Single Rigid Body dynamics model agumented with 
 * end effectors, represented as point masses. This model can be useful 
 * if robot has massive legs.
 *
 * This model makes the assumption that the motion of the limbs does not
 * incur significant momentum and can therefore be neglected. This eliminates
 * the nonlinear dependency on joint angles and allows to express
 * all quantities in Cartesian space.
 *
 * @ingroup Robots
 */
class SingleRigidBodyWithPointEEDynamics : public DynamicModel {
public:
  /**
   * @brief Constructs a specific model.
   * @param mass         The mass of the robot.
   * @param inertia_b    The elements of the 3x3 Inertia matrix around the CoM.
   *                     This matrix maps angular accelerations expressed in
   *                     base frame to moments in base frame.
   * @param ee_masses    The vector of end effector masses.
   */
  SingleRigidBodyWithPointEEDynamics(double mass, const Eigen::Matrix3d& inertia_b, const std::vector<double>& ee_masses);


  virtual ~SingleRigidBodyWithPointEEDynamics () = default;

  BaseAcc GetDynamicViolation() const override;

  Jac GetJacobianWrtBaseLin(const Jac& jac_base_lin_pos,
                            const Jac& jac_acc_base_lin) const override;
  Jac GetJacobianWrtBaseAng(const EulerConverter& base_angular,
                            double t) const override;
  Jac GetJacobianWrtForce(const Jac& jac_force, EE ee) const override;

  Jac GetJacobianWrtEEPos(const Jac& jac_ee_pos, const Jac& jac_ee_acc, EE ee) const override;

private:
  /** Inertia of entire robot around the CoM expressed in a frame anchored
   *  in the base.
   */
  Eigen::SparseMatrix<double, Eigen::RowMajor> I_b;
  /** Masses of end effectors.
   * */
  std::vector<double> ee_masses;
};


} /* namespace towr */

#endif /* TOWR_MODELS_SINGLE_RIBID_BODY_DWITH_POINT_EE_YNAMICS_MODEL_H_ */
