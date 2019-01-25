#ifndef  SINGLE_RIGID_BODY_NON_COM_DYNAMICS_H
#define  SINGLE_RIGID_BODY_NON_COM_DYNAMICS_H

#include <towr/models/dynamic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>

namespace towr {

/**
 * @brief Dynamics model relating forces to base accelerations.
 *
 * This class implements a Single Rigid Body dynamics model, without assumption,
 * that the center of mass coincidices with base frame.
 */
class SingleRigidBodyNonCoMDynamics : public SingleRigidBodyDynamics 
{
	private:
		Vector3d com_pos_b_;
	
	public:
	  /**
	   * @brief Constructs a specific model.
	   * @param mass         The mass of the robot.
	   * @param com_point_b  The center of mass location on base frame.
	   * @param ee_count     The number of endeffectors/forces.
	   * @param inertia_b    The elements of the 3x3 Inertia matrix around the CoM.
	   *                     This matrix maps angular accelerations expressed in
	   *                     base frame to moments in base frame.
	   */
	  SingleRigidBodyNonCoMDynamics (double mass, Vector3d com_point_b, const Eigen::Matrix3d& inertia_b, int ee_count);
	  virtual ~SingleRigidBodyDynamics () = default;

	  void SetCurrent(const ComPos& base_W, const Vector3d base_acc_W,
							  const Matrix3d& w_R_b, const AngVel& omega_W, const Vector3d& omega_dot_W,
							  const EELoad& force_W, const EEPos& pos_W) override;
	  Jac GetJacobianWrtBaseAng(const EulerConverter& base_angular, double t) const override;
};


} /* namespace towr */

#endif  /*SINGLE_RIGID_BODY_NON_COM_DYNAMICS_H*/

