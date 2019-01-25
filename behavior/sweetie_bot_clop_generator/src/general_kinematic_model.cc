#include "general_kinematic_model.h"

namespace towr {

GeneralKinematicModel::GeneralKinematicModel(int n_ee) :
	KinematicModel(n_ee) 
{
	if (n_ee > max_number_of_legs_) throw std::invalid_argument("Too many legs");
	bounding_box_center_.resize(n_ee);
	bounding_box_max_deviation_.resize(n_ee);
}

void GeneralKinematicModel::configureEndEffector(EE ee, const Vector3d& nominal_stance, const Vector3d& bounding_box_p1, const Vector3d& bounding_box_p2) 
{
	int n_ee = nominal_stance_.size();
	if (ee >= n_ee) throw std::out_of_range("invalid end effector number");
	// in current TOWR version nominal pose is always bounding pose center
	nominal_stance_[ee] = 0.5*(bounding_box_p1 + bounding_box_p2);
	bounding_box_center_[ee] = 0.5*(bounding_box_p1 + bounding_box_p2);
	bounding_box_max_deviation_[ee] = 0.5*Eigen::abs( (bounding_box_p2 - bounding_box_p1).array() );
	configured_legs_[ee] = true;
	// in current TOWR version boundin boxes are same, so configure 
	if (configured_legs_.count() == n_ee) {
		max_dev_from_nominal_ = bounding_box_max_deviation_[0];
		for(int i = 1; i < n_ee; i++) max_dev_from_nominal_ = max_dev_from_nominal_.array().min(bounding_box_max_deviation_[i].array());
	}
}

} /* namespace towr */


