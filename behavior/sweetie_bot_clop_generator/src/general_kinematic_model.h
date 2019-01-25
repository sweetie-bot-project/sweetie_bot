#ifndef  GENERAL_KINEMATIC_MODEL_H
#define  GENERAL_KINEMATIC_MODEL_H

#include <bitset>
#include <towr/models/kinematic_model.h>

namespace towr {

	class GeneralKinematicModel : public KinematicModel {
		public:
			static const size_t max_number_of_legs_ = 32;
			using EE = unsigned int;

		protected:
			std::bitset<max_number_of_legs_> configured_legs_;
			std::vector<Vector3d> bounding_box_center_;
			std::vector<Vector3d> bounding_box_max_deviation_;

		public:
			GeneralKinematicModel(int n_ee);

			void configureEndEffector(EE ee, const Vector3d& nominal_stance, const Vector3d& bounding_box_p1, const Vector3d& bounding_box_p2);

			bool isConfigured() {
				int n_ee = nominal_stance_.size();
				return configured_legs_.count() == n_ee;
			}
	};

} /* namespace towr */

#endif  /*GENERAL_KINEMATIC_MODEL_H*/
