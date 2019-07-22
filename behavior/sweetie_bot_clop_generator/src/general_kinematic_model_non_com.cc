#include "general_kinematic_model_non_com.h"

namespace towr {

GeneralKinematicModelNonCoM::AlignedBox3d GeneralKinematicModelNonCoM::GetBoundingBox(EE ee) const {
  AlignedBox3d tmp = bounding_box_.at(ee);
  tmp.translate( -com_ );
  return tmp;
}

Sphere3d GeneralKinematicModelNonCoM::GetBoundingSphere(EE ee) const {
  Sphere3d tmp = bounding_sphere_.at(ee); 
  tmp.center() -= com_;
  return tmp;
}

} /* namespace towr */


