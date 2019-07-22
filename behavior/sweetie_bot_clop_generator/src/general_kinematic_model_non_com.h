#ifndef TOWR_MODELS_GENERAL_KINEMATIC_MODEL_NON_COM_H_
#define TOWR_MODELS_GENERAL_KINEMATIC_MODEL_NON_COM_H_

#include <bitset>
#include <towr/models/general_kinematic_model.h>

namespace towr {

/** * @brief Complex kineamtic model with individual constraints for each leg and movable CoM
 *
 * Standart TOWR kinematic assumes that CoM coincidences with origin of base link frame. 
 * This model allows to set CoM position. All kineamtic parameters are returned in frame
 * with origin in CoM.
 *
 * @ingroup Robots
 */

class GeneralKinematicModelNonCoM : public GeneralKinematicModel {

public:
  GeneralKinematicModelNonCoM(int n_ee) : GeneralKinematicModel(n_ee) {}
  virtual ~GeneralKinematicModelNonCoM() = default;

  /**
   * @brief Set CoM position.
   * @param com CoM position in base link frame.
   **/
  void SetCoM(const Vector3d& com) {
    com_ = com;
  }

  Vector3d GetCoM() const {
    return com_;
  }

  virtual Vector3d GetNominalStanceInBase(EE ee) const override {
    return nominal_stance_.at(ee) - com_;
  }

  virtual AlignedBox3d GetBoundingBox(EE ee) const override;

  virtual Sphere3d GetBoundingSphere(EE ee) const override;

protected:
  Vector3d com_;
};

} /* namespace towr */

#endif /* TOWR_MODELS_GENERAL_KINEMATIC_MODEL_NON_COM_H_ */
