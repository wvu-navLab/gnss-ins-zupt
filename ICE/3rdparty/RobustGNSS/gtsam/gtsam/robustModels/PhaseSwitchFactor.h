/**
 *  @file   PhaseSwitchFactor.h
 *  @author Ryan Watson & Jason Gross
 *  @brief  Header file for Pseudorange Switchable factor
 **/

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>
#include <gtsam/robustModels/switchVariableLinear.h>


using namespace std;
using namespace vertigo;

namespace gtsam {

class GTSAM_EXPORT PhaseSwitchFactor : public NoiseModelFactor3<nonBiasStates, phaseBias, SwitchVariableLinear> {

private:
typedef NoiseModelFactor3<nonBiasStates, phaseBias, SwitchVariableLinear> Base;
Point3 nomXYZ_;
Point3 satXYZ_;
double measured_;
nonBiasStates h_;

public:

typedef boost::shared_ptr<PhaseSwitchFactor> shared_ptr;
typedef PhaseSwitchFactor This;

PhaseSwitchFactor() : measured_(0) {
        h_=(Matrix(1,5)<<1,1,1,1,1).finished();
}

virtual ~PhaseSwitchFactor() {
}

PhaseSwitchFactor(Key a, Key b, Key c, const double deltaObs, const Point3 satXYZ, const Point3 nomXYZ, const SharedNoiseModel& model) :
        Base(model,a,b,c), measured_(deltaObs), satXYZ_(satXYZ) {
        nomXYZ_=nomXYZ;
}
virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                       gtsam::NonlinearFactor::shared_ptr(new PhaseSwitchFactor(*this)));
}

/// vector of errors
Vector evaluateError(const nonBiasStates& q,
                     const phaseBias& b,
                     const SwitchVariableLinear& s,
                     boost::optional<Matrix&> H1 = boost::none,
                     boost::optional<Matrix&> H2 = boost::none,
                     boost::optional<Matrix&> H3 = boost::none) const;

private:

/// Serialization function
friend class boost::serialization::access;
template<class ARCHIVE>
void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
        ar
        & boost::serialization::make_nvp("NoiseModelFactor3",
                                         boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(measured_);
}

}; // PhaseSwitchFactor Factor
} // namespace
