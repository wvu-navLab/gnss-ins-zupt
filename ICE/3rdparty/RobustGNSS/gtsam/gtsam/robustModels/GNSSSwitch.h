/**
 *  @file   GNSSSwitch.h
 *  @author Ryan Watson & Jason Gross
 *  @brief  Header file for GNSS switch factor (i.e., a switchable factor for Psueodrange and Carrier-Phase observations)
 **/

#pragma once
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>
#include <gtsam/robustModels/switchPairLinear.h>
#include <gtsam/robustModels/switchVariableLinear.h>
using namespace vertigo;

namespace gtsam {


class GTSAM_EXPORT GNSSSwitch : public NoiseModelFactor3<nonBiasStates, phaseBias, vertigo::SwitchPairLinear> {

private:

Vector2 measured_;
Point3 satXYZ_, nomXYZ_;
typedef NoiseModelFactor3<nonBiasStates, phaseBias, vertigo::SwitchPairLinear> Base;

public:

typedef boost::shared_ptr<GNSSSwitch> shared_ptr;
typedef GNSSSwitch This;

GNSSSwitch() : measured_() {
}

virtual ~GNSSSwitch() {
}

GNSSSwitch(Key a, Key b, Key c, const Vector2 measurement,
           const Point3 satXYZ, const Point3 nomXYZ, const SharedNoiseModel &model) :
        Base(model, a,b,c), measured_(measurement), satXYZ_(satXYZ), nomXYZ_(nomXYZ) {
}


/** print */
virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
        std::cout << s << "GNSS Factor("
                  << keyFormatter(this->key1()) << ","
                  << keyFormatter(this->key2()) << ")\n"
                  << keyFormatter(this->key3()) << ")\n"
                  << "  measured:  " << measured_.transpose() << "\n"
                  << " noise model: "; this->noiseModel_->print("  noise model: ");
}

virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                       gtsam::NonlinearFactor::shared_ptr(new GNSSSwitch(*this)));
}

Vector evaluateError(const nonBiasStates& q, const phaseBias& g, const vertigo::SwitchPairLinear& s, boost::optional<Matrix&> H1 = boost::none,
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

}; // GNSSSwitch Factor
} // namespace

//
// namespace gtsam {
//
//
// class GTSAM_EXPORT GNSSSwitch : public NoiseModelFactor4<nonBiasStates, phaseBias, vertigo::SwitchVariableLinear, vertigo::SwitchVariableLinear> {
//
// private:
//
// Vector2 measured_;
// Point3 satXYZ_, nomXYZ_;
// typedef NoiseModelFactor4<nonBiasStates,phaseBias,SwitchVariableLinear,SwitchVariableLinear> Base;
//
// public:
//
// typedef boost::shared_ptr<GNSSSwitch> shared_ptr;
// typedef GNSSSwitch This;
//
// GNSSSwitch() : measured_() {
// }
//
// virtual ~GNSSSwitch() {
// }
//
// GNSSSwitch(Key a, Key b, Key c, Key d, const Vector2 measurement,
//            const Point3 satXYZ, const Point3 nomXYZ, const SharedNoiseModel &model) :
//         Base(model, a,b,c,d), measured_(measurement), satXYZ_(satXYZ), nomXYZ_(nomXYZ) {
// }
//
//
// /** print */
// virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
//         std::cout << s << "GNSS Factor("
//                   << keyFormatter(this->key1()) << ","
//                   << keyFormatter(this->key2()) << ")\n"
//                   << keyFormatter(this->key3()) << ")\n"
//                   << keyFormatter(this->key4()) << ")\n"
//                   << "  measured:  " << measured_.transpose() << "\n"
//                   << " noise model: "; this->noiseModel_->print("  noise model: ");
// }
//
// virtual gtsam::NonlinearFactor::shared_ptr clone() const {
//         return boost::static_pointer_cast<gtsam::NonlinearFactor>(
//                        gtsam::NonlinearFactor::shared_ptr(new GNSSSwitch(*this)));
// }
//
// Vector evaluateError(const nonBiasStates& q, const phaseBias& g, const SwitchVariableLinear& sr, const SwitchVariableLinear& sp, boost::optional<Matrix&> H1 = boost::none,
//                      boost::optional<Matrix&> H2 = boost::none,
//                      boost::optional<Matrix&> H3 = boost::none,
//                      boost::optional<Matrix&> H4 = boost::none) const;
//
// private:
//
// /// Serialization function
// friend class boost::serialization::access;
// template<class ARCHIVE>
// void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
//         ar
//         & boost::serialization::make_nvp("NoiseModelFactor4",
//                                          boost::serialization::base_object<Base>(*this));
//         ar & BOOST_SERIALIZATION_NVP(measured_);
// }
//
// }; // GNSSSwitch Factor
// } // namespace
