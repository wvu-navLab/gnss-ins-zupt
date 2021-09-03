/**
 *  @file   GNSSFactor.h
 *  @author Ryan Watson & Jason Gross
 *  @brief  Header file for GNSS Factor (i.e., a factor for Psueodrange and Carrier-Phase observations)
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

namespace gtsam {


class GTSAM_EXPORT GNSSFactor : public NoiseModelFactor2<nonBiasStates, phaseBias> {

private:
typedef NoiseModelFactor2<nonBiasStates, phaseBias> Base;
Point3 satXYZ_;
Point3 nomXYZ_;
Vector2 measured_;
nonBiasStates h_;

public:

typedef boost::shared_ptr<GNSSFactor> shared_ptr;
typedef GNSSFactor This;

GNSSFactor() : measured_() {
        h_=Matrix(2,5);
}

virtual ~GNSSFactor() {
}

GNSSFactor(Key deltaStates, Key bias, const Vector2 measurement,
           const Point3 satXYZ, const Point3 nomXYZ, const SharedNoiseModel &model) :
        Base(model, deltaStates, bias), measured_(measurement)
{
        satXYZ_=satXYZ;
        nomXYZ_=nomXYZ;
}


/** print */
virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
        std::cout << s << "GNSS Factor("
                  << keyFormatter(this->key1()) << ","
                  << keyFormatter(this->key2()) << ")\n"
                  << "  measured:  " << measured_.transpose() << "\n"
                  << " noise model: "; this->noiseModel_->print("  noise model: ");
}

virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                       gtsam::NonlinearFactor::shared_ptr(new GNSSFactor(*this)));
}

Vector evaluateError(const nonBiasStates& q, const phaseBias& g,
                     boost::optional<Matrix&> H1 = boost::none,
                     boost::optional<Matrix&> H2 = boost::none ) const;

private:

/// Serialization function
friend class boost::serialization::access;
template<class ARCHIVE>
void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
        ar
        & boost::serialization::make_nvp("NoiseModelFactor2",
                                         boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(measured_);
}

}; // GNSSFactor Factor
} // namespace
