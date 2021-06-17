/**
 *  @file   GNSSMultiModalFactor.h
 *  @author Ryan Watson
 *  @brief  Header file for GNSS Factor with multimodal uncert. model
 **/

#pragma once
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>


#include <libcluster/merge.h>
#include <libcluster/probutils.h>
#include <libcluster/libcluster.h>
#include <libcluster/distributions.h>

#include <boost/tuple/tuple.hpp>
#include <boost/math/special_functions.hpp>


namespace gtsam {


class GTSAM_EXPORT GNSSMultiModalFactor : public NonlinearFactor {

private:

typedef gtsam::NonlinearFactor Base;
typedef GNSSMultiModalFactor This;

mutable int iter_count_;
Key k1_,k2_;
Point3 satXYZ_;
Point3 nomXYZ_;
nonBiasStates h_;
Vector2 measured_;
mutable gtsam::Matrix2 cov_min_;
vector<merge::mixtureComponents> gmm_;

public:

typedef boost::shared_ptr<GNSSMultiModalFactor> shared_ptr;

GNSSMultiModalFactor() : measured_() {
        h_=Matrix(2,5);
}

GNSSMultiModalFactor(Key deltaStates, Key bias, const Vector2 measurement,
                     const Point3 satXYZ, const Point3 nomXYZ, vector<merge::mixtureComponents>& gmm) :
        Base(cref_list_of<2>(deltaStates)(bias)), k1_(deltaStates), k2_(bias), measured_(measurement), satXYZ_(satXYZ), nomXYZ_(nomXYZ), gmm_(gmm), iter_count_(0) {
}

virtual ~GNSSMultiModalFactor() {
}

/** print */
virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
        std::cout << s << "GNSS Factor("
                  << keyFormatter(k1_) << ","
                  << keyFormatter(k2_) << ")\n"
                  << "  measured:  " << measured_.transpose();
}

virtual bool equals(const NonlinearFactor& f, double tol = 1e-9) const {
        const This *t = dynamic_cast<const This*>(&f);

        if (t && Base::equals(f)) { return k1_ == t->k1_ && k2_ == t->k2_; }
        else{ return false; }
}

virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                       gtsam::NonlinearFactor::shared_ptr(new GNSSMultiModalFactor(*this)));
}

virtual double error(const gtsam::Values& x) const {
        return whitenedError(x).squaredNorm();
}

Vector unwhitenedError(const gtsam::Values& x,
                       boost::optional<std::vector<Matrix>&> H = boost::none) const;

Vector whitenedError(const gtsam::Values& x,
                     boost::optional<std::vector<Matrix>&> H = boost::none) const;

virtual Vector residual(const gtsam::Values& x) const {
        const Vector b = unwhitenedError(x);
        return b;
}

virtual size_t dim() const {
        return 5;
}


std::size_t size() const {
        return 2;
}

bool active(const gtsam::Values& x) const {
        return true;
}



/* ************************************************************************* */
/**
 * Linearize a non-linearFactorN to get a gtsam::GaussianFactor,
 * \f$ Ax-b \approx h(x+\delta x)-z = h(x) + A \delta x - z \f$
 * Hence \f$ b = z - h(x) = - \mathtt{error\_vector}(x) \f$
 */
/* This version of linearize recalculates the noise model each time */
virtual boost::shared_ptr<gtsam::GaussianFactor> linearize(
        const gtsam::Values& x) const {

        iter_count_ += 1;

        if (!active(x))
                return boost::shared_ptr<JacobianFactor>();

        // Call evaluate error to get Jacobians and RHS vector b
        std::vector<Matrix> A(this->size());
        Vector b = unwhitenedError(x, A);

        // Fill in terms, needed to create JacobianFactor below
        std::vector<std::pair<Key, Matrix> > terms(size());
        for (size_t j = 0; j < size(); ++j) {
                terms[j].first = keys()[j];
                terms[j].second.swap(A[j]);
        }

        auto jacobianFactor = GaussianFactor::shared_ptr( new JacobianFactor(terms, -b, noiseModel::Diagonal::Variances((gtsam::Vector(2) << cov_min_(0,0), cov_min_(1,1)).finished()) ));

        return jacobianFactor;
}


private:

/** Serialization function */
friend class boost::serialization::access;
template<class ARCHIVE>
void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
        ar
        & boost::serialization::make_nvp("NonlinearFactor",
                                         boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(measured_);
}

}; // GNSSMultiModalFactor Factor
} // namespace
