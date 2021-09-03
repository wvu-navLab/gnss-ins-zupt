/**
 *  @file   GNSSMaxMix.cpp
 *  @author Ryan
 *  @brief  Implementation file for pseudorange max-mix factor
 **/

#include "GNSSMaxMix.h"

using namespace std;

namespace gtsam {

//***************************************************************************
Vector GNSSMaxMix::evaluateError(const nonBiasStates& q, const phaseBias& g, boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {

        Vector h = obsMap(satXYZ_, nomXYZ_, 1);

        double res_range = (h.transpose() * q) - measured_[0];
        double res_phase = (h.transpose() * q) + g[0] - measured_[1];
        auto error = (Vector(2) << res_range, res_phase).finished();

        noiseModel::Diagonal::shared_ptr hypothesis = noiseModel::Diagonal::Variances(variances_);
        noiseModel::Diagonal::shared_ptr null = noiseModel::Diagonal::Variances(variances_/w_);


        auto m1 = hypothesis->distance(error);
        gtsam::Matrix info1(hypothesis->information());
        auto nu1 = 1.0/sqrt(inverse(info1).determinant());
        auto l1 = nu1 * exp(-0.5*log(m1));

        auto m2 = null->distance(error);
        gtsam::Matrix info2(null->information());
        auto nu2 = 1.0/sqrt(inverse(info2).determinant());
        auto l2 = nu2 * exp(-0.5*log(m2));

        // cout << "Error = " <<  error.transpose() << "  --- l1 = " << l1 << " " << "l2 = " << l2 << endl;

        if (H1)
        {
                Matrix H_g(2,5);
                H_g.row(0) = h;
                H_g.row(1) = h;
                (*H1) = H_g;
        }
        if (H2)
        {
                Matrix H_b(2,1);
                H_b(0,0) = 0.0;
                H_b(1,0) = 1.0;
                (*H2) = H_b;
        }

        if (l2>l1) {
                cout << "outlier " << endl;
                if (H1) *H1 = *H1 * w_;
                error *= sqrt(w_);
        }

        return (Vector(2) << error).finished();
}
} // namespace
