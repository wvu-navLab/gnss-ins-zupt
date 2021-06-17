/**
 *  @file   GNSSDCSFactor.cpp
 *  @author Ryan Watson
 *  @brief  Implementation file for GNSS factor With mulit-modal uncert. model
 **/

#include <gtsam/gnssNavigation/GNSSDCSFactor.h>

using namespace std;
using namespace boost;
using namespace merge;

namespace gtsam {
//***************************************************************************
Vector GNSSDCSFactor::whitenedError(const gtsam::Values& x,
                                    boost::optional<std::vector<Matrix>&> H) const {


        const nonBiasStates& q = x.at<nonBiasStates>(k1_);
        const phaseBias& g = x.at<phaseBias>(k2_);

        Vector h = obsMap(satXYZ_, nomXYZ_, 1);
        Matrix gnssPartials = Z_1x1;

        double res_range = (h.transpose() * q) - measured_[0];
        double res_phase = (h.transpose() * q) + g[0] - measured_[1];

        Eigen::VectorXd res(2);
        res << res_range, res_phase;

        if (H) {
                Matrix H_g(2,5);
                H_g.row(0) = h;
                H_g.row(1) = h;

                Matrix H_b(2,1);
                H_b(0,0) = 0.0;
                H_b(1,0) = 1.0;

                (*H)[0].resize(H_g.rows(), H_g.cols());
                (*H)[1].resize(H_b.rows(), H_b.cols());

                (*H)[0] = H_g;
                (*H)[1] = H_b;

        }


        // DCS Base scaling of cov.
        double v_range, v_phase;

        // check range residaul
        if (std::pow(res(0),2) < k_(0)  || iter_count_ < 2)
        {
                v_range = model_(0,0);
        }
        else
        {
                double scale = (4*std::pow(k_(0),2)) / (std::pow( std::pow(res(0),2) + k_(0),2));
                v_range = model_(0,0)/scale;
        }

        // check phase residaul
        if (std::pow(res(1),2) < k_(1) || iter_count_ < 2)
        {
                v_phase = model_(1,1);
        }
        else
        {
                double scale = (4*std::pow(k_(1),2)) / (std::pow( std::pow(res(1),2) + k_(1),2));
                v_phase = model_(1,1)/scale;
        }


        Eigen::MatrixXd cov_model(2,2);
        cov_model << v_range, 0.0, 0.0, v_phase;

        return (gtsam::noiseModel::Gaussian::Covariance(cov_model))->whiten(res);
}

Vector GNSSDCSFactor::unwhitenedError(const gtsam::Values& x,
                                      boost::optional<std::vector<Matrix>&> H) const {

        const nonBiasStates& q = x.at<nonBiasStates>(k1_);
        const phaseBias& g = x.at<phaseBias>(k2_);

        Vector h = obsMap(satXYZ_, nomXYZ_, 1);
        Matrix gnssPartials = Z_1x1;

        double res_range = (h.transpose() * q) - measured_[0];
        double res_phase = (h.transpose() * q) + g[0] - measured_[1];

        Eigen::VectorXd res(2);
        res << res_range, res_phase;

        if (H) {

                Matrix H_g(2,5);
                H_g.row(0) = h;
                H_g.row(1) = h;

                Matrix H_b(2,1);
                H_b(0,0) = 0.0;
                H_b(1,0) = 1.0;

                (*H)[0].resize(H_g.rows(), H_g.cols());
                (*H)[1].resize(H_b.rows(), H_b.cols());

                (*H)[0] = H_g;
                (*H)[1] = H_b;

        }

        return res;
}

}  //namespace
