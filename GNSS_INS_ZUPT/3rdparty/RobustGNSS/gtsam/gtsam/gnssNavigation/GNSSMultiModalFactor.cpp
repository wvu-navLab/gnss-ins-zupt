/**
 *  @file   GNSSMultiModalFactor.cpp
 *  @author Ryan Watson
 *  @brief  Implementation file for GNSS factor With mulit-modal uncert. model
 **/

#include <gtsam/gnssNavigation/GNSSMultiModalFactor.h>

using namespace std;
using namespace boost;
using namespace merge;

namespace gtsam {
//***************************************************************************
Vector GNSSMultiModalFactor::unwhitenedError(const gtsam::Values& x,
                                             boost::optional<std::vector<Matrix>&> H) const {

        const nonBiasStates& q = x.at<nonBiasStates>(k1_);
        const phaseBias& g = x.at<phaseBias>(k2_);

        Vector h = obsMap(satXYZ_, nomXYZ_, 1);
        Matrix gnssPartials = Z_1x1;

        double res_range = (h.transpose() * q) - measured_[0];
        double res_phase = (h.transpose() * q) + g[0] - measured_[1];

        Eigen::VectorXd res(2);
        res << res_range, res_phase;


        double prob, probMax;
        probMax = 0.0;
        int ind(0);
        gtsam::Matrix cov_min(2,2);
        Eigen::RowVectorXd mean_min(2);
        Eigen::VectorXd res_2(2), res_min(2);

        // Find the most likely model from the GMM.
        for (int i=0; i<gmm_.size(); i++)
        {
                merge::mixtureComponents mixtureComp = gmm_[i];

                Eigen::RowVectorXd mean = mixtureComp.get<3>();
                res_2 << res_range - mean(0), res_phase - mean(1);

                SharedGaussian G = gtsam::noiseModel::Gaussian::Covariance(mixtureComp.get<4>());
                gtsam::Vector errW = G->whiten(res_2);

                double quadform  = (res_2).transpose() * (mixtureComp.get<4>()).inverse() * (res_2);
                double norm = std::pow(std::sqrt(2 * M_PI),-1) * std::pow((mixtureComp.get<4>()).determinant(), -0.5);

                prob =  norm * exp(-0.5 * quadform);

                if (prob >= probMax)
                {
                        ind = i;
                        probMax = prob;
                        cov_min_ = mixtureComp.get<4>();
                        mean_min = mixtureComp.get<3>();
                        res_min = res_2;
                }
        }

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

        return res_min;
        // return res;
}



Vector GNSSMultiModalFactor::whitenedError(const gtsam::Values& x,
                                           boost::optional<std::vector<Matrix>&> H) const {

        Vector res = unwhitenedError(x);
        Vector h = obsMap(satXYZ_, nomXYZ_, 1);
        Matrix gnssPartials = Z_1x1;

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

        return (gtsam::noiseModel::Gaussian::Covariance(cov_min_))->whiten(res);
}





}  //namespace
