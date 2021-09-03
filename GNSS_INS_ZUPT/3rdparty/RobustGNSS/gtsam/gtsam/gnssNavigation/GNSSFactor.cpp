/**
 *  @file   GNSSFactor.cpp
 *  @author Ryan Watson & Jason Gross
 *  @brief  Implementation file for GNSS factor
 **/

#include <gtsam/gnssNavigation/GNSSFactor.h>

using namespace std;

namespace gtsam {
//***************************************************************************
Vector GNSSFactor::evaluateError(const nonBiasStates& q, const phaseBias& g, boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {

        Vector h = obsMap(satXYZ_, nomXYZ_, 1);
        Matrix gnssPartials = Z_1x1;

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
        double res_range = (h.transpose() * q) - measured_[0];
        double res_phase = (h.transpose() * q) + g[0] - measured_[1];

        return (Vector(2) << res_range, res_phase).finished();
}

}  //namespace
