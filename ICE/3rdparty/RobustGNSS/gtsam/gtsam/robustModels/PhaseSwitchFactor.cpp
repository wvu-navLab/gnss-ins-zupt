/**
 *  @file   PhaseSwitchFactor.cpp
 *  @author Ryan Watson & Jason Gross
 *  @brief  Implementation file for carrier-phase factor
 **/

#include <gtsam/robustModels/PhaseSwitchFactor.h>

using namespace std;

namespace gtsam {
//***************************************************************************
Vector PhaseSwitchFactor::evaluateError(const nonBiasStates& q, const phaseBias& g, const SwitchVariableLinear& s, boost::optional<Matrix&> H1, boost::optional<Matrix&> H2, boost::optional<Matrix&> H3) const {

        Vector h = obsMap(satXYZ_, nomXYZ_, 1);
        double est = (h.transpose() * q) + g[0];
        Vector error = (Vector(1) << est - measured_ ).finished();
        error *= s.value();

        cout << error << " " << "s.value = " << s.value() << endl;

        if (H1) { (*H1) = (Matrix(1,5) << h * s.value() ).finished(); }
        Matrix gnssPartials = Z_1x1;
        if (H2) {
                gnssPartials(0) = 1.0 * s.value(); // phase bias
                (*H2) = gnssPartials;
        }
        if (H3) { (Matrix(1,1) << error).finished(); }
        return (Matrix(1,1) << error ).finished();
}

}  //namespace
