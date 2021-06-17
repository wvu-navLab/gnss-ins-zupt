/**
 *  @file   GNSSSwitch.cpp
 *  @author Ryan Watson & Jason Gross
 *  @brief  Implementation file for GNSS switch factor
 **/

#include <gtsam/robustModels/GNSSSwitch.h>

using namespace std;
using namespace vertigo;

//***************************************************************************
namespace gtsam {
Vector GNSSSwitch::evaluateError(const nonBiasStates& q, const phaseBias& g, const vertigo::SwitchPairLinear& s, boost::optional<Matrix&> H1, boost::optional<Matrix&> H2, boost::optional<Matrix&> H3) const {

        Vector5 h = obsMap(satXYZ_, nomXYZ_, 1);

        double res_range = ( (h.transpose() * q) - measured_[0] ) * s.value()[0];
        double res_phase = ( (h.transpose() * q) + g[0] - measured_[1] ) * s.value()[1];

        Vector error = (Vector(2) << res_range, res_phase).finished();

        if (H1) { (*H1) = (Matrix(2,5) << h*s.value()[0], h*s.value()[1]).finished(); }
        if (H2) { (*H2) = (Matrix(2,1) << 0.0, s.value()[1]).finished(); }
        if (H3) { (*H3) = (Vector(2) << error).finished(); }
        return error;
}

}  //namespace


// namespace gtsam {
// Vector GNSSSwitch::evaluateError(const nonBiasStates& q, const phaseBias& g, const SwitchVariableLinear& sr, const SwitchVariableLinear& sp, boost::optional<Matrix&> H1, boost::optional<Matrix&> H2, boost::optional<Matrix&> H3, boost::optional<Matrix&> H4) const {
//
//         Vector5 h = obsMap(satXYZ_, nomXYZ_, 1);
//
//         double res_range = ( (h.transpose() * q) - measured_[0] ) * sr.value();
//         double res_phase = ( (h.transpose() * q) + g[0] - measured_[1] ) * sp.value();
//         Vector2 error = (Vector(2) << res_range, res_phase).finished();
//
//         cout << sr.value() << " " << sp.value() << endl;
//
//         if (H1) {
//                 cout << "H1  " << endl;
//                 (*H1) = (Matrix(2,5) << h*sr.value(), h*sp.value()).finished();
//         }
//         if (H2) {
//                 cout << "H2  " << endl;
//                 (*H2) = (Matrix(2,1) << 0.0, sp.value()).finished();
//         }
//         if (H3) {
//                 cout << "H3  " << endl;
//                 (*H3) = (Vector(1) << error(0)).finished();
//         }
//         if (H4) {
//                 cout << "H4 " << endl;
//                 (*H4) = (Vector(2) << 0.0, error(1)).finished();
//         }
//         return error;
// }
//
// }  //namespace
