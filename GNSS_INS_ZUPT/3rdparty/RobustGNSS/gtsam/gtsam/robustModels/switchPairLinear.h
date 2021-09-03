/**
 *  @file   SwitchPairLinear.h
 *  @author Ryan Watson & Jason Gross
 *  @brief  Header file for Pair of switch factors (simple modification of switchVariableLinear to work with GNSSFactor)
 **/

#ifndef SWITCHPAIRLINEAR_H_
#define SWITCHPAIRLINEAR_H_

#pragma once

#include <gtsam/base/DerivedValue.h>
#include <gtsam/base/Lie.h>

namespace vertigo {

struct SwitchPairLinear : public gtsam::DerivedValue<SwitchPairLinear> {

        /** default constructor */
        SwitchPairLinear() : v_(0.0,0.0) {
        };

        /** wrap a vector */
        SwitchPairLinear(gtsam::Vector2 v) : v_(v) {
        };

        SwitchPairLinear(double x, double y) : v_(x,y) {
        };

        /** access the underlying value */
        gtsam::Vector2 value() const {
                return v_;
        }

        /** access the first underlying value */
        double a() const {
                return v_(0);
        }
        /** access the second underlying value */
        double b() const {
                return v_(1);
        }


        /** print @param s optional string naming the object */
        inline void print(const std::string& name="") const {
                std::cout << name << ": " << v_ << std::endl;
        }

        /** equality up to tolerance */
        inline bool equals(const SwitchPairLinear& expected, double tol=1e-5) const {
                return fabs(expected.a() - v_(0)) <= tol && fabs(expected.b() - v_(1)) <= tol;
        }

        // Manifold requirements

        /** Returns dimensionality of the tangent space */
        inline size_t dim() const {
                return 2;
        }
        inline static size_t Dim() {
                return 2;
        }

        /** Update the SwitchPairLinear with a tangent space update */
        inline SwitchPairLinear retract(const gtsam::Vector& v) const {
                double x = v_(0) + v(0);
                double y = v_(1) + v(1);

                // cout << "BEFORE  _--_ " << "x : " << x << " y : " << y << endl;
                if (x>1.0) x=1.0;
                else if (x<0.0) x=0.0;

                if (y>1.0) y=1.0;
                else if (y<0.0) y=0.0;

                // cout << "AFTER _--_ " << "x : " << x << " y : " << y << endl;

                return SwitchPairLinear(x,y);
        }

        /** @return the local coordinates of another object */
        inline gtsam::Vector localCoordinates(const SwitchPairLinear& t2) const {
                return gtsam::Vector2(t2.value() - value());
        }

        // Group requirements

        /** identity */
        inline static SwitchPairLinear identity() {
                return SwitchPairLinear();
        }

        /** compose with another object */
        inline SwitchPairLinear compose(const SwitchPairLinear& p) const {
                return SwitchPairLinear(v_ + p.v_);
        }

        /** between operation */
        inline SwitchPairLinear between(const SwitchPairLinear& l2,
                                        boost::optional<gtsam::Matrix&> H1=boost::none,
                                        boost::optional<gtsam::Matrix&> H2=boost::none) const {
                if(H1) *H1 = -gtsam::eye(2);
                if(H2) *H2 = gtsam::eye(2);
                return SwitchPairLinear(l2.value() - value());
        }

        /** invert the object and yield a new one */
        inline SwitchPairLinear inverse() const {
                return SwitchPairLinear(-1.0 * value());
        }

        // Lie functions

        /** Expmap around identity */
        static inline SwitchPairLinear Expmap(const gtsam::Vector& v) {
                return SwitchPairLinear(v(0), v(1));
        }

        /** Logmap around identity - just returns with default cast back */
        static inline gtsam::Vector Logmap(const SwitchPairLinear& p) {
                return gtsam::Vector2(p.value());
        }

private:
        gtsam::Vector2 v_;
        double x_, y_;
};
}

namespace gtsam {
// Define Key to be Testable by specializing gtsam::traits
template<typename T> struct traits;
template<> struct traits<vertigo::SwitchPairLinear> {
        static void Print(const vertigo::SwitchPairLinear& key, const std::string& str = "") {
                key.print(str);
        }
        static bool Equals(const vertigo::SwitchPairLinear& key1, const vertigo::SwitchPairLinear& key2, double tol = 1e-8) {
                return key1.equals(key2, tol);
        }
        static int GetDimension(const vertigo::SwitchPairLinear & key) {
                return key.Dim();
        }

        typedef OptionalJacobian<3, 3> ChartJacobian;
        typedef gtsam::Vector TangentVector;
        static TangentVector Local(const vertigo::SwitchPairLinear& origin, const vertigo::SwitchPairLinear& other,
                                   ChartJacobian Horigin = boost::none, ChartJacobian Hother = boost::none) {
                return origin.localCoordinates(other);
        }

        static vertigo::SwitchPairLinear Retract(const vertigo::SwitchPairLinear& g, const TangentVector& v,
                                                 ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
                return g.retract(v);
        }
};
}



#endif /* SWITCHVARIABLELINEAR_H_ */
