#ifndef MERGE_H
#define MERGE_H

#include <vector>
#include <stdexcept>
#include <cmath>
#define _USE_MATH_DEFINES

#include <Eigen/Dense>

#include "libcluster.h"
#include "probutils.h"
#include "distributions.h"

#include <boost/tuple/tuple.hpp>
#include <boost/math/special_functions.hpp>

// /*! \brief Merge similar mixture components as specified in the reference provided below.
//
//    REF ::
//    [1] Song, Mingzhou, and Hongbin Wang. "Highly efficient incremental estimation of Gaussian mixture models for online data stream clustering." Intelligent Computing: Theory and Applications III. Vol. 5803. International Society for Optics and Photonics, 2005.
//
//
// Author: Ryan Watson
//

using namespace std;
using namespace Eigen;
using namespace boost;
using namespace probutils;
using namespace libcluster;
using namespace boost::math;
using namespace distributions;


namespace merge
{

// Total num. obs, Num obs in comp., comp. weight, compt. mean, compt. cov.
typedef boost::tuple<int, int, double, Eigen::RowVectorXd, Eigen::MatrixXd> mixtureComponents;
typedef boost::tuple<mixtureComponents, double> observationModel;

// Implementation of Chi-square check for covarianec matrices.
bool checkCov(Eigen::MatrixXd data, Eigen::MatrixXd qZ, mixtureComponents priorModel, GaussWish currModel, double currWeight, int currModelIdx, double alpha);
bool checkCovGMM(mixtureComponents prior, mixtureComponents test, double alpha);


// Implementation of Fisher-Dist. check for mean vectors
bool checkMean(mixtureComponents priorModel, GaussWish currModel, double alpha);
bool checkMeanGMM(mixtureComponents priorModel, mixtureComponents currModel, double alpha);

// get weight vector from prior model
vector<double> getPriorWeights(vector<mixtureComponents> gmm);

// update num obs per component
vector<mixtureComponents> updateObs(vector<mixtureComponents> gmm, vector<int> numObs);


// prune the GMM to the truncation level
vector<mixtureComponents> pruneMixtureModel(vector<mixtureComponents> gmm, int truncLevel);
bool sortbyobs(mixtureComponents a, mixtureComponents b);


// Check cov. and mean to see if components are approx. equlivant.
bool checkComponent(Eigen::MatrixXd data, Eigen::MatrixXd qZ, mixtureComponents priorModel, GaussWish currModel, double currWeight, int currModelIdx, double alpha);

bool checkComponentGMM(mixtureComponents prior, mixtureComponents test, double alpha);

// Implementaion of Algo. 1  in [1] to merge statistically equlivant components of a mixture model.
vector<mixtureComponents> mergeMixtureModel(Eigen::MatrixXd data, Eigen::MatrixXd qZ, vector<mixtureComponents> priorModel, vector<GaussWish> currModel, StickBreak currWeight, double alpha, int truncLevel);

// Return the highest probability mixture component given an observation
observationModel getMixtureComponent(vector<mixtureComponents> gmm, Eigen::VectorXd observation);

}

#endif // PROBUTILS_H
