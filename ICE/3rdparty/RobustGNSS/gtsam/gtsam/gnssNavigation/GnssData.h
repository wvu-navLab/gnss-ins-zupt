/**
 * @file   GnssData.h
 * @brief  Tools required to read/write GNSS data
 * @author Ryan Watson
 */


#pragma once

#include <gtsam/config.h>
#include <gtsam/dllexport.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/robustModels/GNSSSwitch.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>

#include "boost/foreach.hpp"

#include <cmath>
#include <fstream>
#include <iostream>

using namespace std;
using namespace boost;

#define foreach BOOST_FOREACH

namespace gtsam {

/// Read GNSS data in the rnxToGtsam.cpp format
/// Data = { Week, Sow, Epoch, SVN, SatXYZ, Rho, P.C., L.C., Break_Flag}
typedef boost::tuple<double, int, int, Point3, double, double, double, int> rnxData;
vector<rnxData> readGNSS(const std::string& fileLoc);


/// Read GNSS data in the rnxToGtsam.cpp format
vector<rnxData> readGNSS_SingleFreq(const std::string& fileLoc);


/// Data = { Week, Sow, Epoch, SVN, SatXYZ, Rho, P.C., L.C., Break_Flag, EL}
typedef boost::tuple<double, int, int, Point3, double, double, double, double, double> rnxDataWEl;
vector<rnxDataWEl> readGNSS_SingleFreqTmp(const std::string& fileLoc);

/// Read GNSS data and add fautls to observations
typedef boost::tuple<double, int, int, Point3, double, double, double, int, int> faultyRnxData;
vector<faultyRnxData> readGNSSFaulty(const std::string& fileLoc, const double& mean, const double& stdDev, const double& percentFaulty);


vector<faultyRnxData> readGNSSOracle(const std::string& fileLoc, const double& mean, const double& stdDev, const double& percentFaulty);

/// Write GNSS states to text file
void writeStates(Values &results, vector<string> timeIndex, string outputFile);

/// Write Pos. solution in ENU co-ordinate frame
void writeNavFrame(Values &results, Point3 &nom, vector<string> timeIndex, string outputFile);

/// Write Pos. solution in ECEF co-ordinate frame
void writeEarthFrame(Values &results, Point3 &nom, vector<string> timeIndex, string outputFile);

/// Write switch states to text file
void writeSwitches( Values &results, string outputFile, vector<string> switchIndex);

/// Write switch states to text file
void writeSwitchPair( Values &results, string outputFile, vector<string> switchIndex);

/// Write switch states to text file
void writeAmbiguity( Values &results, string outputFile, vector<string> satIndex);

}
