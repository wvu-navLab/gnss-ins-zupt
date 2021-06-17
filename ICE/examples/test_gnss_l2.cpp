/*
 * @file test_gnss_l2.cpp
 * @brief Iterative GPS Range/Phase Estimator with collected data
 * @author Ryan Watson
 */

// GTSAM related includes.
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/gnssNavigation/GnssData.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/gnssNavigation/GNSSMultiModalFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>


// LibCluster
#include <libcluster/merge.h>
#include <libcluster/probutils.h>
#include <libcluster/libcluster.h>
#include <libcluster/distributions.h>

// BOOST
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/serialization/export.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

// GPSTK
#include <gpstk/ConfDataReader.hpp>

// STD
#include <chrono>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;
using namespace gtsam;
using namespace gpstk;
using namespace boost;
using namespace merge;
using namespace std::chrono;
using namespace libcluster;
using namespace distributions;
namespace NM = gtsam::noiseModel;
namespace po = boost::program_options;
typedef noiseModel::Diagonal diagNoise;

// Intel Threading Building Block
#include <tbb/tbb.h>


using symbol_shorthand::X; // nonBiasStates ( dx, dy, dz, trop, cb )
using symbol_shorthand::G;   // bias states ( Phase Biases )

int main(int argc, char* argv[])
{
        // define std out print color
        bool skipped_update(false);
        vector<int> prn_vec;
        vector<int> factor_count_vec;
        vector<rnxData> data;
        // vector<faultyRnxData> data;
        const string red("\033[0;31m");
        const string green("\033[0;32m");
        string confFile, gnssFile, station;
        double xn, yn, zn, range, phase, rho, gnssTime, prev_time;
        int startKey(0), currKey, startEpoch(0), svn, numBatch(0), state_count(0), update_count(0);
        int nThreads(-1), phase_break, break_count(0), nextKey, factor_count(-1), res_count(-1);
        bool printECEF, printENU, printAmb, first_ob(true);
        Eigen::MatrixXd residuals;
        vector<mixtureComponents> globalMixtureModel;
        int num_zupts = 0;

        string out_file = "/home/navlab-shounak/Desktop/Fusion/t10/l2noisy1.xyz";
        ofstream out_os(out_file);

        cout.precision(12);

        out_os.precision(12);

        po::options_description desc("Available options");
        // desc.add_options()
        //         ("help,h", "Print help message")
        //         ("confFile,c", po::value<string>(&confFile)->default_value(""),
        //         "Input config file" );

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
        po::notify(vm);

        // gnssFile = "/home/navlab-shounak/Desktop/Maria Files/maria_outside.gtsam";
        gnssFile = "/home/navlab-shounak/Desktop/Fusion/t10/noisy1.gtsam";
        // ConfDataReader confReader;
        // confReader.open(confFile);
        //
        // if (confFile.empty() ) {
        //         cout << red << "\n\n Currently, you need to provide a conf file \n"
        //              << "\n\n"  << green << desc << endl;
        // }

        // while ( (station = confReader.getEachSection()) != "" )
        // {
        //         xn = confReader.fetchListValueAsDouble("nominalECEF",station);
        //         yn = confReader.fetchListValueAsDouble("nominalECEF",station);
        //         zn = confReader.fetchListValueAsDouble("nominalECEF",station);
        //         printENU = confReader.getValueAsBoolean("printENU", station);
        //         printAmb = confReader.getValueAsBoolean("printAmb", station);
        //         printECEF = confReader.getValueAsBoolean("printECEF", station);
        //         gnssFile = confReader("dataFile", station);
        // }

        // xn = 856509.2570;
        // yn = -4843016.7781;
        // zn = 4047938.0419;

        //t10 nominal ECEF values

        ifstream inputFile("zupt_Tags.txt");
        vector<double> zupt_tags;

        // test file open
        if (inputFile) {
            double val;

            // read the elements in the file into a vector
            while ( inputFile >> val ) {
                zupt_tags.push_back(val);
            }
        }


        xn = 859153.0167;
        yn = -4836303.7245;
        zn = 4055378.4991;

        printENU = false;
        printAmb = false;
        printECEF = true;

        Point3 nomXYZ(xn, yn, zn);
        Point3 prop_xyz = nomXYZ;

        try {data = readGNSS_SingleFreq(gnssFile); }
        catch(std::exception& e)
        {
                cout << red << "\n\n Cannot read GNSS data file " << endl;
                exit(1);
        }


        std::auto_ptr<tbb::task_scheduler_init> init;
        if(nThreads > 0) {
                init.reset(new tbb::task_scheduler_init(nThreads));
        }
        else
                cout << green << " \n\n Using threads for all processors" << endl;

        ISAM2DoglegParams doglegParams;
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1000;
        ISAM2 isam(parameters);

        double output_time = 0.0;
        double rangeWeight = 2.5;
        double phaseWeight = 0.25;

        ifstream file(gnssFile.c_str());
        string value;

        nonBiasStates prior_nonBias = (gtsam::Vector(5) << 0.0, 0.0, 0.0, 0.0, 0.0).finished();

        noiseModel::Diagonal::shared_ptr zuptNoise = noiseModel::Diagonal::Variances((gtsam::Vector(5) << 1e-5, 1e-5, 1e-5, 1e3, 1e-3).finished());

        phaseBias bias_state(Z_1x1);
        gnssStateVector phase_arc(Z_34x1);
        gnssStateVector bias_counter(Z_34x1);
        for (int i=1; i<34; i++) {bias_counter(i) = bias_counter(i-1) + 10000; }

        nonBiasStates initEst(Z_5x1);
        nonBiasStates between_nonBias_State(Z_5x1);

        Values initial_values, result;

        noiseModel::Diagonal::shared_ptr nonBias_InitNoise = noiseModel::Diagonal::Variances((gtsam::Vector(5) << 0.2, 0.2, 0.2, 3e6, 1e-1).finished());

        noiseModel::Diagonal::shared_ptr nonBias_Reset = noiseModel::Diagonal::Variances((gtsam::Vector(5) << 3e4, 3e4, 3e4, 3e6, 1e-1).finished());

        noiseModel::Diagonal::shared_ptr nonBias_ProcessNoise = noiseModel::Diagonal::Variances((gtsam::Vector(5) << 1.0, 1.0, 1.0, 1e3, 1e-3).finished());

        noiseModel::Diagonal::shared_ptr initNoise = noiseModel::Diagonal::Variances((gtsam::Vector(1) << 3e6).finished());

        NonlinearFactorGraph *graph = new NonlinearFactorGraph();

        residuals.setZero(1000,2);

        // For the l2 model, we will use a mixture model with 1 component
        Eigen::RowVectorXd m(2);
        m << 0.0, 0.0;

        // Add component 1. (i.e., the assumed error covariance)
        Eigen::MatrixXd c(2,2);
        c<< std::pow(rangeWeight,2), 0.0, 0.0, std::pow(phaseWeight,2);
        globalMixtureModel.push_back(boost::make_tuple(0, 0, 0.0, m, c));

        int lastStep = get<0>(data.back());

        std::vector<int> num_obs (1000, 0);

        for(unsigned int i = startEpoch; i < data.size(); i++ ) {


                auto start = high_resolution_clock::now();

                double gnssTime = get<0>(data[i]);
                int currKey = get<1>(data[i]);
                if (first_ob) {
                        first_ob=false;
                        startKey = currKey;
                        graph->add(PriorFactor<nonBiasStates>(X(currKey), initEst,  nonBias_InitNoise));
                        ++factor_count;
                        initial_values.insert(X(currKey), initEst);
                }
                int nextKey = get<1>(data[i+1]);
                int svn = get<2>(data[i]);
                Point3 satXYZ = get<3>(data[i]);
                double rho = get<4>(data[i]);
                double range = get<5>(data[i]);
                double phase = get<6>(data[i]);
                double phase_break = get<7>(data[i]);

                gtsam::Vector2 obs;
                obs << range-rho, phase-rho;

                if ( currKey == startKey || phase_arc[svn]!=phase_break)
                {
                        bias_state[0] = phase-range;
                        if (currKey > startKey) { bias_counter[svn] = bias_counter[svn] +1; }
                        initial_values.insert(G(bias_counter[svn]), bias_state);

                        graph->add(boost::make_shared<PriorFactor<phaseBias> >(G(bias_counter[svn]), bias_state,  initNoise));

                        phase_arc[svn] = phase_break;
                        ++factor_count;
                }

                graph->add(boost::make_shared<GNSSMultiModalFactor>(X(currKey), G(bias_counter[svn]), obs, satXYZ, nomXYZ, globalMixtureModel));

                prn_vec.push_back(svn);
                factor_count_vec.push_back(++factor_count);


                //---------------------------------------------------------------------
                // add between factor here

                // if (i != startEpoch){
                //     int prevKey = get<1>(data[i-1]);
                //     double prevgnssTime = get<0>(data[i-1]);
                //
                //     for (int j = 0; j < zupt_tags.size()-1; j++){
                //         if ((std::abs(zupt_tags[j] - prevgnssTime) < 0.01) && (std::abs(zupt_tags[j+1] - gnssTime) < 0.01)){
                //             graph->add(BetweenFactor<nonBiasStates>(X(currKey),X(prevKey), between_nonBias_State, zuptNoise));
                //             ++factor_count;
                //             num_zupts = num_zupts+1;
                //             cout << " Zupt applied -- " << num_zupts <<  " between times " << prevgnssTime << " <--> " << gnssTime <<  endl;
                //         }
                //     }
                // }

                //---------------------------------------------------------------------

                if (currKey != nextKey && nextKey != 0) {

                        if (currKey > startKey ) {
                                if ( lastStep == nextKey ) { break; }


                                if (state_count < 600 || state_count > (lastStep-600))
                                {
                                        graph->add(PriorFactor<nonBiasStates>(X(currKey), initEst,  nonBias_InitNoise));

                                        ++factor_count;
                                }

                                graph->add(PriorFactor<nonBiasStates>(X(currKey), prior_nonBias, nonBias_Reset));
                                ++factor_count;

                        }

                        isam.update(*graph, initial_values);
                        isam.update();
                        isam.update();
                        result = isam.calculateEstimate();

                        prior_nonBias = result.at<nonBiasStates>(X(currKey));
                        Point3 delta_xyz = (gtsam::Vector(3) << prior_nonBias.x(), prior_nonBias.y(), prior_nonBias.z()).finished();
                        prop_xyz = nomXYZ - delta_xyz;

                        if (printECEF) {
                                out_os << gnssTime << " " << prop_xyz.x() << " " << prop_xyz.y() << " " << prop_xyz.z() << endl;
                        }

                        if (printENU) {
                                Point3 enu = xyz2enu(prop_xyz, nomXYZ);
                                out_os << "enu " << gnssTime << " " << enu.x() << " " << enu.y() << " " << enu.z() << endl;
                        }

                        if (printAmb) {
                                out_os << "gps " << " " << gnssTime << " ";
                                for (int k=0; k<prn_vec.size(); k++) {
                                        out_os << result.at<phaseBias>(G(bias_counter[prn_vec[k]])) << " ";
                                }
                                out_os << endl;
                        }

                        ++state_count;
                        output_time = output_time +1;

                        factor_count_vec.clear();
                        factor_count = -1;

                        graph->resize(0);
                        initial_values.clear();
                        prn_vec.clear();

                        auto stop = high_resolution_clock::now();
                        auto duration = duration_cast<microseconds>(stop - start);

                        // cout << "Delta time: "
                        //      << duration.count() << " microseconds" << endl;

                        initial_values.insert(X(nextKey), prior_nonBias);
                }

        }

        return 0;
}
