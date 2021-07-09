/*
 * @file test_gnss_ice.cpp
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

// remove this later
Eigen::MatrixXd Tw_inv;
Eigen::RowVectorXd means;
// remove this later


int main(int argc, char* argv[])
{
        // define std out print color
        bool skipped_update(false);
        vector<int> prn_vec;
        vector<int> factor_count_vec;
        vector<rnxData> data;
        const string red("\033[0;31m");
        const string green("\033[0;32m");
        string confFile, gnssFile, station;
        double xn, yn, zn, range, phase, rho, gnssTime, prev_time;
        int ob_count(0), state_skip(0), tmp(0);
        int startKey(0), currKey, startEpoch(0), svn, numBatch(0), state_count(0), update_count(0);
        int nThreads(-1), phase_break, break_count(0), nextKey, factor_count(-1), res_count(-1);
        bool printECEF, printENU, printAmb, first_ob(true);
        Eigen::MatrixXd residuals, all_residuals;
        vector<mixtureComponents> globalMixtureModel;

        int num_zupts = 0;

        string res_str = "all.residuals";
        ofstream res_os(res_str);

        string res_out_str = "outliers.residuals";
        ofstream res_out_os(res_out_str);

        // string mean_str = "means.txt";
        // ofstream mean_os(mean_str);
        //
        // string cov_str = "covs.txt";
        // ofstream cov_os(cov_str);

        string out_file = "/home/navlab-shounak/Desktop/Fusion/clean_results_t9/ice_t9_w500_FmodCN.xyz";
        ofstream out_os(out_file);

        cout.precision(12);

        out_os.precision(12);

        po::options_description desc("Available options");
        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
        po::notify(vm);

        gnssFile = "/home/navlab-shounak/Desktop/Fusion/gtsam_data_t9/out9sat4F.gtsam";
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


        //---------------------------------------------------------------------

        //read the zupt times from a file (a set?, they can use the method count())

        // open file
        // ifstream inputFile("zupt_Tags_t11.txt");
        // vector<double> zupt_tags;
        //
        // // test file open
        // if (inputFile) {
        //     double val;
        //
        //     // read the elements in the file into a vector
        //     while ( inputFile >> val ) {
        //         zupt_tags.push_back(val);
        //     }
        // }
        // close the file

        //---------------------------------------------------------------------

        // read the ecef displacements from CoreNav
        std::vector<std::vector<double> > ecefCN;
        std::string line;
        double value2;

        int rowNum = 0;

        // read in matrix
        std::ifstream file2("ecefGtsamt9.txt");
        while(std::getline(file2, line)) {
                std::vector<double> row;
                std::istringstream iss(line);
                while(iss >> value2){
                        row.push_back(value2);
                }
                ecefCN.push_back(row);
                rowNum = rowNum + 1;
        }

        cout << " Number of rows in the ecefGtsamt9.txt file -- " << rowNum << endl;
        // open file
        // ifstream inputFile("ecefGtsamt9.txt");
        // vector<double> x, y, z;
        //
        // // test file open
        // if (inputFile) {
        //     double val1, val2, val3;
        //
        //     // read the elements in the file into a vector
        //     while ( inputFile >> val1 >> val2 >> val3 ) {
        //         x.push_back(val1);
        //         y.push_back(val2);
        //         z.push_back(val3);
        //     }
        // }
        // close the file

        //----------------------------------------------------------------------
        //outside Maria nominal ECEF values
        // xn = 856509.2570;
        // yn = -4843016.7781;
        // zn = 4047938.0419;

        //t9 nominal ECEF values
        xn = 859154.0695;
        yn = -4836304.2164;
        zn = 4055377.5475;

        // 859154.0695, -4836304.2164, 4055377.5475 - t9
        // 859153.0167, -4836303.7245, 4055378.4991 - t10
        // 859156.4189, -4836305.5491, 4055375.2899 - t11


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
        double rangeWeight = 0.5; //2.5;
        double phaseWeight = 0.05; //0.25

        ifstream file(gnssFile.c_str());
        string value;

        nonBiasStates prior_nonBias = (gtsam::Vector(5) << 0.0, 0.0, 0.0, 0.0, 0.0).finished();

        // nonBiasStates ( dx, dy, dz, trop, cb ) = 0 , zupt constraint

        // nonBiasStates zupt_nonBias = (gtsam::Vector(5) << 0.0, 0.0, 0.0, 0.0, 0.0).finished();

        // zupt noise model

        noiseModel::Diagonal::shared_ptr zuptNoise = noiseModel::Diagonal::Variances((gtsam::Vector(5) << 1e-5, 1e-5, 1e-5, 1e3, 1e-3).finished());

        noiseModel::Diagonal::shared_ptr corenavNoise = noiseModel::Diagonal::Variances((gtsam::Vector(5) << 1e-1, 1e-1, 1e-1, 1e3, 1e-3).finished());

        // non-zupt noise model

        noiseModel::Diagonal::shared_ptr non_zuptNoise = noiseModel::Diagonal::Variances((gtsam::Vector(5) << 4.0, 4.0, 4.0, 1e3, 1e-3).finished());

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

        residuals.setZero(500,2);

        // init. mixture model.
        // Init this from file later
        Eigen::RowVectorXd m(2);
        m << 0.0, 0.0;

        // Add comp 1.
        Eigen::MatrixXd c(2,2);
        c<< std::pow(rangeWeight,2), 0.0, 0.0, std::pow(phaseWeight,2);
        globalMixtureModel.push_back(boost::make_tuple(0, 0, 0.0, m, c));

        int lastStep = get<0>(data.back());

        std::vector<int> num_obs (500, 0);

        bool is_zupt = false;

        cout << " The number of epochs in the Gtsam data file -- " << data.size() << endl;

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

                ob_count = ob_count + 1;

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
                //

                if (i != startEpoch){
                    int prevKey = get<1>(data[i-1]);
                    double prevgnssTime = get<0>(data[i-1]);

                    std::vector<double> ecefcurr = ecefCN[i];
                    std::vector<double> ecefprev = ecefCN[i-1];

                    double xcurr = ecefcurr[0];
                    double ycurr = ecefcurr[1];
                    double zcurr = ecefcurr[2];

                    double xprev = ecefprev[0];
                    double yprev = ecefprev[1];
                    double zprev = ecefprev[2];

                    // for (int j = 0; j < zupt_tags.size()-1; j++){
                    //     if ((std::abs(zupt_tags[j] - prevgnssTime) < 0.01) && (std::abs(zupt_tags[j+1] - gnssTime) < 0.01)){
                    //         is_zupt = true;
                    //         graph->add(BetweenFactor<nonBiasStates>(X(currKey),X(prevKey), between_nonBias_State, zuptNoise));
                    //         ++factor_count;
                    //         num_zupts = num_zupts+1;
                    //         cout << "Zupt applied -- " << num_zupts <<  " between times " << prevgnssTime << " <--> " << gnssTime <<  endl;
                    //     }
                    // }
                    // if (is_zupt == false){

                    nonBiasStates corenav_nonBias = (gtsam::Vector(5) << xcurr-xprev, ycurr-yprev, zcurr-zprev, 0.0, 0.0).finished();

                    graph->add(BetweenFactor<nonBiasStates>(X(currKey),X(prevKey), corenav_nonBias, corenavNoise));
                    ++factor_count;
                    // }
                }

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
                        result = isam.calculateEstimate();


                        // Only learn from residuals which don't agree with the model
                        int ind(0);
                        double prob, probMax(0.0);
                        gtsam::Matrix cov_min(2,2);
                        Eigen::RowVectorXd mean_min(2);
                        Eigen::VectorXd res(2);

                        for (int j = 0; j<factor_count_vec.size(); j++)
                        {

                                res = graph->at(factor_count_vec[j])->residual(result);

                                for (int k=0; k<globalMixtureModel.size(); k++)
                                {
                                        merge::mixtureComponents mixtureComp = globalMixtureModel[k];

                                        Eigen::RowVectorXd mean = mixtureComp.get<3>();

                                        double quadform  = (res).transpose() * (mixtureComp.get<4>()).inverse() * (res);
                                        double norm = std::pow(std::sqrt(2 * M_PI),-1) * std::pow((mixtureComp.get<4>()).determinant(), -0.5);

                                        prob =  norm * exp(-0.5 * quadform);

                                        if (prob >= probMax)
                                        {
                                                ind = k;
                                                probMax = prob;
                                                cov_min = mixtureComp.get<4>();
                                                mean_min = mixtureComp.get<3>();
                                        }
                                }

                                res_os << res(0) << " " << res(1) << endl;

                                // Use z-test to see if residuals is considered an outlier
                                double z_r = (res(0))/std::sqrt(cov_min(0,0));
                                double z_p = (res(1))/std::sqrt(cov_min(1,1));

                                // only consider residuals more than 'n' stds from model
                                if (std::abs(z_r) > 3.0 || std::abs(z_p) > 3.0)
                                {
                                        ++res_count;
                                        if (res_count > 499 )
                                        {
                                                residuals.conservativeResize(residuals.rows()+1, residuals.cols());

                                                residuals.row(residuals.rows()-1) = res.transpose();

                                                res_out_os << res(0) << " " << res(1) << endl;

                                                graph->remove(factor_count_vec[j]);
                                                ob_count-=1;
                                        }
                                        else
                                        {
                                                residuals.block(res_count,0,1,2) << res.transpose();

                                                res_out_os << res(0) << " " << res(1) << endl;

                                                graph->remove(factor_count_vec[j]);
                                                ob_count-=1;
                                        }
                                }
                                else
                                {
                                        // if obs. match a model. Update the number of points in
                                        // that cluster.
                                        num_obs.at(ind) = num_obs.at(ind)+1;
                                }
                        }


                        initial_values.clear();
                        if (ob_count >= 5) {


                                ++tmp;
                                ++state_count;

                                isam.update(*graph);
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
                                state_skip = 1;
                                prev_time = gnssTime;

                        }
                        else{
                                ++state_skip;
                        }
                        output_time = output_time +1;

                        factor_count_vec.clear();
                        factor_count = -1;

                        if (res_count > 500)
                        {

                                StickBreak weights;
                                vector<GaussWish> clusters;
                                Eigen::MatrixXd qZ;

                                learnVDP(residuals, qZ, weights, clusters);

                                // update the number of obs in each component
                                globalMixtureModel = updateObs(globalMixtureModel, num_obs);
                                std::fill(num_obs.begin(), num_obs.end(), 0);

                                // merge the curr and prior mixture models.
                                globalMixtureModel = mergeMixtureModel(residuals, qZ, globalMixtureModel, clusters, weights, 0.05, 20);

                                // cout << "\n\n\n\n\n\n" << endl;
                                // cout << "----------------- Merged MODEL ----------------" << endl;
                                for (int i=0; i<globalMixtureModel.size(); i++)
                                {
                                        mixtureComponents mc = (globalMixtureModel)[i];
                                        auto cov = mc.get<4>();
                                        // cout << mc.get<0>() << " " << mc.get<1>() << " "  <<  mc.get<2>() << "    " << mc.get<3>() <<"     "<< cov(0,0) << " " << cov(0,1) << " " << cov(1,1) <<"     "<<"\n\n" << endl;
                                }

                                residuals.setZero(500,2);
                                res_count = -1;

                        }

                        graph->resize(0);
                        prn_vec.clear();

                        auto stop = high_resolution_clock::now();
                        auto duration = duration_cast<microseconds>(stop - start);

                        // cout << "Delta time: "
                        //      << duration.count() << " microseconds" << endl;

                        initial_values.insert(X(nextKey), prior_nonBias);
                        ob_count = 0;
                }

        }

        // cout << "\n\n\n\n\n\n" << endl;
        // cout << "----------------- Final Incremental Mixture MODEL ----------------" << endl;
        // for (int i=0; i<globalMixtureModel.size(); i++)
        // {
        //         mixtureComponents mc = globalMixtureModel[i];
        //
        //         Eigen::MatrixXd cov = mc.get<4>();
        //         Eigen::RowVectorXd mean = mc.get<3>();
        //         mean_os << mean(0) << " " << mean(1) << " " << mean(2) << " " << mean(3) << "\n";
        //         for (int i = 0; i < 2;i++){
        //           cov_os << cov(i,0) << " " << cov(i,1) << "\n" ;
        //         }
        //         // auto cov = mc.get<4>();
        //         // cout << mc.get<0>() << " " << mc.get<1>() << " "  <<  mc.get<2>() << "    " << mc.get<3>() <<"     "<< cov(0,0) << " " << cov(0,1) << " " << cov(1,1) <<"     "<<"\n\n" << endl;
        // }

        return 0;
}
