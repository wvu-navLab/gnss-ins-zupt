/*
 *  @file   rnxToGtsam_singleFreq.cpp
 *  @author Ryan Watson
 *  @brief  Script to convert Rinex File format to format utilized by GTSAM.

 * To do ::
 * Currenlty only works with SP3
 * Need to get nom. pos. from rinex header
 * Get DOY from header for trop estimation
 * need to write to file. Currently only prints to screen.
 */

// GPSTK
#include <gpstk/MJD.hpp>
// #include <gpstk/ModelObs.hpp>
#include <gpstk/PowerSum.hpp>
#include <gpstk/Decimate.hpp>
#include <gpstk/IonoModel.hpp>
#include <gpstk/TropModel.hpp>
#include <gpstk/BasicModel.hpp>
#include <gpstk/CommonTime.hpp>
#include <gpstk/PCSmoother.hpp>
#include <gpstk/IonexModel.hpp>
#include <gpstk/CodeSmoother.hpp>
#include <gpstk/SimpleFilter.hpp>
#include <gpstk/MWCSDetector.hpp>
#include <gpstk/SatArcMarker.hpp>
#include <gpstk/DCBDataReader.hpp>
#include <gpstk/ComputeWindUp.hpp>
#include <gpstk/Rinex3NavData.hpp>
#include <gpstk/RinexNavData.hpp>
#include <gpstk/GNSSconstants.hpp>
#include <gpstk/ComputeLinear.hpp>
#include <gpstk/GPSWeekSecond.hpp>
#include <gpstk/LICSDetector2.hpp>
#include <gpstk/IonoModelStore.hpp>
#include <gpstk/RinexNavStream.hpp>
#include <gpstk/DataStructures.hpp>
#include <gpstk/Rinex3ObsStream.hpp>
#include <gpstk/RinexObsStream.hpp>
#include <gpstk/Rinex3NavStream.hpp>
#include <gpstk/ComputeIonoModel.hpp>
#include <gpstk/ComputeTropModel.hpp>
#include <gpstk/OneFreqCSDetector.hpp>
#include <gpstk/SP3EphemerisStore.hpp>
#include <gpstk/ComputeSatPCenter.hpp>
#include <gpstk/EclipsedSatFilter.hpp>
#include <gpstk/GPSEphemerisStore.hpp>
#include <gpstk/CorrectCodeBiases.hpp>
#include <gpstk/ComputeSatPCenter.hpp>
#include <gpstk/RequireObservables.hpp>
#include <gpstk/CorrectObservables.hpp>
#include <gpstk/LinearCombinations.hpp>
#include <gpstk/GravitationalDelay.hpp>
#include <gpstk/PhaseCodeAlignment.hpp>

// BOOST
#include <boost/program_options.hpp>

// STD
#include <iomanip>
#include <iostream>

using namespace std;
using namespace gpstk;
using namespace boost;

namespace po = boost::program_options;

int main(int argc, char *argv[])
{

        double break_thresh;
        bool usingP1 = false;
        int dec_int, itsBelowThree = 0, count = 0, break_window;
        string rnx_file, nav_file, sp3_file, out_file, iono_file, brdc_nav_file;

        cout << fixed << setprecision(12); // Set a proper output format

        po::options_description desc("Available options");
        desc.add_options()
                ("help,h", "Print help message")
                ("obs", po::value<string>(&rnx_file)->default_value(""),
                "Observation file to read")
                ("sp3", po::value<string>(&sp3_file)->default_value(""),
                "SP3 file to read.")
                ("brdc_nav", po::value<string>(&brdc_nav_file)->default_value(""),
                "Broadcast nav file to read.")
                ("iono", po::value<string>(&iono_file)->default_value(""),
                "IonoMap file to read.")
                ("break_window",  po::value<int>(&break_window)->default_value(1500), "Size of window (in samples) to check for cycle-slips")
                ("break_thresh",  po::value<double>(&break_thresh)->default_value(5.5), "deviation magnitude to classify as phase break")
                ("usingP1", "Are you using C1 instead of P1?")
                ("dec", po::value<int>(&dec_int)->default_value(0),
                "decimate input obs file");
        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
        po::notify(vm);

        usingP1 = (vm.count("usingP1")>0);

        if ( rnx_file.empty() )
        {
                cout << " Must pass in obs file !!! Try --obs " << desc << endl;
                exit(1);
        }
        if ( nav_file.empty() && sp3_file.empty() )
        {
                cout << " Must pass in ephemeris file !!! Try --sp3 or --nav" << desc << endl;
                exit(1);
        }

        // std::cout << "Before reading rinex file" << endl;
        // Create the input observation file stream
        RinexObsStream rin(rnx_file);

        // std::cout << "After reading rinex file" << endl;

        // Declare a "SP3EphemerisStore" object to handle precise ephemeris
        SP3EphemerisStore SP3EphList;

        // Set flags to reject satellites with bad or absent positional
        // values or clocks
        SP3EphList.rejectBadPositions(true);
        SP3EphList.rejectBadClocks(true);

        // Load all the SP3 ephemerides files
        // std::cout << "Before reading sp3 file" << endl;
        // std::cout << " sp3 file -- " << sp3_file << endl;

        SP3EphList.loadFile(sp3_file);

        // std::cout << "After reading sp3 file" << endl;
        // BELL station nominal position
        // Nom. pos. for the greenhouse dataset.
        //Position nominalPos(859154.0695, -4836304.2164, 4055377.5475);// nominal pose for t9
        // Position nominalPos(859153.0167,-4836303.7245,4055378.4991);// nominal pose for t10
        Position nominalPos(859156.4189, -4836305.5491, 4055375.2899);// nominal pose for t11
        // Position nominalPos(856509.2570,-4843016.7781, 4047938.0419); // outside Maria
        // Position nominalPos(856651.3014 -4843072.0103 4047854.2071); //greenhouse Maria
        // Nom. pos. for the dec12 dataset.
        // Position nomXYZ(856295.3346, -4843033.4111, 4048017.6649);

        CorrectCodeBiases corrCode;
        corrCode.setDCBFile("/home/navlab-shounak/Desktop/Fusion/t10/p1p2.DCB", "/home/navlab-shounak/Desktop/Fusion/t10/p1c1.DCB");

        if (!usingP1) {
                corrCode.setUsingC1(true);
        }


        // This is the GNSS data structure that will hold all the
        // GNSS-related information
        gnssRinex gRin;

        RequireObservables requireObs;
        requireObs.addRequiredType(TypeID::L1);
        // requireObs.addRequiredType(TypeID::L2);

        SimpleFilter pObsFilter;
        pObsFilter.setFilteredType(TypeID::C1);

        if ( usingP1 )
        {
                requireObs.addRequiredType(TypeID::P1);
                pObsFilter.addFilteredType(TypeID::P1);
        }
        else
        {
                requireObs.addRequiredType(TypeID::C1);
                pObsFilter.addFilteredType(TypeID::C1);
        }

        // Object to correct for SP3 Sat Phase-center offset
        AntexReader antexread;
        antexread.open( "/home/navlab-shounak/Desktop/Fusion/t10/antenna_corr.atx" );
        ComputeSatPCenter svPcenter(SP3EphList, nominalPos);
        svPcenter.setAntexReader( antexread );

        // Declare an object to correct observables
        CorrectObservables corr(SP3EphList);

        // object def several linear combinations
        LinearCombinations comb;

        // Setup single-freq cycle-slip detection
        OneFreqCSDetector markCSC1(TypeID::C1);
        markCSC1.setMaxNumSigmas(break_thresh);
        markCSC1.setMaxWindowSize(break_window);
        markCSC1.setDeltaTMax(0.11);

        // Object to keep track of satellite arcs
        SatArcMarker markArc;
        markArc.setDeleteUnstableSats(false);
        // markArc.setUnstablePeriod(10.0);

        // Objects to compute gravitational delay effects
        GravitationalDelay grDelay(nominalPos);

        // Object to align phase with code measurements
        PhaseCodeAlignment phaseAlign;

        // Object to remove eclipsed satellites
        EclipsedSatFilter eclipsedSV;

        //Object to compute wind-up effect
        ComputeWindUp windup( SP3EphList,
                              nominalPos );


        // Object to compute prefit-residuals
        ComputeLinear linear3(comb.pcPrefit);
        linear3.addLinear(comb.lcPrefit);

        CodeSmoother smoothC1;

        TypeIDSet tset;
        tset.insert(TypeID::prefitC);
        tset.insert(TypeID::prefitL);

        // Declare a NeillTropModel object, setting the defaults
        NeillTropModel neillTM(355, 39.09, 355);
        ComputeTropModel computeTropo(neillTM);

        // Objects to compute the iono data
        // IonexStore IonexMapList;
        // IonexMapList.loadFile(iono_file);
        // IonexModel ionex(nominalPos, IonexMapList);

        ComputeIonoModel computeIono(nominalPos);
        RinexNavHeader rNavHeader;
        RinexNavStream rnavin(brdc_nav_file.c_str());
        rnavin >> rNavHeader;
        computeIono.setKlobucharModel(rNavHeader.ionAlpha,rNavHeader.ionBeta);

        SimpleFilter pcFilter;

        // Declare a couple of basic modelers
        BasicModel basic(nominalPos, SP3EphList);

        // Loop over all data epochs
        while(rin >> gRin)
        {
                CommonTime gpsT(gRin.header.epoch);
                gpsT.setTimeSystem(TimeSystem::GPS);
                GPSWeekSecond gpstime( gpsT );

                try
                {
                        gRin
                        >> requireObs // Check if required observations are present
                        >> pObsFilter // Filter out spurious data
                        >> markCSC1 // Mark cycle slips
                        >> markArc // Keep track of satellite arcs
                        >> basic // Compute the basic components of model
                        >> eclipsedSV // Remove satellites in eclipse
                        >> svPcenter // Computer delta for sat. phase center
                        >> corr // SP3 Corrections
                        >> corrCode // Correct for differential code biases
                        >> windup // phase windup correction
                        >> grDelay // Compute gravitational delay
                        >> computeTropo // Compute slant trop. for L1 --- neill trop function
                        >> computeIono;  // Compute slant iono. for L1
                        // >> phaseAlign
                        // >> pcFilter;  // screen c1

                }
                catch(Exception& e)
                {
                        //cerr << "Exception at epoch: " << time << "; " << e << endl;
                        continue;
                }
                catch(...)
                {
                        cerr << "Unknown exception at epoch: " << time << endl;
                        continue;
                }
                TypeIDSet types;
                types.insert(TypeID::satX);
                types.insert(TypeID::satY);
                types.insert(TypeID::satZ);
                types.insert(TypeID::C1);
                types.insert(TypeID::L1);
                types.insert(TypeID::rho);
                types.insert(TypeID::tropo);
                types.insert(TypeID::tropoSlant);
                types.insert(TypeID::ionoL1);
                types.insert(TypeID::dtSat);
                types.insert(TypeID::rel);
                types.insert(TypeID::gravDelay);
                types.insert(TypeID::instC1);
                types.insert(TypeID::satArc);
                types.insert(TypeID::satPCenter);
                types.insert(TypeID::windUp);
                gRin.keepOnlyTypeID(types);

                // Iterate through the GNSS Data Structure
                satTypeValueMap::const_iterator it;
                typeValueMap::const_iterator itObs;
                if (gRin.numSats() >= 4)
                {
                        // if ( itsBelowThree > 0 )
                        // {
                        // itsBelowThree = 0;
                        // continue;
                        // }
                        for (it = gRin.body.begin(); it!= gRin.body.end(); it++)
                        {
                                cout << gpstime.week << " ";
                                cout << gpstime.sow << " ";
                                cout << count << " ";
                                cout << (*it).first << " ";

                                typeValueMap::const_iterator itObs;
                                for( itObs  = (*it).second.begin(); itObs != (*it).second.end(); itObs++ )
                                {
                                        // cout << (*itObs).first << " ";
                                        cout << (*itObs).second << " ";
                                }
                                cout << endl;
                        }
                        count++;
                }
                else { itsBelowThree++; }
        }
        return 0;
}
