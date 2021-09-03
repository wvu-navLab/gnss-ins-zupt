/**
 * @file   GnssData.cpp
 * @brief  Tools required to read/write GNSS data
 * @author Ryan Watson
 */

#include <iomanip>      // std::setprecision
#include <gtsam/gnssNavigation/GnssData.h>
#include <random>

using namespace std;

namespace gtsam {

vector<rnxData> readGNSS(const std::string &fileLoc) {
        /*
           inputs ::
           fileLoc ---> path to data file
           output ::
           data ---> gnss data in gtsam format
                           { epoch, svn, satXYZ, computed_range, rangeLC, phaseLC }
         */
        vector<rnxData> data;
        // string data_file = findExampleDataFile(fileLoc);
        // ifstream is(data_file.c_str());
        ifstream is(fileLoc.c_str());

        while (is) {
                string satType;
                int svn, count;
                double break_flag, grav_delay, windup, satPC, trop_slant, c1Del, c2Del;
                double week, sow, satX, satY, satZ, rho, cb, rel, rangeLC, phaseLC;
                Point3 satXYZ, computed_range;
                string constellation;
                // is >> week >> sow  >> count >> constellation
                is >> week >> sow  >> count
                >> satType >> svn >> rangeLC >> phaseLC
                >> rho >> cb >> rel >> grav_delay >> trop_slant >>  windup >> satPC >> satX >> satY >> satZ >> break_flag >> c1Del >> c2Del;
                data.push_back(rnxData(sow, count, svn,Point3(satX,satY,satZ),
                                       (rho - cb  + rel  + grav_delay + trop_slant - satPC), (rangeLC - c1Del + c2Del), (phaseLC - windup*0.017), break_flag));
                // 0.01702215881 == LC wavelength/2*pi
        }
        is.clear();         /* clears the end-of-file and error flags */
        return data;
}


vector<rnxData> readGNSS_SingleFreq(const std::string &fileLoc) {
        /*
           inputs ::
           fileLoc ---> path to data file
           output ::
           data ---> gnss data in gtsam format
                           { epoch, svn, satXYZ, computed_range, rangeLC, phaseLC }
         */
        vector<rnxData> data;
        // string data_file = findExampleDataFile(fileLoc);
        // ifstream is(data_file.c_str());
        ifstream is(fileLoc.c_str());

        while (is) {
                string satType;
                int svn, count;
                double break_flag, grav_delay, windup, satPC, trop_slant, c1Del, c2Del;
                double week, sow, satX, satY, satZ, rho, cb, rel, range, phase, iono_slant;
                Point3 satXYZ, computed_range;
                string constellation;
                // is >> week >> sow  >> count >> constellation
                is >> week >> sow  >> count
                >> satType >> svn >> range >> phase
                >> rho >> cb >> rel >> grav_delay
                >> trop_slant >>  iono_slant >> windup >> satPC >> satX
                >> satY >> satZ >> break_flag >> c1Del;
                data.push_back(rnxData(sow, count, svn,Point3(satX,satY,satZ),
                                       (rho - cb  + rel  + grav_delay + trop_slant - iono_slant - satPC), (range - c1Del), (phase - windup*0.017), break_flag ));
                // 0.01702215881 == LC wavelength/2*pi
        }
        is.clear();         /* clears the end-of-file and error flags */
        return data;
}


vector<rnxDataWEl> readGNSS_SingleFreqTmp(const std::string &fileLoc) {
        /*
           inputs ::
           fileLoc ---> path to data file
           output ::
           data ---> gnss data in gtsam format
                           { epoch, svn, satXYZ, computed_range, rangeLC, phaseLC }
         */
        vector<rnxDataWEl> data;
        // string data_file = findExampleDataFile(fileLoc);
        // ifstream is(data_file.c_str());
        ifstream is(fileLoc.c_str());

        while (is) {
                string satType;
                int svn, count;
                double grav_delay, windup, satPC, trop_slant, c1Del, c2Del, el, break_flag;
                double week, sow, satX, satY, satZ, rho, cb, rel, range, phase, iono_slant;
                Point3 satXYZ, computed_range;
                string constellation;
                // is >> week >> sow  >> count >> constellation
                is >> week >> sow  >> count
                >> satType >> svn >> range >> phase
                >> rho >> cb >> rel >> grav_delay
                >> trop_slant >>  iono_slant >> windup >> satPC >> satX
                >> satY >> satZ >> el >> break_flag >> c1Del;
                data.push_back(rnxDataWEl(sow, count, svn,Point3(satX,satY,satZ),
                                          (rho - cb  + rel  + grav_delay + trop_slant - iono_slant - satPC), (range - c1Del), (phase - windup*0.017), break_flag, el ));
                // 0.01702215881 == LC wavelength/2*pi
        }
        is.clear();         /* clears the end-of-file and error flags */
        return data;
}


vector<faultyRnxData> readGNSSFaulty(const std::string &fileLoc, const double &mean, const double &stdDev, const double &percentFaulty) {
        /*
           inputs ::
           fileLoc ---> path to data file
           mean --> mean of distribution to generate observation faults
           stdDev --> 1 sigma of observation fault distribution
           percentFaulty --> number of faults to add to data set. scale == [0,1]
           output ::
           data ---> gnss data in gtsam format
                           { epoch, svn, satXYZ, computed_range, rangeLC, phaseLC }
         */
        vector<faultyRnxData> data;
        // string data_file = findExampleDataFile(fileLoc);
        // ifstream is(data_file.c_str());
        ifstream is(fileLoc.c_str());

        std::default_random_engine gen1;
        std::default_random_engine gen2;
        std::normal_distribution<double> dist1(mean, stdDev);
        std::normal_distribution<double> dist2(mean, stdDev/10.0);

        double rangeMag, phaseMag;

        srand( (unsigned)time( NULL ) );

        string satType;
        int svn, count, numFault=0, faultInd=0;
        double break_flag, grav_delay, windup, satPC, trop_slant, c1Del, c2Del;
        double week, sow, satX, satY, satZ, rho, cb, rel, rangeLC, phaseLC;
        Point3 satXYZ, computed_range;
        while (is) {
                is >> week >> sow  >> count
                >> satType >> svn >> rangeLC >> phaseLC
                >> rho >> cb >> rel >> grav_delay >> trop_slant >>  windup >> satPC >> satX >> satY >> satZ >> break_flag >> c1Del >> c2Del;
                if ( ((float) rand()/RAND_MAX) < percentFaulty) {
                        if ( ((double) rand() / (RAND_MAX)) > 0.5 ) {
                                rho += dist1(gen1);
                                // rangeLC += dist1(gen1);
                                // phaseLC += dist2(gen2);
                        }
                        else {
                                rho -= dist1(gen1);
                                // rangeLC -= dist1(gen1);
                                // phaseLC -= dist2(gen2);
                        }
                        faultInd = 1;
                        numFault++;
                }
                data.push_back(faultyRnxData(sow, count, svn,Point3(satX,satY,satZ),
                                             (rho - cb  + rel  + grav_delay + trop_slant - satPC), (rangeLC - c1Del + c2Del), (phaseLC - windup*0.017), break_flag, faultInd));
                faultInd = 0;
        }
        is.clear();         /* clears the end-of-file and error flags */
        return data;
}

vector<faultyRnxData> readGNSSOracle(const std::string &fileLoc, const double &mean, const double &stdDev, const double &percentFaulty) {
        /*
           inputs ::
           fileLoc ---> path to data file
           mean --> mean of distribution to generate observation faults
           stdDev --> 1 sigma of observation fault distribution
           percentFaulty --> number of faults to add to data set. scale == [0,1]
           output ::
           data ---> gnss data in gtsam format
                           { epoch, svn, satXYZ, computed_range, rangeLC, phaseLC }
         */
        vector<faultyRnxData> data;
        string data_file = findExampleDataFile(fileLoc);
        ifstream is(data_file.c_str());

        std::default_random_engine generator;
        std::normal_distribution<double> dist1(mean, stdDev);

        double rangeMag, phaseMag;

        srand( (unsigned)time( NULL ) );

        int svn, count, numFault=0, faultInd=0;
        double break_flag, grav_delay, windup, satPC, trop_slant, c1Del, c2Del;
        double week, sow, satX, satY, satZ, rho, cb, rel, rangeLC, phaseLC;
        Point3 satXYZ, computed_range;
        while (is) {
                is >> week >> sow  >> count
                >> svn >> rangeLC >> phaseLC
                >> rho >> cb >> rel >> grav_delay >> trop_slant >>  windup >> satPC >> satX >> satY >> satZ >> break_flag >> c1Del >> c2Del;
                if ( ((float) rand()/RAND_MAX) > percentFaulty) {
                        data.push_back(faultyRnxData(sow, count, svn, Point3(satX,satY,satZ),
                                                     (rho - cb  + rel  + grav_delay + trop_slant - satPC), (rangeLC - c1Del + c2Del), (phaseLC - windup*0.017), break_flag, faultInd));
                        faultInd = 0;
                }
                else { continue; }
        }
        is.clear();         /* clears the end-of-file and error flags */
        return data;
}
void writeStates(Values &results, vector<string> timeIndex, string outputFile){
        /*
           inputs ::
           results -->
           outputFile --> name of file to write state est. to. [string]
         */
        ofstream outFile(outputFile.c_str());
        // outFile << "/** GNSS State Vector\n\n *time (sec), dx, dy, dz, dCB, dTz \n\n **/" << endl;
        int epoch = 0;
        Values::ConstFiltered<nonBiasStates> result_poses = results.filter<nonBiasStates>();
        foreach (const Values::ConstFiltered<nonBiasStates>::KeyValuePair& key_value, result_poses)
        {
                int index = epoch++;
                nonBiasStates p = key_value.value;
                outFile << timeIndex[index]
                        << " "  << p.x() << " " << p.y()
                        << " "  << p.z() << " " << p.cb()
                        << " " << p.tz() << endl;
        }
}

void writeNavFrame(Values &results, Point3 &nom, vector<string> timeIndex, string outputFile){
        ofstream outFile(outputFile.c_str());
        // outFile << "/** ENU\n\n *time (sec), e (m), n (m), u (m)\n\n **/" << endl;
        int epoch = 0;
        Values::ConstFiltered<nonBiasStates> result_poses = results.filter<nonBiasStates>();
        foreach (const Values::ConstFiltered<nonBiasStates>::KeyValuePair& key_value, result_poses)
        {
                nonBiasStates p = key_value.value;
                Point3 delta(p.x(),p.y(),p.z());
                Point3 ecef = (nom - delta);
                Point3 enu = xyz2enu(ecef,nom);
                int index = epoch++;
                outFile << timeIndex[index] << " " << enu.x()
                        << " " << enu.y() << " " << enu.z() << endl;

        }
}

void writeEarthFrame(Values &results, Point3 &nom, vector<string> timeIndex, string outputFile){
        ofstream outFile(outputFile.c_str());
        // outFile << "/** ECEF \n\n *time (sec), prn, X (m), Y (m), Z (m) \n\n **/" << endl;
        int epoch = 0;
        Values::ConstFiltered<nonBiasStates> result_poses = results.filter<nonBiasStates>();
        foreach (const Values::ConstFiltered<nonBiasStates>::KeyValuePair& key_value, result_poses)
        {
                nonBiasStates p = key_value.value;
                Point3 delta(p.x(),p.y(),p.z());
                Point3 ecef = (nom - delta);
                int index = epoch++;
                outFile << timeIndex[index] << " " << std::setprecision(10) << ecef.x()
                        << " " << ecef.y() << " " << ecef.z() << endl;

        }
}

void writeSwitches( Values &results, string outputFile, vector<string> switchIndex){
        /*
           inputs ::
           results --> optimizer output
           outputFile --> name of file to write switch states to [string]
           Optional ::
           switchIndex --> index by epoch and visible satellite (i.e. obs 4 would be Switch_0_4) [vector]
         */
        ofstream outFile(outputFile.c_str());
        // outFile << "/** Switch Values\n\n *time (sec), prn, switch \n\n **/" << endl;
        int epoch = 0;
        Values::ConstFiltered<SwitchVariableLinear> result_switches = results.filter<SwitchVariableLinear>();
        foreach (const Values::ConstFiltered<SwitchVariableLinear>::KeyValuePair& key_value, result_switches) {
                int index = epoch++;
                outFile << switchIndex[index] << " " <<  key_value.value.value() << endl;
        }
}


void writeSwitchPair( Values &results, string outputFile, vector<string> switchIndex){
        /*
           inputs ::
           results --> optimizer output
           outputFile --> name of file to write switch states to [string]
           Optional ::
           switchIndex --> index by epoch and visible satellite (i.e. obs 4 would be Switch_0_4) [vector]
         */
        ofstream outFile(outputFile.c_str());
        // outFile << "/** Switch Values \n\n *time (sec), prn, range switch, phase switch \n\n **/" << endl;
        int epoch = 0;
        Values::ConstFiltered<SwitchPairLinear> result_switches = results.filter<SwitchPairLinear>();
        foreach (const Values::ConstFiltered<SwitchPairLinear>::KeyValuePair& key_value, result_switches) {
                int index = epoch++;
                outFile << switchIndex[index] << " " << key_value.value.a() << " " << key_value.value.b() << endl;
        }
}


void writeAmbiguity(Values &results, string outputFile, vector<string> satIndex){
        ofstream outFile(outputFile.c_str());
        // outFile << "/** \n\n *time (sec), prn, phase bias \n\n **/" << endl;
        int epoch = 0;
        Values::ConstFiltered<phaseBias> result_bias = results.filter<phaseBias>();
        foreach (const Values::ConstFiltered<phaseBias>::KeyValuePair& key_value, result_bias)
        {
                int index = epoch++;
                phaseBias p = key_value.value;
                outFile << satIndex[index] <<  " " << p.value() << endl;

        }
}

}
