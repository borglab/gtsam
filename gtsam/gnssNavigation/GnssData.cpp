/**
 * @file   GnssData.cpp
 * @brief  Tools required to read/write GNSS data
 * @author Ryan Watson
 */

#include <iomanip>      // std::setprecision
#include <gtsam/gnssNavigation/GnssData.h>

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
        string data_file = findExampleDataFile(fileLoc);
        ifstream is(data_file.c_str());

        while (is) {
                int svn, count;
                double break_flag, grav_delay, windup, satPC, trop_slant, c1Del, c2Del;
                double week, sow, satX, satY, satZ, rho, cb, rel, rangeLC, phaseLC;
                Point3 satXYZ, computed_range;
                string constellation;
                is >> week >> sow  >> count >> constellation
                >> svn >> rangeLC >> phaseLC
                >> rho >> cb >> rel >> grav_delay >> trop_slant >>  windup >> satPC >> satX >> satY >> satZ >> break_flag >> c1Del >> c2Del;
                data.push_back(rnxData(sow, count, svn,Point3(satX,satY,satZ),
                                       (rho - cb  + rel  + grav_delay + trop_slant - satPC), (rangeLC - c1Del + c2Del), (phaseLC - windup*0.017), break_flag));
                // 0.01702215881 == LC wavelength/2*pi
        }
        is.clear();         /* clears the end-of-file and error flags */
        return data;
}

void writeStates(Values &results, string outputFile){
        /*
           inputs ::
           results -->
           outputFile --> name of file to write state est. to. [string]
         */
        ofstream outFile(outputFile.c_str());
        int epoch = 0;
        Values::ConstFiltered<nonBiasStates> result_poses = results.filter<nonBiasStates>();
        foreach (const Values::ConstFiltered<nonBiasStates>::KeyValuePair& key_value, result_poses)
        {
                nonBiasStates p = key_value.value;
                outFile << "stateVec " << epoch++
                        << " "  << p.x() << " " << p.y()
                        << " "  << p.z() << " " << p.cb()
                        << " " << p.tz() << endl;
        }
}

void writeNavFrame(Values &results, Point3 &nom, string outputFile){
        ofstream outFile(outputFile.c_str());
        int epoch = 0;
        Values::ConstFiltered<nonBiasStates> result_poses = results.filter<nonBiasStates>();
        foreach (const Values::ConstFiltered<nonBiasStates>::KeyValuePair& key_value, result_poses)
        {
                nonBiasStates p = key_value.value;
                Point3 delta(p.x(),p.y(),p.z());
                Point3 ecef = (nom - delta);
                Point3 enu = xyz2enu(ecef,nom);
                outFile << epoch++ << " " << enu.x()
                        << " " << enu.y() << " " << enu.z() << endl;

        }
}

void writeEarthFrame(Values &results, Point3 &nom, string outputFile){
        ofstream outFile(outputFile.c_str());
        int epoch = 0;
        Values::ConstFiltered<nonBiasStates> result_poses = results.filter<nonBiasStates>();
        foreach (const Values::ConstFiltered<nonBiasStates>::KeyValuePair& key_value, result_poses)
        {
                nonBiasStates p = key_value.value;
                Point3 delta(p.x(),p.y(),p.z());
                Point3 ecef = (nom - delta);
                outFile << epoch++ << " " << std::setprecision(10) << ecef.x()
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
        int epoch = 0;
        Values::ConstFiltered<SwitchVariableLinear> result_switches = results.filter<SwitchVariableLinear>();
        foreach (const Values::ConstFiltered<SwitchVariableLinear>::KeyValuePair& key_value, result_switches) {
                int index = epoch++;
                outFile << switchIndex[index] << " "
                        << index << " " <<  key_value.value.value() << endl;
        }
}

void writeAmbiguity(Values &results, string outputFile, vector<string> satIndex){
        ofstream outFile(outputFile.c_str());
        int epoch = 0;
        Values::ConstFiltered<phaseBias> result_bias = results.filter<phaseBias>();
        foreach (const Values::ConstFiltered<phaseBias>::KeyValuePair& key_value, result_bias)
        {
                int index = epoch++;
                phaseBias p = key_value.value;
                outFile << satIndex[index] <<  " " << key_value.value.value() << endl;

        }
}

}
