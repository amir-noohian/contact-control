#include "joint_computed_torque/logging.hpp"
#include "joint_computed_torque/log_path.hpp"
#include <fstream>
#include <cassert>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>
#include <cstdio>

template<size_t N> void check() {
    using Log=joint_computed_torque::TrackingLog<N>;
    typename Log::jp_type qd(0.3), q(0.1);
    typename Log::jv_type vd(0.4), v(0.1);
    typename Log::ja_type ad(0.5);
    char path[]="/tmp/tracking_test_XXXXXX";
    int fd=mkstemp(path); assert(fd>=0); close(fd);
    {
        barrett::log::Writer<typename Log::Record> writer(path);
        writer.putRecord(typename Log::Record(1.25,qd,q,vd,v,ad));
        writer.close();
    }
    std::ostringstream out;
    Log::exportCsv(path,out);
    std::remove(path);
    std::istringstream rows(out.str());
    std::string header,row,cell;
    std::getline(rows,header); std::getline(rows,row);
    assert(header.find("ddq_des_rad_s2_j"+std::to_string(N))!=std::string::npos);
    assert(header.find("ddq_est") == std::string::npos);
    assert(header.find("ddq_error") == std::string::npos);
    std::istringstream cells(row);
    std::vector<double> vals;
    while(std::getline(cells,cell,',')) vals.push_back(std::stod(cell));
    assert(vals.size()==1+7*N && vals[0]==1.25);
    for (size_t j=0;j<N;++j) {
        assert(std::abs(vals[1+5*N+j]-.2)<1e-12);
        assert(std::abs(vals[1+6*N+j]-.3)<1e-12);
        assert(std::abs(vals[1+4*N+j]-.5)<1e-12);
    }

}
int main() {
    check<4>(); check<7>();
    char directory[]="/tmp/tracking_names_XXXXXX";
    assert(mkdtemp(directory));
    using joint_computed_torque::reserveTrackingCsv;
    const std::string first=reserveTrackingCsv(directory, "dyncomp_tracking");
    const std::string second=reserveTrackingCsv(directory, "dyncomp_tracking");
    const std::string comptorq=reserveTrackingCsv(directory, "comptorq_tracking");
    assert(first==std::string(directory)+"/dyncomp_tracking_0001.csv");
    assert(second==std::string(directory)+"/dyncomp_tracking_0002.csv");
    assert(comptorq==std::string(directory)+"/comptorq_tracking_0001.csv");
    std::ofstream(second) << "preserve";
    std::remove(first.c_str());
    const std::string third=reserveTrackingCsv(directory, "dyncomp_tracking");
    assert(third==std::string(directory)+"/dyncomp_tracking_0003.csv");
    std::string contents;
    std::ifstream(second) >> contents;
    assert(contents=="preserve");
    std::remove(second.c_str()); std::remove(third.c_str());
    std::remove(comptorq.c_str());
    assert(rmdir(directory)==0);
}
