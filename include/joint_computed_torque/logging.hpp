#pragma once
#include <barrett/log.h>
#include <barrett/systems.h>
#include <barrett/units.h>
#include <ostream>

namespace joint_computed_torque {
template<size_t DOF>
struct TrackingLog {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    typedef barrett::systems::TupleGrouper<double, jp_type, jp_type, jv_type,
                                          jv_type, ja_type> Grouper;
    typedef typename Grouper::tuple_type Record;
    static void exportCsv(const char* binary, std::ostream& out) {
        barrett::log::Reader<Record> reader(binary);
        out << "time_s";
        const char* groups[] = {"q_des_rad", "q_meas_rad", "dq_des_rad_s", "dq_meas_rad_s",
            "ddq_des_rad_s2", "q_error_rad", "dq_error_rad_s"};
        for (const char* group : groups)
            for (size_t j=0; j<DOF; ++j) out << ',' << group << "_j" << j+1;
        out << '\n';
        out.precision(17);
        for (size_t n=0; n<reader.numRecords(); ++n) {
            const Record r=reader.getRecord();
            out << boost::get<0>(r);
            write(out, boost::get<1>(r)); write(out, boost::get<2>(r));
            write(out, boost::get<3>(r)); write(out, boost::get<4>(r));
            write(out, boost::get<5>(r));
            write(out, boost::get<1>(r)-boost::get<2>(r));
            write(out, boost::get<3>(r)-boost::get<4>(r));
            out << '\n';
        }
    }
private:
    template<typename T> static void write(std::ostream& out, const T& v) {
        for (size_t j=0; j<DOF; ++j) out << ',' << v[j];
    }
};
}
