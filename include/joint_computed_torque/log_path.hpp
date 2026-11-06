#pragma once
#include <cerrno>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <iomanip>
#include <limits>
#include <memory>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace joint_computed_torque {
// Reserve a new CSV atomically, starting above the largest existing run number.
inline std::string reserveTrackingCsv(const std::string& directory,
                                     const std::string& prefix) {
    std::unique_ptr<DIR, int(*)(DIR*)> entries(opendir(directory.c_str()), closedir);
    if (!entries) throw std::system_error(errno, std::generic_category(), "Open log directory");
    const std::regex pattern(prefix + "_([0-9]+)\\.csv");
    unsigned long long largest = 0;
    while (true) {
        errno = 0;
        const dirent* entry = readdir(entries.get());
        if (!entry) {
            if (errno) throw std::system_error(errno, std::generic_category(), "Read log directory");
            break;
        }
        std::smatch match;
        const std::string name(entry->d_name);
        if (std::regex_match(name, match, pattern))
            largest = std::max(largest, std::stoull(match[1].str()));
    }
    for (;;) {
        if (largest == std::numeric_limits<unsigned long long>::max())
            throw std::overflow_error("Tracking log numbers exhausted");
        std::ostringstream path;
        path << directory << "/" << prefix << "_" << std::setfill('0')
             << std::setw(4) << ++largest << ".csv";
        const int fd = open(path.str().c_str(), O_WRONLY | O_CREAT | O_EXCL, 0664);
        if (fd >= 0) { close(fd); return path.str(); }
        if (errno != EEXIST)
            throw std::system_error(errno, std::generic_category(), "Create tracking CSV");
    }
}
}
