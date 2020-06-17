#include <sstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#pragma once

namespace gtsam {
    template<class T>
    std::string serialize(const T& input) {
        std::ostringstream out_archive_stream;
        boost::archive::text_oarchive out_archive(out_archive_stream);
        out_archive << input;
        return out_archive_stream.str();
    }
    template<class T>
    T deserialize(const std::string& serialized, T& output) {
        std::istringstream in_archive_stream(serialized);
        boost::archive::text_iarchive in_archive(in_archive_stream);
        in_archive >> output;
        return output;
    }
}