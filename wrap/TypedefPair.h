#pragma once

#include "Qualified.h"

namespace wrap {
struct TypedefPair {
    Qualified oldType, newType;
    std::string includeFile;

    TypedefPair() {}
    TypedefPair(const Qualified& oldType, const Qualified& newType,
                const std::string& includeFile)
        : oldType(oldType), newType(newType), includeFile(includeFile) {}

    void emit_cython_pxd(FileWriter& file) const {
        file.oss << "cdef extern from \"" << includeFile << "\" namespace \""
                 << oldType.qualifiedNamespaces("::") << "\":\n";
        file.oss << "\tctypedef " << oldType.cythonClass() << " "
                 << newType.cythonClass() << "\n";
    }
};
}