#pragma once

#include "Qualified.h"

namespace wrap {
struct TypedefPair {
    Qualified oldType, newType;
    std::string includeFile;

    TypedefPair() {}
    TypedefPair(const Qualified& _oldType, const Qualified& _newType,
                const std::string& includeFile)
        : oldType(_oldType), newType(_newType), includeFile(includeFile) {
          if (!oldType.isNonBasicType() &&
              std::find(Qualified::BasicTypedefs.begin(),
                        Qualified::BasicTypedefs.end(),
                        newType) == Qualified::BasicTypedefs.end())
            Qualified::BasicTypedefs.push_back(newType);
    }

    void emit_cython_pxd(FileWriter& file) const {
        file.oss << "cdef extern from \"" << includeFile << "\" namespace \""
                 << oldType.qualifiedNamespaces("::") << "\":\n";
        file.oss << "    ctypedef " << oldType.pxdClassName() << " "
                 << newType.pxdClassName() << "\n";
    }
};
}
