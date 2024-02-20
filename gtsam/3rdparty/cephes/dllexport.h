// Macros for exporting DLL symbols on Windows
// Usage example:
// In header file:
//   class CEPHES_EXPORT MyClass { ... };
//   
// Results in the following declarations:
// When included while compiling the library itself:
//   class __declspec(dllexport) MyClass { ... };
// When included while compiling other code against the library:
//   class __declspec(dllimport) MyClass { ... };

#pragma once

#ifdef _WIN32
#      define CEPHES_EXPORT __declspec(dllimport)
#      define CEPHES_EXTERN_EXPORT __declspec(dllimport)
#else
#ifdef __APPLE__
#  define CEPHES_EXPORT __attribute__((visibility("default")))
#  define CEPHES_EXTERN_EXPORT extern
#else
#  define CEPHES_EXPORT
#  define CEPHES_EXTERN_EXPORT extern
#endif
#endif
