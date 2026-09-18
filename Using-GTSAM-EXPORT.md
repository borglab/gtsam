# Exporting GTSAM Symbols on Windows

CMake generates the export macros used by GTSAM public headers. In a Windows
shared-library build, a macro expands to `__declspec(dllexport)` while its
library is being built and to `__declspec(dllimport)` for consumers. In a static
build, it expands to nothing.

Use the macro belonging to the library that owns the symbol:

- `GTSAM_EXPORT` for the stable `gtsam` library.
- `GTSAM_UNSTABLE_EXPORT` for `gtsam_unstable`.

## Usage rules

1. Add the library's export macro to the header declaration of a public free
   function whose definition is compiled in a `.cpp` file:

   ```cpp
   GTSAM_EXPORT ReturnType myFunction();
   ```

2. Add the macro to a public class when the library owns out-of-line member
   definitions that consumers must import:

   ```cpp
   class GTSAM_EXPORT MyClass {
    public:
     void method();
   };
   ```

   A class-level macro exports its applicable members; do not repeat the macro
   on each member declaration.

3. Inline functions, header-only classes, and unspecialized templates are
   normally compiled in the consuming translation unit and should not be marked
   merely to force them into the DLL. For an explicit template specialization
   with an out-of-line definition owned by the library, put the export macro on
   that specialization.

4. Review base classes before exporting a derived class. MSVC warning C4275 and
   downstream link failures can occur when an exported class derives from a
   non-exported base. This is especially relevant to implementation types from
   header-only libraries such as Eigen. Prefer composition or export only the
   non-inline GTSAM interface when a base cannot participate in the DLL ABI.

## Diagnosing downstream linker failures

Incorrect export annotations often appear only when a different target links
against the built DLL. A typical failure is `LNK2019: unresolved external
symbol` in a test, wrapper, or downstream project even though the GTSAM library
itself compiled successfully.

For the missing symbol, check:

- that its declaration uses the macro for the library containing its definition;
- that the definition is compiled into that library;
- that class-level and member-level annotations are not conflicting;
- that an explicit template specialization is exported when its definition is
  owned by the library; and
- that the consumer and library agree on shared versus static configuration.

The generated macro definitions are in `gtsam/dllexport.h` and
`gtsam_unstable/dllexport.h` in the build or install tree. See Microsoft's
[warning C4275 documentation](https://learn.microsoft.com/en-us/cpp/error-messages/compiler-warnings/compiler-warning-level-2-c4275)
for the exported-derived-class diagnostic.
