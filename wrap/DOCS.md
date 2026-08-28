## Wrap Module Definition

### Important

The python wrapper supports keyword arguments for functions/methods. Hence, the argument names matter. An implementation restriction is that in overloaded methods or functions, arguments of different types *have* to have different names.

### Requirements

- Classes must start with an uppercase letter.
    - The wrapper can wrap a typedef, e.g. `typedef TemplatedClass<Arg> EasyName;`.

- Only one Method/Constructor per line, though methods/constructors can extend across multiple lines.

- Methods can return
    - Eigen types: `gtsam::Matrix`, `gtsam::Vector`.
    - C/C++ basic types: `string`, `bool`, `size_t`, `int`, `double`, `char`, `unsigned char`.
    - `void`
    - Any class with which be copied with `std::make_shared()`.
    - `std::shared_ptr` of any object type.

- Constructors
    - Overloads are supported, but arguments of different types *have* to have different names.
    - A class with no constructors can be returned from other functions but not allocated directly in MATLAB.

- Methods
    - Constness has no effect.
    - Specify by-value (not reference) return types, even if C++ method returns reference.
    - MATLAB maps a disengaged `std::optional<T>` to `[]` and an engaged
      optional to the normal wrapped representation of `T`.
      `std::optional<std::pair<T, U>>` follows the existing pair convention:
      two MATLAB outputs, both `[]` when the optional is disengaged.
    - Must start with a letter (upper or lowercase).
    - Overloads are supported.

- Static methods
    - Must start with a letter (upper or lowercase) and use the "static" keyword, e.g. `static void func()`.
    - The first letter will be made uppercase in the generated MATLAB interface.
    - Overloads are supported, but arguments of different types *have* to have different names.

- Arguments to functions can be any of
    - Eigen types: `gtsam::Matrix`, `gtsam::Vector`.
    - Eigen types and classes as an optionally const reference.
    - C/C++ basic types: `string`, `bool`, `size_t`, `size_t`, `double`, `char`, `unsigned char`.
    - Any class with which be copied with `std::make_shared()` (except Eigen).
    - `std::shared_ptr` of any object type (except Eigen).
    - For MATLAB, `std::optional<T>` accepts `[]` for `std::nullopt` or the
      normal MATLAB representation of `T` for an engaged value. Consequently,
      an engaged optional containing an empty MATLAB array cannot be
      distinguished from `std::nullopt`.

- Properties or Variables
    - You can specify class variables in the interface file as long as they are in the `public` scope, e.g.

    ```cpp
    class Sample {
        double seed;
    };
    ```

    - Class variables are read-write so they can be updated directly in Python.
    - For the Matlab wrapper, specifying the full property type (including namespaces) is required.

    ```cpp
    class TriangulationResult {
      gtsam::SharedNoiseModel noiseModel;
    };
    ```

    - If the property is part of an enum within the class, the type should be specified as `gtsam::Class::Enum`. Similarly for templated types where `This` is used, e.g. `gtsam::This::Enum`.

    ```cpp
    class TriangulationResult {
      enum Status { VALID, DEGENERATE, BEHIND_CAMERA, OUTLIER, FAR_POINT };
      gtsam::TriangulationResult::Status status;
    };

    template<PARAMS>
    virtual class GncParams {
      enum Verbosity {
        SILENT,
        SUMMARY,
        VALUES
      };
      gtsam::This::Verbosity verbosity;
    };
    ```

- Operator Overloading (Python only)
    - You can overload operators just like in C++.

    ```cpp
    class Overload {
        Overload operator*(const Overload& other) const;
    };
    ```
    - Supported operators are the intersection of those supported in C++ and in Python.
    - Operator overloading definitions have to be marked as `const` methods.

- Pointer types
    - To declare a simple/raw pointer, simply add an `@` to the class name, e.g.`Pose3@`.
    - To declare a shared pointer (e.g. `gtsam::noiseModel::Base::shared_ptr`), use an asterisk (i.e. `*`). E.g. `gtsam::noiseModel::Base*` to define the wrapping for the `Base` noise model shared pointer.

- Comments can use either C++ or C style, with multiple lines.

- Namespace definitions
    - Names of namespaces must start with a lowercase letter.
    - Start a namespace with `namespace example_ns {`, where `example_ns` is the namespace name.
    - End a namespace with exactly `}`
    - Namespaces can be nested.

- Namespace usage
     - Namespaces can be specified for classes in arguments and return values.
     - In each case, the namespace must be fully specified, e.g., `namespace1::namespace2::ClassName`.

- Includes in C++ wrappers
    - All includes will be collected and added in a single file.
    - All namespaces must have angle brackets, e.g. `#include <path>`.
    - No default includes will be added.

- Global/Namespace functions
    - Functions specified outside of a class are **global**.
    - Can be overloaded with different arguments.
    - Can have multiple functions of the same name in different namespaces.
    - Functions can be templated and have multiple template arguments, e.g.
        ```cpp
        template<T, R, S>
        ```

- Global variables
    - Similar to global functions, the wrapper supports global variables as well.
    - Currently we only support primitive types, such as `double`, `int`, `string`, etc.
    - E.g.
        ```cpp
        const double kGravity = -9.81;
        ```

- Using classes defined in other modules
    - If you are using a class `OtherClass` not wrapped in an interface file, add `class OtherClass;` as a forward declaration to avoid a dependency error.
    - `OtherClass` may not be in the same project. If this is the case, include the header for the appropriate project `#include <other_project/OtherClass.h>`.

- Virtual inheritance
    - Specify fully-qualified base classes, i.e. `virtual class Derived : ns::Base {` where `ns` is the namespace.
    - Mark with `virtual` keyword, e.g. `virtual class Base {`, and also `virtual class Derived : ns::Base {`.
    - Base classes can be templated, e.g. `virtual class Dog: ns::Animal<Pet> {};`. This is useful when you want to inherit from specialized classes.
    - Forward declarations must also be marked virtual, e.g. `virtual class ns::Base;` and
      also `virtual class ns::Derived;`.
    - Pure virtual (abstract) classes should list no constructors in the interface file.
    - Virtual classes must have a `clone()` function in C++ (though it does not have to be included
      in the interface file). `clone()` will be called whenever an object copy is needed, instead
      of using the copy constructor (which is used for non-virtual objects).
    - Signature of clone function - `clone()` will be called virtually, so must appear at least at the top of the inheritance tree

        ```cpp
        virtual std::shared_ptr<CLASS_NAME> clone() const;
        ```

- Templates
    - Basic templates are supported either with an explicit list of types to instantiate,
      e.g.

      ```cpp
      template<T = {gtsam::Pose2, gtsam::Rot2, gtsam::Point3}> class Class1 { ... };
      ```

      or with typedefs, e.g.

      ```cpp
      template<T, U> class Class2 { ... };
      typedef Class2<Type1, Type2> MyInstantiatedClass;
      ```
    - Serialization can be enabled for one typedef without marking every
      specialization of the template serializable:

      ```cpp
      template<T> class Class3 { ... };
      @serializable
      typedef Class3<Type1> SerializableClass3;
      typedef Class3<Type2> PlainClass3;
      ```

      `@serializable` is wrapper metadata. It generates the same Python pickle
      and MATLAB save/load support as a `void serialize() const;` marker on a
      concrete wrapper class, but applies only to the annotated typedef.
    - Templates can also be defined for constructors, methods, properties and static methods.
    - In the class definition, appearances of the template argument(s) will be replaced with their
      instantiated types, e.g. `void setValue(const T& value);`.
    - Values scoped within templates are supported. E.g. one can use the form `T::Value` where T is a template, as an argument to a method.
    - To refer to the instantiation of the template class itself, use `This`, i.e. `static This Create();`.
    - To create new instantiations in other modules, you must copy-and-paste the whole class definition
      into the new module, but use only your new instantiation types.
    - When forward-declaring template instantiations, use the generated/typedef'd name, e.g.

      ```cpp
      class gtsam::Class1Pose2;
      class gtsam::MyInstantiatedClass;
      ```
    - Template arguments can be templates themselves, e.g.

      ```cpp
      // Typedef'd PinholeCamera
      template<CALIBRATION>
      class PinholeCamera { ... };
      typedef gtsam::PinholeCamera<gtsam::Cal3_S2> PinholeCameraCal3_S2;

      template<CAMERA>
      class SfmFactor { ... };
      // This is valid.
      typedef gtsam::SfmFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>> BasicSfmFactor;
      ```

- `Boost.serialization` within the wrapper:
    - You need to mark classes as being serializable in the markup file (see `gtsam.i` for examples).
    - There are two options currently, depending on the class.  To "mark" a class as serializable,
      add a function with a particular signature so that `wrap` will catch it.
       - Add `void serialize()` to a class to create serialization functions for a class.
         Adding this flag subsumes the `serializable()` flag below.
         
         Requirements:
            - A default constructor must be publicly accessible.
            - Must not be an abstract base class.
            - The class must have an actual boost.serialization `serialize()` function.

       - Add `void serializable()` to a class if you only want the class to be serialized as a
         part of a container (such as `noiseModel`). This version does not require a publicly
         accessible default constructor.

- Forward declarations and class definitions for **Pybind**:
    - Need to specify the base class (both this forward class and base class are declared in an external Pybind header)
    - This is so that Pybind can generate proper inheritance.

    - Example for when wrapping a gtsam-based project:

        ```cpp
         // forward declarations
         virtual class gtsam::NonlinearFactor
         virtual class gtsam::NoiseModelFactor : gtsam::NonlinearFactor
         // class definition
         #include <MyFactor.h>
         virtual class MyFactor : gtsam::NoiseModelFactor {...};
         ```

   - **DO NOT** re-define an overriden function already declared in the external (forward-declared) base class. This will cause an ambiguity problem in the Pybind header file.

- Splitting wrapper over multiple files
    - The Pybind11 wrapper supports splitting the wrapping code over multiple files.
    - To be able to use classes from another module, simply import the C++ header file in that wrapper file.
    - Unfortunately, this means that aliases can no longer be used.
    - Similarly, there can be multiple `preamble.h` and `specializations.h` files. Each of these should match the module file name.

## Pybind Callable Adapters

The Python generator uses explicit full-signature C++ callable-pointer casts for
ordinary wrapper declarations. For example:

```cpp
class Example {
  int size() const;
  static Example Create();
};

int globalFunction(int value);
```

generates bindings equivalent to:

```cpp
static_cast<int (Example::*)() const>(&Example::size)
static_cast<Example (*)()>(&Example::Create)
static_cast<int (*)(int)>(&::globalFunction)
```

The return type, argument types, class type, and member constness are always
part of the cast. This selects an exact overload even when other overloads exist
in the C++ header but are not listed in the `.i` file.

A wrapper `.i` declaration does not always reproduce the underlying C++
signature exactly. Add `@pybind_lambda` to one method, static method, or global
function when its wrapper signature is intentionally an adapter:

```cpp
class Example {
  @pybind_lambda
  int lookup(int index) const;

  @pybind_lambda
  static Example Load(string filename);

  template<T = {double}>
  @pybind_lambda
  T convert(T value) const;
};

@pybind_lambda
int globalFunction(int value);
```

Place the annotation after any `template<...>` declaration and immediately
before the callable. The annotated instance method is emitted using the existing
forwarding-lambda path, for example:

```cpp
.def("lookup",
     [](Example* self, int index) {
       return self->lookup(index);
     },
     py::arg("index"))
```

The annotation is useful when the interface intentionally:

- omits underlying C++ parameters that have defaults;
- accepts values where the C++ function accepts references;
- synthesizes a value-returning container operation such as `at` or `front`;
- uses types that are not equivalent to the callable's declared signature; or
- otherwise adapts an inherited or templated member's signature.

Do not annotate a callable merely because it is overloaded, including when an
overload exists only in the C++ header. An unannotated `.i` declaration whose
complete function type matches the desired C++ overload uses the generated
full-signature cast. Likewise, existing automatic lambdas for template
specializations, renamed bindings, print/repr, serialization, and synthesized
dunder methods do not need this marker.

`@pybind_lambda` affects only pybind output. MATLAB generation ignores the
stored marker. Template instantiation preserves the marker on every instantiated
callable. Python-visible names, keyword escaping, default arguments, argument
policies, return-value policies such as `reference_internal`, and generated
docstrings are appended exactly as they are for the normal binding path.

Wrap does not inspect the C++ AST and cannot determine whether an interface
signature exactly matches the real declaration. Use the annotation only after
comparing the `.i` declaration with the C++ header or after diagnosing a
generated-code compilation error. Unknown annotations and annotations on
constructors, properties, classes, enums, or other unsupported declarations are
parser errors.

### TODO
- Handle `gtsam::Rot3M` conversions to quaternions.
- Parse return of const ref arguments.
- Parse `std::string` variants and convert directly to special string.
- Add generalized serialization support via `boost.serialization` with hooks to MATLAB save/load.
