# Using GTSAM_EXPORT

To create a DLL in windows, the `GTSAM_EXPORT` keyword has been created and needs to be applied to different methods and classes in the code to expose this code outside of the DLL.  However, there are several intricacies that make this more difficult than it sounds.  In general, if you follow the following three rules, GTSAM_EXPORT should work properly.  The rest of the document also describes (1) the common error message encountered when you are not following these rules and (2) the reasoning behind these usage rules.

## Usage Rules
1.  Put `GTSAM_EXPORT` in front of any function that you want exported in the DLL _if and only if_ that function is declared in a .cpp file, not just a .h file.
2.  Use `GTSAM_EXPORT` in a class definition (i.e. `class GSTAM_EXPORT MyClass {...}`) only if:
    * At least one of the functions inside that class is declared in a .cpp file and not just the .h file.
    * You can `GTSAM_EXPORT` any class it inherits from as well.  (Note that this implictly requires the class does not derive from a "header-only" class.  Note that Eigen is a "header-only" library, so if your class derives from Eigen, _do not_ use `GTSAM_EXPORT` in the class definition!) 
3.  If you have defined a class using `GTSAM_EXPORT`, do not use `GTSAM_EXPORT` in any of its individual function declarations.  (Note that you _can_ put `GTSAM_EXPORT` in the definition of individual functions within a class as long as you don't put `GTSAM_EXPORT` in the class definition.)
4. For template specializations, you need to add `GTSAM_EXPORT` to each individual specialization.

## When is GTSAM_EXPORT being used incorrectly
Unfortunately, using `GTSAM_EXPORT` incorrectly often does not cause a compiler or linker error in the library that is being compiled, but only when you try to use that DLL in a different library.  For example, an error in `gtsam/base` will often show up when compiling the `check_base_program` or the MATLAB wrapper, but not when compiling/linking gtsam itself.  The most common errors will say something like:

```
Error	LNK2019	unresolved external symbol "public: void __cdecl gtsam::SO3::print(class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> > const &)const " (?print@SO3@gtsam@@QEBAXAEBV?$basic_string@DU?$char_traits@D@std@@V?$allocator@D@2@@std@@@Z) referenced in function "public: static void __cdecl gtsam::Testable<class gtsam::SO3>::Print(class gtsam::SO3 const &,class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> > const &)" (?Print@?$Testable@VSO3@gtsam@@@gtsam@@SAXAEBVSO3@2@AEBV?$basic_string@DU?$char_traits@D@std@@V?$allocator@D@2@@std@@@Z)	check_geometry_program	C:\AFIT\lib\gtsam\build\gtsam\geometry\tests\testSO3.obj
```

Let's analyze this error statement.  First, there is an unresolved symbol `gtsam::SO3::print`.  This can occur because _either_ `GTSAM_EXPORT` was not added to the print function definition when it should have been, _OR_ because `GTSAM_EXPORT` was added to the print function definition when it is fully declared in the header.  This error was detected while compiling `check_geometry_program` and pulling in `...\testSO3.obj`.  Specifically, within the function call `gtsam::Testable<class gtsam::SO3>::Print (...)`.  Note that this error did _not_ occur when compiling the library that actually has SO3 inside of it.

## But Why?
I believe that how the compiler/linker interacts with GTSAM_EXPORT can be explained by understanding the rules that MSVC operates under.  

But first, we need to understand exactly what `GTSAM_EXPORT` is.  `GTSAM_EXPORT` is a `#define` macro that is created by CMAKE when GTSAM is being compiled on a Windows machine.  Inside the GTSAM project, GTSAM export will be set to a "dllexport" command.  A "dllexport" command tells the compiler that this function should go into the DLL and be visible externally.  In any other library, `GTSAM_EXPORT`  will be set to a "dllimport" command, telling the linker it should find this function in a DLL somewhere.  This leads to the first rule the compiler uses.  (Note that I say "compiler rules" when the rules may actually be in the linker, but I am conflating the two terms here when I speak of the "compiler rules".) 

***Compiler Rule #1*** If a `dllimport` command is used in defining a function or class, that function or class _must_ be found in a DLL.

Rule #1 doesn't seem very bad, until you combine it with rule #2

***Compiler Rule #2*** Anything declared in a header file is not included in a DLL.

When these two rules are combined, you get some very confusing results.  For example, a class which is completely defined in a header (e.g. Foo) cannot use `GTSAM_EXPORT` in its definition.  If Foo is defined with `GTSAM_EXPORT`, then the compiler _must_ find Foo in a DLL.  Because Foo is a header-only class, however, it can't find it, leading to a very confusing "I can't find this symbol" type of error.  Note that the linker says it can't find the symbol even though the compiler found the header file that completely defines the class.

Also note that when a class that you want to export inherits from another class that is not exportable, this can cause significant issues.  According to this [MSVC Warning page](https://docs.microsoft.com/en-us/cpp/error-messages/compiler-warnings/compiler-warning-level-2-c4275?view=vs-2019), it may not strictly be a rule, but we have seen several linker errors when a class that is defined with `GTSAM_EXPORT` extended an Eigen class.  In general, it appears that any inheritance of non-exportable class by an exportable class is a bad idea.

## Conclusion
Hopefully, this little document clarifies when `GTSAM_EXPORT` should and should not be used whenever future GTSAM code is being written.  Following the usage rules above, we have been able to get all of the libraries, together with their test and wrapping libraries, to compile/link successfully.