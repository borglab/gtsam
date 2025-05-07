# Calling the GeographicLib C++ library from Excel

You can call GeographicLib functions from Excel.  Thanks to Thomas
Warner <warnerta@gmail.com>, for showing me how.  This prescription
has only been tested for Excel running on a Windows machine.  Please
let me know if you figure out how to get this working on MacOS
versions of Excel.

Here's the overview

* Write and compile little interface routines to invoke the
  functionality you want.

* Copy the resulting DLLs to where Excel can find them.

* Write an interface script in Visual Basic.  This tells Visual Basic
  about your interfrace routines and it includes definitions of the actual
  functions you will see exposed in Excel.

Here are the step-by-step instructions for compiling and using the
sample routines given here (which solve the direct and inverse geodesic
problems and the corresponding rhumb line problems):

1. Install binary distribution for GeographicLib (either 64-bit or
   32-bit to match your version of Excel).

2. Install a recent version of cmake.

3. Start a command prompt window and run
   ```bash
   mkdir BUILD
   cd BUILD
   cmake -G "Visual Studio 16" -A x64 ..
   ```
   This configures your build.  Any of Visual Studio 14, 15, or 16
   (corresponding the VS 2015, 2017, 2019) will work.  If your Excel is
   32-bit, change `-A x64` to `-A win32`.  If necessary include `-D
   CMAKE_PREFIX_PATH=DIR` to specify where GeographicLib is installed
   (specified when you ran the GeographicLib installer).  Compile the
   interface with
   ```bash
   cmake --build . --config Release
   ```

4. Copy
   ```bash
   Release\cgeodesic.dll   # the interface routines
   Release\GeographicLib.dll  # the main GeographicLib library
   ```
   to the directory where the Excel executable lives.  You can find this
   directory by launching Excel, launching Task Manager, right-clicking on
   Excel within Task Manager and selecting Open file location.  It's
   probably something like
   ```bash
   C:\Program Files\Microsoft Office\root\Office16
   ```
   and you will probably need administrator privileges to do the copy.
   If it's in `Program Files (x86)`, then you have a 32-bit version of
   Excel and you need to compile your interface routines in 32-bit by
   specitying `-A win32` when you first run cmake.

5. Open the Excel workbook within which you would like to use the
   geodesic and rhumb routines.<br>
   Type `Alt-F11` to open Excel's Visual Basic editor.<br>
   In the left sidebar, right-click on `VBAProject (%yourworksheetname%)`
   and select Import File<br>
   Browse to `Geodesic.bas`, select it and click Open<br>
   Save your Workbook as Excel Macro-Enabled Workbook (`*.xlsm`)

6. You will now have 10 new functions available:
   * Solve the direct geodesic problem for
     ```
     lat2: geodesic_direct_lat2(lat1, lon1, azi1, s12)
     lon2: geodesic_direct_lon2(lat1, lon1, azi1, s12)
     azi2: geodesic_direct_azi2(lat1, lon1, azi1, s12)
     ```
   * Solve the inverse geodesic problem for
     ```
     s12: geodesic_inverse_s12(lat1, lon1, lat2, lon2)
     azi1: geodesic_inverse_azi1(lat1, lon1, lat2, lon2)
     azi2: geodesic_inverse_azi2(lat1, lon1, lat2, lon2)
     ```
   * Solve the direct rhumb problem for
     ```
     lat2: rhumb_direct_lat2(lat1, lon1, azi12, s12)
     lon2: rhumb_direct_lon2(lat1, lon1, azi12, s12)
     ```
   * Solve the inverse rhumb problem for
     ```
     s12: rhumb_inverse_s12(lat1, lon1, lat2, lon2)
     azi12: rhumb_inverse_azi12(lat1, lon1, lat2, lon2)
     ```
   Latitudes, longitudes, and azimuths are in degrees.  Distances are
   in meters.
