#include "stdafx.h"

using namespace System;
using namespace System::Reflection;
using namespace System::Runtime::CompilerServices;
using namespace System::Runtime::InteropServices;
using namespace System::Security::Permissions;

//
// General Information about an assembly is controlled through the following
// set of attributes. Change these attribute values to modify the information
// associated with an assembly.
//
[assembly:AssemblyTitleAttribute("NETGeographic")];
[assembly:AssemblyDescriptionAttribute(".NET Interface for GeographicLib")];
[assembly:AssemblyConfigurationAttribute("1.33")];
[assembly:AssemblyCompanyAttribute("Open Source")];
[assembly:AssemblyProductAttribute("NETGeographic")];
[assembly:AssemblyCopyrightAttribute("GeographicLib copyright (c) Charles Karney 2013\nNETGeographicLib copyright (c) Scott Heiman 2013")];
[assembly:AssemblyTrademarkAttribute("")];
[assembly:AssemblyCultureAttribute("")];

//
// Version information for an assembly consists of the following four values:
//
//      Major Version
//      Minor Version
//      Build Number
//      Revision
//
// You can specify all the value or you can default the Revision and Build Numbers
// by using the '*' as shown below:

[assembly:AssemblyVersionAttribute("1.33.*")];

[assembly:ComVisible(false)];

[assembly:CLSCompliantAttribute(true)];

#if _MSC_VER < 1800
// RequestMinium marked as obsolete in Visual Studio 12 (2013)
[assembly:SecurityPermission(SecurityAction::RequestMinimum, UnmanagedCode = true)];
#endif
