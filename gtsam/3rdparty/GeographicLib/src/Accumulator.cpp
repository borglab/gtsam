/**
 * \file Accumulator.cpp
 * \brief Implementation for GeographicLib::Accumulator class
 *
 * Copyright (c) Charles Karney (2013) <charles@karney.com> and licensed under
 * the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/Accumulator.hpp>

namespace GeographicLib {

  /// \cond SKIP

  // Need to instantiate Accumulator to get the code into the shared library
  // (without this, NETGeographic complains about not finding the == and !=
  // operators).
  template class GEOGRAPHICLIB_EXPORT Accumulator<Math::real>;

  /// \endcond

} // namespace GeographicLib
