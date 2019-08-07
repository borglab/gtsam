#pragma once
/**
 * \file NETGeographicLib/Accumulator.h
 * \brief Header for NETGeographicLib::Accumulator class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

namespace NETGeographicLib
{
    /*!
    \brief .NET wrapper for GeographicLib::Accumulator.

    This class allows .NET applications to access GeographicLib::Accumulator<double>.

    This allow many numbers of floating point type \e double to be added together
    with twice the normal precision.  The effective
    precision of the sum is 106 bits or about 32 decimal places.

    The implementation follows J. R. Shewchuk,
    <a href="https://doi.org/10.1007/PL00009321"> Adaptive Precision
    Floating-Point Arithmetic and Fast Robust Geometric Predicates</a>,
    Discrete & Computational Geometry 18(3) 305--363 (1997).

    C# Example:
    \include example-Accumulator.cs
    Managed C++ Example:
    \include example-Accumulator.cpp
    Visual Basic Example:
    \include example-Accumulator.vb

    <B>INTERFACE DIFFERENCES:</B><BR>
    Since assignment operators (=,+=,-=,*=) are not supported in managed classes;
    - the Assign() method replaces the = operator,
    - the Sum() method replaces the += and -= operators, and
    - the Multiply() method replaces the *= operator,

    Use Result() instead of the () operator to obtain the summed value from the accumulator.
    */
    public ref class Accumulator
    {
    private:
        // Pointer to the unmanaged GeographicLib::Accumulator.
        GeographicLib::Accumulator<double>* m_pAccumulator;
        // The finalizer releases the unmanaged object when this class is destroyrd.
        !Accumulator(void);
    public:
        //! \brief Constructor.
        Accumulator(void);
        //! \brief Destructor calls the finalizer.
        ~Accumulator() { this->!Accumulator(); }
        /*!
        \brief Assigns a value to an accumulator.
        \param[in] a The value to be assigned.
        */
        void Assign( double a );
        //! \brief Returns the accumulated value.
        double Result();
        /*!
        \brief Adds a value to the accumulator.
        \param[in] a The value to be added.
        */
        void Sum( double a );
        /*!
        \brief Multiplication by an integer
        \param[in] i The multiplier.
        */
        void Multiply( int i );
        /*!
        \brief Equality operator.
        \param[in] lhs The accumulator.
        \param[in] a The value to be compared to.
        \return true if the accumulated value is equal to a.
        */
        static bool operator == ( Accumulator^ lhs, double a );
        /*!
        \brief Inequality operator.
        \param[in] lhs The accumulator.
        \param[in] a The value to be compared to.
        \return true if the accumulated value is not equal to a.
        */
        static bool operator != ( Accumulator^ lhs, double a );
        /*!
        \brief Less than operator.
        \param[in] lhs The accumulator.
        \param[in] a The value to be compared to.
        \return true if the accumulated value is less than a.
        */
        static bool operator < ( Accumulator^ lhs, double a );
        /*!
        \brief Less than or equal to operator.
        \param[in] lhs The accumulator.
        \param[in] a The value to be compared to.
        \return true if the accumulated value is less than or equal to a.
        */
        static bool operator <= ( Accumulator^ lhs, double a );
        /*!
        \brief Greater than operator.
        \param[in] lhs The accumulator.
        \param[in] a The value to be compared to.
        \return true if the accumulated value is greater than a.
        */
        static bool operator > ( Accumulator^ lhs, double a );
        /*!
        \brief Greater than or equal to operator.
        \param[in] lhs The accumulator.
        \param[in] a The value to be compared to.
        \return true if the accumulated value is greater than or equal to a.
        */
        static bool operator >= ( Accumulator^ lhs, double a );
    };
} //namespace NETGeographicLib
