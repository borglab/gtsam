/**
 * Unit test for Boost's awesome Spirit parser
 * Author: Frank Dellaert
 **/

#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_push_back_actor.hpp>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace BOOST_SPIRIT_CLASSIC_NS;

typedef rule<BOOST_SPIRIT_CLASSIC_NS::phrase_scanner_t> Rule;

/* ************************************************************************* */
// lexeme_d turns off white space skipping
// http://www.boost.org/doc/libs/1_37_0/libs/spirit/classic/doc/directives.html
Rule name_p       = lexeme_d[alpha_p >> *(alnum_p | '_')];
Rule className_p  = lexeme_d[upper_p >> *(alnum_p | '_')];
Rule methodName_p = lexeme_d[lower_p >> *(alnum_p | '_')];

Rule basisType_p = (str_p("string") | "bool" | "size_t" | "int" | "double" | "Vector" | "Matrix");

/* ************************************************************************* */
TEST( spirit, real ) {
  // check if we can parse 8.99 as a real
  CHECK(parse("8.99", real_p, space_p).full);
  // make sure parsing fails on this one
  CHECK(!parse("zztop", real_p, space_p).full);
}

/* ************************************************************************* */
TEST( spirit, string ) {
  // check if we can parse a string
  CHECK(parse("double", str_p("double"), space_p).full);
}

/* ************************************************************************* */
TEST( spirit, sequence ) {
  // check that we skip white space
  CHECK(parse("int int", str_p("int") >> *str_p("int"), space_p).full);
  CHECK(parse("int --- - -- -", str_p("int") >> *ch_p('-'), space_p).full);
  CHECK(parse("const \t string", str_p("const") >> str_p("string"), space_p).full);

  // not that (see spirit FAQ) the vanilla rule<> does not deal with whitespace
  rule<>vanilla_p = str_p("const") >> str_p("string");
  CHECK(!parse("const \t string", vanilla_p, space_p).full);

  // to fix it, we need to use <phrase_scanner_t>
  rule<phrase_scanner_t>phrase_level_p = str_p("const") >> str_p("string");
  CHECK(parse("const \t string", phrase_level_p, space_p).full);
}

/* ************************************************************************* */
// parser for interface files

// const string reference reference
Rule constStringRef_p = 
  str_p("const") >> "string" >> '&';

// class reference
Rule classRef_p = className_p >> '&';

// const class reference
Rule constClassRef_p = str_p("const") >> classRef_p;

// method parsers
Rule constMethod_p = basisType_p >> methodName_p >> '(' >> ')' >> "const" >> ';';

/* ************************************************************************* */
TEST( spirit, basisType_p ) {
  CHECK(!parse("Point3", basisType_p, space_p).full);
  CHECK(parse("string", basisType_p, space_p).full);
}

/* ************************************************************************* */
TEST( spirit, className_p ) {
  CHECK(parse("Point3", className_p, space_p).full);
}

/* ************************************************************************* */
TEST( spirit, classRef_p ) {
  CHECK(parse("Point3 &", classRef_p, space_p).full);
  CHECK(parse("Point3&", classRef_p, space_p).full);
}

/* ************************************************************************* */
TEST( spirit, constMethod_p ) {
  CHECK(parse("double norm() const;", constMethod_p, space_p).full);
}

/* ************************************************************************* */
int main() { TestResult tr; TestRegistry::runAllTests(tr); return 0; }
/* ************************************************************************* */
