/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     QPParser.cpp
 * @author   Ivan Dario Jimenez
 * @date     3/5/16
 */

#define BOOST_SPIRIT_USE_PHOENIX_V3 1

#include <gtsam_unstable/linear/QPSParser.h>
#include <gtsam_unstable/linear/QPSParserException.h>
#include <gtsam_unstable/linear/RawQP.h>

#include <boost/spirit/include/qi.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/phoenix/bind.hpp>
#include <boost/spirit/include/classic.hpp>

namespace bf = boost::fusion;
namespace qi = boost::spirit::qi;

namespace gtsam {
typedef qi::grammar<boost::spirit::basic_istream_iterator<char>> base_grammar;

struct QPSParser::MPSGrammar: base_grammar {
  typedef std::vector<char> Chars;
  RawQP * rqp_;
  boost::function<void(bf::vector<Chars, Chars, Chars> const&)> setName;
  boost::function<void(bf::vector<Chars, char, Chars, Chars, Chars> const &)> addRow;
  boost::function<
      void(bf::vector<Chars, Chars, Chars, Chars, Chars, double, Chars> const &)> rhsSingle;
  boost::function<
      void(
          bf::vector<Chars, Chars, Chars, Chars, Chars, double, Chars, Chars,
              Chars, double>)> rhsDouble;
  boost::function<
      void(bf::vector<Chars, Chars, Chars, Chars, Chars, double, Chars>)> colSingle;
  boost::function<
      void(
          bf::vector<Chars, Chars, Chars, Chars, double, Chars, Chars, Chars,
              double> const &)> colDouble;
  boost::function<
      void(bf::vector<Chars, Chars, Chars, Chars, Chars, double, Chars> const &)> addQuadTerm;
  boost::function<
      void(
          bf::vector<Chars, Chars, Chars, Chars, Chars, Chars, Chars, double> const &)> addBound;
  boost::function<
      void(bf::vector<Chars, Chars, Chars, Chars, Chars, Chars, Chars> const &)> addBoundFr;
  MPSGrammar(RawQP * rqp) :
      base_grammar(start), rqp_(rqp), setName(
          boost::bind(&RawQP::setName, rqp, ::_1)), addRow(
          boost::bind(&RawQP::addRow, rqp, ::_1)), rhsSingle(
          boost::bind(&RawQP::addRHS, rqp, ::_1)), rhsDouble(
          boost::bind(&RawQP::addRHSDouble, rqp, ::_1)), colSingle(
          boost::bind(&RawQP::addColumn, rqp, ::_1)), colDouble(
          boost::bind(&RawQP::addColumnDouble, rqp, ::_1)), addQuadTerm(
          boost::bind(&RawQP::addQuadTerm, rqp, ::_1)), addBound(
          boost::bind(&RawQP::addBound, rqp, ::_1)), addBoundFr(
          boost::bind(&RawQP::addBoundFr, rqp, ::_1)) {
    using namespace boost::spirit;
    using namespace boost::spirit::qi;
    character = lexeme[alnum | '_' | '-' | '.'];
    title = lexeme[character >> *(blank | character)];
    word = lexeme[+character];
    name = lexeme[lit("NAME") >> *blank >> title >> +space][setName];
    row = lexeme[*blank >> character >> +blank >> word >> *blank][addRow];
    rhs_single = lexeme[*blank >> word >> +blank >> word >> +blank >> double_
        >> *blank][rhsSingle];
    rhs_double = lexeme[(*blank >> word >> +blank >> word >> +blank >> double_
        >> +blank >> word >> +blank >> double_)[rhsDouble] >> *blank];
    col_single = lexeme[*blank >> word >> +blank >> word >> +blank >> double_
        >> *blank][colSingle];
    col_double = lexeme[*blank
        >> (word >> +blank >> word >> +blank >> double_ >> +blank >> word
            >> +blank >> double_)[colDouble] >> *blank];
    quad_l = lexeme[*blank >> word >> +blank >> word >> +blank >> double_
        >> *blank][addQuadTerm];
    bound = lexeme[(*blank >> word >> +blank >> word >> +blank >> word >> +blank
        >> double_)[addBound] >> *blank];
    bound_fr = lexeme[*blank >> word >> +blank >> word >> +blank >> word
        >> *blank][addBoundFr];
    rows = lexeme[lit("ROWS") >> *blank >> eol >> +(row >> eol)];
    rhs = lexeme[lit("RHS") >> *blank >> eol
        >> +((rhs_double | rhs_single) >> eol)];
    cols = lexeme[lit("COLUMNS") >> *blank >> eol
        >> +((col_double | col_single) >> eol)];
    quad = lexeme[lit("QUADOBJ") >> *blank >> eol >> +(quad_l >> eol)];
    bounds = lexeme[lit("BOUNDS") >> +space >> +((bound | bound_fr) >> eol)];
    ranges = lexeme[lit("RANGES") >> +space];
    end = lexeme[lit("ENDATA") >> *space];
    start = lexeme[name >> rows >> cols >> rhs >> -ranges >> bounds >> quad
        >> end];
  }

  qi::rule<boost::spirit::basic_istream_iterator<char>, char()> character;
  qi::rule<boost::spirit::basic_istream_iterator<char>, Chars()> word, title;
  qi::rule<boost::spirit::basic_istream_iterator<char> > row, end, col_single,
      col_double, rhs_single, rhs_double, ranges, bound, bound_fr, bounds, quad,
      quad_l, rows, cols, rhs, name, start;
};

QP QPSParser::Parse() {
  RawQP rawData;
  std::fstream stream(fileName_.c_str());
  stream.unsetf(std::ios::skipws);
  boost::spirit::basic_istream_iterator<char> begin(stream);
  boost::spirit::basic_istream_iterator<char> last;

  if (!parse(begin, last, MPSGrammar(&rawData)) || begin != last) {
    throw QPSParserException();
  }

  return rawData.makeQP();
}

}
