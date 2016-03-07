//
// Created by ivan on 3/5/16.
//

#include <gtsam_unstable/linear/QPSParser.h>
#include <boost/spirit/include/qi.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/phoenix/bind.hpp>

#define BOOST_SPIRIT_USE_PHOENIX_V3 1

using namespace boost::spirit;
using namespace boost::spirit::qi;
using namespace boost::spirit::qi::labels;

namespace gtsam {

struct QPSParser::MPSGrammar: grammar<basic_istream_iterator<char>> {
  MPSGrammar(RawQP * rqp) :
      MPSGrammar::base_type(start), rqp_(rqp) {
    character = lexeme[char_("a-zA-Z") | char_('_')];
    title = lexeme[+character
        >> *(blank | character | char_('-') | char_('.') | char_("0-9"))];
    word = lexeme[(character
        >> *(char_("0-9") | char_('-') | char_('.') | character))];
    name =
        lexeme[lit("NAME") >> *blank
            >> title[boost::phoenix::bind(&RawQP::setName, rqp_, qi::_1)]
            >> +space];
    row = lexeme[*blank
        >> (character >> +blank >> word)[boost::phoenix::bind(&RawQP::addRow,
            rqp_, qi::_1, qi::_3)] >> *blank];
    rhs_single = lexeme[*blank
        >> (word >> +blank >> word >> +blank >> double_)[boost::phoenix::bind(
            &RawQP::addRHS, rqp_, qi::_1, qi::_3, qi::_5)] >> *blank];
    rhs_double = lexeme[*blank
        >> (word >> +blank >> word >> +blank >> double_ >> +blank >> word
            >> +blank >> double_)[boost::phoenix::bind(&RawQP::addRHS, rqp_,
            qi::_1, qi::_3, qi::_5, qi::_7, qi::_9)] >> *blank];
    col_single = lexeme[*blank
        >> (word >> +blank >> word >> +blank >> double_)[boost::phoenix::bind(
            &RawQP::addColumn, rqp_, qi::_1, qi::_3, qi::_5)] >> *blank];
    col_double = lexeme[*blank
        >> (word >> +blank >> word >> +blank >> double_ >> +blank >> word
            >> +blank >> double_)[boost::phoenix::bind(&RawQP::addColumn, rqp_,
            qi::_1, qi::_3, qi::_5, qi::_7, qi::_9)] >> *blank];
    quad_l = lexeme[*blank
        >> (word >> +blank >> word >> +blank >> double_)[boost::phoenix::bind(
            &RawQP::addQuadTerm, rqp_, qi::_1, qi::_3, qi::_5)] >> *blank];
    bound =
        lexeme[*blank
            >> (word >> +blank >> word >> +blank >> word >> +blank >> double_)[boost::phoenix::bind(
                &RawQP::addBound, rqp_, qi::_1, qi::_5, qi::_7)] >> *blank];
    bound_fr = lexeme[*blank
        >> (word >> +blank >> word >> +blank >> word)[boost::phoenix::bind(
            &RawQP::addBound, rqp_, qi::_1, qi::_5)] >> *blank];
    rows = lexeme[lit("ROWS") >> *blank >> eol >> +(row >> eol)];
    rhs = lexeme[lit("RHS") >> *blank >> eol
        >> +((rhs_double | rhs_single) >> eol)];
    cols = lexeme[lit("COLUMNS") >> *blank >> eol
        >> +((col_double | col_single) >> eol)];
    quad = lexeme[lit("QUADOBJ") >> *blank >> eol >> +(quad_l >> eol)];
    bounds = lexeme[lit("BOUNDS") >> +space >> +((bound | bound_fr) >> eol)];
    ranges = lexeme[lit("RANGES") >> +space];
    start = lexeme[name >> rows >> cols >> rhs >> ranges >> bounds >> quad
        >> lit("ENDATA") >> +space];
  }
  RawQP * rqp_;

  rule<basic_istream_iterator<char>, char()> character;
  rule<basic_istream_iterator<char>, std::vector<char>()> word;
  rule<basic_istream_iterator<char>, std::vector<char>()> title;
  rule<basic_istream_iterator<char> > row;
  rule<basic_istream_iterator<char> > col_single;
  rule<basic_istream_iterator<char> > col_double;
  rule<basic_istream_iterator<char> > rhs_single;
  rule<basic_istream_iterator<char> > rhs_double;
  rule<basic_istream_iterator<char> > ranges;
  rule<basic_istream_iterator<char> > bound;
  rule<basic_istream_iterator<char> > bound_fr;
  rule<basic_istream_iterator<char> > bounds;
  rule<basic_istream_iterator<char> > quad;
  rule<basic_istream_iterator<char> > quad_l;
  rule<basic_istream_iterator<char> > rows;
  rule<basic_istream_iterator<char> > cols;
  rule<basic_istream_iterator<char> > rhs;
  rule<basic_istream_iterator<char> > name;
  rule<basic_istream_iterator<char> > start;
};

QP QPSParser::Parse() {
  RawQP rawData;
  boost::spirit::basic_istream_iterator<char> begin(stream);
  boost::spirit::basic_istream_iterator<char> last;

  if (!parse(begin, last, MPSGrammar(&rawData)) && begin == last) {
    throw QPSParserException();
  }

  return rawData.makeQP();
}

}
