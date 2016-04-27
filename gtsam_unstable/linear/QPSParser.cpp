//
// Created by ivan on 3/5/16.
//
#define BOOST_SPIRIT_USE_PHOENIX_V3 1

#include <gtsam_unstable/linear/QPSParser.h>
#include <boost/spirit/include/qi.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/phoenix/bind.hpp>
#include <boost/spirit/include/classic.hpp>

namespace gtsam {
typedef boost::spirit::qi::grammar<boost::spirit::basic_istream_iterator<char>> base_grammar;

struct QPSParser::MPSGrammar: base_grammar {
  RawQP * rqp_;
  boost::function<
      void(
          boost::fusion::vector<std::vector<char>, std::vector<char>,
              std::vector<char>> const&)> setName;
  boost::function<
      void(
          boost::fusion::vector<std::vector<char>, char, std::vector<char>,
              std::vector<char>, std::vector<char>> const &)> addRow;
  boost::function<
      void(
          boost::fusion::vector<std::vector<char>, std::vector<char>,
              std::vector<char>, std::vector<char>, std::vector<char>, double,
              std::vector<char> > const &)> rhsSingle;
  boost::function<
      void(
          boost::fusion::vector<std::vector<char>, std::vector<char>,
              std::vector<char>, std::vector<char>, std::vector<char>, double,
              std::vector<char>, std::vector<char>, std::vector<char>, double>)> rhsDouble;
  boost::function<
      void(
          boost::fusion::vector<std::vector<char>, std::vector<char>,
              std::vector<char>, std::vector<char>, std::vector<char>, double,
              std::vector<char>>)> colSingle;
  boost::function<
      void(
          boost::fusion::vector<std::vector<char>, std::vector<char>,
              std::vector<char>, std::vector<char>, double, std::vector<char>,
              std::vector<char>, std::vector<char>, double> const &)> colDouble;
  boost::function<
      void(
          boost::fusion::vector<std::vector<char>, std::vector<char>,
              std::vector<char>, std::vector<char>, std::vector<char>, double,
              std::vector<char>> const &)> addQuadTerm;
  boost::function<
      void(
          boost::fusion::vector<std::vector<char>, std::vector<char>,
              std::vector<char>, std::vector<char>, std::vector<char>,
              std::vector<char>, std::vector<char>, double> const &)> addBound;
  boost::function<
      void(
          boost::fusion::vector<std::vector<char>, std::vector<char>,
              std::vector<char>, std::vector<char>, std::vector<char>,
              std::vector<char>, std::vector<char>> const &)> addBoundFr;
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
    rhs_single = lexeme[*blank >>  word >> +blank >> word >> +blank>> double_
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
    bound = lexeme[*blank >> word >> +blank >> word >> +blank >> word >> +blank
        >> double_ >> *blank][addBound];
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

  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char>, char()> character;
  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char>,
      std::vector<char>()> word;
  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char>,
      std::vector<char>()> title;
  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char> > row;
  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char> > end;
  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char> > col_single;
  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char> > col_double;
  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char> > rhs_single;
  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char> > rhs_double;
  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char> > ranges;
  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char> > bound;
  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char> > bound_fr;
  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char> > bounds;
  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char> > quad;
  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char> > quad_l;
  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char> > rows;
  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char> > cols;
  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char> > rhs;
  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char> > name;
  boost::spirit::qi::rule<boost::spirit::basic_istream_iterator<char> > start;
};

QP QPSParser::Parse() {
  RawQP rawData;
  std::fstream stream(fileName_.c_str());
  stream.unsetf(std::ios::skipws);
  boost::spirit::basic_istream_iterator<char> begin(stream);
  boost::spirit::basic_istream_iterator<char> last;

  if (!parse(begin, last, MPSGrammar(&rawData)) || begin != last) {
    throw QPSParserException();
  } else {
    std::cout << "Parse Successful." << std::endl;
  }

  return rawData.makeQP();
}

}
