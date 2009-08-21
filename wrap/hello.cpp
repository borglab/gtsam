// example for wrapping python with boost
// from http://www.boost.org/doc/libs/1_37_0/libs/python/doc/tutorial/doc/html/index.html

char const* greet()
{
   return "hello, world";
}

#include <boost/python.hpp>

BOOST_PYTHON_MODULE(hello_ext)
{
    using namespace boost::python;
    def("greet", greet);
}
