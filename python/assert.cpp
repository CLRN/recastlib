#include "DetourAssert.h"

#define BOOST_PYTHON_STATIC_LIB
#include <boost/python.hpp>

void pythonAssert(const char* expression, const char* file, int line)
{
    using namespace boost::python;

    const auto error = std::string(file) + ":" + std::to_string(line);
    PyErr_SetString(PyExc_AssertionError, error.c_str());
    throw_error_already_set();
}

dtAssertFailFunc* dtAssertFailGetCustom()
{
    return pythonAssert;
}