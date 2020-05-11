/*cppimport
<%
setup_pybind11(cfg)
from pathlib import Path
import pybind11
mypath=Path(filepath)
psimpath= mypath.parent.parent.parent
#flightsoftwarepath= psimpath.parent/'FlightSoftware'
# 'dependencies' controls what get checked for changes before recompile
cfg['dependencies'] = [str(p) for p in (psimpath/'src').rglob('*') if p.is_file()]
cfg['dependencies'] += [str(p) for p in (psimpath/'lib').rglob('*') if p.is_file()]
cfg['dependencies'] += [str(p) for p in (psimpath/'include').rglob('*') if p.is_file()]
#cfg['dependencies'] += [str(p) for p in (flightsoftwarepath/'src').rglob('*') if p.is_file()]
#cfg['dependencies'] += [str(p) for p in (flightsoftwarepath/'lib').rglob('*') if p.is_file()]

cfg['sources'] = ['lin_ext.cpp', 'orb_ext.cpp']
cfg['sources'] += [str(p) for p in (psimpath/'src').rglob('*.cpp') if (p.is_file() and p.parent.name!='targets')]
#cfg['sources'] += [str(p) for p in (flightsoftwarepath/'src/fsw').rglob('*.cpp') if (p.is_file() and p.parent.name!='targets')]
#cfg['sources'] += [str(p) for p in (flightsoftwarepath/'src/common').rglob('*.cpp') if (p.is_file() and p.parent.name!='targets')]

cfg['include_dirs'] = [str(psimpath/'include'), 
                       str(psimpath/'lib/lin/include')]
cfg['include_dirs'] += [pybind11.get_include(), pybind11.get_include(True)]
#cfg['include_dirs'] += [str(flightsoftwarepath/'src'), 
#                         str(flightsoftwarepath/'lib/common/libsbp/include'),
#                         str(flightsoftwarepath/'lib/common/ArduinoJson/src'),
#                         str(flightsoftwarepath/'lib/common/concurrentqueue')]

# compiler flags
cfg['compiler_args']= ['-std=c++14']
cfg['compiler_args']+= ['-DDESKTOP','-DFUNCTIONAL_TEST','-DUNIT_TEST']

#platform dependent compiler flags
from sys import platform
if platform == "linux" or platform == "linux2":
    # linux
    cfg['compiler_args'] += ['-fvisibility=hidden']
elif platform == "darwin":
    # OS X
    cfg['compiler_args'] += ['-mmacosx-version-min=10.9','-fvisibility=hidden']
elif platform == "win32":
    pass
    # Windows...
%>
*/
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <string>
#include <stdio.h>
#include <lin/core.hpp>
#include <lin/generators.hpp>

namespace py = pybind11;

void init_lin_ext(py::module &);
void init_orb_ext(py::module &);
//void init_common_ext(py::module &);

PYBIND11_MODULE(pwrap, m) {
    init_lin_ext(m);
    init_orb_ext(m);
    //init_common_ext(m);
}