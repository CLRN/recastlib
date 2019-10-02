#include "config.h"
#include "mmap.h"

void export_mmaps()
{
    using namespace boost::python;

    class_<MMAP::MMapManager>("MMapManager", init<std::string>())
            .def("load", &MMAP::MMapManager::loadMap)
            .def("unload", &MMAP::MMapManager::unloadMap)
            .def("mesh", &MMAP::MMapManager::GetNavMesh,
                 return_value_policy<reference_existing_object>());
}
