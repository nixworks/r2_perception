#include "ros/ros.h"
#include "Fusion.h"

FusionServer::FusionServer(unsigned int loop_rate/*=10*/) :_loop_rate(loop_rate){

}

bool FusionServer::load(std::string filename){
    void *dynLibrary = dlopen(filename.c_str(), RTLD_LAZY | RTLD_GLOBAL);
    const char* dlsymError = dlerror();
    if ((dynLibrary == NULL) || (dlsymError)) {
        //TODO ROS log
        // This is almost surely due to a user configuration error.
        // User errors are always logged as warnings.
        return false;
    }

    // reset error
    dlerror();

    // search for 'load' & 'unload' symbols
    FusionModule* module = (FusionModule* (*) (void)) dlsym(dynLibrary, "fusion_module_load");
    dlsymError = dlerror();
    if (dlsymError) {
        //TODO ROS log
        return false;
    }

    // push publish to the list of publishers
    //TODO make sure the module is not added more than once.
    modules.push_back(module);

    // load and init module
    module->run();

    return true;
}

void FusionServer::unload(std::string fusion){

}
