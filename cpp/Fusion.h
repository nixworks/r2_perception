
// Dynamic reconfigure includes.
#include "dynamic_reconfigure/server.h"
// Auto-generated from cfg/ directory.
//#include "r2_perception/hearing_pipelineConfig.h"
//#include "r2_perception/vision_pipelineConfig.h"



#ifndef _FUSION_H_
#define _FUSION_H_
/*
 * Design Rationale
 * - Modularity - Need to adapt to the addition of new sensors with minimal
 *   configuration.
 * - Plugin architectures - Loading and unloading of fusers should be smooth.
 * - Opencog integration? TODO
 *
 *   Non functional requirements
 *   ---------------------------
 *   - Composability 
 *   - Plug and play
 *   - Message based
 *   -  
 *
 *   Functional requirements
 *   -----------------------
 *   - Fusion combination 
 *      - face + saliency
 *      - hand + saliency
 *      - saliency crossing
 */
#define DECLARE_MODULE(MODNAME)                                   \
    /* load/unload functions for the Module interface */          \
extern "C" const char* fusion_module_id(void) {                   \
    return "fusion_module::" #MODNAME;                            \
}                                                                 \
extern "C" FusionModule * fusion_module_load() {                  \
    return new MODNAME();                                         \
}                                                                 \
extern "C" void fusion_module_unload(FusionModule* m) {           \
    delete m;                                                     \
}                                                                 \
inline const char * MODNAME::id(void) {                           \
    return "fusion_module::" #MODNAME;                            \
}

class FusionModule {
    public:
        // Implements fusion algorithms
        void virtual run(ros::NodeHandle* n) = 0;
        void virtual publish (void) = 0;
//        virtual ~FusionModule();
};

/*
 * FusionModule->run
 * FusionModule->publish
 */
#endif
