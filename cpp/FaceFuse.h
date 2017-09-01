#include "Fusion.h"

#include "ros/ros.h"
#include "ros/time.h"

#include "r2_perception/CandidateFace.h"
#include "r2_perception/EstablishedFace.h"

#include "timeoctomap/TimeOctomap.h"

#include <chrono>

using namespace r2_perception;


/**
 * create three timeoctomaps
 * iterate through each face key
 * calculate distance and fuse the points
 * store the fused in the third timeoctomap
 *
 *
 */
class FaceFuse : virtual public FusionModule {
    private:
        ros::Subscriber _sub_leye_cface;
        ros::Subscriber _sub_reye_cface;
        ros::Subscriber _sub_rlsense_cface;
        ros::Subscriber _sub_wideangle_cface;

        // (face_id, saliency_data)
        using cfaces = std::map<unsigned int, CandidateFace::ConstPtr>;
        // (camera_id, cfaces)
        std::map<unsigned int, cfaces>  cam_cfaces;
        // Established face
        EstablishedFace eface;
        std::vector<EstablishedFace> efaces;
        struct FaceLink{
            unsigned int camera_id;
            unsigned int cface_id;
        };

        opencog::TimeOctomap<EstablishedFace> octmapMain;
        ros::Publisher * pub_eface;

        void init_eface(void){
            eface.position.x = 0.0;
            eface.position.y = 0.0;
            eface.position.z = 0.0;
            eface.confidence = 0.0;
            eface.smile = 0.0;
            eface.frown = 0.0;
            eface.expressions.push_back("");
            eface.age = 0.0;
            eface.age_confidence = 0.0;
            eface.gender = 0;
            eface.gender_confidence = 0.0;
            eface.identity = 0;
            eface.identity_confidence = 0;
        }

        void processCFaces(void); //XXX When this function is invoked as a plugin how does it affect the control flow?
        void rocessFaceCBWithOctomap(void);
    public:
        // Candidate Face topic call back handler
        void CFaceCB(const CandidateFace::ConstPtr &msg);

        FaceFuse() : octmapMain(100, 0.1, std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::milliseconds(100)))
                     {};
        // Implement Publish

        void publish (void);
        void run(ros::NodeHandle* node);
};

