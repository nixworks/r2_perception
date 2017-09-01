#include "Fusion.h"

#include "r2_perception/CandidateHand.h"
#include "r2_perception/EstablishedHand.h"

class HandFuse : public FusionModule {
    // (hands_id, saliency_data)
    using chands = std::map<unsigned int, r2_perception::CandidateHand::ConstPtr>;
    // (camera_id, chands)
    std::map<unsigned int, chands> cam_chands;
    // Established hands
    r2_perception::EstablishedHand ehand;

    void CHandCB(const r2_perception::CandidateFace::ConstPtr &msg){}

    public:
        void run(ros::NodeHandle* n);
        void publish (void);

};

