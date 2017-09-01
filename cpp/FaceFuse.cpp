#include "FaceFuse.h"

using namespace opencog;

#define FACE_FUSE_DISTANCE 0.2

void FaceFuse::CFaceCB(const CandidateFace::ConstPtr &msg){

    auto it = cam_cfaces.find(msg->camera_id);
    if(it != cam_cfaces.end()){
        //XXX do we need this mutex?
        //std::lock_guard<std::mutex> lock(face_mtx);
        (it->second)[msg->cface_id] = msg;
    } else {
        cam_cfaces[msg->camera_id] = {{msg->cface_id, msg}};
    }
    // To be fused at later stage. 
    auto d = msg->position;
    std::vector<EstablishedFace> fmsgs;
    TimeSlice<EstablishedFace> tmsg = octmapMain.get_current_timeslice(); 
    
    processCFaces();
    
    // Delete points close to Fused ones in the octomap and store the new ones.
    for (typename AtomOcTree<EstablishedFace>::tree_iterator it2 = tmsg.map_tree.begin_tree(),
            endit2 = tmsg.map_tree.end_tree();
            it2 != endit2;
            ++it2) {
        auto it = std::find_if(efaces.begin(), efaces.end(),[it2](const EstablishedFace& ef){ 
                EstablishedFace e = it2->getData();
                float dx = ef.position.x - e.position.x;
                float dy = ef.position.y - e.position.y;
                float dz = ef.position.z - e.position.z;
                float distance = sqrt(dx*dx + dy*dy + dz*dz);
                return (FACE_FUSE_DISTANCE <= distance);
                });

        if( it == efaces.end()){
            tmsg.map_tree.deleteNode(it2.getCoordinate());
        }
    }

    for ( auto ef : efaces){
        auto p = eface.position;
        // Add to octomap as well
        octmapMain.insert_atom(point3d(p.x, p.y, p.z), ef);
    }
}

/* Called from the main module periodically*/
void FaceFuse::publish(void){
    processCFaces();
    for(const EstablishedFace& eface : efaces){
        pub_eface->publish(eface);
    }
    //Clear all sent messages.
    efaces.clear();
}

void FaceFuse::run(ros::NodeHandle* n){
    _sub_leye_cface = n->subscribe("lefteye/cface", 1000, &FaceFuse::CFaceCB, this);
    _sub_reye_cface = n->subscribe("righteye/cface", 1000, &FaceFuse::CFaceCB, this);
    _sub_rlsense_cface = n->subscribe("realsense/cface", 1000, &FaceFuse::CFaceCB, this);
    _sub_wideangle_cface = n->subscribe("wideangle/cface", 1000, &FaceFuse::CFaceCB, this);

    *pub_eface = n->advertise<EstablishedFace>("face", 5);
}

void FaceFuse::processCFaces(void){
    std::vector<std::vector<FaceLink>> facegroups;

    for(const auto& p1 : cam_cfaces){
        for(const auto& p2 : cam_cfaces){
            unsigned int cam_id1 = p1.first;
            unsigned int cam_id2 = p2.first;
            // the IDs are numeric hashes based on the unique pipeline name
            if( cam_id1 < cam_id2 ){ 
                std::map<unsigned int, CandidateFace::ConstPtr> fdata1 = p1.second;
                std::map<unsigned int, CandidateFace::ConstPtr> fdata2 = p2.second;
                for(const auto& pfd1 : fdata1){
                    for(const auto& pfd2 : fdata2){
                        float dx = pfd1.second->position.x - pfd2.second->position.x;
                        float dy = pfd1.second->position.y - pfd2.second->position.y;
                        float dz = pfd1.second->position.z - pfd2.second->position.z;
                        float distance = sqrt(dx*dx + dy*dy + dz*dz);

                        if(distance < FACE_FUSE_DISTANCE){
                            //TODO find cface code here.
                            FaceLink flink_2;
                            flink_2.camera_id = cam_id2;
                            flink_2.cface_id = pfd2.second->cface_id;

                            // iterate overall combinations of faces and group them.
                            auto it = std::find_if(facegroups.begin(), facegroups.end(),
                                    [=](std::vector<FaceLink> vec){
                                    auto it = std::find_if(vec.begin(), vec.end(),
                                        [=](const FaceLink& fl){ 
                                        return pfd1.second->camera_id == cam_id1
                                        and fl.cface_id == pfd1.second->cface_id;
                                        });

                                    return it != vec.end(); 
                                    });

                            if(it != facegroups.end()){
                                (*it).push_back(flink_2);
                            }else{
                                FaceLink flink_1;
                                flink_1.camera_id =  cam_id1;
                                flink_1.cface_id = pfd1.second->cface_id;
                                std::vector<FaceLink> groups;
                                groups.push_back(flink_1);        
                                groups.push_back(flink_2);     
                                facegroups.push_back(groups);   
                            }
                        }
                    }
                }
            }
        }
    }

    for (std::vector<FaceLink>& group : facegroups){
        init_eface();
        for(const FaceLink& link : group){
            auto id = link.cface_id;
            eface.position.x += cam_cfaces[link.camera_id][link.cface_id]->position.x;
            eface.position.y += cam_cfaces[link.camera_id][link.cface_id]->position.y;
            eface.position.z +=  cam_cfaces[link.camera_id][link.cface_id]->position.z;
            eface.confidence += cam_cfaces[link.camera_id][link.cface_id]->confidence;
            eface.smile += cam_cfaces[link.camera_id][link.cface_id]->smile;
            eface.frown += cam_cfaces[link.camera_id][link.cface_id]->frown;
            //FIXME eface.expressions.insert(eface.expressions.end(), (cfaces[id]->expressions).begin(), 
            //                         (cfaces[id]->expressions).end());
            eface.age += cam_cfaces[link.camera_id][link.cface_id]->age;
            eface.age_confidence += cam_cfaces[link.camera_id][link.cface_id]->age_confidence;
        }

        size_t n = group.size();
        eface.position.x /= n;
        eface.position.y /= n;
        eface.position.z /= n;
        eface.confidence /= n;
        eface.smile /= n;
        eface.frown /= n;
        eface.age /= n;
        eface.age_confidence /= n;

        efaces.push_back(eface);

    }

    // TODO: gender is the most likely of any of the group
    // TODO: identity is the most likely of any of the group
    // create established face for all faces not referenced in any group
    for(const auto& p : cam_cfaces){
        for (const std::vector<FaceLink>& group : facegroups){
            cfaces cfs = p.second;
            bool found = false;
            for(const auto& p2 : cfs ){
                for(const FaceLink& link : group){
                    if (link.camera_id == p.first and link.cface_id == p2.first){}
                    found = true;
                    break;
                }
                if(found) {
                    break;
                } else {
                    //FIXME
                    //eface.session_id = self.session_id
                    //eface.ts = ts
                    eface.position.x = (p2.second)->position.x;
                    eface.position.y = (p2.second)->position.y;
                    eface.position.z = (p2.second)->position.z;
                    eface.confidence = (p2.second)->confidence;
                    eface.smile = (p2.second)->smile;
                    eface.frown = (p2.second)->frown;
                    //FIXME
                    //eface.expressions = (p.second)->expressions;
                    eface.age = (p2.second)->age;
                    eface.age_confidence = (p2.second)->age_confidence;
                    eface.gender = (p2.second)->gender;
                    eface.gender_confidence = (p2.second)->gender_confidence;
                    eface.identity = (p2.second)->identity;
                    eface.identity_confidence = (p2.second)->identity_confidence;

                    efaces.push_back(eface);
                }
            }
        }
    }
}

int main(int argc, char** args){
    ros::Rate loop_rate(10);
    FaceFuse module;
    ros::NodeHandle node;
    module.run(&node);

    while (ros::ok()){
        module.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
