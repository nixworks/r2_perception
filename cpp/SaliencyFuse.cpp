#include "SaliencyFuse.h"

SaliencyFuse::run(ros::NodeHandle* n){
    _sub_csaliency = n->subscribe("csalieny", 1000, &SaliencyFuse::CSaliencyCB, this);
    _pub_saliency = n->advertise<EstablishedSaliency>("esaliency",5);
}

SaliencyFuse::publish(void){
    while(not _esaliencies.empty()){
      pub_saliency->publish(_esaliencies.front()); 
      esaliencies.pop();
    }
}


void SaliencyFuse::CSaliencyCB(const r2_perception::CandidateSaliency::ConstPtr &msg){
    csaliencyMsgBuffer.push_back(msg);
}

// Fuse each saliency msg with all the subsequent messages in the buffer whenever they
// meet the Fusing criteria stated in the doc.
void SaliencyFuse::FuseCSalencyVec(void){
    // Read these from confg. statrting points
    float xa,ya,za; //x,y,z coordinates for starting point a
    float xb,yb,zb;  //x,y,z coordinates for starting point b
    float K; // threshold distance to be read from config file.

    for(unsigned int i = 0; i < csaliencyMsgBuffer.size() ; i++){
        CandidateSaliency cs1 = csaliencyMsgBuffer[i];
        float na_x = cs1.direction.x;
        float na_y = cs1.direction.y;
        float nb_z = cs1.direction.z;
        for(unsigned int j = i+1 ; j < csaliencyMsgBuffer.size()-1 ; j++){
            CandidateSaliency cs2 = csaliencyMsgBuffer[j];
            float nb_x = cs2.direction.x;
            float nb_y = cs2.direction.y;
            float nb_z = cs2.direction.z;
            // Do the calculation here.
            float c = xb.nb_x + yb.nb_y + zb.nb_z;
            float d = nb_x.nb_x + nb_y.nb_y + nb_z.nb_z;
            float e = xa.nb_x + ya.nb_y + za.nb_z;
            float f = nb_x.na_x + nb_y.na_y + nb_z.na_z;
            float g = xb.na_x + yb.na_y + zb.na_z;
            float h = f; 
            float i = xa.na_x + ya.na_y + za.na_z;
            float j =  na_x.na_x + na_y.na_y + na_z.na_z; 

            float a = (i* d - g* d - e* h + c* h) / (f* h - j* d);
            float b = (e - c + a* f) / d;

            //Pa = Ba + aNa
            float pa_x = (xa + a*na_x), pa_y = (ya + a* na_y), pa_z = (za + a * na_z);
            //Pb = Bb + bNb
            float pb_x = (xb + b*nb_x), pb_y = (yb + b * nb_y), pb_z = (zb + b * nb_z);

            float distance = sqrt(pow(pa_x - pb_x, 2) + pow(pa_y - pb_y, 2) + pow(pa_z - pb_z, 2))/2;

            // Fuse the points if they are not too distant to each other.
            if ( distancne <= K ){
                // Pf = 1/2(Pa + Pb)
                // TODO use confidence as a weight
                float pf_x = (pa_x + pb_x)/2, pf_y = (pa_y + pb_y)/2, pf_z = (pa_z + pb_z)/2;
                // Set i to the next unfused saliency message.
                i = j+1;
                //Create Established Saliency messages.
                EstblishedSaliency f;
                f.direction.x = pf_x; 
                f.direction.y = pf_y; 
                f.direction.z = pf_z; 

                f.motion = cs1.confidence* cs1.motion + cs2.confidence * cs2.motion;
                f.vmap = cs1.vmap;
                f.umap = cs1.umap;
                // TODO fill in the rest of the params defined in the
                // EstablishedSaliency message.
                _esaliencies.push(f);
            } else {
                EstablishedSaliency i,j;
                i.direction = cs1.direction; 
                i.vmap = cs1.vmap;
                i.umap = cs1.umap; 
                i.motion = cs1.motion;

                j.direction = cs2.direction; 
                j.vmap = cs2.vmap;
                j.umap = cs2.umap;
                j.motion = cs2.motion;

                // TODO fill in the rest of the params defined in the
                // EstablishedSaliency message.
                _esaliencies.push(i);
                _esaliencies.push(j);
            }
        } 

        // clear the buffer
        csaliencyMsgBuffer.clear();
    }
}
