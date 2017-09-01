#include "ros/ros.h"
#include "Fusion.h"

class FusionServer{
    private:
        std::vector<FusionModule*> _modules;
        ros::Rate _loop_rate;

    public:
        FusionServer(unsigned int loop_rate = 10);
        ~FusionServer();

        // Dynamical loader for ROS fusion nodes
        bool load(std::string path);
        void unload(std::string fusion);

        // This is where all the ros topics are published.
        void run(void){
            while (ros::ok()){
                for(const FusionModule* module : _modules){
                    module->publish();
                } 
                ros::spinOnce();
                loop_rate.sleep();
            }
        }
};


