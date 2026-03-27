#include <ros/ros.h>
#include "artifacts_bci/ArtifactDetector.h"

int main(int argc, char** argv) {

    
    // ros initialization
    ros::init(argc, argv, "artifactDetector_node");

    ArtifactDetector bci_artifact_detector;
    
    if(bci_artifact_detector.configure("ArtifactCfg") == false) {
        ROS_ERROR("[ArtifactDetector] Configuration failed");
        ros::shutdown();
        return 0;
    }
    
    bci_artifact_detector.run();
    
    ros::shutdown();
    return 0;
}
