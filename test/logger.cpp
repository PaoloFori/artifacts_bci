#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <artifacts_bci/artifact_presence.h> 
#include "artifacts_bci/utils.hpp" 
#include <string>
#include <vector>


class LoggerNode {
public:
    LoggerNode(ros::NodeHandle& nh) {
        if (!nh.getParam("output_filename", output_filename_)) {
            output_filename_ = "features_output.csv";
            ROS_WARN("Parameter 'output_filename' doesn't found. Default: %s", output_filename_.c_str());
        }

        std::string topic = "/artifact_presence";
        sub_ = nh.subscribe(topic, 1, &LoggerNode::callback, this);

        this->artifacts_ = std::vector<int>(this->max_expected_size_, 0);
    }

    ~LoggerNode() {
        if (this->artifacts_.empty()) {
            ROS_WARN("No data received.");
            return;
        }

        int n_samples = this->artifacts_.size();

        ROS_INFO("Received %d campioni. Creation final matrix [samples x 1]: (%d x 1)...",
                 n_samples, n_samples);

        Eigen::MatrixXi final_vector = Eigen::Map<Eigen::VectorXi>(this->artifacts_.data(), this->artifacts_.size());

        writeCSV<int>(output_filename_, final_vector);
    }


    void callback(const artifacts_bci::artifact_presence::ConstPtr& msg) {
        bool c_has_artifact = msg->has_artifact;
        std::cout << "Received message with seq: " << msg->seq << std::endl;
        if(c_has_artifact)
            this->artifacts_[msg->seq] = 1;
    }

private:
    ros::Subscriber sub_;
    std::string output_filename_;
    
    const int max_expected_size_ = 200;
    std::vector<int> artifacts_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_subscriber");
    ros::NodeHandle nh("~"); 

    LoggerNode logger(nh);

    ros::spin(); 

    return 0; 
}