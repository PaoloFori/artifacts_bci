#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <artifacts_bci/artifact_presence.h>
#include "artifacts_bci/utils.hpp"
#include <string>
#include <vector>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cstdlib>

class LoggerNode {
public:
    LoggerNode(ros::NodeHandle& nh) {
        if (!nh.getParam("output_filename", output_filename_)) {
            output_filename_ = "artifacts_output.csv";
            ROS_WARN("[Logger] 'output_filename' not set. Default: %s", output_filename_.c_str());
        }
        // Create output directory if it doesn't exist
        auto slash = output_filename_.rfind('/');
        if (slash != std::string::npos) {
            std::string dir = output_filename_.substr(0, slash);
            std::system(("mkdir -p " + dir).c_str());
        }

        // first_seq file sits next to the output CSV
        first_seq_filename_ = output_filename_.substr(0, output_filename_.rfind('.')) + "_first_seq.txt";

        sub_ = nh.subscribe("/artifact_presence", 10, &LoggerNode::callback, this);
        ROS_INFO("[Logger] Saving artifact flags to: %s", output_filename_.c_str());
        ROS_INFO("[Logger] First-seq offset will be saved to: %s", first_seq_filename_.c_str());
    }

    ~LoggerNode() {
        if (artifacts_.empty()) {
            ROS_WARN("[Logger] No data received.");
            return;
        }

        // Save artifacts indexed by seq (positions 0..first_seq-1 are zero – never received)
        uint32_t n = max_seq_ + 1;
        Eigen::VectorXi out = Eigen::VectorXi::Zero(n);
        for (uint32_t i = 0; i < n && i < artifacts_.size(); i++)
            out(i) = artifacts_[i];

        writeCSV<int>(output_filename_, out);
        ROS_INFO("[Logger] Saved %u frames to %s", n, output_filename_.c_str());

        // Save first_seq so MATLAB can compute the correct alignment offset
        std::ofstream fs(first_seq_filename_);
        if (fs.is_open()) {
            fs << first_seq_ << "\n";
            fs.close();
            ROS_INFO("[Logger] First seq received: %u → saved to %s", first_seq_, first_seq_filename_.c_str());
        } else {
            ROS_ERROR("[Logger] Could not write first_seq file: %s", first_seq_filename_.c_str());
        }
    }

    void callback(const artifacts_bci::artifact_presence::ConstPtr& msg) {
        uint32_t seq = msg->seq;

        if (first_call_) {
            first_seq_ = seq;
            first_call_ = false;
            ROS_INFO("[Logger] First seq received: %u", seq);
        }

        if (seq >= artifacts_.size())
            artifacts_.resize(seq + 1, 0);

        artifacts_[seq] = msg->has_artifact ? 1 : 0;
        max_seq_ = std::max(max_seq_, seq);
    }

private:
    ros::Subscriber sub_;
    std::string output_filename_;
    std::string first_seq_filename_;
    std::vector<int> artifacts_;
    uint32_t max_seq_   = 0;
    uint32_t first_seq_ = 0;
    bool first_call_    = true;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_logger_artifacts");
    ros::NodeHandle nh("~");

    LoggerNode logger(nh);
    ros::spin();

    return 0;
}
