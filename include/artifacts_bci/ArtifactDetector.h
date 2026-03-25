#ifndef ARTIFACT_DETECTOR_H
#define ARTIFACT_DETECTOR_H

#include <ros/ros.h>
#include <rosneuro_msgs/NeuroFrame.h>
#include <rosneuro_buffers_ringbuffer/RingBuffer.h>
#include <rosneuro_filters_butterworth/Butterworth.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include "artifacts_bci/artifact_presence.h"
#include "rosneuro_filters_car/Car.h"


class ArtifactDetector {
public:
    enum class ApplyResults {BufferNotFull = 0, Error = 1, Success = 2};
    ArtifactDetector();
    ~ArtifactDetector();

    void on_received_data(const rosneuro_msgs::NeuroFrame &msg);
    bool configure(const std::string& param_name);
    ApplyResults apply(void);
    void run(void);
    void set_message();

    bool getParam(const std::string& name, std::string& value) const;
    bool getParam(const std::string& name, bool& value) const;
    bool getParam(const std::string& name, double& value) const;
    bool getParam(const std::string& name, int& value) const;
    bool getParam(const std::string& name, unsigned  int& value) const;
    bool getParam(const std::string& name, std::vector<double>& value) const;
    bool getParam(const std::string& name, std::vector<int>& value) const;
    bool getParam(const std::string& name, std::vector<std::string>& value) const;
    bool getParam(const std::string& name, XmlRpc::XmlRpcValue& value) const;

protected:

    bool loadConfiguration(XmlRpc::XmlRpcValue& config);
    bool configure(XmlRpc::XmlRpcValue& config);
    bool configure(void);
    bool setName(XmlRpc::XmlRpcValue& config);

    std::map<std::string, XmlRpc::XmlRpcValue> params_;

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    std::vector<rosneuro::Buffer<float>*> buffers_;
    bool has_new_data_;
    rosneuro::DynamicMatrix<float> data_in_;
    int chunkSize_;
    int nchannels_;
    bool has_artifact_;
    double th_hEOG_, th_vEOG_, th_peaks_;
    std::vector<int> EOG_ch_; // channel indexes for EOG detection are in 1-based notion
    int seq_id_;
    bool is_configured_;

    rosneuro::Car<float> car_filter_;

    rosneuro::Butterworth<double> filter_low_EOG_;
    rosneuro::Butterworth<double> filter_high_EOG_;
    rosneuro::Butterworth<double> filter_high_peaks_;

    artifacts_bci::artifact_presence out_;
    std::string run_mode_, signal_type_;

    std::string name_;

};


#endif