#include <ros/ros.h>
#include <rosneuro_msgs/NeuroFrame.h>
#include <eigen3/Eigen/Dense> 
#include <vector>
#include <string>
#include "artifacts_bci/utils.hpp" 

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_publisher_csv");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); 

    std::string topic = "/neurodata";
    std::string csv_filename;
    int n_samples;     
    double sample_rate;

    if (!private_nh.getParam("csv_file", csv_filename)) {
        ROS_ERROR("Parametro 'csv_file' non impostato! Specificare il percorso del CSV.");
        return 1;
    }
    if (!private_nh.getParam("chunk_size", n_samples)) {
        ROS_ERROR("Parametro 'chunk_size' non impostato! (es. 32)");
        return 1;
    }
    if (!private_nh.getParam("sample_rate", sample_rate)) {
        ROS_ERROR("Parametro 'sample_rate' non impostato! (es. 512)");
        return 1;
    }

    ROS_INFO("Loading dati from: %s", csv_filename.c_str());
    Eigen::MatrixXd full_data;
    try {
        full_data = readCSV<double>(csv_filename); 
    } catch (const std::exception& e) {
        ROS_ERROR("Error durign the CSV reading: %s", e.what());
        return 1;
    }
    
    int n_channels = full_data.cols();
    int total_samples = full_data.rows();
    ROS_INFO("Loaded data: %d samples x %d channels.", total_samples, n_channels);

    // Standard 32-ch LiveAmp labels – needed by ArtifactDetector to resolve EOG channel names
    const std::vector<std::string> ch_labels_32 = {
        "Fp1","Fp2","F3","Fz","F4","FC1","FC2","C3","Cz","C4",
        "CP1","CP2","P3","Pz","P4","POz","O1","O2","CPz","F1",
        "F2","FC3","FCz","FC4","C1","C2","CP3","CP4","P5","P1","P2","P6"
    };
    std::vector<std::string> ch_labels;
    if(n_channels == static_cast<int>(ch_labels_32.size())){
        ch_labels = ch_labels_32;
    } else {
        for(int i = 0; i < n_channels; i++)
            ch_labels.push_back("ch" + std::to_string(i + 1));
        ROS_WARN("Non-standard channel count (%d) – using generic labels ch1..ch%d", n_channels, n_channels);
    }

    ros::Publisher pub = nh.advertise<rosneuro_msgs::NeuroFrame>(topic, 1);
    ros::Rate loop_rate(sample_rate / n_samples);

    ROS_INFO("Waiting for a  subscriber on topic '%s'...", topic.c_str());
    while (ros::ok() && pub.getNumSubscribers() == 0) {
        ros::Duration(0.5).sleep(); 
        ROS_INFO_THROTTLE(5.0, "Still waiting...");
    }
    ROS_INFO("Subscriber connected. Start pubblication.");

    int current_sample = 0;
    while (ros::ok()) {
        
        if (current_sample + n_samples > total_samples) {
            ROS_INFO("Fine del file CSV.");
            break;
        }

        Eigen::MatrixXd chunk = full_data.block(current_sample, 0, n_samples, n_channels);

        rosneuro_msgs::NeuroFrame msg;
        msg.header.stamp = ros::Time::now();
        msg.header.seq = current_sample / n_samples;
        msg.sr = sample_rate;
        msg.eeg.info.nchannels = n_channels;
        msg.eeg.info.nsamples  = n_samples;
        msg.eeg.info.labels    = ch_labels;
        msg.neuroheader.seq = current_sample/n_samples;
        
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> chunk_float;
        chunk_float = chunk.cast<float>();
        msg.eeg.data.assign(chunk_float.data(), chunk_float.data() + chunk_float.size());
        pub.publish(msg);
        std::cout << "seq sended: " << msg.neuroheader.seq << std::endl;
        
        current_sample += n_samples;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}