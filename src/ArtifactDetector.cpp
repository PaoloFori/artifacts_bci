#include "artifacts_bci/ArtifactDetector.h"

ArtifactDetector::ArtifactDetector(void) : nh_() { 
    this->pub_ = this->nh_.advertise<artifacts_bci::artifact_presence>("/artifact_presence", 1);
    this->sub_ = this->nh_.subscribe("/neurodata", 1, &ArtifactDetector::on_received_data, this);

    this->has_new_data_ = false;
    this->has_artifact_ = false;
    this->is_configured_ = false;
    this->is_signal_configured_ = false;

    this->name_ = "UnconfiguredArtifactDetector";
}
ArtifactDetector::~ArtifactDetector(){
    for(auto& buffer : this->buffers_)
        delete buffer;
}

void ArtifactDetector::run(){
    ros::Rate r(512);
    if(this->is_configured_ == false){
        ROS_ERROR("[%s] ArtifactDetector not configured correctly", this->name_.c_str());
        return;
    }

    while(ros::ok()){
        if(this->has_new_data_){
            ArtifactDetector::ApplyResults res = this->apply();
            this->has_new_data_ = false;
            
            if(res == ArtifactDetector::ApplyResults::Error){
                ROS_ERROR("[%s] Error in ArtifactDetector processing", this->name_.c_str());
                break;
            }else if(res == ArtifactDetector::ApplyResults::BufferNotFull){
                this->set_message();
                this->pub_.publish(this->out_);
                ROS_WARN("[%s] Buffer not full", this->name_.c_str());
                continue;
            }
            this->pub_.publish(this->out_);
            this->has_artifact_ = false;
        }
        ros::spinOnce();
        r.sleep();
    }
}

void ArtifactDetector::set_message(){
    this->out_.header.stamp = ros::Time::now();
    this->out_.seq = this->seq_id_;
    this->out_.has_artifact = this->has_artifact_;
}

ArtifactDetector::ApplyResults ArtifactDetector::apply(void){

    try{
        int bufferSize, EOG_ch_size;
        this->buffers_[0]->getParam(std::string("size"), bufferSize);
        EOG_ch_size = this->EOG_ch_.size();

        Eigen::MatrixXf data_car;
        data_car = this->car_filter_.apply(this->data_in_.transpose());

        Eigen::MatrixXd data1, eog_data, peaks_data;
        data1 = this->filter_low_EOG_.apply(data_car.cast<double>());
        eog_data = this->filter_high_EOG_.apply(data1);
        peaks_data = this->filter_high_peaks_.apply(data_car.cast<double>());

        this->buffers_[0]->add(eog_data.cast<float>()); // [samples x channels]
        this->buffers_[1]->add(peaks_data.cast<float>()); // [samples x channels]

        if(!this->buffers_[0]->isfull()){
            return ArtifactDetector::ApplyResults::BufferNotFull;
        }
        Eigen::MatrixXd dfet;

        if(EOG_ch_size > 0){
            dfet = Eigen::MatrixXd::Zero(bufferSize, EOG_ch_size);

            eog_data = this->buffers_[0]->get().cast<double>();

            // EOG
            for(int i = 0; i < EOG_ch_size; i++){
                dfet.col(i) = eog_data.col(this->EOG_ch_[i]).cast<double>();
            }
            Eigen::VectorXd heog, veog;
            heog = dfet.col(0) - dfet.col(1); 
            if(EOG_ch_size == 2){
                veog = (dfet.col(0) + dfet.col(1)) / 2.0f; 
            }else if (EOG_ch_size == 3){
                veog = (dfet.col(0) + dfet.col(1)) / 2.0f - dfet.col(2); 
            }

            if(heog.cwiseAbs().maxCoeff() > this->th_hEOG_ || veog.cwiseAbs().maxCoeff() > this->th_vEOG_){
                this->has_artifact_ = true;
            }
        }

        // peaks
        peaks_data = this->buffers_[1]->get().cast<double>();
        int nchannels_noEOG = peaks_data.cols() - EOG_ch_size;
        dfet = Eigen::MatrixXd::Zero(bufferSize, nchannels_noEOG);
        int j = 0;
        for(int i = 0; i < peaks_data.cols(); i++){
            if(std::find(this->EOG_ch_.begin(), this->EOG_ch_.end(), i) != this->EOG_ch_.end()){
                continue; // skip EOG channels
            }
            dfet.col(j) = peaks_data.col(i).cast<double>();
            j++;
        }

        // decision message
        if(dfet.cwiseAbs().maxCoeff() > this->th_peaks_){
            this->has_artifact_ = true;
        }
        this->set_message();

        return ArtifactDetector::ApplyResults::Success;

    }catch(std::exception& e){
        ROS_ERROR("[%s] Error in ArtifactDetector processing: %s", this->name_.c_str(), e.what());
        return ArtifactDetector::ApplyResults::Error;
    }
}

void ArtifactDetector::on_received_data(const rosneuro_msgs::NeuroFrame &msg){

    if(!this->is_signal_configured_){
        if(!this->configure_signal(msg)){
            ROS_ERROR("[%s] Failed to configure signal from NeuroFrame", this->name_.c_str());
            return;
        }
    }

    this->has_new_data_ = true;
    this->has_artifact_ = false;

    float* ptr_in;
    float* ptr_eog;
    ptr_in = const_cast<float*>(msg.eeg.data.data());
    ptr_eog = const_cast<float*>(msg.exg.data.data());

    if(this->run_mode_ == "online" || (this->run_mode_ == "offline" && this->signal_type_ == "eeg")){
        this->data_in_ = Eigen::Map<rosneuro::DynamicMatrix<float>>(ptr_in, this->nchannels_, this->chunkSize_);
    }else if(this->run_mode_ == "offline" && this->signal_type_ == "eeg_eog"){
        Eigen::MatrixXf eeg_data = Eigen::Map<rosneuro::DynamicMatrix<float>>(ptr_in, this->nchannels_ - 1, this->chunkSize_);
        Eigen::MatrixXf eog_data = Eigen::Map<Eigen::Matrix<float, 1, -1>>(ptr_eog, 1, this->chunkSize_);
        this->data_in_ = Eigen::MatrixXf(this->nchannels_, this->chunkSize_);
        this->data_in_.block(0, 0, this->nchannels_-1, this->chunkSize_) = eeg_data;
        this->data_in_.row(this->nchannels_-1) = eog_data;
    }
    this->seq_id_ = msg.neuroheader.seq;
}

bool ArtifactDetector::configure_signal(const rosneuro_msgs::NeuroFrame& msg){
    this->nchannels_ = msg.eeg.info.nchannels;
    this->chunkSize_ = msg.eeg.info.nsamples;
    double sampleRate = static_cast<double>(msg.sr);

    if(this->run_mode_ == "offline" && this->signal_type_ == "eeg_eog"){
        this->nchannels_ += 1; // EEG channels + 1 EOG from exg
    }

    // Resolve EOG channel names → 0-based indices using NeuroFrame labels
    const auto& labels = msg.eeg.info.labels;
    if(labels.empty()){
        ROS_ERROR("[%s] NeuroFrame eeg.info.labels is empty – cannot resolve EOG channel names", this->name_.c_str());
        return false;
    }
    this->EOG_ch_.clear();
    for(const auto& name : this->EOG_ch_names_){
        bool found = false;
        for(int i = 0; i < static_cast<int>(labels.size()); i++){
            std::string a = name, b = labels[i];
            std::transform(a.begin(), a.end(), a.begin(), ::tolower);
            std::transform(b.begin(), b.end(), b.begin(), ::tolower);
            if(a == b){
                this->EOG_ch_.push_back(i); // 0-based
                found = true;
                break;
            }
        }
        if(!found){
            ROS_ERROR("[%s] EOG channel '%s' not found in NeuroFrame labels", this->name_.c_str(), name.c_str());
            return false;
        }
    }
    ROS_INFO("[%s] EOG channels resolved: %s → indices (0-based): %s",
             this->name_.c_str(),
             [&]{ std::string s; for(auto& n : this->EOG_ch_names_) s += n + " "; return s; }().c_str(),
             [&]{ std::string s; for(auto i : this->EOG_ch_) s += std::to_string(i) + " "; return s; }().c_str());

    try{
        this->filter_low_EOG_    = rosneuro::Butterworth<double>(rosneuro::ButterType::LowPass,  this->filterOrder_EOG_,   this->freq_low_EOG_,    sampleRate);
        this->filter_high_EOG_   = rosneuro::Butterworth<double>(rosneuro::ButterType::HighPass, this->filterOrder_EOG_,   this->freq_high_EOG_,   sampleRate);
        this->filter_high_peaks_ = rosneuro::Butterworth<double>(rosneuro::ButterType::HighPass, this->filterOrder_peaks_, this->freq_high_peaks_, sampleRate);

        this->car_filter_ = rosneuro::Car<float>();
        this->car_filter_.configure(this->EOG_ch_);

        for(int i = 0; i < 2; i++){
            this->buffers_.push_back(new rosneuro::RingBuffer<float>());
            if(!this->buffers_.back()->configure("RingBufferCfgArtifact")){
                ROS_ERROR("[%s] Buffer %d not configured correctly", this->name_.c_str(), i);
                return false;
            }
        }
    }catch(std::exception& e){
        ROS_ERROR("[%s] Error in configure_signal: %s", this->name_.c_str(), e.what());
        return false;
    }

    ROS_INFO("[%s] Signal configured from NeuroFrame: nchannels=%d, chunkSize=%d, sampleRate=%.1f",
             this->name_.c_str(), this->nchannels_, this->chunkSize_, sampleRate);

    this->is_signal_configured_ = true;
    return true;
}

bool ArtifactDetector::configure(void){

    if(!ArtifactDetector::getParam(std::string("run_mode"), this->run_mode_)){
        ROS_ERROR("[%s] Missing 'run_mode' parameter, which is a mandatory parameter", this->name_.c_str());
        return false;
    }
    if (!ArtifactDetector::getParam(std::string("signal_type"), this->signal_type_)){
        ROS_ERROR("[%s] Cannot find param signal_type", this->name_.c_str());
        return false;
    }
    if (!ArtifactDetector::getParam(std::string("th_hEOG"), this->th_hEOG_)) {
        ROS_ERROR("[%s] Cannot find param th_hEOG", this->name_.c_str());
        return false;
    }
    if (!ArtifactDetector::getParam(std::string("th_vEOG"), this->th_vEOG_)) {
        ROS_ERROR("[%s] Cannot find param th_vEOG", this->name_.c_str());
        return false;
    }
    if (!ArtifactDetector::getParam(std::string("th_peaks"), this->th_peaks_)) {
        ROS_ERROR("[%s] Cannot find param th_peaks", this->name_.c_str());
        return false;
    }
    if (!ArtifactDetector::getParam(std::string("EOG_ch_names"), this->EOG_ch_names_)){
        ROS_ERROR("[%s] Cannot find param EOG_ch_names (expected a list of channel name strings)", this->name_.c_str());
        return false;
    }
    // EOG_ch_ indices are resolved in configure_signal() once NeuroFrame labels are available

    if (!ArtifactDetector::getParam(std::string("freq_high_EOG"), this->freq_high_EOG_)) {
        ROS_ERROR("[%s] Cannot find param freq_high_EOG", this->name_.c_str());
        return false;
    }
    if (!ArtifactDetector::getParam(std::string("freq_low_EOG"), this->freq_low_EOG_)) {
        ROS_ERROR("[%s] Cannot find param freq_low_EOG", this->name_.c_str());
        return false;
    }
    if (!ArtifactDetector::getParam(std::string("freq_high_peaks"), this->freq_high_peaks_)) {
        ROS_ERROR("[%s] Cannot find param freq_high_peaks", this->name_.c_str());
        return false;
    }
    if (!ArtifactDetector::getParam(std::string("filterOrder_EOG"), this->filterOrder_EOG_)) {
        ROS_ERROR("[%s] Cannot find param filterOrder_EOG", this->name_.c_str());
        return false;
    }
    if (!ArtifactDetector::getParam(std::string("filterOrder_peaks"), this->filterOrder_peaks_)) {
        ROS_ERROR("[%s] Cannot find param filterOrder_peaks", this->name_.c_str());
        return false;
    }

    // nchannels, chunkSize, sampleRate are derived from the first NeuroFrame in configure_signal()
    return true;
}

bool ArtifactDetector::configure(const std::string& param_name) {
    bool retval = false;
    XmlRpc::XmlRpcValue config;
    if (!this->nh_.getParam(param_name, config)) {
        ROS_ERROR("[%s] Could not find parameter %s on the server, are you sure that it was pushed up correctly?", this->name_.c_str(), param_name.c_str());
        return false;
    }

    retval = this->configure(config);
    return retval;
}

bool ArtifactDetector::configure(XmlRpc::XmlRpcValue& config) {
    if (this->is_configured_) {
        ROS_ERROR("[%s] ArtifactDetector %s already being reconfigured", this->name_.c_str(), this->name_.c_str());
    }
    this->is_configured_ = false;

    bool retval = this->loadConfiguration(config);
    retval = retval && this->configure();
    this->is_configured_ = retval;
    return retval;
}

bool ArtifactDetector::loadConfiguration(XmlRpc::XmlRpcValue& config) {
    if(config.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("[%s] An ArtifactDetector configuration must be a map with fields name, and params", this->name_.c_str());
        return false;
    }

    if (!setName(config)) {
        return false;
    }

    if(config.hasMember("params")) {
        XmlRpc::XmlRpcValue params = config["params"];
        if(params.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR("[%s] params must be a map", this->name_.c_str());
            return false;
        } else {
            for(XmlRpc::XmlRpcValue::iterator it = params.begin(); it != params.end(); ++it) {
                ROS_DEBUG("[%s] Loading param %s", this->name_.c_str(), it->first.c_str());
                this->params_[it->first] = it->second;
            }
        }
    }
    return true;
}

bool ArtifactDetector::setName(XmlRpc::XmlRpcValue& config) {
    if(!config.hasMember("name")) {
        ROS_ERROR("[%s] ArtifactDetector didn't have name defined, this is required", this->name_.c_str());
        return false;
    }

    this->name_ = std::string(config["name"]);
    ROS_DEBUG("[%s] Configuring ArtifactDetector with name %s", this->name_.c_str(), this->name_.c_str());
    return true;
}

bool ArtifactDetector::getParam(const std::string& name, std::string& value) const {
    auto it = this->params_.find(name);
    if (it == this->params_.end()) {
        return false;
    }

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeString) {
        return false;
    }

    auto tmp = it->second;
    value = std::string(tmp);
    return true;
}


bool ArtifactDetector::getParam(const std::string& name, bool& value) const {
    auto it = this->params_.find(name);
    if (it == this->params_.end()) {
        return false;
    }

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
        return false;
    }

    auto tmp = it->second;
    value = (bool)(tmp);
    return true;
}

bool ArtifactDetector::getParam(const std::string& name, double& value) const {
    auto it = this->params_.find(name);
    if (it == this->params_.end()) {
        return false;
    }

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeDouble && it->second.getType() != XmlRpc::XmlRpcValue::TypeInt) {
        return false;
    }

    auto tmp = it->second;
    value = it->second.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(tmp) : (double)(tmp);
    return true;
}

bool ArtifactDetector::getParam(const std::string& name, int& value) const {
    auto it = this->params_.find(name);
    if (it == this->params_.end()) {
        return false;
    }

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeInt) {
        return false;
    }

    auto tmp = it->second;
    value = tmp;
    return true;
}

bool ArtifactDetector::getParam(const std::string& name, unsigned int& value) const {
    int signed_value;
    if (!this->getParam(name, signed_value))
        return false;
    if (signed_value < 0)
        return false;
    value = signed_value;
    return true;
}

bool ArtifactDetector::getParam(const std::string& name, std::vector<double>& value) const {
    auto it = this->params_.find(name);
    if (it == this->params_.end()) {
        return false;
    }

    value.clear();

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeArray)
        return false;

    XmlRpc::XmlRpcValue double_array = it->second;

    for (auto i = 0; i < double_array.size(); ++i){
        if(double_array[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && double_array[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
            return false;
        }

        double double_value = double_array[i].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(double_array[i]) : (double)(double_array[i]);
        value.push_back(double_value);
    }

    return true;
}

bool ArtifactDetector::getParam(const std::string& name, std::vector<int>& value) const {
    auto it = this->params_.find(name);
    if (it == this->params_.end()) {
        return false;
    }

    value.clear();

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeArray)
        return false;

    XmlRpc::XmlRpcValue double_array = it->second;

    for (auto i = 0; i < double_array.size(); ++i){
        if(double_array[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && double_array[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
            return false;
        }

        int double_value = double_array[i].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(double_array[i]) : (int)(double_array[i]);
        value.push_back(double_value);
    }

    return true;
}

bool ArtifactDetector::getParam(const std::string& name, std::vector<std::string>& value) const {
    auto it = this->params_.find(name);
    if (it == this->params_.end()) {
        return false;
    }

    value.clear();

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeArray)
        return false;

    XmlRpc::XmlRpcValue string_array = it->second;

    for (auto i = 0; i < string_array.size(); ++i) {
        if(string_array[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
            return false;
        }

        value.push_back(string_array[i]);
    }

    return true;
}

bool ArtifactDetector::getParam(const std::string& name, XmlRpc::XmlRpcValue& value) const {
    auto it = this->params_.find(name);
    if (it == this->params_.end()) {
        return false;
    }

    auto tmp = it->second;
    value = tmp;
    return true;
}
