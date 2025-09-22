#include "../include/frequency_locked_kalman_filter.h"

FrequencyLockedKalmanFilter::FrequencyLockedKalmanFilter(const std::string& ns, double target_frequency, double max_prediction_time, double likelihood_threshold)
    : namespace_(ns), is_initialized_(false), likelihood_threshold_(likelihood_threshold) {
    kf_ = new iirob_filters::MultiChannelKalmanFilter<double>();
    if (target_frequency > 0) {
        target_period_ = ros::Duration(1.0 / target_frequency);
    } else {
        target_period_ = ros::Duration(0);
        ROS_WARN("Target frequency is non-positive. Prediction logic will be disabled.");
    }

    max_prediction_duration_ = ros::Duration(max_prediction_time);
    grace_period_duration_ = ros::Duration(10.0 * max_prediction_time);
}

FrequencyLockedKalmanFilter::~FrequencyLockedKalmanFilter() {
    delete kf_;
}

bool FrequencyLockedKalmanFilter::isInitialized() const {
    return is_initialized_;
}

double FrequencyLockedKalmanFilter::getLikelihood(const std::vector<double>& measurement) {
    if (!is_initialized_) {
        return -1.0; // Not ready to calculate likelihood
    }
    double likelihood;
    if (kf_->likelihood(measurement, likelihood)) {
        return likelihood;
    }
    return -1.0; // Error case
}

std::vector<KalmanState> FrequencyLockedKalmanFilter::update_and_predict_frequency_gap(const std::vector<double>& measurement, const ros::Time& measurement_stamp) {
    std::vector<KalmanState> output_states;

    // --- Initialization Step ---
    if (!is_initialized_) {
        
        // First configure with namespace only to load parameters and get state dimension
        if (!kf_->configure(namespace_)) {
            ROS_ERROR("Failed to configure Kalman Filter with namespace '%s'.", namespace_.c_str());
            return output_states;
        }
        
        // The underlying 'configure' will fail and log an error if measurement size is wrong.
        // Expand measurement vector to be a valid initial state for the Kalman filter
        std::vector<double> initial_state = measurement;

        // Infer state dimension from Kalman filter after configuration
        int state_dim = kf_->getCurrentState(initial_state) ? initial_state.size() : 0;
        if (state_dim > 0) {
            initial_state.resize(state_dim, 0.0);
            // Copy measurement into the beginning of the initial state vector
            for (size_t i = 0; i < std::min(measurement.size(), static_cast<size_t>(state_dim)); ++i) {
            initial_state[i] = measurement[i];
            }
        } else {
            ROS_ERROR("Failed to infer state dimension from Kalman filter configuration.");
            return output_states;
        }

        // Now configure with the initial state vector
        if (kf_->configure(initial_state, namespace_)) {
            ROS_INFO("Kalman Filter initialized with first measurement in namespace '%s'.", namespace_.c_str());
            is_initialized_ = true;
            last_update_stamp_ = measurement_stamp;
            initialization_stamp_ = measurement_stamp;
            
            // Get the initial state back from the filter to ensure consistency
            std::vector<double> current_state;
            kf_->getCurrentState(current_state);
            output_states.push_back({measurement_stamp, current_state});
        } else {
            ROS_ERROR("Failed to configure/initialize Kalman Filter in namespace '%s'.", namespace_.c_str());
        }
        return output_states;
    }

    // --- Prediction Step (to fill gaps) ---
    ros::Duration time_gap = measurement_stamp - last_update_stamp_;
    if (time_gap > target_period_ && target_period_.toSec() > 0) {
        ros::Duration prediction_gap = time_gap;
        // Limit the prediction time to the configured maximum
        if (prediction_gap > max_prediction_duration_) {
            ROS_WARN("Large time gap detected (%.2f s), limiting prediction time to %.2f s.", prediction_gap.toSec(), max_prediction_duration_.toSec());
            prediction_gap = max_prediction_duration_;
        }

        int num_predictions_needed = static_cast<int>(prediction_gap.toSec() / target_period_.toSec());

        for (int i = 0; i < num_predictions_needed; ++i) {
            std::vector<double> predicted_state;
            double prediction_dt = target_period_.toSec() * (i + 1);
            
            // Compute prediction based on the state BEFORE the current update
            kf_->computePrediction(predicted_state, prediction_dt);

            if (!predicted_state.empty()) {
                ros::Time predicted_stamp = last_update_stamp_ + ros::Duration(prediction_dt);
                output_states.push_back({predicted_stamp, predicted_state});
            }
        }
    }

    // --- Update Step with Outlier Rejection ---
    double sensor_dt = time_gap.toSec();
    if (sensor_dt <= 0) {
        ROS_WARN_THROTTLE(1.0, "Non-positive sensor_dt (%.4f s) detected. Using target_period.", sensor_dt);
        sensor_dt = target_period_.toSec();
    }

    double likelihood;
    kf_->likelihood(measurement, likelihood);
    
    bool is_in_grace_period = (measurement_stamp - initialization_stamp_) <= grace_period_duration_;
    bool is_timed_out = (measurement_stamp - last_update_stamp_) > max_prediction_duration_;
    bool likelihood_is_ok = likelihood >= likelihood_threshold_;

    if (likelihood_is_ok || is_in_grace_period || is_timed_out) {
        // Accept the measurement
        if (is_timed_out && !likelihood_is_ok) {
            ROS_WARN("Forcing acceptance of measurement with low likelihood (%.4f) after %.2fs timeout.", likelihood, max_prediction_duration_.toSec());
        }
        
        std::vector<double> corrected_state;
        if (kf_->update(measurement, corrected_state, sensor_dt, true)) {
            output_states.push_back({measurement_stamp, corrected_state});
            last_update_stamp_ = measurement_stamp;
        } else {
            ROS_WARN("Kalman Filter update failed.");
        }
    } else {
        // Reject the measurement and predict forward instead
        ROS_WARN("Rejecting measurement with likelihood %.4f (threshold: %.2f)", likelihood, likelihood_threshold_);
    }

    return output_states;
}