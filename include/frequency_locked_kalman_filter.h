#ifndef FREQUENCY_LOCKED_KALMAN_FILTER_H
#define FREQUENCY_LOCKED_KALMAN_FILTER_H

#include <ros/ros.h>
#include <iirob_filters/kalman_filter.h>
#include <vector>
#include <string>

// Define a type for the output: a pair of timestamp and state vector
using KalmanState = std::pair<ros::Time, std::vector<double>>;

class FrequencyLockedKalmanFilter {
public:
    /**
     * @brief Constructor for the FrequencyLockedKalmanFilter.
     * @param ns The ROS parameter namespace for the filter configuration.
     * @param target_frequency The desired output frequency for the filter.
     * @param max_prediction_time The maximum duration to predict into the future and to reject outliers.
     * @param likelihood_threshold The likelihood value below which measurements are rejected.
     */
    FrequencyLockedKalmanFilter(const std::string& ns, double target_frequency, double max_prediction_time, double likelihood_threshold);

    /**
     * @brief Destructor.
     */
    ~FrequencyLockedKalmanFilter();

    /**
     * @brief Updates the filter with a new measurement and generates predictions to fill any time gaps.
     * @param measurement The new measurement vector.
     * @param measurement_stamp The timestamp of the measurement.
     * @return A vector of states. The first element is the corrected state from the update.
     *         Subsequent elements are the predicted states to fill the time gap up to the new measurement.
     */
    std::vector<KalmanState> update_and_predict_frequency_gap(const std::vector<double>& measurement, const ros::Time& measurement_stamp);

    /**
     * @brief Gets the likelihood of a measurement without updating the filter's state.
     * @param measurement The measurement vector to evaluate.
     * @return The likelihood value. Returns -1.0 if not initialized or on error.
     */
    double getLikelihood(const std::vector<double>& measurement);

    /**
     * @brief Checks if the underlying Kalman filter has been initialized.
     */
    bool isInitialized() const;

private:
    // The Kalman Filter instance
    iirob_filters::MultiChannelKalmanFilter<double>* kf_;

    std::string namespace_;
    ros::Time last_update_stamp_;
    ros::Time initialization_stamp_;
    ros::Time last_accepted_stamp_;
    ros::Duration target_period_;
    ros::Duration max_prediction_duration_;
    ros::Duration grace_period_duration_;
    bool is_initialized_;
    double likelihood_threshold_;
};

#endif // FREQUENCY_LOCKED_KALMAN_FILTER_H