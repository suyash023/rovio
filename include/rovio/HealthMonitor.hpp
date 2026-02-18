/** Paste the ETH licence preamble here
 * @file HealthMonitor.hpp
 * @author Suyash Yeotikar
 * @date Feb 16 2026
 */
#include "rovio/RovioFilter.hpp"
#include "rovio/FilterStates.hpp"

#include "rovio_interfaces/msg/health.hpp"

#include <rclcpp/time.hpp>

#ifndef ROVIO_HEALTHMONITOR_HPP
#define ROVIO_HEALTHMONITOR_HPP
template<unsigned int nMax_, int nLevels_, int patchSize_, int nCam_, int nPose_>
class HealthMonitor {
public:
  typedef rovio::RovioFilter<rovio::FilterState<nMax_,nLevels_,patchSize_,nCam_,nPose_>> mtFilter;

  float trackedFeatureRatio; //< Ratio of tracked features to max features
  float validFeatureRatio; //< Ratio of valid features to max features
  float NISZScoreRMSE; //< RMSE of NIS Z-score
  float featureDepthCovMedian; //< Median of depth covariances for all valid features
  float unhealthyVelocityDeviation; //< Deviation from unhealthy velocity threshold
  float accelDeviation; //< Deviation from the accel threshold
  float pixelCovRatio; //< Ratio of features with pixel covariances above a threshold.
  bool healthMsgValid; //< Boolean variable to control the publishing of the heaalth message.
public:

  HealthMonitor();

  /**
   * @brief Function to populate the health message for ROVIO.
   * @param filterState shared ptr to current state vector of ROVIO
   * @param healthMsg health message to be populated
   * @return None
   */
  void populateHealthMsg(const std::shared_ptr<mtFilter> mpFilter_,
    rovio_interfaces::msg::Health::SharedPtr healthMsg, std::string imu_frame) {
    if ( !this->healthMsgValid) {
      return;
    }
    healthMsg->accel_deviation = this->accelDeviation;
    healthMsg->speed_deviation = this->unhealthyVelocityDeviation;
    healthMsg->pixel_covariance_ratio = this->pixelCovRatio;
    healthMsg->accel_deviation = this->accelDeviation;
    healthMsg->nis_z_score_rmse =  this->NISZScoreRMSE;
    healthMsg->feature_depth_cov_median = featureDepthCovMedian;
    healthMsg->tracked_feature_ratio = this->trackedFeatureRatio;
    healthMsg->total_feature_ratio = this->trackedFeatureRatio;
    healthMsg->header.frame_id = imu_frame;
    healthMsg->header.stamp = rclcpp::Time(static_cast<uint64_t>(1e9 * mpFilter_->safe_.t_));
  }



  /**
   * @brief Function to compute the median of the features depths covariances.
   * @param state Current state vector of ROVIO
   * @return float value that is the median of the depth covariances
   */
  float computeFeatureDepthCovMedian(const std::shared_ptr<mtFilter> mpFilter_ ) {}

  /**
   * @brief Function to compute the valid feature ratio
   * @param state Current state vector of ROVIO
   * @return float ratio of valid to max features.
   */
  float computeValidFeatureRatio(const std::shared_ptr<mtFilter> mpFilter_) {}

  /**
   * @brief Function to compute the tracked feature ratio.
   * @param state Current state vector of ROVIO
   * @return float ratio of tracked to max features.
   */
  float computeTrackedFeatureRatio(const std::shared_ptr<mtFilter> mpFilter_) {}

  /**
   * @breif Function to compute the RMSE of NIS z-score
   * @param state Current state vector of ROVIO
   * @return float RMSE of NIS zscore
   */
  float computeNISZScore(const std::shared_ptr<mtFilter> mpFilter_) {}

  /**
   * @brief Function to compute the ratio of features above a pixel covariance threshold
   * @param mtFilter &state
   * @return float ratio of number of features below pixel covariance threshold to max features
   */
  float computePixelCovRatio( const std::shared_ptr<mtFilter> mpFilter_) {}


  /**
   * @brief Function to compute the deviation of speed from threshold value
   * @param threshold value of velocity
   * @param velocity estimated ROVIO
   */
  float computeUnhealthyVelocityDeviation(const float thresholdSpeed, Eigen::Vector3d rovioVelocity);


  /**
   * @brief Function to compute the deviation of IMU accelration from threshold value
   * Helpful to detect spikes
   * @param threshold value of acceleration
   * @param IMU accel reading
   */
  float computeAccelDeviation(const float thresholdAccel, Eigen::Vector3d IMUAccceleration );
};


#endif // ROVIO_HEALTHMONITOR_HPP
