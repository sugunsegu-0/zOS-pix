/*
 * Copyright (c) 2014, 2015, 2016, Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROBOT_LOCALIZATION_ROS_FILTER_H
#define ROBOT_LOCALIZATION_ROS_FILTER_H


#include "filter_common.h"
#include "filter_base.h"

#include "localization/localizationDS.hpp"


#include <Eigen/Dense>

#include <ecal/ecal.h>
#include <ecal/msg/string/publisher.h>
#include <ecal/msg/string/subscriber.h>

#include <fstream>
#include <map>
#include <memory>
#include <numeric>
#include <queue>
#include <string>
#include <vector>
#include <deque>

namespace RobotLocalization
{


struct CallbackData
{
  CallbackData(const std::string &topicName,
               const std::vector<int> &updateVector,
               const int updateSum,
               const bool differential,
               const bool relative,
               const bool pose_use_child_frame,
               const double rejectionThreshold) :
    topicName_(topicName),
    updateVector_(updateVector),
    updateSum_(updateSum),
    differential_(differential),
    relative_(relative),
    pose_use_child_frame_(pose_use_child_frame),
    rejectionThreshold_(rejectionThreshold)
  {
  }

  std::string topicName_;
  std::vector<int> updateVector_;
  int updateSum_;
  bool differential_;
  bool relative_;
  bool pose_use_child_frame_;
  double rejectionThreshold_;
};

typedef std::priority_queue<MeasurementPtr, std::vector<MeasurementPtr>, Measurement> MeasurementQueue;
typedef std::deque<MeasurementPtr> MeasurementHistoryDeque;
typedef std::deque<FilterStatePtr> FilterStateHistoryDeque;

template<class T> class RosFilter
{
  public:

    eCAL::string::CSubscriber<std::string> subscriber;
    //! @brief Constructor
    //!
    //! The RosFilter constructor makes sure that anyone using
    //! this template is doing so with the correct object type
    //!
    explicit RosFilter(std::string node_name,
                       std::vector<double> args = std::vector<double>());

    //! @brief Constructor
    //!
    //! The RosFilter constructor makes sure that anyone using
    //! this template is doing so with the correct object type
    //!
    explicit RosFilter(std::vector<double> args = std::vector<double>());

    //! @brief Destructor
    //!
    //! Clears out the message filters and topic subscribers.
    //!
    ~RosFilter();

    T filter_;

    //! @brief Initialize filter
    //

    void initialize();

    //! @brief Resets the filter to its initial state
    //!
    void reset();
    //! @brief Callback method for receiving all acceleration (IMU) messages
    //! @param[in] msg - The ROS IMU message to take in.
    //! @param[in] callbackData - Relevant static callback data
    //! @param[in] targetFrame - The target frame_id into which to transform the data
    //!
    
    void enqueueMeasurement(const std::string &topicName,
                            const Eigen::VectorXd &measurement,
                            const Eigen::MatrixXd &measurementCovariance,
                            const std::vector<int> &updateVector,
                            const double mahalanobisThresh,
                            const double);

    //! @brief Method for zeroing out 3D variables within measurements
    //! @param[out] measurement - The measurement whose 3D variables will be zeroed out
    //! @param[out] measurementCovariance - The covariance of the measurement
    //! @param[out] updateVector - The boolean update vector of the measurement
    //!
    //! If we're in 2D mode, then for every measurement from every sensor, we call this.
    //! It sets the 3D variables to 0, gives those variables tiny variances, and sets
    //! their updateVector values to 1.
    //!
    void forceTwoD(Eigen::VectorXd &measurement,
                   Eigen::MatrixXd &measurementCovariance,
                   std::vector<int> &updateVector);

    //! @brief Retrieves the EKF's output for broadcasting
    //! @param[out] message - The standard ROS odometry message to be filled
    //! @return true if the filter is initialized, false otherwise
    //!
    bool getFilteredOdometryMessage(Odometry &msg);

    //! @brief Processes all measurements in the measurement queue, in temporal order
    //!
    //! @param[in] currentTime - The time at which to carry out integration (the current time)
    //!
    void integrateMeasurements(const double currentTime);

    //! @brief Differentiate angular velocity for angular acceleration
    //!
    //! @param[in] currentTime - The time at which to carry out differentiation (the current time)
    //!
    //! Maybe more state variables can be time-differentiated to estimate higher-order states,
    //! but now we only focus on obtaining the angular acceleration. It implements a backward-
    //! Euler differentiation.
    //!
    void differentiateMeasurements(const double currentTime);

    //! @brief Loads all parameters from file
    //!
    void loadParams();

    //! @brief Callback method for receiving all odometry messages
    //! @param[in] msg - The ROS odometry message to take in.
    //! @param[in] topicName - The topic name for the odometry message (only used for debug output)
    //! @param[in] poseCallbackData - Relevant static callback data for pose variables
    //! @param[in] twistCallbackData - Relevant static callback data for twist variables
    //!
    //! This method simply separates out the pose and twist data into two new messages, and passes them into their
    //! respective callbacks
    //!
    void odometryCallback(const std::string &msg, const std::string &topicName,
      const CallbackData &poseCallbackData, const CallbackData &twistCallbackData);

    //! @brief Callback method for receiving all pose messages
    //! @param[in] msg - The ROS stamped pose with covariance message to take in
    //! @param[in] callbackData - Relevant static callback data
    //! @param[in] targetFrame - The target frame_id into which to transform the data
    //! @param[in] poseSourceFrame - The source frame_id from which to transform the data
    //! @param[in] imuData - Whether this data comes from an IMU
    //!
    void poseCallback(const PoseWithCovarianceStamped &msg,
                      const CallbackData &callbackData,
                      const std::string &targetFrame,
                      const std::string &poseSourceFrame,
                      const bool imuData);

    //! @brief Callback method for receiving all twist messages
    //! @param[in] msg - The ROS stamped twist with covariance message to take in.
    //! @param[in] callbackData - Relevant static callback data
    //! @param[in] targetFrame - The target frame_id into which to transform the data
    //!
    void twistCallback(const TwistWithCovarianceStamped &msg,
                       const CallbackData &callbackData,
                       const std::string &targetFrame);

    void imuCallback(const std::string &msg, const std::string &topicName,
      const CallbackData &poseCallbackData, const CallbackData &twistCallbackData,
      const CallbackData &accelCallbackData);  

    void accelerationCallback(const Imu &msg,
                            const CallbackData &callbackData,
                            const std::string &targetFrame);

    bool prepareAcceleration(const Imu &msg,
                             const std::string &topicName,
                             const std::string &targetFrame,
                             const bool relative,
                             std::vector<int> &updateVector,
                             Eigen::VectorXd &measurement,
                             Eigen::MatrixXd &measurementCovariance);  

    //! @brief Validates filter outputs for NaNs and Infinite values
    //! @param[out] message - The standard ROS odometry message to be validated
    //! @return true if the filter output is valid, false otherwise
    //!
    bool validateFilterOutput(const Odometry &message);
    int matSize;

  protected:
    //! @brief Finds the latest filter state before the given timestamp and makes it the current state again.
    //!
    //! This method also inserts all measurements between the older filter timestamp and now into the measurements
    //! queue.
    //!
    //! @param[in] time - The time to which the filter state should revert
    //! @return True if restoring the filter succeeded. False if not.
    //!
    bool revertTo(const double time);

    //! @brief Saves the current filter state in the queue of previous filter states
    //!
    //! These measurements will be used in backwards smoothing in the event that older measurements come in.
    //! @param[in] filter - The filter base object whose state we want to save
    //!
    void saveFilterState(FilterBase &filter);

    //! @brief Removes measurements and filter states older than the given cutoff time.
    //! @param[in] cutoffTime - Measurements and states older than this time will be dropped.
    //!
    void clearExpiredHistory(const double cutoffTime);

    //! @brief Clears measurement queue
    //!
    void clearMeasurementQueue();

    void copyCovariance(const double *arr,
                        Eigen::MatrixXd &covariance,
                        const std::string &topicName,
                        const std::vector<int> &updateVector,
                        const size_t offset,
                        const size_t dimension);

    //! @brief Utility method for copying covariances from Eigen matrices to ROS
    //! covariance arrays
    //!
    //! @param[in] covariance - The source matrix for the covariance data
    //! @param[in] arr - The destination array
    //! @param[in] dimension - The number of values to copy
    //!
    void copyCovariance(const Eigen::MatrixXd &covariance,
                        double *arr,
                        const size_t dimension);

    //! @brief Loads fusion settings from the config file
    //! @param[in] topicName - The name of the topic for which to load settings
    //! @return The boolean vector of update settings for each variable for this topic
    //!
    std::vector<int> loadUpdateConfig(const std::string &topicName);

    bool preparePose(const PoseWithCovarianceStamped &msg,
                     const std::string &topicName,
                     const std::string &targetFrame,
                     const std::string &sourceFrame,
                     const bool differential,
                     const bool relative,
                     const bool imuData,
                     std::vector<int> &updateVector,
                     Eigen::VectorXd &measurement,
                     Eigen::MatrixXd &measurementCovariance);

    //! @brief Prepares a twist message for integration into the filter
    //! @param[in] msg - The twist message to prepare
    //! @param[in] topicName - The name of the topic over which this message was received
    //! @param[in] targetFrame - The target tf frame
    //! @param[in,out] updateVector - The update vector for the data source
    //! @param[out] measurement - The twist data converted to a measurement
    //! @param[out] measurementCovariance - The covariance of the converted measurement
    //! @return true indicates that the measurement was successfully prepared, false otherwise
    //!
    bool prepareTwist(const TwistWithCovarianceStamped &msg,
                      const std::string &topicName,
                      const std::string &targetFrame,
                      std::vector<int> &updateVector,
                      Eigen::VectorXd &measurement,
                      Eigen::MatrixXd &measurementCovariance);


    //! @brief callback function which is called for periodic updates
    //!
    Odometry periodicUpdate();

    //! @brief Start the Filter disabled at startup
    //!
    //! If this is true, the filter reads parameters and prepares publishers and subscribes
    //! but does not integrate new messages into the state vector.
    //! The filter can be enabled later using a service.
    bool disabledAtStartup_;

    //! @brief Whether the filter is enabled or not. See disabledAtStartup_.
    bool enabled_;

    //! Whether we'll allow old measurements to cause a re-publication of the updated state
    bool permitCorrectedPublication_;

    //! @brief By default, the filter predicts and corrects up to the time of the latest measurement. If this is set
    //! to true, the filter does the same, but then also predicts up to the current time step.
    //!
    bool predictToCurrentTime_;

    //! @brief Whether we publish the transform from the world_frame to the base_link_frame
    //!
    //! @brief Whether to reset the filters when backwards jump in time is detected
    //!
    //! This is usually the case when logs are being used and a jump in the logi
    //! is done or if a log files restarts from the beginning.
    //!
    bool resetOnTimeJump_;

    double currentTimeSec;

    //! @brief Whether or not we use smoothing
    //!
    bool smoothLaggedData_;

    //! @brief Whether the filter should process new measurements or not.
    bool toggledOn_;

    //! @brief Whether or not we're in 2D mode
    //!
    //! If this is true, the filter binds all 3D variables (Z,
    //! roll, pitch, and their respective velocities) to 0 for
    //! every measurement.
    //!
    bool twoDMode_;

    //! @brief Whether or not we use a control term
    //!

    //! @brief When true, do not print warnings for tf lookup failures.
    //!
    bool tfSilentFailure_;

    //! @brief The max (worst) dynamic diagnostic level.
    //!
    int dynamicDiagErrorLevel_;

    //! @brief The max (worst) static diagnostic level.
    //!
    int staticDiagErrorLevel_;

    //! @brief The frequency of the run loop
    //!
    double frequency_;

    //! @brief What is the acceleration in Z due to gravity (m/s^2)? Default is +9.80665.
    //!
    double gravitationalAcc_;

    //! @brief The depth of the history we track for smoothing/delayed measurement processing
    //!
    //! This is the guaranteed minimum buffer size for which previous states and measurements are kept.
    //!
    double historyLength_;

    //! @brief minimal frequency
    //!
    double minFrequency_;

    

    //! @brief maximal frequency
    double maxFrequency_;

    //! @brief tf frame name for the robot's body frame
    //!
    std::string baseLinkFrameId_;

    //! @brief tf frame name for the robot's body frame
    //!
    //! When the final state is computed, we "override" the output transform and message to have this frame for its
    //! child_frame_id. This helps to enable disconnected TF trees when multiple EKF instances are being run.
    //!
    std::string baseLinkOutputFrameId_;

    //! @brief tf frame name for the robot's map (world-fixed) frame
    //!
    std::string mapFrameId_;

    //! @brief tf frame name for the robot's odometry (world-fixed) frame
    //!
    std::string odomFrameId_;

    //! @brief tf frame name that is the parent frame of the transform that this node will calculate and broadcast.
    //!
    std::string worldFrameId_;

    //! @brief Used for outputting debug messages
    //!
    std::ofstream debugStream_;

    //! @brief Contains the state vector variable names in string format
    //!
    std::vector<std::string> stateVariableNames_;

    //! @brief Vector to hold our subscribers until they go out of scope
    //!

    //! @brief This object accumulates dynamic diagnostics, e.g., diagnostics relating
    //! to sensor data.
    //!
    //! The values are considered transient and are cleared at every iteration.
    //!
    std::map<std::string, std::string> dynamicDiagnostics_;

    //! @brief Stores the first measurement from each topic for relative measurements
    //!
    //! When a given sensor is being integrated in relative mode, its first measurement
    //! is effectively treated as an offset, and future measurements have this first
    //! measurement removed before they are fused. This variable stores the initial
    //! measurements. Note that this is different from using differential mode, as in
    //! differential mode, pose data is converted to twist data, resulting in boundless
    //! error growth for the variables being fused. With relative measurements, the
    //! vehicle will start with a 0 heading and position, but the measurements are still
    //! fused absolutely.
    // std::map<std::string, tf2::Transform> initialMeasurements_;

    //! @brief Store the last time a message from each topic was received
    //!
    //! If we're getting messages rapidly, we may accidentally get
    //! an older message arriving after a newer one. This variable keeps
    //! track of the most recent message time for each subscribed message
    //! topic. We also use it when listening to odometry messages to
    //! determine if we should be using messages from that topic.
    //!
    std::map<std::string, double > lastMessageTimes_;

    //! @brief We also need the previous covariance matrix for differential data
    //!
    std::map<std::string, Eigen::MatrixXd> previousMeasurementCovariances_;

    //! @brief Stores the last measurement from a given topic for differential integration
    //!
    //! To carry out differential integration, we have to (1) transform
    //! that into the target frame (probably the frame specified by
    //! @p odomFrameId_), (2) "subtract"  the previous measurement from
    //! the current measurement, and then (3) transform it again by the previous
    //! state estimate. This holds the measurements used for step (2).
    //!

    //! @brief If including acceleration for each IMU input, whether or not we remove acceleration due to gravity
    //!
    std::map<std::string, bool> removeGravitationalAcc_;

    //! @brief This object accumulates static diagnostics, e.g., diagnostics relating
    //! to the configuration parameters.
    //!
    //! The values are treated as static and always reported (i.e., this object is never cleared)
    //!
    std::map<std::string, std::string> staticDiagnostics_;

    //! @brief Last time mark that time-differentiation is calculated
    //!
    double  lastDiffTime_;

    std::string ser_odom_msg;

    //! @brief Last record of filtered angular velocity
    //!
    Vector3 lastStateTwistRot_;

    //! @brief Calculated angular acceleration from time-differencing
    //!
    Vector3 angular_acceleration_;

    //! @brief Covariance of the calculated angular acceleration
    //!
    Eigen::MatrixXd angular_acceleration_cov_;

    //! @brief The most recent control input
    //!
    Eigen::VectorXd latestControl_;

    //! @brief Message that contains our latest transform (i.e., state)
    //!
    //! We use the vehicle's latest state in a number of places, and often
    //! use it as a transform, so this is the most convenient variable to
    //! use as our global "current state" object
    //!

    //! @brief last call of periodicUpdate
    //!
    double  lastDiagTime_;

    //! @brief The time of the most recent published state
    //!
    double  lastPublishedStamp_;

    //! @brief Store the last time set pose message was received
    //!
    //! If we receive a setPose message to reset the filter, we can get in
    //! strange situations wherein we process the pose reset, but then even
    //! after another spin cycle or two, we can get a new message with a time
    //! stamp that is *older* than the setPose message's time stamp. This means
    //! we have to check the message's time stamp against the lastSetPoseTime_.
    double  lastSetPoseTime_;

    //! @brief The time of the most recent control input
    //!
    double  latestControlTime_;

    //! @brief For future (or past) dating the world_frame->base_link_frame transform
    //!
    double  tfTimeOffset_;

    //! @brief Parameter that specifies the how long we wait for a transform to become available.
    //!
    double  tfTimeout_;

    //! @brief Service that allows another node to toggle on/off filter processing while still publishing.
    //! Uses a robot_localization ToggleFilterProcessing service.
    //!

    //! @brief timer calling periodicUpdate
    //!
    double periodicUpdateTimer_;

    //! @brief An implicitly time ordered queue of past filter states used for smoothing.
    //
    // front() refers to the filter state with the earliest timestamp.
    // back() refers to the filter state with the latest timestamp.
    FilterStateHistoryDeque filterStateHistory_;

    //! @brief A deque of previous measurements which is implicitly ordered from earliest to latest measurement.
    // when popped from the measurement priority queue.
    // front() refers to the measurement with the earliest timestamp.
    // back() refers to the measurement with the latest timestamp.
    MeasurementHistoryDeque measurementHistory_;

    //! @brief We process measurements by queueing them up in
    //! callbacks and processing them all at once within each
    //! iteration
    //!
    MeasurementQueue measurementQueue_;

    //! @brief Our filter (EKF, UKF, etc.)
    //!
    


};

}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_ROS_FILTER_H
