
#include "ros_filter.h"
#include "filter_utilities.h"

#include "ekf.h"
#include "localization/localizationDS.hpp"


#include <ecal/ecal.h>
#include <ecal/msg/string/publisher.h>
#include <ecal/msg/string/subscriber.h>
#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <limits>
#include "commons.hpp"

#if defined(_WIN32) && defined(ERROR)
  #undef ERROR
#endif

namespace RobotLocalization
{
  template<typename T>
  RosFilter<T>::RosFilter(std::string node_name,
                          std::vector<double> args) :
      filter_(args)
  {
    stateVariableNames_.push_back("X");
    stateVariableNames_.push_back("Y");
    stateVariableNames_.push_back("Z");
    stateVariableNames_.push_back("ROLL");
    stateVariableNames_.push_back("PITCH");
    stateVariableNames_.push_back("YAW");
    stateVariableNames_.push_back("X_VELOCITY");
    stateVariableNames_.push_back("Y_VELOCITY");
    stateVariableNames_.push_back("Z_VELOCITY");
    stateVariableNames_.push_back("ROLL_VELOCITY");
    stateVariableNames_.push_back("PITCH_VELOCITY");
    stateVariableNames_.push_back("YAW_VELOCITY");
    stateVariableNames_.push_back("X_ACCELERATION");
    stateVariableNames_.push_back("Y_ACCELERATION");
    stateVariableNames_.push_back("Z_ACCELERATION");

  }

  template<typename T>
  RosFilter<T>::RosFilter(std::vector<double> args) :
      RosFilter<T>::RosFilter("ekf", args)
  {}

  template<typename T>
  RosFilter<T>::~RosFilter()
  {
  }

  template<typename T>
  void RosFilter<T>::initialize()
  {
    loadParams();

    cout << "EKF Initialization" << endl; 

    eCAL::CPublisher ekf_odom_pub("odom", "std::string");

    // Initialize gps/odom topic  
    bool differential = false;
    bool relative = false;
    bool pose_use_child_frame = false;
    std::string odomTopicName = "odom";
    double poseMahalanobisThresh = 5;
    double twistMahalanobisThresh = 5;
    bool nodelayOdom = true;
    int odomQueueSize = 1;

    std::vector<int> updateVector(STATE_SIZE, 0);
    updateVector = {true,  true, false, false, false, false, false, false, false, false, false, false, false, false, false};

    std::vector<int> poseUpdateVec = updateVector;
    std::fill(poseUpdateVec.begin() + POSITION_V_OFFSET, poseUpdateVec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);
    std::vector<int> twistUpdateVec = updateVector;
    std::fill(twistUpdateVec.begin() + POSITION_OFFSET, twistUpdateVec.begin() + POSITION_OFFSET + POSE_SIZE, 0);

    int poseUpdateSum = std::accumulate(poseUpdateVec.begin(), poseUpdateVec.end(), 0);
    int twistUpdateSum = std::accumulate(twistUpdateVec.begin(), twistUpdateVec.end(), 0);

    const CallbackData gps_poseCallbackData(odomTopicName + "_pose", poseUpdateVec, poseUpdateSum, differential,
      relative, pose_use_child_frame, poseMahalanobisThresh);
    const CallbackData gps_twistCallbackData(odomTopicName + "_twist", twistUpdateVec, twistUpdateSum, false, false,
      false, twistMahalanobisThresh);

    eCAL::string::CSubscriber<std::string> gps_odom_sub("gps/odom");
    gps_odom_sub.AddReceiveCallback(std::bind(&RosFilter::odometryCallback, this, std::placeholders::_2,
                odomTopicName, gps_poseCallbackData, gps_twistCallbackData));

    // Initialize imu/data topic 
    differential = false;
    relative = false;
    std::string imuTopicName = "imu/data";
    poseMahalanobisThresh = 0.8;
    twistMahalanobisThresh = 0.8;
    double accelMahalanobisThresh = 0.8;
    bool nodelayImu = false;
    int imuQueueSize = 5;
    bool removeGravAcc = false;

    updateVector = {false, false, false, true,  true,  true, false, false, false, true,  true,  true, true,  true,  true };
    
    poseUpdateVec = updateVector;
    // IMU message contains no information about position, filter everything except orientation
    std::fill(poseUpdateVec.begin() + POSITION_OFFSET, poseUpdateVec.begin() + POSITION_OFFSET + POSITION_SIZE,0);
    std::fill(poseUpdateVec.begin() + POSITION_V_OFFSET,poseUpdateVec.begin() + POSITION_V_OFFSET + TWIST_SIZE,0);
    std::fill(poseUpdateVec.begin() + POSITION_A_OFFSET,poseUpdateVec.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE,0);

    twistUpdateVec = updateVector;
    // IMU message contains no information about linear speeds, filter everything except angular velocity
    std::fill(twistUpdateVec.begin() + POSITION_OFFSET,twistUpdateVec.begin() + POSITION_OFFSET + POSE_SIZE,0);
    std::fill(twistUpdateVec.begin() + POSITION_V_OFFSET,twistUpdateVec.begin() + POSITION_V_OFFSET + LINEAR_VELOCITY_SIZE,0);
    std::fill(twistUpdateVec.begin() + POSITION_A_OFFSET,twistUpdateVec.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE,0);

    std::vector<int> accelUpdateVec = updateVector;
    std::fill(accelUpdateVec.begin() + POSITION_OFFSET,accelUpdateVec.begin() + POSITION_OFFSET + POSE_SIZE,0);
    std::fill(accelUpdateVec.begin() + POSITION_V_OFFSET,accelUpdateVec.begin() + POSITION_V_OFFSET + TWIST_SIZE,0);

    poseUpdateSum = std::accumulate(poseUpdateVec.begin(), poseUpdateVec.end(), 0);
    twistUpdateSum = std::accumulate(twistUpdateVec.begin(), twistUpdateVec.end(), 0);
    int accelUpdateSum = std::accumulate(accelUpdateVec.begin(), accelUpdateVec.end(), 0);
    
    const CallbackData imu_poseCallbackData(imuTopicName + "_pose", poseUpdateVec, poseUpdateSum, differential,
      relative, false, poseMahalanobisThresh);
    const CallbackData imu_twistCallbackData(imuTopicName + "_twist", twistUpdateVec, twistUpdateSum, differential,
      relative, false, twistMahalanobisThresh);
    const CallbackData imu_accelCallbackData(imuTopicName + "_acceleration", accelUpdateVec, accelUpdateSum,
      differential, relative, false, accelMahalanobisThresh);


    // eCAL::string::CSubscriber<std::string> imu_data_sub("imu/data");
    // imu_data_sub.AddReceiveCallback(std::bind(&RosFilter::imuCallback, this, std::placeholders::_2,
    //             imuTopicName, imu_poseCallbackData, imu_twistCallbackData, imu_accelCallbackData));

    // Initialize imu/data topic 
    differential = false;
    relative = false;
    std::string twistTopicName = "wheel/odom";
    twistMahalanobisThresh = 0.8;
    bool nodelayTwist = false;
    int twistQueueSize = 5;
    

    std::fill(twistUpdateVec.begin() + POSITION_OFFSET, twistUpdateVec.begin() + POSITION_OFFSET + POSE_SIZE, 0);
    twistUpdateSum = std::accumulate(twistUpdateVec.begin(), twistUpdateVec.end(), 0);
    const CallbackData wheel_callbackData(twistTopicName, twistUpdateVec, twistUpdateSum, false, false,
      false, twistMahalanobisThresh);

    // eCAL::string::CSubscriber<std::string> wheel_data_sub("wheel/data");
    // wheel_data_sub.AddReceiveCallback(std::bind(&RosFilter::twistCallback, this, std::placeholders::_2,
    //             wheel_callbackData, baseLinkFrameId_));



    while(eCAL::Ok())
    { 
      auto start = std::chrono::system_clock::now();
      eCAL::Process::SleepMS(1000/10);

      Odometry filteredPosition = periodicUpdate();

      Serialize<Odometry> ekf_odom;
      std::stringstream ss_odom;
      ekf_odom.serialize(filteredPosition,ss_odom);
      ser_odom_msg = ss_odom.str().data();
      ekf_odom_pub.Send(ser_odom_msg);

      cout << std::setprecision(8) << std::fixed << endl;  // set output to fixed floating point, 8 decimal precision
      // cout << "stamp " << filteredPosition.header.time<< endl;
      // cout << "frame_id " << filteredPosition.header.frame_id << endl;
      // cout << "x "  << filteredPosition.pose.pose.position.x << endl;
      // cout << "y "  << filteredPosition.pose.pose.position.y << endl;
      // cout << "z "  << filteredPosition.pose.pose.position.z << endl;
      // cout << "qx "  << filteredPosition.pose.pose.orientation.x << endl;
      // cout << "qy"  << filteredPosition.pose.pose.orientation.y << endl;
      // cout << "qz"  << filteredPosition.pose.pose.orientation.z << endl;
      // cout << "qw"  << filteredPosition.pose.pose.orientation.w << endl;
      // cout << "position_covariance "  << filteredPosition.pose.covariance[0] << "," << filteredPosition.pose.covariance[4] << "," << filteredPosition.pose.covariance[8] << endl;
      // cout << "..................... " << endl;

      auto end = std::chrono::system_clock::now();
      std::chrono::duration<double> diff = end - start;
      // cout << "Time to process last frame (seconds): " << diff.count() << " FPS: " << 1.0 / diff.count()  << endl;;

    }

  }

  template<typename T>
  void RosFilter<T>::reset()
  {
    // Get rid of any initial poses (pretend we've never had a measurement)
    previousMeasurementCovariances_.clear();

    clearMeasurementQueue();

    filterStateHistory_.clear();
    measurementHistory_.clear();

    // Also set the last set pose time, so we ignore all messages
    // that occur before it
    lastSetPoseTime_ = double(0.0);

    // reset filter to uninitialized state
    filter_.reset();
  }
  
  // @todo: Replace with AccelWithCovarianceStamped
  template<typename T>
  void RosFilter<T>::accelerationCallback(const Imu &msg, const CallbackData &callbackData,
    const std::string &targetFrame)
  {
    // If we've just reset the filter, then we want to ignore any messages
    // that arrive with an older timestamp
    if (msg.header.time <= lastSetPoseTime_)
    {
      return;
    }

    const std::string &topicName = callbackData.topicName_;

    // if (lastMessageTimes_.count(topicName) == 0)
    // {
    //   lastMessageTimes_.insert(std::pair<std::string, ros::Time>(topicName, msg->header.stamp));
    // }

    // Make sure this message is newer than the last one
    // if (msg.header.time >= lastMessageTimes_[topicName])
    // {
    Eigen::VectorXd measurement(STATE_SIZE);
    Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);

    measurement.setZero();
    measurementCovariance.setZero();

    // Make sure we're actually updating at least one of these variables
    std::vector<int> updateVectorCorrected = callbackData.updateVector_;

    // Prepare the twist data for inclusion in the filter
    if (prepareAcceleration(msg, topicName, targetFrame, callbackData.relative_, updateVectorCorrected, measurement,
          measurementCovariance))
    {
      // Store the measurement. Add an "acceleration" suffix so we know what kind of measurement
      // we're dealing with when we debug the core filter logic.
      enqueueMeasurement(topicName,
                          measurement,
                          measurementCovariance,
                          updateVectorCorrected,
                          callbackData.rejectionThreshold_,
                          msg.header.time);

    }

    // lastMessageTimes_[topicName] = msg.header.stamp;
    // }
    // else if (resetOnTimeJump_ )
    // {
    //   reset();
    // }

  }


  template<typename T>
  void RosFilter<T>::enqueueMeasurement(const std::string &topicName,
                                        const Eigen::VectorXd &measurement,
                                        const Eigen::MatrixXd &measurementCovariance,
                                        const std::vector<int> &updateVector,
                                        const double mahalanobisThresh,
                                        const double)
  {
    MeasurementPtr meas = MeasurementPtr(new Measurement());

    meas->topicName_ = topicName;
    meas->measurement_ = measurement;
    meas->covariance_ = measurementCovariance;
    meas->updateVector_ = updateVector;
    auto now = std::chrono::high_resolution_clock::now();
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();    
    meas->time_ = nanoseconds/1e9;
    meas->mahalanobisThresh_ = mahalanobisThresh;
    meas->latestControl_ = latestControl_;
    // meas->latestControlTime_ = latestControlTime_.toSec();
    measurementQueue_.push(meas);
  }

  template<typename T>
  void RosFilter<T>::forceTwoD(Eigen::VectorXd &measurement,
                               Eigen::MatrixXd &measurementCovariance,
                               std::vector<int> &updateVector)
  {
    measurement(StateMemberZ) = 0.0;
    measurement(StateMemberRoll) = 0.0;
    measurement(StateMemberPitch) = 0.0;
    measurement(StateMemberVz) = 0.0;
    measurement(StateMemberVroll) = 0.0;
    measurement(StateMemberVpitch) = 0.0;
    measurement(StateMemberAz) = 0.0;

    measurementCovariance(StateMemberZ, StateMemberZ) = 1e-6;
    measurementCovariance(StateMemberRoll, StateMemberRoll) = 1e-6;
    measurementCovariance(StateMemberPitch, StateMemberPitch) = 1e-6;
    measurementCovariance(StateMemberVz, StateMemberVz) = 1e-6;
    measurementCovariance(StateMemberVroll, StateMemberVroll) = 1e-6;
    measurementCovariance(StateMemberVpitch, StateMemberVpitch) = 1e-6;
    measurementCovariance(StateMemberAz, StateMemberAz) = 1e-6;

    updateVector[StateMemberZ] = 1;
    updateVector[StateMemberRoll] = 1;
    updateVector[StateMemberPitch] = 1;
    updateVector[StateMemberVz] = 1;
    updateVector[StateMemberVroll] = 1;
    updateVector[StateMemberVpitch] = 1;
    updateVector[StateMemberAz] = 1;
  }

  template<typename T>
  bool RosFilter<T>::getFilteredOdometryMessage(Odometry &message)
  {
    // If the filter has received a measurement at some point...
    if (filter_.getInitializedStatus())
    {
      // Grab our current state and covariance estimates
      const Eigen::VectorXd &state = filter_.getState();
      const Eigen::MatrixXd &estimateErrorCovariance = filter_.getEstimateErrorCovariance();

      // Convert from roll, pitch, and yaw back to quaternion for
      // orientation values
      Quaternion quat;
      double cr = cos(state(StateMemberRoll) * 0.5);
      double sr = sin(state(StateMemberRoll) * 0.5);
      double cp = cos(state(StateMemberPitch) * 0.5);
      double sp = sin(state(StateMemberPitch) * 0.5);
      double cy = cos(state(StateMemberYaw) * 0.5);
      double sy = sin(state(StateMemberYaw) * 0.5);

      quat.w = cr * cp * cy + sr * sp * sy;
      quat.x = sr * cp * cy - cr * sp * sy;
      quat.y = cr * sp * cy + sr * cp * sy;
      quat.z = cr * cp * sy - sr * sp * cy;

      // Fill out the message
      message.pose.pose.position.x = state(StateMemberX);
      message.pose.pose.position.y = state(StateMemberY);
      message.pose.pose.position.z = state(StateMemberZ);
      message.pose.pose.orientation.x = quat.x;
      message.pose.pose.orientation.y = quat.y;
      message.pose.pose.orientation.z = quat.z;
      message.pose.pose.orientation.w = quat.w;
      message.twist.twist.linear.x = state(StateMemberVx);
      message.twist.twist.linear.y = state(StateMemberVy);
      message.twist.twist.linear.z = state(StateMemberVz);
      message.twist.twist.angular.x = state(StateMemberVroll);
      message.twist.twist.angular.y = state(StateMemberVpitch);
      message.twist.twist.angular.z = state(StateMemberVyaw);

      // Our covariance matrix layout doesn't quite match
      for (size_t i = 0; i < POSE_SIZE; i++)
      {
        for (size_t j = 0; j < POSE_SIZE; j++)
        {
          message.pose.covariance[POSE_SIZE * i + j] = estimateErrorCovariance(i, j);
        }
      }

      // POSE_SIZE and TWIST_SIZE are currently the same size, but we can spare a few
      // cycles to be meticulous and not index a twist covariance array on the size of
      // a pose covariance array
      for (size_t i = 0; i < TWIST_SIZE; i++)
      {
        for (size_t j = 0; j < TWIST_SIZE; j++)
        {
          message.twist.covariance[TWIST_SIZE * i + j] = estimateErrorCovariance(i + POSITION_V_OFFSET, j + POSITION_V_OFFSET);
        }
      }

      message.header.time = filter_.getLastMeasurementTime();
      message.header.frame_id = worldFrameId_;
      message.child_frame_id = baseLinkOutputFrameId_;
    }

    return filter_.getInitializedStatus();
  }

  template<typename T>
  void RosFilter<T>::imuCallback(const std::string &msg,
                                 const std::string &topicName,
                                 const CallbackData &poseCallbackData,
                                 const CallbackData &twistCallbackData,
                                 const CallbackData &accelCallbackData)
  {
    std::stringstream ss_imu_in(msg);
    Serialize<Imu> imu_data;
    Imu imu;
    imu = imu_data.deserialize(ss_imu_in,imu);

    // cout << "frame_id " << imu.header.frame_id << endl;
    // cout << "stamp " << imu.header.time << endl;
    // cout << "ax " << imu.linear_acceleration.x << endl;
    // cout << "ay " << imu.linear_acceleration.y << endl;
    // cout << "az" << imu.linear_acceleration.z << endl;
    
    // If we've just reset the filter, then we want to ignore any messages
    // that arrive with an older timestamp
    if (imu.header.time <= lastSetPoseTime_)
    {
      cout << "IMU Received message that preceded the most recent pose reset. Ignoring..." << endl;

      return;
    }

    // As with the odometry message, we can separate out the pose- and twist-related variables
    // in the IMU message and pass them to the pose and twist callbacks (filters)

    if (poseCallbackData.updateSum_ > 0)
    {
      // Per the IMU message specification, if the IMU does not provide orientation,
      // then its first covariance value should be set to -1, and we should ignore
      // that portion of the message. robot_localization allows users to explicitly
      // ignore data using its parameters, but we should also be compliant with
      // message specs.

      if (::fabs(imu.orientation_covariance[0] + 1) < 1e-9)
      {
        cout << "Received IMU message with -1 as its first covariance value for orientation. Ignoring orientation..." << endl;
      }
      else
      {
        // Extract the pose (orientation) data, pass it to its filter
        PoseWithCovarianceStamped posPtr;
        posPtr.header.frame_id = imu.header.frame_id;
        posPtr.header.time = imu.header.time;
        posPtr.pose.pose.orientation = imu.orientation;

        // Copy the covariance for roll, pitch, and yaw
        for (size_t i = 0; i < ORIENTATION_SIZE; i++)
        {
          for (size_t j = 0; j < ORIENTATION_SIZE; j++)
          {
            posPtr.pose.covariance[POSE_SIZE * (i + ORIENTATION_SIZE) + (j + ORIENTATION_SIZE)] =
                imu.orientation_covariance[ORIENTATION_SIZE * i + j];
          }
        }

        poseCallback(posPtr, poseCallbackData, baseLinkFrameId_,
                     baseLinkFrameId_, true);
      }
    }

    if (twistCallbackData.updateSum_ > 0)
    {
      // Ignore rotational velocity if the first covariance value is -1
      if (::fabs(imu.angular_velocity_covariance[0] + 1) < 1e-9)
      {
        cout << "Received IMU message with -1 as its first covariance value for angular velocity. Ignoring angular velocity..." << endl;
      }
      else
      {
        // Repeat for velocity
        TwistWithCovarianceStamped twistPtr;
        twistPtr.header.frame_id = imu.header.frame_id;
        twistPtr.header.time = imu.header.time;
        twistPtr.twist.twist.angular = imu.angular_velocity;

        // Copy the covariance
        for (size_t i = 0; i < ORIENTATION_SIZE; i++)
        {
          for (size_t j = 0; j < ORIENTATION_SIZE; j++)
          {
            twistPtr.twist.covariance[TWIST_SIZE * (i + ORIENTATION_SIZE) + (j + ORIENTATION_SIZE)] =
              imu.angular_velocity_covariance[ORIENTATION_SIZE * i + j];
          }
        }

        twistCallback(twistPtr, twistCallbackData, baseLinkFrameId_);
      }
    }

    if (accelCallbackData.updateSum_ > 0)
    {
      // Ignore linear acceleration if the first covariance value is -1
      if (::fabs(imu.linear_acceleration_covariance[0] + 1) < 1e-9)
      {
        cout << "Received IMU message with -1 as its first covariance value for linear acceleration. Ignoring linear acceleration..." << endl;
      }
      else
      {
        // Pass the message on
        accelerationCallback(imu, accelCallbackData, baseLinkFrameId_);
      }
    }

  }

  template<typename T>
  void RosFilter<T>::integrateMeasurements(const double currentTime)
  {
    auto now = std::chrono::high_resolution_clock::now();
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();       
    
    const double currentTimeSec = nanoseconds/1e9;

    bool predictToCurrentTime = predictToCurrentTime_;

    // If we have any measurements in the queue, process them
    if (!measurementQueue_.empty())
    {
      // Check if the first measurement we're going to process is older than the filter's last measurement.
      // This means we have received an out-of-sequence message (one with an old timestamp), and we need to
      // revert both the filter state and measurement queue to the first state that preceded the time stamp
      // of our first measurement.
      const MeasurementPtr& firstMeasurement = measurementQueue_.top();
      int restoredMeasurementCount = 0;
      if (smoothLaggedData_ && firstMeasurement->time_ < filter_.getLastMeasurementTime())
      {
        
        int originalCount = static_cast<int>(measurementQueue_.size());
        const double firstMeasurementTime =  firstMeasurement->time_;
        const std::string firstMeasurementTopic =  firstMeasurement->topicName_;
        if (!revertTo(firstMeasurementTime - 1e-9))  // revertTo may invalidate firstMeasurement
        {
          restoredMeasurementCount = 0;
        }

        restoredMeasurementCount = static_cast<int>(measurementQueue_.size()) - originalCount;
      }

      while (!measurementQueue_.empty() )
      {
        MeasurementPtr measurement = measurementQueue_.top();

        // If we've reached a measurement that has a time later than now, it should wait until a future iteration.
        // Since measurements are stored in a priority queue, all remaining measurements will be in the future.
        if (measurement->time_ > currentTime)
        {
          break;
        }

        measurementQueue_.pop();

        // This will call predict and, if necessary, correct
        filter_.processMeasurement(*(measurement.get()));

        // Store old states and measurements if we're smoothing
        if (smoothLaggedData_)
        {
          // Invariant still holds: measurementHistoryDeque_.back().time_ < measurementQueue_.top().time_
          measurementHistory_.push_back(measurement);

          // We should only save the filter state once per unique timstamp
          if (measurementQueue_.empty() ||
              ::fabs(measurementQueue_.top()->time_ - filter_.getLastMeasurementTime()) > 1e-9)
          {
            saveFilterState(filter_);
          }
        }
      }
    }
    else if (filter_.getInitializedStatus())
    {
      // In the event that we don't get any measurements for a long time,
      // we still need to continue to estimate our state. Therefore, we
      // should project the state forward here.
      double lastUpdateDelta = currentTimeSec - filter_.getLastMeasurementTime();

      // If we get a large delta, then continuously predict until
      if (lastUpdateDelta >= filter_.getSensorTimeout())
      {
        predictToCurrentTime = true;
      }
    }
    

    if (filter_.getInitializedStatus() && predictToCurrentTime)
    {
      double lastUpdateDelta = currentTimeSec - filter_.getLastMeasurementTime();

      filter_.validateDelta(lastUpdateDelta);
      filter_.predict(currentTimeSec, lastUpdateDelta);

      // Update the last measurement time and last update time
      filter_.setLastMeasurementTime(filter_.getLastMeasurementTime() + lastUpdateDelta);
    }

  }

  template<typename T>
  void RosFilter<T>::loadParams()
  {
    /* For diagnostic purposes, collect information about how many different
     * sources are measuring each absolute pose variable and do not have
     * differential integration enabled.
     */
    std::map<StateMembers, int> absPoseVarCounts;
    absPoseVarCounts[StateMemberX] = 0;
    absPoseVarCounts[StateMemberY] = 0;
    absPoseVarCounts[StateMemberZ] = 0;
    absPoseVarCounts[StateMemberRoll] = 0;
    absPoseVarCounts[StateMemberPitch] = 0;
    absPoseVarCounts[StateMemberYaw] = 0;

    // Same for twist variables
    std::map<StateMembers, int> twistVarCounts;
    twistVarCounts[StateMemberVx] = 0;
    twistVarCounts[StateMemberVy] = 0;
    twistVarCounts[StateMemberVz] = 0;
    twistVarCounts[StateMemberVroll] = 0;
    twistVarCounts[StateMemberVpitch] = 0;
    twistVarCounts[StateMemberVyaw] = 0;


    // Check for custom gravitational acceleration value
    gravitationalAcc_ = 9.80665;

    // These params specify the name of the robot's body frame (typically
    // base_link) and odometry frame (typically odom)
    mapFrameId_ = std::string("map");
    odomFrameId_ = std::string("odom");
    baseLinkFrameId_ = std::string("base_link");
    baseLinkOutputFrameId_ =  baseLinkFrameId_;
    worldFrameId_ =  odomFrameId_;

    // Whether we'll allow old measurements to cause a re-publication of the updated state
    permitCorrectedPublication_ = false;

    // Update frequency and sensor timeout
    double sensorTimeout;
    frequency_ = 30.0;
    sensorTimeout = 1.0 / frequency_;
    filter_.setSensorTimeout(sensorTimeout);

    // Determine if we're in 2D mode
    twoDMode_ = true;

    // Smoothing window size
    smoothLaggedData_ = false;
    historyLength_ = 0.0;

    // Wether we reset filter on jump back in time
    resetOnTimeJump_ = false;

    if (!smoothLaggedData_ && ::fabs(historyLength_) > 1e-9)
    {
      std::cout << "Filter history interval of " << historyLength_ <<" specified, but smooth_lagged_data is set to false. Lagged data will not be smoothed." << endl;
    }

    if (smoothLaggedData_ && historyLength_ < -1e9)
    {
      std::cout << "Negative history interval of " << historyLength_ << " specified. Absolute value will be assumed." << endl;
    }

    historyLength_ = ::fabs(historyLength_);

    predictToCurrentTime_ = false;

    bool dynamicProcessNoiseCovariance = false;
    filter_.setUseDynamicProcessNoiseCovariance(dynamicProcessNoiseCovariance);

    std::vector<double> initialState(STATE_SIZE, 0.0);

    // Check if the filter should start or not
    disabledAtStartup_ = false;
    enabled_ = !disabledAtStartup_;

    // Init the last measurement time so we don't get a huge initial delta
    auto now = std::chrono::high_resolution_clock::now();
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();       

    filter_.setLastMeasurementTime(nanoseconds/1e9);

    // Now pull in each topic to which we want to subscribe.
    // Start with odom.
    size_t topicInd = 0;
    bool moreParams = false;
      
    bool differential = false;
    bool relative = false;
    bool pose_use_child_frame = false;
    std::string odomTopicName = "odom";
    double poseMahalanobisThresh = 5;
    double twistMahalanobisThresh = 5;
    bool nodelayOdom = true;
    int odomQueueSize = 1;

    std::vector<int> updateVector(STATE_SIZE, 0);
    updateVector = {true,  true, false, false, false, false, false, false, false, false, false, false, false, false, false};

    std::vector<int> poseUpdateVec = updateVector;
    std::fill(poseUpdateVec.begin() + POSITION_V_OFFSET, poseUpdateVec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);
    std::vector<int> twistUpdateVec = updateVector;
    std::fill(twistUpdateVec.begin() + POSITION_OFFSET, twistUpdateVec.begin() + POSITION_OFFSET + POSE_SIZE, 0);

    int poseUpdateSum = std::accumulate(poseUpdateVec.begin(), poseUpdateVec.end(), 0);
    int twistUpdateSum = std::accumulate(twistUpdateVec.begin(), twistUpdateVec.end(), 0);

    const CallbackData poseCallbackData(odomTopicName + "_pose", poseUpdateVec, poseUpdateSum, differential,
      relative, pose_use_child_frame, poseMahalanobisThresh);
    const CallbackData twistCallbackData(odomTopicName + "_twist", twistUpdateVec, twistUpdateSum, false, false,
      false, twistMahalanobisThresh);


    if (poseUpdateSum > 0)
    {
      if (differential)
      {
        twistVarCounts[StateMemberVx] += poseUpdateVec[StateMemberX];
        twistVarCounts[StateMemberVy] += poseUpdateVec[StateMemberY];
        twistVarCounts[StateMemberVz] += poseUpdateVec[StateMemberZ];
        twistVarCounts[StateMemberVroll] += poseUpdateVec[StateMemberRoll];
        twistVarCounts[StateMemberVpitch] += poseUpdateVec[StateMemberPitch];
        twistVarCounts[StateMemberVyaw] += poseUpdateVec[StateMemberYaw];
      }
      else
      {
        absPoseVarCounts[StateMemberX] += poseUpdateVec[StateMemberX];
        absPoseVarCounts[StateMemberY] += poseUpdateVec[StateMemberY];
        absPoseVarCounts[StateMemberZ] += poseUpdateVec[StateMemberZ];
        absPoseVarCounts[StateMemberRoll] += poseUpdateVec[StateMemberRoll];
        absPoseVarCounts[StateMemberPitch] += poseUpdateVec[StateMemberPitch];
        absPoseVarCounts[StateMemberYaw] += poseUpdateVec[StateMemberYaw];
      }
    }

    if (twistUpdateSum > 0)
    {
      twistVarCounts[StateMemberVx] += twistUpdateVec[StateMemberVx];
      twistVarCounts[StateMemberVy] += twistUpdateVec[StateMemberVx];
      twistVarCounts[StateMemberVz] += twistUpdateVec[StateMemberVz];
      twistVarCounts[StateMemberVroll] += twistUpdateVec[StateMemberVroll];
      twistVarCounts[StateMemberVpitch] += twistUpdateVec[StateMemberVpitch];
      twistVarCounts[StateMemberVyaw] += twistUpdateVec[StateMemberVyaw];
    }

      
      
    // Load up the process noise covariance (from the launch file/parameter server)
    Eigen::MatrixXd processNoiseCovariance(STATE_SIZE, STATE_SIZE);
    processNoiseCovariance.setZero();

    // int matSize = processNoiseCovariance.rows();
    // x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az
    for (int i = 0; i < processNoiseCovariance.rows(); i++)
    {
      for (int j = 0; j < processNoiseCovariance.rows(); j++)
      {
          processNoiseCovariance(i, j) = 0.05;
      }
    }

    filter_.setProcessNoiseCovariance(processNoiseCovariance);
    

    // Load up the process noise covariance (from the launch file/parameter server)
    Eigen::MatrixXd initialEstimateErrorCovariance(STATE_SIZE, STATE_SIZE);
    initialEstimateErrorCovariance.setZero();

    // int matSize = initialEstimateErrorCovariance.rows();

    for (int i = 0; i < initialEstimateErrorCovariance.rows(); i++)
    {
      for (int j = 0; j < initialEstimateErrorCovariance.rows(); j++)
      {
        initialEstimateErrorCovariance(i, j) = 1e-9;
      }
    }

    filter_.setEstimateErrorCovariance(initialEstimateErrorCovariance);
  }

  template<typename T>
  void RosFilter<T>::odometryCallback(const std::string &msg, const std::string &topicName,
    const CallbackData &poseCallbackData, const CallbackData &twistCallbackData)
  {

    cout << msg << endl;
    std::stringstream ss_odom_in(msg);
    Serialize<Odometry> data;
    Odometry odom;
    odom = data.deserialize(ss_odom_in,odom);

    cout << "in" << endl;

    // If we've just reset the filter, then we want to ignore any messages
    // that arrive with an older timestamp
    if (odom.header.time <= lastSetPoseTime_)
    {
      cout << "The " << topicName << " message has a timestamp equal to or before the last filter reset, " <<
                "this message will be ignored. This may indicate an empty or bad timestamp. (message time: " <<
                 ")" << endl;
      return;
    }

    PoseWithCovarianceStamped posPtr;

    if (poseCallbackData.updateSum_ > 0)
    {
      // Grab the pose portion of the message and pass it to the poseCallback

      PoseWithCovarianceStamped *posPtr = new PoseWithCovarianceStamped;
      posPtr->header = odom.header;
      posPtr->pose = odom.pose;
      cout << odom.pose.covariance[0] << endl;
      // PoseWithCovarianceStampedConstPtr pptr(posPtr);
      if (poseCallbackData.pose_use_child_frame_)
      {
        poseCallback(*posPtr, poseCallbackData, worldFrameId_, odom.child_frame_id, false);
      }
      else
      {
        poseCallback(*posPtr, poseCallbackData, worldFrameId_, baseLinkFrameId_, false);
      }
    }

    if (twistCallbackData.updateSum_ > 0)
    {
      // Grab the twist portion of the message and pass it to the twistCallback
      // TwistWithCovarianceStamped *twistPtr = new TwistWithCovarianceStamped();
      TwistWithCovarianceStamped twistPtr;
      twistPtr.header = odom.header;
      twistPtr.twist = odom.twist;
      
      twistCallback(twistPtr, twistCallbackData, baseLinkFrameId_);
    }

  }

  template<typename T>
  void RosFilter<T>::poseCallback(const PoseWithCovarianceStamped &msg,
                                  const CallbackData &callbackData,
                                  const std::string &targetFrame,
                                  const std::string &poseSourceFrame,
                                  const bool imuData)
  {
    const std::string &topicName = callbackData.topicName_;

    // If we've just reset the filter, then we want to ignore any messages
    // that arrive with an older timestamp
    if (msg.header.time <= lastSetPoseTime_)
    {
      return;
    }


    Eigen::VectorXd measurement(STATE_SIZE);
    Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);

    measurement.setZero();
    measurementCovariance.setZero();

    // Make sure we're actually updating at least one of these variables
    std::vector<int> updateVectorCorrected = callbackData.updateVector_;

    // Prepare the pose data for inclusion in the filter
    if (preparePose(msg,
                    topicName,
                    targetFrame,
                    poseSourceFrame,
                    callbackData.differential_,
                    callbackData.relative_,
                    imuData,
                    updateVectorCorrected,
                    measurement,
                    measurementCovariance))
    {
      // Store the measurement. Add a "pose" suffix so we know what kind of measurement
      // we're dealing with when we debug the core filter logic.
      enqueueMeasurement(topicName,
                          measurement,
                          measurementCovariance,
                          updateVectorCorrected,
                          callbackData.rejectionThreshold_,
                          msg.header.time);

    }


  }

  template<typename T>
  Odometry RosFilter<T>::periodicUpdate()
  {

    //Get system time for time stamp.
    auto now = std::chrono::high_resolution_clock::now();
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
            
    double curTime = nanoseconds/1e9;

    if (toggledOn_)
    {
      // Now we'll integrate any measurements we've received if requested, and update angular acceleration.
      integrateMeasurements(curTime);
    }
    else
    {
      // clear out measurements since we're not currently processing new entries
      clearMeasurementQueue();

      // Reset last measurement time so we don't get a large time delta on toggle on
      if (filter_.getInitializedStatus())
      {
        //Get system time for time stamp.
        auto now = std::chrono::high_resolution_clock::now();
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
            
        double curTime = nanoseconds/1e9;
    
        filter_.setLastMeasurementTime(nanoseconds/1e9);
      }
    }

    // Get latest state and publish it
    Odometry filteredPosition;

    bool corrected_data = false;

    if (getFilteredOdometryMessage(filteredPosition))
    {
      if (!validateFilterOutput(filteredPosition))
      {
        cout << "Critical Error, NaNs were detected in the output state of the filter." << " This was likely due to poorly conditioned process, noise, or sensor covariances." << endl;
      }
    }
    return filteredPosition;

  }

  template<typename T>
  void RosFilter<T>::twistCallback(const TwistWithCovarianceStamped &msg,
                                   const CallbackData &callbackData,
                                   const std::string &targetFrame)
  {
    const std::string &topicName = callbackData.topicName_;

    // If we've just reset the filter, then we want to ignore any messages
    // that arrive with an older timestamp
    if (msg.header.time <= lastSetPoseTime_)
    {
      return;
    }


    Eigen::VectorXd measurement(STATE_SIZE);
    Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);

    measurement.setZero();
    measurementCovariance.setZero();

    // Make sure we're actually updating at least one of these variables
    std::vector<int> updateVectorCorrected = callbackData.updateVector_;

    // Prepare the twist data for inclusion in the filter
    if (prepareTwist(msg, topicName, targetFrame, updateVectorCorrected, measurement, measurementCovariance))
    {
      // Store the measurement. Add a "twist" suffix so we know what kind of measurement
      // we're dealing with when we debug the core filter logic.
      enqueueMeasurement(topicName,
                          measurement,
                          measurementCovariance,
                          updateVectorCorrected,
                          callbackData.rejectionThreshold_,
                          msg.header.time);

    }
    

  }

  template<typename T>
  void RosFilter<T>::copyCovariance(const double *arr,
                                    Eigen::MatrixXd &covariance,
                                    const std::string &topicName,
                                    const std::vector<int> &updateVector,
                                    const size_t offset,
                                    const size_t dimension)
  {
    for (size_t i = 0; i < dimension; i++)
    {
      for (size_t j = 0; j < dimension; j++)
      {
        covariance(i, j) = arr[dimension * i + j];

        
        {
          std::string iVar = stateVariableNames_[offset + i];

          if (covariance(i, j) > 1e3 && (updateVector[offset  + i] || updateVector[offset  + j]))
          {
            std::string jVar = stateVariableNames_[offset + j];

          }
        }
      }
    }
  }

  template<typename T>
  void RosFilter<T>::copyCovariance(const Eigen::MatrixXd &covariance,
                                 double *arr,
                                 const size_t dimension)
  {
    for (size_t i = 0; i < dimension; i++)
    {
      for (size_t j = 0; j < dimension; j++)
      {
        arr[dimension * i + j] = covariance(i, j);
      }
    }
  }


  template<typename T>
  bool RosFilter<T>::prepareAcceleration(const Imu &msg,
                           const std::string &topicName,
                           const std::string &targetFrame,
                           const bool relative,
                           std::vector<int> &updateVector,
                           Eigen::VectorXd &measurement,
                           Eigen::MatrixXd &measurementCovariance)
  {
    bool retVal = false;

    // Get the measurement into a vector which are gravity compensated
    
    // 3. We'll need to rotate the covariance as well
    Eigen::MatrixXd covarianceRotated(ACCELERATION_SIZE, ACCELERATION_SIZE);
    covarianceRotated.setZero();

    this->copyCovariance(&(msg.linear_acceleration_covariance[0]),
                         covarianceRotated,
                         topicName,
                         updateVector,
                         POSITION_A_OFFSET,
                         ACCELERATION_SIZE);

    const Eigen::VectorXd &state = filter_.getState();


    // 6. Store our corrected measurement and covariance
    measurement(StateMemberAx) = msg.linear_acceleration.x;
    measurement(StateMemberAy) = msg.linear_acceleration.y;
    measurement(StateMemberAz) = msg.linear_acceleration.z;

    // Copy the covariances
    measurementCovariance.block(POSITION_A_OFFSET, POSITION_A_OFFSET, ACCELERATION_SIZE, ACCELERATION_SIZE) =
      covarianceRotated.block(0, 0, ACCELERATION_SIZE, ACCELERATION_SIZE);

    // 7. Handle 2D mode
    if (twoDMode_)
    {
      forceTwoD(measurement, measurementCovariance, updateVector);
    }
    

    retVal = true;
    return retVal;
  }


  template<typename T>
  bool RosFilter<T>::preparePose(const PoseWithCovarianceStamped &msg,
                                 const std::string &topicName,
                                 const std::string &targetFrame,
                                 const std::string &sourceFrame,
                                 const bool differential,
                                 const bool relative,
                                 const bool imuData,
                                 std::vector<int> &updateVector,
                                 Eigen::VectorXd &measurement,
                                 Eigen::MatrixXd &measurementCovariance)
  {
    bool retVal = false;

    // 1. Get the measurement into a tf-friendly transform (pose) object

    Quaternion quat;

    quat.x = msg.pose.pose.orientation.x;
    quat.y = msg.pose.pose.orientation.y;
    quat.z = msg.pose.pose.orientation.z;
    quat.w = msg.pose.pose.orientation.w;

    // 5a. We'll need to rotate the covariance as well. Create a container and copy over the
    // covariance data
    Eigen::MatrixXd covariance(POSE_SIZE, POSE_SIZE);
    covariance.setZero();
    copyCovariance(&(msg.pose.covariance[0]), covariance, topicName, updateVector, POSITION_OFFSET, POSE_SIZE);

    cout << covariance << endl;
    // 7i. Finally, copy everything into our measurement and covariance objects
    measurement(StateMemberX) = msg.pose.pose.position.x;
    measurement(StateMemberY) = msg.pose.pose.position.y;
    measurement(StateMemberZ) = msg.pose.pose.position.z;

    double roll, pitch, yaw;
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z);
    double cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y);
    roll =  atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (quat.w * quat.y - quat.z * quat.x);
    if (abs(sinp) >= 1)
        pitch =  copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch =  asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
    yaw = atan2(siny_cosp, cosy_cosp);

    measurement(StateMemberRoll) = roll;
    measurement(StateMemberPitch) = pitch;
    measurement(StateMemberYaw) = yaw;

    measurementCovariance.block(0, 0, POSE_SIZE, POSE_SIZE) = covariance.block(0, 0, POSE_SIZE, POSE_SIZE);

    // 8. Handle 2D mode
    if (twoDMode_)
    {
      forceTwoD(measurement, measurementCovariance, updateVector);
    }

    retVal = true;


    return retVal;
  }

  template<typename T>
  bool RosFilter<T>::prepareTwist(const TwistWithCovarianceStamped &msg,
                                  const std::string &topicName,
                                  const std::string &targetFrame,
                                  std::vector<int> &updateVector,
                                  Eigen::VectorXd &measurement,
                                  Eigen::MatrixXd &measurementCovariance)
  {
    bool retVal = false;

    // 3. We'll need to rotate the covariance as well
    Eigen::MatrixXd covariance(TWIST_SIZE, TWIST_SIZE);
    covariance.setZero();

    copyCovariance(&(msg.twist.covariance[0]),
                   covariance,
                   topicName,
                   updateVector,
                   POSITION_V_OFFSET,
                   TWIST_SIZE);

    // 6. Store our corrected measurement and covariance
    measurement(StateMemberVx) = msg.twist.twist.linear.x;
    measurement(StateMemberVy) = msg.twist.twist.linear.y;
    measurement(StateMemberVz) = msg.twist.twist.linear.z;
    measurement(StateMemberVroll) = msg.twist.twist.angular.x;
    measurement(StateMemberVpitch) = msg.twist.twist.angular.y;
    measurement(StateMemberVyaw) = msg.twist.twist.angular.z;

    // Copy the covariances
    measurementCovariance.block(POSITION_V_OFFSET, POSITION_V_OFFSET, TWIST_SIZE, TWIST_SIZE) =
        covariance.block(0, 0, TWIST_SIZE, TWIST_SIZE);

    // 7. Handle 2D mode
    if (twoDMode_)
    {
      forceTwoD(measurement, measurementCovariance, updateVector);
    }

    retVal = true;

    return retVal;
  }

  template<typename T>
  void RosFilter<T>::saveFilterState(FilterBase& filter)
  {
    FilterStatePtr state = FilterStatePtr(new FilterState());
    state->state_ = Eigen::VectorXd(filter.getState());
    state->estimateErrorCovariance_ = Eigen::MatrixXd(filter.getEstimateErrorCovariance());
    state->lastMeasurementTime_ = filter.getLastMeasurementTime();
    state->latestControl_ = Eigen::VectorXd(filter.getControl());
    state->latestControlTime_ = filter.getControlTime();
    filterStateHistory_.push_back(state);
  }

  template<typename T>
  bool RosFilter<T>::revertTo(const double time)
  {
    size_t history_size = filterStateHistory_.size();

    // Walk back through the queue until we reach a filter state whose time stamp is less than or equal to the
    // requested time. Since every saved state after that time will be overwritten/corrected, we can pop from
    // the queue. If the history is insufficiently short, we just take the oldest state we have.
    FilterStatePtr lastHistoryState;
    while (!filterStateHistory_.empty() && filterStateHistory_.back()->lastMeasurementTime_ > time)
    {
      lastHistoryState = filterStateHistory_.back();
      filterStateHistory_.pop_back();
    }

    // If the state history is not empty at this point, it means that our history was large enough, and we
    // should revert to the state at the back of the history deque.
    bool retVal = false;
    if (!filterStateHistory_.empty())
    {
      retVal = true;
      lastHistoryState = filterStateHistory_.back();
    }
    else
    {

      if (lastHistoryState)
      {
      }
    }

    // If we have a valid reversion state, revert
    if (lastHistoryState)
    {
      // Reset filter to the latest state from the queue.
      const FilterStatePtr &state = lastHistoryState;
      filter_.setState(state->state_);
      filter_.setEstimateErrorCovariance(state->estimateErrorCovariance_);
      filter_.setLastMeasurementTime(state->lastMeasurementTime_);

      // Repeat for measurements, but push every measurement onto the measurement queue as we go
      int restored_measurements = 0;
      while (!measurementHistory_.empty() && measurementHistory_.back()->time_ > time)
      {
        // Don't need to restore measurements that predate our earliest state time
        if (state->lastMeasurementTime_ <= measurementHistory_.back()->time_)
        {
          measurementQueue_.push(measurementHistory_.back());
          restored_measurements++;
        }

        measurementHistory_.pop_back();
      }

    }

    return retVal;
  }

  template<typename T>
  bool RosFilter<T>::validateFilterOutput(const Odometry &message)
  {
    return !std::isnan(message.pose.pose.position.x) && !std::isinf(message.pose.pose.position.x) &&
           !std::isnan(message.pose.pose.position.y) && !std::isinf(message.pose.pose.position.y) &&
           !std::isnan(message.pose.pose.position.z) && !std::isinf(message.pose.pose.position.z) &&
           !std::isnan(message.pose.pose.orientation.x) && !std::isinf(message.pose.pose.orientation.x) &&
           !std::isnan(message.pose.pose.orientation.y) && !std::isinf(message.pose.pose.orientation.y) &&
           !std::isnan(message.pose.pose.orientation.z) && !std::isinf(message.pose.pose.orientation.z) &&
           !std::isnan(message.pose.pose.orientation.w) && !std::isinf(message.pose.pose.orientation.w) &&
           !std::isnan(message.twist.twist.linear.x) && !std::isinf(message.twist.twist.linear.x) &&
           !std::isnan(message.twist.twist.linear.y) && !std::isinf(message.twist.twist.linear.y) &&
           !std::isnan(message.twist.twist.linear.z) && !std::isinf(message.twist.twist.linear.z) &&
           !std::isnan(message.twist.twist.angular.x) && !std::isinf(message.twist.twist.angular.x) &&
           !std::isnan(message.twist.twist.angular.y) && !std::isinf(message.twist.twist.angular.y) &&
           !std::isnan(message.twist.twist.angular.z) && !std::isinf(message.twist.twist.angular.z);
  }

  template<typename T>
  void RosFilter<T>::clearExpiredHistory(const double cutOffTime)
  {
    

    int poppedMeasurements = 0;
    int poppedStates = 0;

    while (!measurementHistory_.empty() && measurementHistory_.front()->time_ < cutOffTime)
    {
      measurementHistory_.pop_front();
      poppedMeasurements++;
    }

    while (!filterStateHistory_.empty() && filterStateHistory_.front()->lastMeasurementTime_ < cutOffTime)
    {
      filterStateHistory_.pop_front();
      poppedStates++;
    }

  }

  template<typename T>
  void RosFilter<T>::clearMeasurementQueue()
  {
    while (!measurementQueue_.empty())
    {
      measurementQueue_.pop();
    }
    return;
  }
}  // namespace RobotLocalization

// Instantiations of classes is required when template class code
// is placed in a .cpp file.
template class RobotLocalization::RosFilter<RobotLocalization::Ekf>;
