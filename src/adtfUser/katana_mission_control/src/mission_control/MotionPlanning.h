#ifndef _MISSION_CONTROL_MOTION_PLANNING_H
#define _MISSION_CONTROL_MOTION_PLANNING_H

#include "katanaCommon/katanaCommon.h"
#include "RoadBase.h"
#include "RoadJunction.h"
#include "SignalAverage.h"
#include "VelocityControl.h"

using namespace std;

namespace katana
{

/**
 * @brief The MotionPlanning class
 */
class MotionPlanning
{
  friend class MissionControl;

public:
  //! Convenience pointer
  typedef std::shared_ptr<MotionPlanning> Ptr;

  //! Pointer for extended points
  typedef std::shared_ptr<ExtendedPose2dVector> ExtendedPose2dVectorPtr;
  typedef std::shared_ptr<const ExtendedPose2dVector> ExtendedPose2dVectorConstPtr;

  //! Constructor
  MotionPlanning()
    : m_driven_distance_since_last_update(0xffffffff)
    , m_isJunctionInStrip(false)
    , m_driving_strip(DEFAULT_DRIVING_STRIP)
  {
  }

  //! Destructor
  virtual ~MotionPlanning()
  {
  }

  //! Access
  double& defaultSpeed()                  { return m_default_speed; }
  const double& defaultSpeed() const      { return m_default_speed; }

  u_int32_t& drivingStrip()               { return m_driving_strip; }
  const u_int32_t& drivingStrip() const   { return m_driving_strip; }

  //! Create a completely new driving strip from the given road patches
  void generateRoadDrivingTrajectory(const vector<RoadBase::ConstPtr> &road_bases);

  //!
  bool isReadyToDrive() const              { return m_standard_trajectory.size() >= MIN_EXTENDED_POINTS_NEEDED_FOR_LATERAL_CONTROLLER; }

  //! Is a junction in the last generated trajectory
  bool isJunctionInStrip()		   { return m_isJunctionInStrip; }

  //! Interpolate trajectory, calculate orientation, ...
  void prepareTrajectory(oadrive::core::Trajectory2d& trajectory) const;

  //! Set a trajectory
  void setTrajectory(Trajectory2d& traj)		    { m_standard_trajectory = traj; }

  /** READ ACCESS **/
  //!
  _position_type getDistanceSinceLastUpdate() const   { return m_driven_distance_since_last_update; }

  //! get most recent pose
  const PoseWithTime& getPose() const                         { return m_last_pose; }

  //! read current trajectory
  const Trajectory2d& getTrajectory() const    { return m_standard_trajectory; }

  //! set velocity, achieving that the vehicle drives forward a certain distance
  void setVelocityToDriveDistance(Trajectory2d& trajectory, std::size_t start_at, double desired_distance, double desired_speed) const;

  //! create reverse trajectory from given trajectory
  oadrive::core::Trajectory2d createReverseTrajectory(const Trajectory2d& trajectory) const;

private:
  //! Gets updated by mission control (independent by current state)
  void poseChanged(const PoseWithTime& p)
  {
    m_driven_distance_since_last_update += (p.pose.translation() - m_last_pose.pose.translation()).norm();
    m_last_pose = p;
  }

  //!
  //! \brief createTrajectoryToRoad: linear Trajectory polygon beginning at current vehicle pose, ending at appropriate point on road
  //! \param road bases, obtained from world (get next patches). If current vehicle pose is already located any of this patches,
  //!        an empty trajectory will be returned, if vehicle is after the end of the road, an empty trajectory will be returned
  //! \return Trajectory
  //!
  //Trajectory2d createTrajectoryToRoad(const vector<RoadBase::ConstPtr>& road_bases, const Trajectory2d& strip) const;

  //! holds last generated trajectory from road patches
  Trajectory2d m_standard_trajectory;

  //! driven distance since the last update of m_extended_points_ptr
  _position_type m_driven_distance_since_last_update;

  //! current vehicle pose
  PoseWithTime m_last_pose;

  //! is a junction in the last planned motion
  bool m_isJunctionInStrip;

  //! Default speed
  double m_default_speed;

  //! Current driving strip
  u_int32_t m_driving_strip;
};


} // ns

#endif //_MISSION_CONTROL_MOTION_PLANNING_H
