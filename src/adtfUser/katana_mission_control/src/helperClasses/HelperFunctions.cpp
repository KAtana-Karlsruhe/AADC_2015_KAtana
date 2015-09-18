#include "HelperFunctions.h"

#include <oadrive_core/Pose.h>
#include <oadrive_core/Interpolator.h>

using namespace oadrive::core;

namespace katana {

HelperFunctions::HelperFunctions()
{

}

HelperFunctions::~HelperFunctions()
{

}

/*TrafficSign HelperFunctions::getTrafficSign(int32_t identifier)
{
  TrafficSign sign;
  switch (identifier)
  {
    case 1:
      sign = TrafficSign::JUNCTION_GIVE_WAY;
      break;
    case 2:
      sign = TrafficSign::JUNCTION_PRIORITY;
      break;
    case 3:
      sign = TrafficSign::JUNCTION_STOP_GIVE_WAY;
      break;
    case 4:
      sign = TrafficSign::PARKING_AHEAD;
      break;
    case 5:
      sign = TrafficSign::PRESCRIBED_DIRECTION;
      break;
    case 6:
      sign = TrafficSign::JUNCTION_PRIORITY_FROM_RIGHT;
      break;
    default:
      std::cout << "Received unkown identifier: " << identifier << std::endl;
      sign = TrafficSign::UNKNOWN;
  }

  return sign;
}*/

JuryAction HelperFunctions::getJuryAction(int8_t action)
{
  if(action == (int8_t)JuryAction::STOP) {
    return STOP;
  } else if(action == (int8_t)JuryAction::REQUEST_READY) {
    return REQUEST_READY;
  } else if(action == (int8_t)JuryAction::RUN) {
    return RUN;
  } else {
    assert(false && "Received unkown jury action");
    return JuryAction::STOP;
  }
}

World::RoadPatchContainer HelperFunctions::getVirtualPatches(System::Ptr system, RoadBase::ConstPtr junction)
{
  assert(junction->getPatchType() == PatchType::JUNCTION && "getVirtualPatches called with no junction");

  // Get position of endpose in endposecontainer
  int position = 1; // STRAIGHT
  if(junction->getDirection() == Action::RIGHT) {
    position = 2; // RIGHT
  } else if (junction->getDirection() == Action::LEFT) {
    position = 0; //LEFT
  }

  // Calculate Anchorpose of new Pach start = mid of junction
  Pose2d startPose = junction->getEndPoseContainer().at(position);

  // Id of the junction and start id for the "virtual" straight patches
  u_int32_t startID = junction->getId();

  std::vector<katana::RoadBase::Ptr> working_copy;

  for(u_int32_t i = 1; i <= 2; ++i) {
    // instantiate a new "virtual" patch
    RoadBase::Ptr segment = system->getWorld()->instantiateRoadBase(PatchType::STRAIGHT, startPose, (startID + i), true);
    startPose = segment->getEndPose();
    working_copy.push_back(segment);
  }

  #ifndef NDEBUG
    WorldPlausibilityChecker::check_ids(working_copy);
  #endif

  return working_copy;
}

bool HelperFunctions::projectPoseOnTrajectory(const Position2d& position, const PoseVector &pose_vector, Pose2d& projection)
{
  double shortest_distance = std::numeric_limits<double>::max();
  double distance;
  double best_t = 0;
  size_t nearest_index = 0;

  for (std::size_t i = 0; i < pose_vector.size() - 1; i++)
  {
    const Position2d ab = pose_vector[i+1].translation() - pose_vector[i].translation();
    const Position2d ap = position - pose_vector[i].translation();

    const double t = ab.dot(ap)/ab.squaredNorm();

    if (t < 0.0)
    {
      distance = (position - pose_vector[i].translation()).norm();
    }
    else if (t > 1.0)
    {
      distance = (position - pose_vector[i+1].translation()).norm();
    }
    else
    {
      distance = (position - (pose_vector[i].translation() + t * ab)).norm();
    }

    if (distance < shortest_distance)
    {
      shortest_distance = distance;
      nearest_index = i;
      best_t = t;
    }
  }


  if (best_t < 0.0)
  {
    projection = pose_vector[nearest_index];
  }
  else if (best_t > 1.0)
  {
    projection = pose_vector[nearest_index+1];
  }
  else
  {
    projection = Interpolator::interpolateLinear(pose_vector[nearest_index], pose_vector[nearest_index+1], best_t);
  }

  return (nearest_index != pose_vector.size() - 2) || (best_t <= 1.0);
}

} //ns
