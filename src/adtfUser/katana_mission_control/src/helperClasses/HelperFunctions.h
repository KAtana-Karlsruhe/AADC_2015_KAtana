#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include "katanaCommon/katanaCommon.h"
#include "mission_control/world.h"
#include "mission_control/System.h"
#include "WorldPlausibilityChecker.h"

namespace katana {

class HelperFunctions
{
  public:
    HelperFunctions();

    ~HelperFunctions();

    //static katana::TrafficSign getTrafficSign(int32_t identifier);

    static katana::JuryAction getJuryAction(int8_t action);

    static katana::World::RoadPatchContainer getVirtualPatches(katana::System::Ptr system, katana::RoadBase::ConstPtr junction);

    //! Get Projection of pose on given strip
    typedef std::vector<oadrive::core::Pose2d> PoseVector;
    static bool projectPoseOnTrajectory(const oadrive::core::Position2d& position, const PoseVector& pose_vector, Pose2d& projection);
};

} //ns
#endif // HELPERFUNCTIONS_H
