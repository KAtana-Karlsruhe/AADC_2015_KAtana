#ifndef WORLDPLAUSIBILITYCHECKER_H
#define WORLDPLAUSIBILITYCHECKER_H

#include "../src/mission_control/world.h"

namespace katana
{
  
class WorldPlausibilityChecker
{
  public:
    
    WorldPlausibilityChecker();
    
    ~WorldPlausibilityChecker();
    
    //! Check if newRoadbase is plausible considering the already known roadBases
    static bool plausibility_check_for_new_Patch(RoadBase::ConstPtr const newRoadbase, World::RoadPatchContainer const roadbases);
  
    //! Check if newRoadbase is similar to Roadbase it is overwriting
    static bool plausibility_check_for_known_Patch(RoadBase::ConstPtr const newRoadbase, RoadBase::ConstPtr const oldRoadbase, World::RoadPatchContainer const roadbases);
    
    static void check_ids(World::RoadPatchContainer const roadbases);
    static void check_ids(std::vector<katana::RoadBase::ConstPtr> const roadbases);
};
}

#endif // WORLDPLAUSIBILITYCHECKER_H
