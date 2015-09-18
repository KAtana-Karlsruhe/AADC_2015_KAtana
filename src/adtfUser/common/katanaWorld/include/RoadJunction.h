// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-12-07
 *
 */
//----------------------------------------------------------------------

#ifndef _KATANA_WORLD_ROADJUNCTION_H
#define _KATANA_WORLD_ROADJUNCTION_H

#include "RoadBase.h"

namespace katana
{
/**
 * @brief Junction, obtained by pattern matching.
 * Defined by driving strip polygons @see RoadBase
 */

enum class RIGHT_OF_WAY : u_int8_t {
  OWN,
  STRAIGHT,
  LEFT,
  RIGHT,
  PRIORITY_FROM_RIGHT,
  UNDEFINED                   // yield right of way to every other vehicle (precaution)
};

class RoadJunction : public RoadBase
{
public:
  //! Shared pointer shorthand
  typedef std::shared_ptr<RoadJunction> Ptr;
  typedef std::shared_ptr<const RoadJunction> ConstPtr;

  //! Right of way
  typedef std::pair<RIGHT_OF_WAY, RIGHT_OF_WAY> RightOfWayDirection;

  //!
  enum class TURN_ACTION : u_int8_t
  {
    LEFT = 0,
    STRAIGHT = 1,
    RIGHT = 2
  };

  //! Constructor
  RoadJunction() = delete;
  RoadJunction(const RoadPatch::ConstPtr& road_patch, const Pose2d& pose, u_int32_t id)
    : RoadBase(road_patch, pose, id)
    , m_direction(Action::UNKNOWN)
  {
    // currently the junction needs exactly 3 diversions
    assert(getEndPoseContainer().size() == 3);

    setRightOfWay(RightOfWayDirection(RIGHT_OF_WAY::UNDEFINED, RIGHT_OF_WAY::UNDEFINED));
    m_is_stop = true; // precaution
    m_missing_diversion = DIVERSION::UNDEFINED; //< assume 4-diversion junction
  }

  //! Destructor
  virtual ~RoadJunction()    {}

  //!
  virtual bool isJunction() const override    { return true; }

  //! Set right of way
  void setRightOfWay(const RightOfWayDirection& right_of_way);
  void setRightOfWay(TrafficSign ts);

  //!
  std::vector<DIVERSION> yieldRightOfWayTo(DIVERSION where_to_go) const;
  //! is stop?
  bool getIsStop() const  { return m_is_stop; }
  
  //! set the driving direction
  //TODO: use yieldRightOfWayTo
  virtual void setDirection(Action direction) override	{ m_direction = direction; }
  virtual Action getDirection() const override		{ return m_direction; }
  
  virtual RoadPatch::TrajectoryPtr getDrivingStrip(DrivingStripId drivingStrip) const override;
  
protected:
  //! Driving direction
  Action m_direction; 

private:
  //! Junction type
  DIVERSION m_missing_diversion;

  //! Right of way
  RightOfWayDirection m_right_of_way;

  //! STOP-junction
  bool m_is_stop;

};


} // ns

#endif //_KATANA_WORLD_ROADJUNCTION_H
