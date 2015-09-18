// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-03-17
 *
 */
//----------------------------------------------------------------------

#ifndef _MISSION_CONTROL_LANETRACKERJOBMANAGER_H
#define _MISSION_CONTROL_LANETRACKERJOBMANAGER_H

#include "katanaCommon/katanaCommon.h"
#include "Pose.h"
#include "PinConversions.h"

#include "RoadBase.h"

#include <queue>

namespace katana
{

//! Patches transmit function
typedef std::function<void(const std::vector<katana::RoadBase::ConstPtr>& patches,
                           u_int8_t status,
                           u_int8_t number_of_stitches,
                           u_int8_t patches_to_search,
                           double matching_threshold)> TransmitPatchesFunc;

//! Patch answer vector
typedef std::vector<sPatch> PatchVector;
typedef std::shared_ptr<PatchVector> PatchVectorPtr;
typedef std::shared_ptr<const PatchVector> PatchVectorConstPtr;

class LanetrackerJobManager
{
public:

  struct LTJob
  {
    std::vector<katana::RoadBase::ConstPtr> patches;
    u_int8_t status;
    u_int8_t number_of_stitches;
    u_int8_t patches_to_search;
    double matching_threshold;
  };

  //! Convenience pointer
  typedef std::shared_ptr<LanetrackerJobManager> Ptr;

  //! Constructor
  LanetrackerJobManager() = delete;

  //! No copying, instance handles jobs for one lanetracker
  LanetrackerJobManager(const LanetrackerJobManager& rs) = delete;
  LanetrackerJobManager& operator=(const LanetrackerJobManager& rhs) = delete;

  LanetrackerJobManager(const TransmitPatchesFunc& transmit_func)
    : m_output_patches(transmit_func)
    , m_next_id(0)
    , m_last_id(0)
    , m_working(false)
    , m_ready(false)
  {

  }

  //! Destructor
  virtual ~LanetrackerJobManager()     {}

  //! Ping lanetracker until ready
  bool pingLanetracker() const;

  //! Add job
  u_int32_t submitJob(const LTJob& job);
  u_int32_t submitJob(const std::vector<katana::RoadBase::ConstPtr>& patches,
                      u_int8_t status,
                      u_int8_t number_of_stitches,
                      u_int8_t patches_to_search,
                      double matching_threshold)
  {
    return submitJob(LTJob{patches, status, number_of_stitches, patches_to_search, matching_threshold});
  }

  //! Job finished...
  bool finishedJob(const PatchVectorPtr& patch_vector, PerceptionState state, u_int32_t& job_id);

  //! Information
  bool isLTWorking() const    { return m_working; }
  u_int32_t getLTQueueLength() const    { return m_jobs.size(); }
  bool isLTReady() const      { return m_ready; }

  //! Last finished job
  PerceptionState getLastPerceptionState() const    { return m_last_state; }
  PatchVectorConstPtr getLastLTAnswer() const       { return m_last_answer; }

private:
  //! Jobs to submit
  std::queue<LTJob> m_jobs;

  //! Call lanetracker
  void callLanetracker(const LTJob& job) const
  {
    m_output_patches(job.patches, job.status, job.number_of_stitches, job.patches_to_search, job.matching_threshold);
  }

  //! Output patches
  TransmitPatchesFunc m_output_patches;

  //! Unique job identifier
  //! ID for next submitted job
  u_int32_t m_next_id;
  //! ID for last finished job
  u_int32_t m_last_id;

  //! Last answer from lanetracker
  PatchVectorPtr m_last_answer;
  PerceptionState m_last_state;

  //! Flag if lt is currently working
  bool m_working;

  //! Flag if lanetracker is ready
  bool m_ready;

};


} // ns

#endif //_MISSION_CONTROL_LANETRACKERJOBMANAGER_H
