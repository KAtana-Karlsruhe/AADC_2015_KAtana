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

#include "LanetrackerJobManager.h"

#ifdef KATANA_MC_LANETRACKER_JOBS_DEBUG
#include <iostream>
#endif

namespace katana
{

u_int32_t LanetrackerJobManager::submitJob(const LTJob &job)
{
  assert(m_ready && "Lanetracker not ready yet, but job was submitted.");

  if (!m_working)
  {
    m_working = true;
    callLanetracker(job);
  }
  else
  {
    m_jobs.push(job);
#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
    std::cout <<"[LanetrackerJobManager] Added job to queue, id " <<m_next_id <<"! Number of jobs in queue: " <<m_jobs.size() <<std::endl;
#endif
  }
  return m_next_id++;
}

bool LanetrackerJobManager::finishedJob(const PatchVectorPtr& patch_vector, PerceptionState state, u_int32_t& job_id)
{
  if (!m_ready)
  {
    assert(state == PerceptionState::DO_NOTHING && "Lanetracker answered in initial phase with something else than DO_NOTHING");

#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
    std::cout <<"[LanetrackerJobManager] Lanetracker answered to ping, seems ready!" <<std::endl;
#endif

    m_ready = true;

    return false;
  }

  m_last_state = state;
  m_last_answer = patch_vector;

#ifdef KATANA_MC_LANETRACKER_JOBS_DEBUG
  std::cout <<"[LanetrackerJobManager] Finished job, ID: " <<m_last_id <<std::endl;
#endif

  if (!m_jobs.empty())
  {
    callLanetracker(m_jobs.front());
    m_jobs.pop();
  }
  else
  {
    m_working = false;
#ifdef KATANA_MC_LANETRACKER_JOBS_DEBUG
    std::cout <<"[LanetrackerJobManager] No more jobs in queue! Lanetracker is sleeping..." <<std::endl;
#endif
  }

  job_id = m_last_id;
  ++m_last_id;

  return true;
}

bool LanetrackerJobManager::pingLanetracker() const
{
  if (m_ready)
    return true;

  callLanetracker(LTJob{std::vector<katana::RoadBase::ConstPtr>(), PerceptionState::DO_NOTHING, 0, 0, -1.0});

  return false;
}

} // ns
