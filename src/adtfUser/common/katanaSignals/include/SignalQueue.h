// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-01-13
 *
 */
//----------------------------------------------------------------------

#ifndef KATANA_COMMON_SIGNALQUEUE_H
#define KATANA_COMMON_SIGNALQUEUE_H

#include <stdlib.h>
#include <cmath>
#include <queue>

namespace katana
{

template<typename tValue, typename tTimestamp>
class SignalQueue
{
public:
  typedef std::pair<tValue, tTimestamp> Sample;
  typedef std::queue<Sample> SampleContainer;

  SignalQueue() = delete;
  SignalQueue(tTimestamp keep_duration)
    : m_keep_duration(keep_duration)
  {
    if (m_keep_duration < 0)
      m_keep_duration = 0;
  }

  virtual ~SignalQueue()    {}

  //! Clear
  void clear()
  {
    while (!m_samples.empty())
      m_samples.pop();

  }

  //! Returns last in queue
  bool getOldest(tValue& value) const
  {
    if (m_samples.empty())
      return false;

    value = m_samples.front();

    return true;
  }

  //! Returns first in queue
  bool getNewest(tValue& value) const
  {
    if (m_samples.empty())
      return false;

    value = m_samples.back();

    return true;
  }

  void addSample(tValue value, tTimestamp timestamp)
  {
    addSample(Sample(value, timestamp));
  }

  void addSample(Sample sample);

private:
  //!
  void removeOld();

  //!
  SampleContainer m_samples;

  //!
  tTimestamp m_keep_duration;

};

template <typename tValue, typename tTimestamp>
void SignalQueue<tValue, tTimestamp>::addSample(Sample sample)
{
  // push new sample
  m_samples.push(sample);

  // remove old values if necessary
  removeOld();
}

template <typename tValue, typename tTimestamp>
void SignalQueue<tValue, tTimestamp>::removeOld()
{
  while(!m_samples.empty() && (m_samples.front().second < (m_samples.back().second - m_keep_duration) || m_samples.front().second > m_samples.back().second))
  {
    m_samples.pop();
  }
}

} // ns

#endif //KATANA_COMMON_SIGNALQUEUE_H
