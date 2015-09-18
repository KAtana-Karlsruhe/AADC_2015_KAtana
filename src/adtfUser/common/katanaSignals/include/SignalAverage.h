// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-11-25
 *
 */
//----------------------------------------------------------------------

#ifndef KATANA_COMMON_SIGNAL_H
#define KATANA_COMMON_SIGNAL_H

#include <stdlib.h>
#include <cmath>
#include <queue>

namespace katana
{

template<typename tValue, typename tTimestamp>
class SignalAverage
{
public:
  typedef std::pair<tValue, tTimestamp> Sample;
  typedef std::queue<Sample> SampleContainer;

  SignalAverage()
    : m_keep_duration(100)
  {

  }

  SignalAverage(tTimestamp keep_duration)
    : m_keep_duration(keep_duration)
    , m_sum(0.0)
    , m_temp(0.0)
  {
    if (m_keep_duration < 0)
      m_keep_duration = 0;
  }

  virtual ~SignalAverage()    {}

  //! Clear
  void clear()
  {
    while (!m_samples.empty())
      m_samples.pop();

    m_sum = 0.0;
  }

  void setInterval(tTimestamp interval)
  {
    m_keep_duration = interval;
  }

  //!
  u_int32_t getContainerSize() const  { return m_samples.size(); }
  bool isEmpty() const                { return m_samples.empty(); }

  //! Returns current average
  const tValue& get() const      { return m_temp; }

  //! Return sum
  const tValue& getSum() const   { return m_sum; }

  //! Returns last in queue
  const Sample& getOldest() const   { return m_samples.front(); }

  //! Returns newest in queue
  const Sample& getNewest() const   { return m_samples.back(); }

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

  //!
  tValue m_sum;

  //!
  tValue m_temp;

};

template <typename tValue, typename tTimestamp>
void SignalAverage<tValue, tTimestamp>::addSample(Sample sample)
{
  // push new sample
  m_samples.push(sample);

  // remove old values if necessary
  removeOld();

  if (m_samples.empty())
  {
    m_temp = 0.0;
    m_sum = 0.0;
  }
  else
  {
    // update sum
    m_sum += sample.first;

    // update value
    m_temp = m_sum / m_samples.size();
  }
}

template <typename tValue, typename tTimestamp>
void SignalAverage<tValue, tTimestamp>::removeOld()
{
  while(!m_samples.empty() && (m_samples.front().second < (m_samples.back().second - m_keep_duration) || m_samples.front().second > m_samples.back().second))
  {
    m_sum -= m_samples.front().first;
    m_samples.pop();
  }
}

} // ns

#endif //KATANA_COMMON_SIGNAL_H
