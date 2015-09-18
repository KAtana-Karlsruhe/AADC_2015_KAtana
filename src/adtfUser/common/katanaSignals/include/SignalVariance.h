// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Raphael Frisch <frishc@fzi.de>
 * \date    2015-03-11
 *
 */
//----------------------------------------------------------------------

#ifndef KATANA_COMMON_SIGNAL_VARIANCE_H
#define KATANA_COMMON_SIGNAL_VARIANCE_H

#include <stdlib.h>
#include <cmath>
#include <queue>
#include <iostream>

#include <boost/circular_buffer.hpp>

//#include <icl_core/RingBuffer.h>

namespace katana
{


class SignalVariance
{
public:
  typedef boost::circular_buffer<double> SampleContainer;


  SignalVariance()
    : m_sum(0.0)
    , m_temp(0.0)
  {

  }

  virtual ~SignalVariance()    {}

  //! Clear
  void clear()
  {
    m_samples.clear();
    m_sum = 0.0;
  }

  void setNumber(u_int32_t number)
  {
    m_samples.set_capacity(number);
  }

  //!
  u_int32_t getContainerSize() const  { return m_samples.size(); }
  bool isEmpty() const                { return m_samples.empty(); }

  //! Returns current average
  const double& get() const      { return m_temp; }

  //! Return sum
  const double& getSum() const   { return m_sum; }

  void addSample(double value);

  double calculateVariance() const;

private:
//  //!
//  void removeOld();

  //!
  SampleContainer m_samples;

  //!
  double m_sum;

  //!
  double m_temp;

};

double SignalVariance::calculateVariance() const
{
  double variance = 0;

  for(boost::circular_buffer<double>::const_iterator it = m_samples.begin(); it != m_samples.end(); ++it)
  {
    const double diff = (*it) - m_temp;
    variance += diff*diff/m_samples.size();
  }

  return variance;
}

void SignalVariance::addSample(double value)
{
  if(m_samples.full())
  {
   m_sum -= m_samples.front();
   m_samples.pop_front();
  }
  // push new sample
  m_samples.push_back(value);

  // update sum
  m_sum += value;
  // update value
  m_temp = m_sum / m_samples.size();

}

} // ns

#endif //KATANA_COMMON_SIGNAL_VARIANCE_H
