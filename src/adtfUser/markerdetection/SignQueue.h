#ifndef SIGNQUEUE_H
#define SIGNQUEUE_H

#include <stdint.h>
#include <vector>
#include <marker.h>




class SignQueue
{
public:

  typedef std::pair<aruco::Marker, int32_t> SignElement;

  SignQueue() {
    m_slidingWindow = 5;
  }

  void addElement(SignElement element)	{
    m_elements.push_back(element);
    deleteOldElements(element.second);
  }

private:

  void deleteOldElements(int32_t current);

  std::vector<SignElement> m_elements;

  // lenght of the sliding window
  int m_slidingWindow;

};

#endif // SIGNQUEUE_H
