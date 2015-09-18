#include "SignQueue.h"

void SignQueue::deleteOldElements(int32_t current)
{
  for(std::vector<SignElement>::iterator it = m_elements.begin(); it != m_elements.end(); ++it) {
    if(current - it->second > m_slidingWindow) {
      it = m_elements.erase(it);
    }
  }
}
