#include "my-aodv-id-cache.h"

#include <algorithm>

namespace ns3 {
namespace myaodv {

bool IdCache::IsDuplicate(Ipv4Address addr, uint32_t id) {
    Purge();
    for(std::vector<UniqueId>::const_iterator i = m_idCache.begin(); i != m_idCache.end(); ++i) {
        if(i->m_context == addr && i->m_id == id) {
            return true;
        }
    }
    UniqueId uniqueId = {addr, id, m_lifetime + Simulator::Now()};
    m_idCache.push_back(uniqueId);
    return false;
}

void IdCache::Purge() {
    m_idCache.erase(remove_if(m_idCache.begin(), m_idCache.end(), IsExpired()), m_idCache.end());
}

uint32_t
IdCache::GetSize() {
    Purge();
    return m_idCache.size();
}

}  // namespace myaodv
}  // namespace ns3
