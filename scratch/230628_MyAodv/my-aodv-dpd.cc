#include "my-aodv-dpd.h"

namespace ns3 {
namespace myaodv {

bool DuplicatePacketDetection::IsDuplicate(Ptr<const Packet> p, const Ipv4Header& header) {
    return m_idCache.IsDuplicate(header.GetSource(), p->GetUid());
}

void DuplicatePacketDetection::SetLifetime(Time lifetime) {
    m_idCache.SetLifetime(lifetime);
}

Time DuplicatePacketDetection::GetLifetime() const {
    return m_idCache.GetLifeTime();
}

}  // namespace myaodv
}  // namespace ns3
