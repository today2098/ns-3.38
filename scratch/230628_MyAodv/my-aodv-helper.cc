#include "my-aodv-helper.h"

#include "my-aodv-routing-protocol.h"

#include "ns3/ipv4-list-routing.h"
#include "ns3/names.h"
#include "ns3/node-list.h"
#include "ns3/ptr.h"

namespace ns3 {

MyAodvHelper::MyAodvHelper()
    : Ipv4RoutingHelper() {
    m_agentFactory.SetTypeId("ns3::myaodv::RoutingProtocol");
}

MyAodvHelper*
MyAodvHelper::Copy() const {
    return new MyAodvHelper(*this);
}

Ptr<Ipv4RoutingProtocol>
MyAodvHelper::Create(Ptr<Node> node) const {
    Ptr<myaodv::RoutingProtocol> agent = m_agentFactory.Create<myaodv::RoutingProtocol>();
    node->AggregateObject(agent);
    return agent;
}

void MyAodvHelper::Set(std::string name, const AttributeValue& value) {
    m_agentFactory.Set(name, value);
}

int64_t
MyAodvHelper::AssignStreams(NodeContainer c, int64_t stream) {
    int64_t currentStream = stream;
    Ptr<Node> node;
    for(NodeContainer::Iterator i = c.Begin(); i != c.End(); ++i) {
        node = (*i);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        NS_ASSERT_MSG(ipv4, "Ipv4 not installed on node");
        Ptr<Ipv4RoutingProtocol> proto = ipv4->GetRoutingProtocol();
        NS_ASSERT_MSG(proto, "Ipv4 routing not installed on node");
        Ptr<myaodv::RoutingProtocol> aodv = DynamicCast<myaodv::RoutingProtocol>(proto);
        if(aodv) {
            currentStream += aodv->AssignStreams(currentStream);
            continue;
        }
        // Aodv may also be in a list
        Ptr<Ipv4ListRouting> list = DynamicCast<Ipv4ListRouting>(proto);
        if(list) {
            int16_t priority;
            Ptr<Ipv4RoutingProtocol> listProto;
            Ptr<myaodv::RoutingProtocol> listAodv;
            for(uint32_t i = 0; i < list->GetNRoutingProtocols(); i++) {
                listProto = list->GetRoutingProtocol(i, priority);
                listAodv = DynamicCast<myaodv::RoutingProtocol>(listProto);
                if(listAodv) {
                    currentStream += listAodv->AssignStreams(currentStream);
                    break;
                }
            }
        }
    }
    return (currentStream - stream);
}

}  // namespace ns3
