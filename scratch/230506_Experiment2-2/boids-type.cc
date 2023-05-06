#include "boids-type.h"

#include "ns3/core-module.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("BoidsType");

NS_OBJECT_ENSURE_REGISTERED(BoidsType);

TypeId BoidsType::GetTypeId(void) {
    static TypeId tid = TypeId("ns3::BoidsType")
                            .SetParent<Object>()
                            .AddConstructor<BoidsType>()
                            .AddAttribute("Type",
                                          "Boids type of node.",
                                          EnumValue(BoidsTypeEnum::BOID),
                                          MakeEnumAccessor(&BoidsType::m_boidsType),
                                          MakeEnumChecker(BoidsTypeEnum::BOID, "Boid",
                                                          BoidsTypeEnum::PREY, "Prey",
                                                          BoidsTypeEnum::ENEMY, "Enemy",
                                                          BoidsTypeEnum::IGNORE, "Ignore"));
    return tid;
}

BoidsType::BoidsType() : m_boidsType(BoidsTypeEnum::BOID) {
    NS_LOG_FUNCTION(this);
}

enum BoidsType::BoidsTypeEnum BoidsType::GetBoidsType(void) const {
    NS_LOG_FUNCTION(this);
    return m_boidsType;
}

void BoidsType::SetBoidsType(enum BoidsTypeEnum boidsType) {
    NS_LOG_FUNCTION(this << boidsType);
    m_boidsType = boidsType;
}

}  // namespace ns3
