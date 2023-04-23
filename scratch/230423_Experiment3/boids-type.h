#ifndef BOIDS_TYPE_H
#define BOIDS_TYPE_H

#include "ns3/core-module.h"

namespace ns3 {

class BoidsType : public Object {
public:
    enum BoidsTypeEnum {
        BOID,
        PREY,
        ENEMY,
        IGNORE
    };

private:
    enum BoidsTypeEnum m_boidsType;

public:
    static TypeId GetTypeId(void);

    BoidsType();

    BoidsTypeEnum GetBoidsType(void) const;
    void SetBoidsType(enum BoidsTypeEnum boidsType);
};

}  // namespace ns3

#endif
