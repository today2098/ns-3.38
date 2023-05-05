#ifndef BOIDS_MOBILITY_MODEL_H
#define BOIDS_MOBILITY_MODEL_H

#include "boids-type.h"

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"

namespace ns3 {

class BoidsMobilityModel : public MobilityModel {
    double m_zs;                      // m_sz:=(Zone of "Separation").
    double m_za;                      // m_za:=(Zone of "Alignment").
    double m_zc;                      // m_zc:=(Zone of "Cohesion").
    double m_ze;                      // m_ze:=(Zone of "Enemy").
    double m_ws;                      // m_ws:=(Weight of "Separation").
    double m_wa;                      // m_wa:=(Weight of "Alignment").
    double m_wc;                      // m_wc:=(Weight of "Cohesion")
    double m_we;                      // m_we:=(Weight of "Enemy").
    double m_wct;                     // m_wct:(Weight of "Center").
    double m_alpha;                   // m_alpha:=(Wight of boids rule). [0,1]
    double m_dist;                    // m_dist:=(Boids間距離).
    double m_dist_enemy;              // m_dist_enemy:=(外敵間距離).
    Vector m_center;                  // m_center:=(中心座標).
    bool m_enable3D;                  // m_enable3D:=(移動方向を3次元として扱うか).
    double m_minZ;                    // m_minZ:=(最低高度). m_enable3D=trueのとき有効．
    double m_maxZ;                    // m_maxZ:=(最高高度). m_enable3D=trueのとき有効．
    double m_maxSpeed;                // m_maxSpeed:=(最高速度). If the value is -1, the speed is unlimited.
    ConstantVelocityHelper m_helper;  // m_helper:=(Helper for this object).
    bool m_flag;                      // m_flag:=(更新にBoids Modelを用いるか).
    Time m_interval;                  // m_interval:=(更新間隔).
    EventId m_event;                  // m_event:=(次の更新イベント).
    Ptr<RandomVariableStream> m_eps;

    void DoInitialize() override;
    void DoDispose() override;

    void DoInitializePrivate();
    void UpdateInfo();
    void Update();

    Vector Separation();
    Vector Alignment();
    Vector Cohesion();
    Vector Enemy();
    Vector Center();

    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;
    int64_t DoAssignStreams(int64_t) override;

public:
    static TypeId GetTypeId();

    BoidsMobilityModel();
};

}  // namespace ns3

#endif
