#include "boids-mobility-model.h"

#include "boids-type.h"

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("BoidsMobilityModel");

constexpr double EPS = 1e-10;
constexpr double DIST = 40.0;
constexpr double DIST_ENEMY = 50.0;

Vector operator*(Vector v, double a) {
    return Vector(v.x * a, v.y * a, v.z * a);
}

Vector operator*(double a, Vector v) {
    return v * a;
}

Vector operator/(Vector v, double a) {
    return Vector(v.x / a, v.y / a, v.z / a);
}

NS_OBJECT_ENSURE_REGISTERED(BoidsMobilityModel);

TypeId BoidsMobilityModel::GetTypeId() {
    static TypeId tid = TypeId("ns3::BoidsMobilityModel")
                            .SetParent<MobilityModel>()
                            .SetGroupName("Mobility")
                            .AddConstructor<BoidsMobilityModel>()
                            .AddAttribute("ZoneS",
                                          "Zone of 'Separation.' [m]",
                                          DoubleValue(100),
                                          MakeDoubleAccessor(&BoidsMobilityModel::m_zs),
                                          MakeDoubleChecker<double>(0))
                            .AddAttribute("ZoneA",
                                          "Zone of 'Alignment.' [m]",
                                          DoubleValue(100),
                                          MakeDoubleAccessor(&BoidsMobilityModel::m_za),
                                          MakeDoubleChecker<double>(0))
                            .AddAttribute("ZoneC",
                                          "Zone of 'Cohesion.' [m]",
                                          DoubleValue(100),
                                          MakeDoubleAccessor(&BoidsMobilityModel::m_zc),
                                          MakeDoubleChecker<double>(0))
                            .AddAttribute("ZoneE",
                                          "Zone of 'Enemy.' [m]",
                                          DoubleValue(100),
                                          MakeDoubleAccessor(&BoidsMobilityModel::m_ze),
                                          MakeDoubleChecker<double>(0))
                            .AddAttribute("WeightS",
                                          "Weight of 'Separation'",
                                          DoubleValue(0.1),
                                          MakeDoubleAccessor(&BoidsMobilityModel::m_ws),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("WeightA",
                                          "Weight of 'Alignment'",
                                          DoubleValue(0.2),
                                          MakeDoubleAccessor(&BoidsMobilityModel::m_wa),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("WeightC",
                                          "Weight of 'Cohesion'",
                                          DoubleValue(0.3),
                                          MakeDoubleAccessor(&BoidsMobilityModel::m_wc),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("WeightE",
                                          "Weight of 'Enemy'",
                                          DoubleValue(0.2),
                                          MakeDoubleAccessor(&BoidsMobilityModel::m_we),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("WeightCt",
                                          "Weight of 'Center'",
                                          DoubleValue(0.2),
                                          MakeDoubleAccessor(&BoidsMobilityModel::m_wct),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("Alpha",
                                          "Wight of boids rule. [0,1]",
                                          DoubleValue(0.5),
                                          MakeDoubleAccessor(&BoidsMobilityModel::m_alpha),
                                          MakeDoubleChecker<double>(0, 1))
                            .AddAttribute("Enable3D",
                                          "Enable 3D, if true",
                                          BooleanValue(false),
                                          MakeBooleanAccessor(&BoidsMobilityModel::m_enable3D),
                                          MakeBooleanChecker())
                            .AddAttribute("Center",
                                          "Center coordinate of boids",
                                          VectorValue(Vector(0.0, 0.0, 30.0)),
                                          MakeVectorAccessor(&BoidsMobilityModel::m_center),
                                          MakeVectorChecker())
                            .AddAttribute("MinZ",
                                          "",
                                          DoubleValue(-1),
                                          MakeDoubleAccessor(&BoidsMobilityModel::m_minZ),
                                          MakeDoubleChecker<double>(0))
                            .AddAttribute("MaxZ",
                                          "",
                                          DoubleValue(-1),
                                          MakeDoubleAccessor(&BoidsMobilityModel::m_maxZ),
                                          MakeDoubleChecker<double>(0))
                            .AddAttribute("MaxSpeed",
                                          "",
                                          DoubleValue(-1),
                                          MakeDoubleAccessor(&BoidsMobilityModel::m_maxSpeed),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("Interval",
                                          "Interval of update.",
                                          TimeValue(Seconds(0.5)),
                                          MakeTimeAccessor(&BoidsMobilityModel::m_interval),
                                          MakeTimeChecker())
                            .AddAttribute("Eps",
                                          "",
                                          StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.1]"),
                                          MakePointerAccessor(&BoidsMobilityModel::m_eps),
                                          MakePointerChecker<RandomVariableStream>());
    return tid;
}

BoidsMobilityModel::BoidsMobilityModel() {}

void BoidsMobilityModel::DoInitialize() {
    DoInitializePrivate();
    MobilityModel::DoInitialize();
}

void BoidsMobilityModel::DoDispose() {
    MobilityModel::DoDispose();
}

void BoidsMobilityModel::DoInitializePrivate() {
    m_helper.Update();
    auto speed = Vector(0, 0, 0);
    m_helper.SetVelocity(speed);
    m_helper.Unpause();
    NotifyCourseChange();

    Simulator::ScheduleNow(&BoidsMobilityModel::UpdateInfo, this);
    m_event = Simulator::Schedule(Seconds(0.1), &BoidsMobilityModel::Update, this);
}

void BoidsMobilityModel::UpdateInfo() {
    m_flag = true;

    Simulator::Schedule(m_interval, &BoidsMobilityModel::UpdateInfo, this);
}

void BoidsMobilityModel::Update() {
    m_helper.Update();
    auto position = m_helper.GetCurrentPosition();
    auto speed = m_helper.GetVelocity();
    auto new_speed = speed;
    if(m_flag) {
        new_speed = (1 - m_alpha) * speed + m_alpha * (m_ws * Separation() + m_wa * Alignment() + m_wc * Cohesion() + m_we * Enemy() + m_wct * Center());
        m_flag = false;
    }
    new_speed = new_speed + Vector(m_eps->GetValue() / m_interval.GetSeconds(), m_eps->GetValue() / m_interval.GetSeconds(), (m_enable3D ? m_eps->GetValue() / m_interval.GetSeconds() : 0));
    if(m_enable3D) {
        if(m_minZ != -1 and position.z < m_minZ and new_speed.z < 0.0) new_speed.z = 0.0;
        if(m_maxZ != -1 and position.z > m_maxZ and new_speed.z > 0.0) new_speed.z = 0.0;
    }
    if(m_maxSpeed >= 0.0 and new_speed.GetLength() > m_maxSpeed) {
        new_speed = new_speed / new_speed.GetLength() * m_maxSpeed;
    }
    m_helper.SetVelocity(new_speed);
    m_helper.Unpause();
    NotifyCourseChange();

    m_event.Cancel();
    m_event = Simulator::Schedule(Seconds(0.1), &BoidsMobilityModel::Update, this);
}

Vector BoidsMobilityModel::Separation() {
    Vector res(0.0, 0.0, 0.0);
    auto position = m_helper.GetCurrentPosition();
    int cnt_neighbors = 0;
    for(auto itr = NodeList::Begin(); itr != NodeList::End(); ++itr) {
        auto neighbor = *itr;
        auto type = neighbor->GetObject<BoidsType>();
        if(type and type->GetBoidsType() == BoidsType::BoidsTypeEnum::BOID) {
            auto neighbor_model = neighbor->GetObject<MobilityModel>();
            auto neighbor_position = neighbor_model->GetPosition();
            auto distance = CalculateDistance(position, neighbor_position);
            if(EPS < distance and distance <= m_zs) {
                res = res + (position - neighbor_position) / distance * DIST * (DIST / distance);
                cnt_neighbors++;
            }
        }
    }
    // res = res / cnt_neighbors;
    return res;
}

Vector BoidsMobilityModel::Alignment() {
    Vector res(0.0, 0.0, 0.0);
    auto position = m_helper.GetCurrentPosition();
    auto speed = m_helper.GetVelocity();
    int cnt_neighbors = 0;
    for(auto itr = NodeList::Begin(); itr != NodeList::End(); ++itr) {
        auto neighbor = *itr;
        auto type = neighbor->GetObject<BoidsType>();
        if(type and type->GetBoidsType() == BoidsType::BoidsTypeEnum::BOID) {
            auto neighbor_model = neighbor->GetObject<MobilityModel>();
            auto neighbor_position = neighbor_model->GetPosition();
            auto distance = CalculateDistance(position, neighbor_position);
            if(EPS < distance and distance <= m_za) {
                auto neighbor_speed = neighbor_model->GetVelocity();
                res = res + (neighbor_speed - speed);
                cnt_neighbors++;
            }
        }
    }
    res = res / cnt_neighbors;
    return res;
}

Vector BoidsMobilityModel::Cohesion() {
    Vector tmp(0.0, 0.0, 0.0);
    auto position = m_helper.GetCurrentPosition();
    int cnt_neighbors = 0;
    for(auto itr = NodeList::Begin(); itr != NodeList::End(); ++itr) {
        auto neighbor = *itr;
        auto type = neighbor->GetObject<BoidsType>();
        if(type and type->GetBoidsType() == BoidsType::BoidsTypeEnum::BOID) {
            auto neighbor_model = neighbor->GetObject<MobilityModel>();
            auto neighbor_position = neighbor_model->GetPosition();
            auto distance = CalculateDistance(position, neighbor_position);
            if(EPS < distance and distance <= m_zc) {
                tmp = tmp + neighbor_position;
                cnt_neighbors++;
            }
        }
    }
    tmp = tmp / cnt_neighbors;
    Vector res = tmp - position;
    return res;
}

Vector BoidsMobilityModel::Enemy() {
    Vector res(0.0, 0.0, 0.0);
    auto position = m_helper.GetCurrentPosition();
    for(auto itr = NodeList::Begin(); itr != NodeList::End(); ++itr) {
        auto neighbor = *itr;
        auto type = neighbor->GetObject<BoidsType>();
        if(type and type->GetBoidsType() == BoidsType::BoidsTypeEnum::ENEMY) {
            auto neighbor_model = neighbor->GetObject<MobilityModel>();
            auto neighbor_position = neighbor_model->GetPosition();
            auto distance = CalculateDistance(position, neighbor_position);
            if(EPS < distance and distance <= m_ze) {
                res = res + (position - neighbor_position) / distance * DIST_ENEMY * (DIST_ENEMY / distance);
            }
        }
    }
    return res;
}

Vector BoidsMobilityModel::Center() {
    auto position = m_helper.GetCurrentPosition();
    auto res = m_center - position;
    return res;
}

Vector BoidsMobilityModel::DoGetPosition() const {
    m_helper.Update();
    return m_helper.GetCurrentPosition();
}

void BoidsMobilityModel::DoSetPosition(const Vector &position) {
    m_helper.SetPosition(position);
    NotifyCourseChange();
}

Vector BoidsMobilityModel::DoGetVelocity() const {
    return m_helper.GetVelocity();
}

int64_t BoidsMobilityModel::DoAssignStreams(int64_t stream) {
    m_eps->SetStream(stream);
    return 1;
}

}  // namespace ns3
