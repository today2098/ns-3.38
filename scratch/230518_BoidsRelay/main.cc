#include "boids-mobility-model.h"

#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
using namespace ns3;

#include <filesystem>
#include <string>
#include <vector>

NS_LOG_COMPONENT_DEFINE("Main");

class NetSim {
    const double EPS = 1e-10;
    std::string OUTPUT_DIR = "output/boids-relay/";  // OUTPUT_DIR:=(ログファイルの出力ディレクトリ).
    std::string ANIM_DIR = "output/animation/";      // ANIM_DIR:=(netanim用出力ディレクトリ).

    bool m_tracing;        // m_tracing:=(トレーシングを有効にするか).
    bool m_logging;        // m_logging:=(ロギングを有効にするか).
    std::string m_prefix;  // m_prefix:=(ログファイルの接頭辞).

    int m_vn;                      // m_vn:=(ノード数).
    double m_dist;                 // m_dist:=(ノード間距離).
    double m_height;               // m_height:=(ノード群の高さ).
    bool m_disableBoids;           // m_disableBoids:=(ボイズモデルを無効にするか).
    bool m_enable3D;               // m_enable3D:=(3次元空間の移動を有効にするか).
    bool m_enableEnemy;            // m_enableEnemy:=(外敵を作成するか).
    NodeContainer m_nodes;         // m_nodes:=(ノードコンテナ).
    NetDeviceContainer m_devices;  // m_devices:=(デバイスコンテナ).
    Ipv4InterfaceContainer m_ifs;  // m_ifs:=(インターフェイスコンテナ).
    double m_simStop;              // m_simStop:=(シミュレーション終了時間).

    std::map<std::string, Ptr<OutputStreamWrapper>> m_streams;  // m_streams[]:=(出力ストリーム).

    void TracePosition(Time interval);
    void TraceDistance(Time interval, int u, int v);
    void TracePhyTxBegin(std::string context, Ptr<const Packet> packet, double txPowerW);
    void TracePhyRxBegin(std::string context, Ptr<const Packet> packet, RxPowerWattPerChannelBand rxPowerW);
    void TracePacket(std::string context, Ptr<const Packet> packet);

    void CreateNodes(void);
    void ConfigureL2(void);
    void ConfigureL3(void);
    void ConfigureL7(void);

public:
    NetSim(int argc, char *argv[]);
    ~NetSim();

    void Run(void);
};

NetSim::NetSim(int argc, char *argv[]) {
    NS_LOG_FUNCTION(this << argc);

    m_tracing = false;
    m_logging = false;
    m_prefix = "boids-relay";

    m_vn = 7;  // BS (2 nodes) and Boids (5 nodes).
    m_dist = 35.0;
    m_height = 30.0;
    m_disableBoids = false;
    m_enable3D = false;
    m_enableEnemy = false;
    m_simStop = 100.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("tracing", "Enable tracing, if true", m_tracing);
    cmd.AddValue("logging", "Enable logging, if true", m_logging);
    cmd.AddValue("prefix", "Prefix of log files", m_prefix);
    cmd.AddValue("disableBoids", "Disable Boids Model, if true", m_disableBoids);
    cmd.AddValue("3d", "Enable 3D, if true", m_enable3D);
    cmd.AddValue("enemy", "Enable enemy, if true", m_enableEnemy);
    cmd.Parse(argc, argv);
}

NetSim::~NetSim() {
    NS_LOG_FUNCTION(this);
}

void NetSim::TracePosition(Time interval) {
    for(auto itr = NodeList::Begin(); itr != NodeList::End(); ++itr) {
        auto node = *itr;
        auto position = node->GetObject<MobilityModel>()->GetPosition();

        std::ostringstream oss;
        oss << OUTPUT_DIR << m_prefix << "_position_" << node->GetId() << ".csv";
        if(m_streams.find(oss.str()) == m_streams.end()) {
            AsciiTraceHelper ascii;
            m_streams[oss.str()] = ascii.CreateFileStream(oss.str());
            *m_streams[oss.str()]->GetStream() << "time,x,y,z\n";
            *m_streams[oss.str()]->GetStream() << std::fixed << std::setprecision(4);
        }
        *m_streams[oss.str()]->GetStream() << Simulator::Now().GetSeconds() << "," << position.x << "," << position.y << "," << position.z << std::endl;
    }

    Simulator::Schedule(interval, &NetSim::TracePosition, this, interval);
}

void NetSim::TraceDistance(Time interval, int u, int v) {
    std::ostringstream oss;
    oss << OUTPUT_DIR << m_prefix << "_distance_" << u << "_" << v << ".csv";
    if(m_streams.find(oss.str()) == m_streams.end()) {
        AsciiTraceHelper ascii;
        m_streams[oss.str()] = ascii.CreateFileStream(oss.str());
        *m_streams[oss.str()]->GetStream() << "time,distance\n";
        *m_streams[oss.str()]->GetStream() << std::fixed << std::setprecision(4);
    }
    auto posi_u = m_nodes.Get(u)->GetObject<MobilityModel>()->GetPosition();
    auto posi_v = m_nodes.Get(v)->GetObject<MobilityModel>()->GetPosition();
    auto dist = CalculateDistance(posi_u, posi_v);
    *m_streams[oss.str()]->GetStream() << Simulator::Now().GetSeconds() << "," << dist << std::endl;

    Simulator::Schedule(interval, &NetSim::TraceDistance, this, interval, u, v);
}

void NetSim::TracePhyTxBegin(std::string context, Ptr<const Packet> packet, double txPowerW) {
    NS_LOG_FUNCTION(this << context << packet << txPowerW);

    std::ostringstream oss;
    oss << ">-----------------------------\n";
    packet->Print(oss);
    oss << "\n-----------------------------<";
    NS_LOG_UNCOND(oss.str());
}

void NetSim::TracePhyRxBegin(std::string context, Ptr<const Packet> packet, RxPowerWattPerChannelBand rxPowerW) {
    NS_LOG_FUNCTION(this << context << packet);
}

void NetSim::TracePacket(std::string context, Ptr<const Packet> packet) {
    NS_LOG_FUNCTION(this << context << packet);
}

void NetSim::CreateNodes(void) {
    NS_LOG_FUNCTION(this);

    // BS.
    NodeContainer bs(2);
    m_nodes.Add(bs);

    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "GridWidth", UintegerValue(5),
                                  "MinX", DoubleValue(-80.0),
                                  "MinY", DoubleValue(0.0),
                                  "DeltaX", DoubleValue(160.0),
                                  "Z", DoubleValue(0.0));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(bs);

    // Boids.
    NodeContainer boids(5);
    m_nodes.Add(boids);

    for(auto itr = m_nodes.Begin(); itr != m_nodes.End(); ++itr) {
        auto node = *itr;
        auto type = CreateObject<BoidsType>();
        type->SetBoidsType(BoidsType::BoidsTypeEnum::BOID);
        node->AggregateObject(type);
    }

    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "GridWidth", UintegerValue(5),
                                  "MinX", DoubleValue(-2 * m_dist),
                                  "MinY", DoubleValue(0.0),
                                  "DeltaX", DoubleValue(m_dist),
                                  "Z", DoubleValue(m_height));
    if(m_disableBoids) {
        mobility.SetMobilityModel("ns3::BoidsMobilityModel",
                                  "WeightS", DoubleValue(0.0),
                                  "WeightA", DoubleValue(0.0),
                                  "WeightC", DoubleValue(0.0),
                                  "WeightE", DoubleValue(0.2),
                                  "WeightCt", DoubleValue(0.2),
                                  "Enable3D", BooleanValue(m_enable3D),
                                  "MinZ", DoubleValue(m_height),
                                  "MaxZ", DoubleValue(m_height + 40.0),
                                  "MaxSpeed", DoubleValue(15.0),
                                  "Interval", TimeValue(Seconds(0.5)));
    } else {
        mobility.SetMobilityModel("ns3::BoidsMobilityModel",
                                  "ZoneS", DoubleValue(70.0),
                                  "ZoneA", DoubleValue(70.0),
                                  "ZoneC", DoubleValue(70.0),
                                  "WeightS", DoubleValue(0.1),
                                  "WeightA", DoubleValue(1.0),
                                  "WeightC", DoubleValue(0.3),
                                  "WeightE", DoubleValue(0.2),
                                  "WeightCt", DoubleValue(0.2),
                                  "Dist", DoubleValue(35.0),
                                  "Enable3D", BooleanValue(m_enable3D),
                                  "MinZ", DoubleValue(m_height),
                                  "MaxZ", DoubleValue(m_height + 40.0),
                                  "MaxSpeed", DoubleValue(15.0),
                                  "Interval", TimeValue(Seconds(0.5)));
    }
    mobility.Install(boids);

    for(auto itr = boids.Begin(); itr != boids.End(); ++itr) {
        auto node = *itr;
        auto mm = node->GetObject<BoidsMobilityModel>();
        auto position = mm->GetPosition();
        mm->SetAttribute("Center", VectorValue(Vector(position.x, position.y, position.z)));
    }

    // Enenmy.
    auto enemy = CreateObject<Node>();

    auto type = CreateObject<BoidsType>();
    type->SetBoidsType(BoidsType::BoidsTypeEnum::ENEMY);
    enemy->AggregateObject(type);

    mobility.SetMobilityModel("ns3::WaypointMobilityModel");
    mobility.Install(enemy);

    // Velocity of enemy is 10 m/s.
    auto model = enemy->GetObject<WaypointMobilityModel>();
    model->AddWaypoint(Waypoint(Seconds(0.0), Vector(-500.0, 30.0, m_height)));
    if(m_enableEnemy) model->AddWaypoint(Waypoint(Seconds(m_simStop), Vector(500.0, 30.0, m_height)));
}

void NetSim::ConfigureL2(void) {
    NS_LOG_FUNCTION(this);

    auto wifiChannel = YansWifiChannelHelper::Default();
    auto channel = wifiChannel.Create();

    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel(channel);

    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue("OfdmRate6Mbps"),
                                 "ControlMode", StringValue("OfdmRate6Mbps"));
    // wifi.SetRemoteStationManager("ns3::IdealWifiManager");
    m_devices = wifi.Install(wifiPhy, wifiMac, m_nodes);
}

void NetSim::ConfigureL3(void) {
    NS_LOG_FUNCTION(this);

    AodvHelper aodv;

    InternetStackHelper internet;
    internet.SetRoutingHelper(aodv);
    internet.Install(m_nodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    m_ifs = ipv4.Assign(m_devices);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();
}

void NetSim::ConfigureL7(void) {
    NS_LOG_FUNCTION(this);

    uint16_t port = 12345;

    // BulkSendHelper bulk("ns3::TcpSocketFactory", InetSocketAddress("10.1.1.2", port));
    // auto bulkApp = bulk.Install(m_nodes.Get(0));
    // bulkApp.Start(Seconds(20.0));
    // bulkApp.Stop(Seconds(80.0));

    // PacketSinkHelper sink("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
    // auto sinkApp = sink.Install(m_nodes.Get(1));
    // sinkApp.Start(Seconds(5.0));
    // sinkApp.Stop(Seconds(95.0));

    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress("10.1.1.2", port));
    // onoff.SetAttribute("DataRate", DataRateValue(DataRate("1Mbps")));
    auto onoffApp = onoff.Install(m_nodes.Get(0));
    onoffApp.Start(Seconds(20.0));
    onoffApp.Stop(Seconds(80.0));

    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
    auto sinkApp = sink.Install(m_nodes.Get(1));
    sinkApp.Start(Seconds(5.0));
    sinkApp.Stop(Seconds(95.0));
}

void NetSim::Run(void) {
    NS_LOG_FUNCTION(this);

    Time::SetResolution(Time::NS);
    Packet::EnablePrinting();
    Packet::EnableChecking();

    CreateNodes();
    ConfigureL2();
    ConfigureL3();
    ConfigureL7();

    Simulator::Stop(Seconds(m_simStop));

    FlowMonitorHelper flowmon;
    auto monitor = flowmon.InstallAll();

    if(m_tracing) {
        Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin", MakeCallback(&NetSim::TracePhyTxBegin, this));
        Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxEnd", MakeCallback(&NetSim::TracePacket, this));
        Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxBegin", MakeCallback(&NetSim::TracePhyRxBegin, this));
        Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxEnd", MakeCallback(&NetSim::TracePacket, this));
        Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxDrop", MakeCallback(&NetSim::TracePacket, this));
    }

    std::filesystem::create_directories(OUTPUT_DIR);
    Simulator::ScheduleNow(&NetSim::TracePosition, this, Seconds(1.0));
    Simulator::ScheduleNow(&NetSim::TraceDistance, this, Seconds(1.0), 0, 2);
    Simulator::ScheduleNow(&NetSim::TraceDistance, this, Seconds(1.0), 2, 3);
    Simulator::ScheduleNow(&NetSim::TraceDistance, this, Seconds(1.0), 3, 4);
    Simulator::ScheduleNow(&NetSim::TraceDistance, this, Seconds(1.0), 4, 5);
    Simulator::ScheduleNow(&NetSim::TraceDistance, this, Seconds(1.0), 5, 6);
    Simulator::ScheduleNow(&NetSim::TraceDistance, this, Seconds(1.0), 6, 1);

    if(m_logging) {
        YansWifiPhyHelper wifiPhy;
        // wifiPhy.EnablePcapAll(OUTPUT_DIR + m_prefix);
        wifiPhy.EnableAsciiAll(OUTPUT_DIR + m_prefix);
    }

    std::filesystem::create_directories(ANIM_DIR);
    AnimationInterface anim(ANIM_DIR + m_prefix + ".xml");

    Simulator::Run();

    std::ostringstream oss;
    oss << ">-----------------------------\n";
    monitor->CheckForLostPackets();
    auto classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    const auto &stats = monitor->GetFlowStats();
    for(const auto &[id, stat] : stats) {
        const auto &ft = classifier->FindFlow(id);
        if(ft.destinationPort != 12345) continue;
        oss << "Flow " << id << " (" << ft.sourceAddress << ":" << ft.sourcePort << " -> " << ft.destinationAddress << ":" << ft.destinationPort << ")\n";
        oss << "  Protocol:           " << (int)ft.protocol << "\n";
        oss << "  Tx Packets:         " << stat.txPackets << " packets\n";
        oss << "  Tx Bytes:           " << stat.txBytes << " bytes\n";
        oss << "  Rx Packets:         " << stat.rxPackets << " packets\n";
        oss << "  Rx Bytes:           " << stat.rxBytes << " bytes\n";
        if(stat.delaySum.GetSeconds() > EPS) {
            oss << "  Throughput:         " << (double)8 * stat.rxBytes / stat.delaySum.GetSeconds() / 1e6 << " Mbps\n";
        }
        if(stat.rxPackets > 0) {
            oss << "  Mean Delay:         " << Time(stat.delaySum / stat.rxPackets).As(Time::MS) << "\n";
            oss << "  Mean Jitter:        " << Time(stat.jitterSum / stat.rxPackets).As(Time::MS) << "\n";
        }
        oss << "  Packet Loss Ration: " << (double)stat.lostPackets / stat.txPackets * 100 << " %\n";
    }
    oss << "-----------------------------<";
    NS_LOG_UNCOND(oss.str());

    Simulator::Destroy();
}

int main(int argc, char *argv[]) {
    LogComponentEnable("Main", LOG_LEVEL_LOGIC);
    LogComponentEnableAll(LOG_PREFIX_ALL);

    NetSim netsim(argc, argv);
    netsim.Run();

    return 0;
}
