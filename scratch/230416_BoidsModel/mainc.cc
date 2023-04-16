#include "boids-mobility-model.h"

#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
using namespace ns3;

#include <filesystem>

NS_LOG_COMPONENT_DEFINE("Main");

class NetSim {
    std::string OUTPUT_DIR = "output/boids-model/";  // OUTPUT_DIR:=(ログファイルの出力ディレクトリ).
    std::string ANIM_DIR = "output/animation/";      // ANIM_DIR:=(netanim用出力ディレクトリ).

    bool m_tracing;        // m_tracing:=(トレーシングを有効にする).
    bool m_verbose;        // m_verbose:=(詳細なロギングを有効にする).
    std::string m_prefix;  // m_prefix:=(ログファイルの接頭辞).

    int m_vn;                            // m_vn:=(ノード数).
    double m_height;                     // m_height:=(ノード群の高さ).
    bool m_enable3D;                     // m_enable3D:=(3次元空間の移動を有効にする).
    bool m_enableEnemy;                  // m_enableEnemy:=(外敵を作成する).
    NodeContainer m_nodes;               // m_nodes:=(ノードコンテナ).
    NetDeviceContainer m_devices;        // m_devices:=(デバイスコンテナ).
    Ipv4InterfaceContainer m_ifs;        // m_ifs:=(インターフェイスコンテナ).
    std::vector<Ptr<Socket>> m_sockets;  // m_sockets[v]:=(ノードvのソケット).
    double m_simStop;                    // m_simStop:=(シミュレーション終了時間).
    double m_appStart;                   // m_appStart:=(アプリ開始時間).

    std::map<std::string, Ptr<OutputStreamWrapper>> m_streams;  // m_streams[]:=(出力ストリーム).
    std::vector<long long> m_totalRx;                           // m_totalRx[v]:=(ノードvが受信したデータサイズ).

    void TracePosition(Time interval);

    void SendPacket(Ptr<Socket> socket, Ipv4Address dest, uint16_t port, int dataSize, double interval, int cnt);
    void HandleRead(Ptr<Socket> socket);

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
    m_verbose = false;
    m_prefix = "boids-model";

    m_vn = 30;
    m_height = 30.0;
    m_enable3D = false;
    m_enableEnemy = false;
    m_simStop = 100.0;
    m_appStart = 20.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("tracing", "Enable tracing, if true", m_tracing);
    cmd.AddValue("verbose", "Enable verbose logging, if true", m_verbose);
    cmd.AddValue("prefix", "Prefix of log files", m_prefix);
    cmd.AddValue("3d", "Enable 3D, if true", m_enable3D);
    cmd.AddValue("enemy", "Enable enemy, if true", m_enableEnemy);
    cmd.Parse(argc, argv);

    m_sockets.resize(m_vn);
    m_totalRx.assign(m_vn, 0);
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

void NetSim::SendPacket(Ptr<Socket> socket, Ipv4Address dest, uint16_t port, int dataSize, double interval, int cnt) {
    if(cnt == 0) return;

    auto node = socket->GetNode();
    auto packet = Create<Packet>(dataSize);
    socket->SendTo(packet, 0, InetSocketAddress(dest, port));

    std::ostringstream oss;
    oss << "node" << node->GetId() << " sends packet(uid: " << packet->GetUid() << ", size: " << packet->GetSize() << " bytes) "
        << "from " << dest << ":" << port;
    NS_LOG_INFO(oss.str());

    Simulator::Schedule(Seconds(interval), &NetSim::SendPacket, this, socket, dest, port, dataSize, interval, cnt - 1);
}

void NetSim::HandleRead(Ptr<Socket> socket) {
    auto node = socket->GetNode();
    Ptr<Packet> packet;
    Address from;
    while(packet = socket->RecvFrom(from)) {
        if(packet->GetSize() > 0) {
            m_totalRx[node->GetId()] += packet->GetSize();
        }

        std::ostringstream oss;
        auto addr = InetSocketAddress::ConvertFrom(from);
        oss << "node" << node->GetId() << " receives packet(uid: " << packet->GetUid() << ", size: " << packet->GetSize() << " bytes) "
            << "from " << addr.GetIpv4() << ":" << addr.GetPort();
        NS_LOG_INFO(oss.str());
    }
}

void NetSim::CreateNodes(void) {
    NS_LOG_FUNCTION(this);

    // Tower.
    auto tower = CreateObject<Node>();
    m_nodes.Add(tower);

    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX", DoubleValue(0),
                                  "MinY", DoubleValue(0),
                                  "Z", DoubleValue(m_height - 10.0));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(tower);

    // Boids.
    NodeContainer boids(m_vn - 1);
    m_nodes.Add(boids);

    for(auto itr = boids.Begin(); itr != boids.End(); ++itr) {
        auto node = *itr;
        auto type = CreateObject<BoidsType>();
        type->SetBoidsType(BoidsType::BoidsTypeEnum::BOID);
        node->AggregateObject(type);
    }

    mobility.SetPositionAllocator("ns3::UniformDiscPositionAllocator",
                                  "rho", DoubleValue(100.0),
                                  "Z", DoubleValue(m_height));
    if(m_enable3D) {
        mobility.SetMobilityModel("ns3::BoidsMobilityModel",
                                  "Enable3D", BooleanValue(true),
                                  "Center", VectorValue(Vector(0.0, 0.0, m_height - 10.0)),
                                  "MinZ", DoubleValue(m_height),
                                  "MaxZ", DoubleValue(m_height + 10.0),
                                  "MaxSpeed", DoubleValue(15.0),
                                  "Interval", TimeValue(Seconds(1.0)));
    } else {
        mobility.SetMobilityModel("ns3::BoidsMobilityModel",
                                  "MaxSpeed", DoubleValue(15.0),
                                  "Interval", TimeValue(Seconds(1.0)));
    }
    mobility.Install(boids);

    // Enenmy.
    auto enemy = CreateObject<Node>();

    auto type = CreateObject<BoidsType>();
    type->SetBoidsType(BoidsType::BoidsTypeEnum::ENEMY);
    enemy->AggregateObject(type);

    mobility.SetMobilityModel("ns3::WaypointMobilityModel");
    mobility.Install(enemy);

    auto model = enemy->GetObject<WaypointMobilityModel>();

    if(m_enableEnemy) {
        model->AddWaypoint(Waypoint(Seconds(0.0), Vector(-200.0, 20.0, m_height)));
        model->AddWaypoint(Waypoint(Seconds(70.0), Vector(500.0, 20.0, m_height)));
    } else {
        model->AddWaypoint(Waypoint(Seconds(0.0), Vector(-500.0, 20.0, m_height)));
    }
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
    aodv.Set("EnableHello", BooleanValue(false));
    // aodv.Set("EnableBroadcast", BooleanValue(false));

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

    for(int i = 0; i < m_vn; ++i) {
        auto node = m_nodes.Get(i);
        auto sockFactory = node->GetObject<UdpSocketFactory>();

        auto &sock = m_sockets[i];
        sock = sockFactory->CreateSocket();
        sock->Bind(InetSocketAddress(Ipv4Address::GetAny(), port));
        sock->SetAllowBroadcast(true);
        sock->SetRecvCallback(MakeCallback(&NetSim::HandleRead, this));
    }
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

    int dataSize = 100;
    double interval = 1.0;
    int cnt = 10;
    Simulator::Schedule(Seconds(m_appStart), &NetSim::SendPacket, this, m_sockets[0], "10.1.1.255", 12345, dataSize, interval, cnt);

    if(m_verbose) {
        std::filesystem::create_directories(OUTPUT_DIR);

        YansWifiPhyHelper wifiPhy;
        // wifiPhy.EnablePcapAll(OUTPUT_DIR + m_prefix);
        wifiPhy.EnableAsciiAll(OUTPUT_DIR + m_prefix);

        AsciiTraceHelper ascii;
        auto stream = ascii.CreateFileStream(OUTPUT_DIR + m_prefix + "_routing-table.tr");
        AodvHelper aodv;
        aodv.PrintRoutingTableAllEvery(Seconds(1.0), stream);

        Simulator::ScheduleNow(&NetSim::TracePosition, this, Seconds(1.0));
    }

    std::filesystem::create_directories(ANIM_DIR);
    AnimationInterface anim(ANIM_DIR + m_prefix + ".xml");
    anim.SetStartTime(Seconds(0.0));
    anim.SetStopTime(Seconds(m_simStop));

    Simulator::Run();

    // Result.
    std::ostringstream oss;
    oss << "------------------------------\n";
    double ration = 0.0;
    for(int i = 0; i < m_vn; ++i) {
        oss << "[node " << i << "] totalRx: " << m_totalRx[i] << " bytes\n";
        ration += (double)m_totalRx[i] / cnt / dataSize;
    }
    ration /= m_vn;
    oss << "ration: " << ration << "\n";
    oss << "------------------------------";
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
