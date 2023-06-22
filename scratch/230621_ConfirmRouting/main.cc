#include "ns3/aodv-module.h"
#include "ns3/core-module.h"
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

//
//   o ----> o ----> o
//   n0      n1      n2
//
// This sinario is for operarion confirmation of routing.
// Node0 sends UDP packets to node2.
//

class NetSim {
    std::string OUTPUT_DIR = "output/confirm_routing/";  // OUTPUT_DIR:=(ログファイルの出力ディレクトリ).
    std::string ANIM_DIR = "output/animation/";          // ANIM_DIR:=(netanim用出力ディレクトリ).

    bool m_tracing;        // m_tracing:=(トレーシングを有効にするか).
    bool m_logging;        // m_logging:=(ロギングを有効にするか).
    std::string m_prefix;  // m_prefix:=(ログファイルの接頭辞).

    int m_vn;                            // m_vn:=(ノード数).
    std::string m_routing;               // m_routing:=(ルーティングプロトコル)．
    NodeContainer m_nodes;               // m_nodes:=(ノードコンテナ).
    NetDeviceContainer m_devices;        // m_devices:=(デバイスコンテナ).
    Ipv4InterfaceContainer m_ifs;        // m_ifs:=(インターフェイスコンテナ).
    std::vector<Ptr<Socket>> m_sockets;  // m_sockets[v]:=(ノードvのソケット).
    double m_simStop;                    // m_simStop:=(シミュレーション終了時間).
    double m_appStart;                   // m_appStart:=(アプリ開始時間).

    void TracePhyTxBegin(std::string context, Ptr<const Packet> packet, double txPowerW);
    void TracePhyRxBegin(std::string context, Ptr<const Packet> packet, RxPowerWattPerChannelBand rxPowerW);
    void TracePacket(std::string context, Ptr<const Packet> packet);

    void SendPacket(Ptr<Socket> socket, Ipv4Address dest, uint16_t port, int dataSize);
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
    m_logging = false;
    m_prefix = "confirm_routing";

    m_vn = 3;
    m_routing = "aodv";
    m_simStop = 60.0;
    m_appStart = 10.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("tracing", "Enable tracing, if true", m_tracing);
    cmd.AddValue("logging", "Enable logging, if true", m_logging);
    cmd.AddValue("prefix", "prefix of log files", m_prefix);
    cmd.AddValue("routing", "[aodv/static]", m_routing);
    cmd.Parse(argc, argv);
    NS_ASSERT(m_routing == "aodv" or m_routing == "static");

    m_sockets.resize(m_vn);
}

NetSim::~NetSim() {
    NS_LOG_FUNCTION(this);
}

void NetSim::TracePhyTxBegin(std::string context, Ptr<const Packet> packet, double txPowerW) {
    NS_LOG_FUNCTION(this << context << packet << txPowerW);

    std::clog << "------------------------------\n";
    packet->Print(std::clog);
    std::clog << "\n------------------------------" << std::endl;
}

void NetSim::TracePhyRxBegin(std::string context, Ptr<const Packet> packet, RxPowerWattPerChannelBand rxPowerW) {
    NS_LOG_FUNCTION(this << context << packet);
}

void NetSim::TracePacket(std::string context, Ptr<const Packet> packet) {
    NS_LOG_FUNCTION(this << context << packet);
}

void NetSim::SendPacket(Ptr<Socket> socket, Ipv4Address dest, uint16_t port, int dataSize) {
    auto node = socket->GetNode();
    auto packet = Create<Packet>(dataSize);
    socket->SendTo(packet, 0, InetSocketAddress(dest, port));

    std::ostringstream oss;
    oss << "node" << node->GetId() << " sends packet (uid: " << packet->GetUid() << ", size: " << packet->GetSize() << " bytes) "
        << "to " << dest << ":" << port;
    NS_LOG_INFO(oss.str());
}

void NetSim::HandleRead(Ptr<Socket> socket) {
    auto node = socket->GetNode();
    Ptr<Packet> packet;
    Address from;
    while(packet = socket->RecvFrom(from)) {
        if(packet->GetSize() > 0) {
        }

        std::ostringstream oss;
        auto addr = InetSocketAddress::ConvertFrom(from);
        oss << "node" << node->GetId() << " receives packet (uid: " << packet->GetUid() << ", size: " << packet->GetSize() << " bytes) "
            << "from " << addr.GetIpv4() << ":" << addr.GetPort();
        NS_LOG_INFO(oss.str());
    }
}

void NetSim::CreateNodes(void) {
    NS_LOG_FUNCTION(this);

    m_nodes.Create(m_vn);

    double distance = 40.0;
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "GridWidth", UintegerValue(m_vn),
                                  "MinX", DoubleValue(0),
                                  "MinY", DoubleValue(0),
                                  "DeltaX", DoubleValue(distance));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(m_nodes);
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

    Ipv4StaticRoutingHelper staticRouting;

    InternetStackHelper internet;
    if(m_routing == "aodv") internet.SetRoutingHelper(aodv);
    else internet.SetRoutingHelper(staticRouting);
    internet.Install(m_nodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    m_ifs = ipv4.Assign(m_devices);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    if(m_routing == "static") {
        for(int i = 0; i < m_vn; ++i) {
            auto node = m_nodes.Get(i);
            auto routing = staticRouting.GetStaticRouting(node->GetObject<Ipv4>());
            for(int j = 0; j < i; ++j) routing->AddHostRouteTo(m_ifs.GetAddress(j), m_ifs.GetAddress(i - 1), 1);
            for(int j = i + 1; j < m_vn; ++j) routing->AddHostRouteTo(m_ifs.GetAddress(j), m_ifs.GetAddress(i + 1), 1);
        }
    }
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
    Simulator::Schedule(Seconds(m_appStart), &NetSim::SendPacket, this, m_sockets[0], "10.1.1.3", 12345, 100);

    if(m_tracing) {
        Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin", MakeCallback(&NetSim::TracePhyTxBegin, this));
        Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxBegin", MakeCallback(&NetSim::TracePhyRxBegin, this));
        Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxEnd", MakeCallback(&NetSim::TracePacket, this));
        Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxEnd", MakeCallback(&NetSim::TracePacket, this));
    }

    std::filesystem::create_directories(ANIM_DIR);
    AnimationInterface anim(ANIM_DIR + m_prefix + ".xml");

    if(m_logging) {
        std::filesystem::create_directories(OUTPUT_DIR);

        YansWifiPhyHelper wifiPhy;
        // wifiPhy.EnablePcapAll(OUTPUT_DIR + "pcap");
        wifiPhy.EnableAsciiAll(OUTPUT_DIR + "pcap");

        AsciiTraceHelper ascii;
        auto stream = ascii.CreateFileStream(OUTPUT_DIR + "rtable.tr");
        Ipv4StaticRoutingHelper staticRouting;
        staticRouting.PrintRoutingTableAllEvery(Seconds(1.0), stream);
    }

    Simulator::Run();
    Simulator::Destroy();
}

int main(int argc, char *argv[]) {
    LogComponentEnable("Main", LOG_LEVEL_LOGIC);
    LogComponentEnable("AodvRoutingProtocol", LOG_LEVEL_LOGIC);
    LogComponentEnable("Ipv4StaticRouting", LOG_LEVEL_LOGIC);
    LogComponentEnableAll(LOG_PREFIX_ALL);

    NetSim netsim(argc, argv);
    netsim.Run();

    return 0;
}
