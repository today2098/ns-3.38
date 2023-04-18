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
// "DCF"
//
//   n0      n1
//   o ----- o
//    \     /
//     \   /
//      \ /
//       o
//       n2
//

class NetSim {
    std::string OUTPUT_DIR = "output/packet-collision/";  // OUTPUT_DIR:=(ログファイルの出力ディレクトリ).
    std::string ANIM_DIR = "output/animation/";           // ANIM_DIR:=(netanim用出力ディレクトリ).

    bool m_tracing;        // m_tracing:=(トレーシングを有効にする).
    bool m_verbose;        // m_verbose:=(詳細なロギングを有効にする).
    std::string m_prefix;  // m_prefix:=(ログファイルの接頭辞).

    int m_vn;                            // m_vn:=(ノード数).
    double m_dist;                       // m_r:=(ノード間距離).
    NodeContainer m_nodes;               // m_nodes:=(ノードコンテナ).
    NetDeviceContainer m_devices;        // m_devices:=(デバイスコンテナ).
    Ipv4InterfaceContainer m_ifs;        // m_ifs:=(インターフェイスコンテナ).
    std::vector<Ptr<Socket>> m_sockets;  // m_sockets[v]:=(ノードvのソケット).
    double m_simStop;                    // m_simStop:=(シミュレーション終了時間).
    double m_appStart;                   // m_appStart:=(アプリ開始時間).
    int m_cnt;                           // m_cnt:=(送信回数).
    bool m_sim;                          // m_sim:=(同時に送信する).
    bool m_normal;                       // m_normal:=(ノード1のみが送信する場合).
    std::vector<long long> m_totalRx;    // m_totalRx[v]:=(ノードvが受信したデータサイズ).

    void TracePhyTxBegin(std::string context, Ptr<const Packet> packet, double txPowerW);
    void TracePhyRxBegin(std::string context, Ptr<const Packet> packet, RxPowerWattPerChannelBand rxPowerW);
    void TracePacket(std::string context, Ptr<const Packet> packet);
    void TraceBackoff(std::string context, uint32_t newValue, uint8_t);

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
    m_prefix = "packet-collision";

    m_vn = 3;
    m_dist = 40.0;
    m_simStop = 60.0;
    m_appStart = 10.0;
    m_cnt = 1;
    m_sim = false;
    m_normal = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("tracing", "Enable tracing, if true", m_tracing);
    cmd.AddValue("verbose", "Enable verbose logging, if true", m_verbose);
    cmd.AddValue("prefix", "prefix of log files", m_prefix);
    cmd.AddValue("cnt", "Number to send packet", m_cnt);
    cmd.AddValue("sim", "Node0 and node1 send packet at the same time", m_sim);
    cmd.AddValue("normal", "Only node1 sends packet (node0 sends no packet)", m_normal);
    cmd.Parse(argc, argv);

    m_sockets.resize(m_vn);
    m_totalRx.assign(m_vn, 0);
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

void NetSim::TraceBackoff(std::string context, uint32_t newValue, uint8_t) {
    NS_LOG_FUNCTION(this << context << newValue);
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

    m_nodes.Create(m_vn);

    auto posiList = CreateObject<ListPositionAllocator>();
    posiList->Add(Vector(0, 0, 0));
    posiList->Add(Vector(m_dist, 0, 0));
    posiList->Add(Vector(m_dist / 2, m_dist / 2 * std::sqrt(3.0), 0));

    MobilityHelper mobility;
    mobility.SetPositionAllocator(posiList);
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
    auto ssid = Ssid("my-ssid");
    wifiMac.SetType("ns3::AdhocWifiMac",
                    "Ssid", SsidValue(ssid));

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a);
    // wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
    //                              "DataMode", StringValue("OfdmRate6Mbps"),
    //                              "ControlMode", StringValue("OfdmRate6Mbps"));
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");
    m_devices = wifi.Install(wifiPhy, wifiMac, m_nodes);
}

void NetSim::ConfigureL3(void) {
    NS_LOG_FUNCTION(this);

    AodvHelper aodv;
    aodv.Set("EnableHello", BooleanValue(false));
    aodv.Set("EnableBroadcast", BooleanValue(false));

    InternetStackHelper internet;
    // internet.SetRoutingHelper(aodv);
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

    int dataSize = 2000;
    double interval = 0.001;
    int cnt = m_cnt;
    if(!m_normal) Simulator::Schedule(Seconds(m_appStart), &NetSim::SendPacket, this, m_sockets[0], "10.1.1.255", 12345, dataSize, interval, cnt);
    Simulator::Schedule(Seconds(m_appStart + (m_sim ? 0.0 : 0.00001)), &NetSim::SendPacket, this, m_sockets[1], "10.1.1.255", 12345, dataSize, interval, cnt);

    Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin", MakeCallback(&NetSim::TracePhyTxBegin, this));
    // Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/Txop/CwTrace", MakeCallback(&NetSim::TraceBackoff, this));
    // Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/Txop/BackoffTrace", MakeCallback(&NetSim::TraceBackoff, this));
    if(m_tracing) {
        Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxBegin", MakeCallback(&NetSim::TracePhyRxBegin, this));
        Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxEnd", MakeCallback(&NetSim::TracePacket, this));
        Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxEnd", MakeCallback(&NetSim::TracePacket, this));
    }

    if(m_verbose) {
        std::filesystem::create_directories(OUTPUT_DIR);

        YansWifiPhyHelper wifiPhy;
        wifiPhy.EnablePcapAll(OUTPUT_DIR + m_prefix);

        AsciiTraceHelper ascii;
        auto stream = ascii.CreateFileStream(OUTPUT_DIR + m_prefix + "_routing-table.tr");
        AodvHelper aodv;
        aodv.PrintRoutingTableAllEvery(Seconds(1.0), stream);
    }

    std::filesystem::create_directories(ANIM_DIR);
    AnimationInterface anim(ANIM_DIR + m_prefix + ".xml");
    anim.SetStartTime(Seconds(0.0));
    anim.SetStopTime(Seconds(30.0));

    Simulator::Run();

    // Result.
    std::ostringstream oss;
    oss << "------------------------------\n";
    for(int i = 0; i < m_vn; ++i) oss << "[node " << i << "] totalRx: " << m_totalRx[i] << " bytes\n";
    oss << "------------------------------";
    NS_LOG_UNCOND(oss.str());

    Simulator::Destroy();
}

int main(int argc, char *argv[]) {
    LogComponentEnable("Main", LOG_LEVEL_LOGIC);
    // LogComponentEnable("WifiPhy", LOG_LEVEL_LOGIC);
    // LogComponentEnable("FrameExchangeManager", LOG_LEVEL_LOGIC);
    // LogComponentEnable("ChannelAccessManager", LOG_LEVEL_LOGIC);
    // LogComponentEnable("Txop", LOG_LEVEL_LOGIC);
    // LogComponentEnable("WifiMac", LOG_LEVEL_LOGIC);
    // LogComponentEnable("AdhocWifiMac", LOG_LEVEL_LOGIC);
    LogComponentEnableAll(LOG_PREFIX_ALL);

    // RngSeedManager::SetSeed(1);

    NetSim netsim(argc, argv);
    netsim.Run();

    return 0;
}
