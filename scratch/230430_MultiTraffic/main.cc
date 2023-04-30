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
    std::string OUTPUT_DIR = "output/multi-traffic/";  // OUTPUT_DIR:=(ログファイルの出力ディレクトリ).
    std::string ANIM_DIR = "output/animation/";        // ANIM_DIR:=(netanim用出力ディレクトリ).

    bool m_tracing;        // m_tracing:=(トレーシングを有効にするか).
    bool m_logging;        // m_logging:=(ロギングを有効にするか).
    std::string m_prefix;  // m_prefix:=(ログファイルの接頭辞).

    int m_vn;                           // m_vn:=(ノード数).
    double m_dist;                      // m_dist:=(ノード間距離).
    NodeContainer m_nodes;              // m_nodes:=(ノードコンテナ).
    NetDeviceContainer m_devices;       // m_devices:=(デバイスコンテナ).
    Ipv4InterfaceContainer m_ifs;       // m_ifs:=(インターフェイスコンテナ).
    std::vector<Ptr<Socket>> m_rxSock;  // m_rxSock[v]:=(ノードvの受信用ソケット).
    std::vector<Ptr<Socket>> m_txSock;  // m_txSock[v]:=(ノードvの送信用ソケット).
    double m_simStop;                   // m_simStop:=(シミュレーション終了時間).
    double m_appStart;                  // m_appStart:=(アプリ開始時間).

    std::vector<long long> m_totalTx;  // m_totalTx[v]:=(ノードvが送信されるデータサイズ).
    std::vector<long long> m_totalRx;  // m_totalRx[v]:=(ノードvが受信したデータサイズ).

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
    m_prefix = "multi-traffic";

    m_vn = 25;
    m_dist = 35.0;
    m_simStop = 100.0;
    m_appStart = 50.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("tracing", "Enable tracing, if true", m_tracing);
    cmd.AddValue("logging", "Enable logging, if true", m_logging);
    cmd.AddValue("prefix", "Prefix of log files", m_prefix);
    cmd.Parse(argc, argv);

    m_rxSock.resize(m_vn);
    m_txSock.resize(m_vn);
    m_totalTx.assign(m_vn, 0);
    m_totalRx.assign(m_vn, 0);
}

NetSim::~NetSim() {
    NS_LOG_FUNCTION(this);
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

void NetSim::SendPacket(Ptr<Socket> socket, Ipv4Address dest, uint16_t port, int dataSize) {
    auto node = socket->GetNode();
    auto packet = Create<Packet>(dataSize);
    socket->SendTo(packet, 0, InetSocketAddress(dest, port));

    std::ostringstream oss;
    oss << "node" << node->GetId() << " sends packet (uid: " << packet->GetUid() << ", size: " << packet->GetSize() << " bytes) "
        << "from " << dest << ":" << port;
    NS_LOG_INFO(oss.str());
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
        oss << "node" << node->GetId() << " receives packet (uid: " << packet->GetUid() << ", size: " << packet->GetSize() << " bytes) "
            << "from " << addr.GetIpv4() << ":" << addr.GetPort();
        NS_LOG_INFO(oss.str());
    }
}

void NetSim::CreateNodes(void) {
    NS_LOG_FUNCTION(this);

    m_nodes.Create(m_vn);

    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "GridWidth", UintegerValue(5),
                                  "MinX", DoubleValue(-2 * m_dist),
                                  "MinY", DoubleValue(-2 * m_dist),
                                  "DeltaX", DoubleValue(m_dist),
                                  "DeltaY", DoubleValue(m_dist));
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

        m_rxSock[i] = sockFactory->CreateSocket();
        m_rxSock[i]->Bind(InetSocketAddress(Ipv4Address::GetAny(), port));
        m_rxSock[i]->SetRecvCallback(MakeCallback(&NetSim::HandleRead, this));
        m_rxSock[i]->ShutdownSend();

        m_txSock[i] = sockFactory->CreateSocket();
        m_txSock[i]->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
        m_txSock[i]->SetAllowBroadcast(true);
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

    int dataSize = 3000;
    double interval = 0.0;
    int cnt = 30;
    auto rng_node = CreateObject<UniformRandomVariable>();
    auto rng_interval = CreateObject<UniformRandomVariable>();
    for(int i = 0; i < cnt; ++i) {
        int src = rng_node->GetInteger(0, m_vn - 1);
        int dest = src;
        while(src == dest) dest = rng_node->GetInteger(0, m_vn - 1);
        auto time = rng_interval->GetValue(m_appStart, m_appStart + 10.0);
        Simulator::Schedule(Seconds(time), &NetSim::SendPacket, this, m_txSock[src], m_ifs.GetAddress(dest), 12345, dataSize);
        m_totalTx[dest] += dataSize;
    }

    FlowMonitorHelper flowmon;
    auto monitor = flowmon.InstallAll();

    if(m_tracing) {
        Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin", MakeCallback(&NetSim::TracePhyTxBegin, this));
        Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxEnd", MakeCallback(&NetSim::TracePacket, this));
        Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxBegin", MakeCallback(&NetSim::TracePhyRxBegin, this));
        Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxEnd", MakeCallback(&NetSim::TracePacket, this));
        Config::Connect("NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxDrop", MakeCallback(&NetSim::TracePacket, this));
    }

    if(m_logging) {
        std::filesystem::create_directories(OUTPUT_DIR);

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

    oss.str("");
    oss.clear(std::ostringstream::goodbit);
    oss << ">-----------------------------\n";
    double ration = 0.0;
    for(int i = 0; i < m_vn; ++i) {
        oss << "[node " << i << "] totalRx: " << m_totalRx[i] << " bytes";
        if(m_totalTx[i] > 0) oss << " (" << (double)m_totalRx[i] / m_totalTx[i] << ")";
        oss << "\n";
        ration += m_totalRx[i];
    }
    ration /= dataSize * cnt;
    oss << "ration: " << ration << "\n";
    oss << "-----------------------------<";
    NS_LOG_UNCOND(oss.str());

    Simulator::Destroy();
}

int main(int argc, char *argv[]) {
    LogComponentEnable("Main", LOG_LEVEL_LOGIC);
    // LogComponentEnable("WifiPhy", LOG_LEVEL_LOGIC);
    // LogComponentEnable("FrameExchangeManager", LOG_LEVEL_LOGIC);
    // LogComponentEnable("ChannelAccessManager", LOG_LEVEL_LOGIC);
    // LogComponentEnable("Txop", LOG_LEVEL_LOGIC);
    // LogComponentEnable("QosTxop", LOG_LEVEL_LOGIC);
    // LogComponentEnable("WifiMac", LOG_LEVEL_LOGIC);
    // LogComponentEnable("AdhocWifiMac", LOG_LEVEL_LOGIC);
    // LogComponentEnable("NetDevice", LOG_LEVEL_LOGIC);
    // LogComponentEnable("WifiNetDevice", LOG_LEVEL_LOGIC);
    // LogComponentEnable("Ipv4Interface", LOG_LEVEL_LOGIC);
    // LogComponentEnable("ArpL3Protocol", LOG_LEVEL_LOGIC);
    // LogComponentEnable("Ipv4L3Protocol", LOG_LEVEL_LOGIC);
    // LogComponentEnable("UdpL4Protocol", LOG_LEVEL_LOGIC);
    // LogComponentEnable("TcpL4Protocol", LOG_LEVEL_LOGIC);
    // LogComponentEnable("Socket", LOG_LEVEL_LOGIC);
    // LogComponentEnable("TcpSocket", LOG_LEVEL_LOGIC);
    // LogComponentEnable("TcpSocketBase", LOG_LEVEL_LOGIC);
    LogComponentEnableAll(LOG_PREFIX_ALL);

    // RngSeedManager::SetSeed(1);

    NetSim netsim(argc, argv);
    netsim.Run();

    return 0;
}
