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
    std::string OUTPUT_DIR = "output/simple-flow/";  // OUTPUT_DIR:=(ログファイルの出力ディレクトリ).
    std::string ANIM_DIR = "output/animation/";      // ANIM_DIR:=(netanim用出力ディレクトリ).

    bool m_tracing;        // m_tracing:=(トレーシングを有効にするか).
    bool m_logging;        // m_logging:=(ロギングを有効にするか).
    std::string m_prefix;  // m_prefix:=(ログファイルの接頭辞).

    int m_vn;                      // m_vn:=(ノード数).
    double m_dist;                 // m_dist:=(ノード間距離).
    NodeContainer m_nodes;         // m_nodes:=(ノードコンテナ).
    NetDeviceContainer m_devices;  // m_devices:=(デバイスコンテナ).
    Ipv4InterfaceContainer m_ifs;  // m_ifs:=(インターフェイスコンテナ).
    bool m_enableTcp;              // m_enableTcp:=(TCPを用いるか).
    double m_simStop;              // m_simStop:=(シミュレーション終了時間).

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
    m_prefix = "simple-flow";

    m_vn = 2;
    m_dist = 40.0;
    m_enableTcp = false;
    m_simStop = 100.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("tracing", "Enable tracing, if true", m_tracing);
    cmd.AddValue("logging", "Enable logging, if true", m_logging);
    cmd.AddValue("prefix", "Prefix of log files", m_prefix);
    cmd.AddValue("tcp", "Enable using TCP, if true", m_enableTcp);
    cmd.Parse(argc, argv);
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

void NetSim::CreateNodes(void) {
    NS_LOG_FUNCTION(this);

    m_nodes.Create(m_vn);

    auto posiList = CreateObject<ListPositionAllocator>();
    posiList->Add(Vector(0, 0, 0));
    posiList->Add(Vector(m_dist, 0, 0));

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

    Ipv4StaticRoutingHelper staticRouting;

    InternetStackHelper internet;
    internet.SetRoutingHelper(staticRouting);
    internet.Install(m_nodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    m_ifs = ipv4.Assign(m_devices);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    staticRouting.GetStaticRouting(m_nodes.Get(0)->GetObject<Ipv4>())->AddHostRouteTo("10.1.1.2", "10.1.1.2", 1);
    staticRouting.GetStaticRouting(m_nodes.Get(1)->GetObject<Ipv4>())->AddHostRouteTo("10.1.1.1", "10.1.1.1", 1);
}

void NetSim::ConfigureL7(void) {
    NS_LOG_FUNCTION(this);

    uint16_t port = 12345;

    if(m_enableTcp) {  // use TCP.
        PacketSinkHelper sink("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));

        auto sinkApp = sink.Install(m_nodes.Get(1));
        sinkApp.Start(Seconds(5.0));
        sinkApp.Stop(Seconds(90.0));

        BulkSendHelper bulk("ns3::TcpSocketFactory", InetSocketAddress("10.1.1.2", port));
        bulk.SetAttribute("MaxBytes", UintegerValue(10000));

        auto bulkApp = bulk.Install(m_nodes.Get(0));
        bulkApp.Start(Seconds(10.0));
        bulkApp.Stop(Seconds(80.0));
    } else {  // use UDP.
        PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));

        auto sinkApp = sink.Install(m_nodes.Get(1));
        sinkApp.Start(Seconds(5.0));
        sinkApp.Stop(Seconds(90.0));

        OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress("10.1.1.2", port));
        onoff.SetAttribute("MaxBytes", UintegerValue(10000));

        auto onoffApp = onoff.Install(m_nodes.Get(0));
        onoffApp.Start(Seconds(10.0));
        onoffApp.Stop(Seconds(80.0));
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
    auto classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    const auto &stats = monitor->GetFlowStats();
    for(const auto &[id, stat] : stats) {
        const auto &ft = classifier->FindFlow(id);
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
    LogComponentEnable("PacketSink", LOG_LEVEL_LOGIC);
    LogComponentEnable("OnOffApplication", LOG_LEVEL_LOGIC);
    LogComponentEnable("BulkSendApplication", LOG_LEVEL_LOGIC);
    LogComponentEnableAll(LOG_PREFIX_ALL);

    // RngSeedManager::SetSeed(1);

    NetSim netsim(argc, argv);
    netsim.Run();

    return 0;
}
