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
// "Simple AODV"
//
// Node0 sneds a packet to node2. Then, AODV is used as routing.
//
//   n0        n1        n2
//   o ------> o ------> o
//       40m       40m
//

class NetSim {
    std::string OUTPUT_DIR = "output/simple_aodv/";  // OUTPUT_DIR:=(ログファイルの出力ディレクトリ).
    std::string ANIM_DIR = "output/animation/";      // ANIM_DIR:=(netanim用出力ディレクトリ).

    bool m_tracing;        // m_tracing:=(トレーシングを有効にする).
    bool m_verbose;        // m_verbose:=(詳細なロギングを有効にする).
    std::string m_prefix;  // m_prefix:=(ログファイルの接頭辞).

    int m_vn;                            // m_vn:=(ノード数).
    double m_dist;                       // m_dist:=(ノード間距離).
    NodeContainer m_nodes;               // m_nodes:=(ノードコンテナ).
    NetDeviceContainer m_devices;        // m_devices:=(デバイスコンテナ).
    Ipv4InterfaceContainer m_ifs;        // m_ifs:=(インターフェイスコンテナ).
    std::vector<Ptr<Socket>> m_sockets;  // m_sockets[v]:=(ノードvのソケット).
    double m_simStop;                    // m_simStop:=(シミュレーション終了時間).
    double m_appStart;                   // m_appStart:=(アプリ開始時間).

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
    m_prefix = "simple_aodv";

    m_vn = 3;
    m_dist = 40.0;
    m_simStop = 60.0;
    m_appStart = 10.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("tracing", "Enable tracing, if true", m_tracing);
    cmd.AddValue("verbose", "Enable verbose logging, if true", m_verbose);
    cmd.AddValue("prefix", "prefix of log files", m_prefix);
    cmd.Parse(argc, argv);

    m_sockets.resize(m_vn);
}

NetSim::~NetSim() {
    NS_LOG_FUNCTION(this);
}

int main(int argc, char *argv[]) {
}
