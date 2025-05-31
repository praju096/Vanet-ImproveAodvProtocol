#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/aodv-module.h"
#include "ns3/aodv-routing-protocol.h"
#include "ns3/v4ping-helper.h"
#include "ns3/ns2-mobility-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("VANET_FuzzySUMO");

NodeContainer nodes;
NetDeviceContainer devices;
Ipv4InterfaceContainer interfaces;
std::map<uint32_t, double> latestRssi;

// --- Fuzzy score calculation
double CalculateFuzzyScore(double speed, double rssi, double delay, double congestion)
{
    double speedNorm = speed / 50.0;
    double rssiNorm = (rssi + 100) / 70.0;
    double delayNorm = std::min(delay / 0.5, 1.0);
    double congestionNorm = std::min(congestion / 10.0, 1.0);

    double score = (0.25 * (1.0 - speedNorm)) + (0.25 * rssiNorm) + (0.25 * (1.0 - delayNorm)) + (0.25 * (1.0 - congestionNorm));
    return score;
}

// --- Capture RSSI
void PhyRxTrace (std::string context, Ptr<const Packet> packet,
                 uint16_t channelFreqMhz, WifiTxVector txVector,
                 MpduInfo mpdu, SignalNoiseDbm signalNoise, uint16_t staId)
{
    uint32_t nodeId = Simulator::GetContext ();
    double rssiDbm = signalNoise.signal; // Now directly available
    latestRssi[nodeId] = rssiDbm;
}

// --- Density Monitoring
void MonitorNodes ()
{
    uint32_t totalNeighbors = 0;

    for (NodeContainer::Iterator i = nodes.Begin (); i != nodes.End (); ++i)
    {
        Ptr<Node> node = *i;
        uint32_t nodeId = node->GetId ();

        // Speed
        Ptr<MobilityModel> mob = node->GetObject<MobilityModel> ();
        double speed = mob->GetVelocity ().GetLength ();

        // Congestion (neighbor count)
        uint32_t neighborCount = 0;
        double range = 250.0;

        for (NodeContainer::Iterator j = nodes.Begin (); j != nodes.End (); ++j)
        {
            if (node == *j) continue;
            Ptr<MobilityModel> otherMob = (*j)->GetObject<MobilityModel> ();
            double distance = mob->GetDistanceFrom (otherMob);
            if (distance <= range)
                neighborCount++;
        }
        totalNeighbors += neighborCount;

        // RSSI
        double rssi = -80.0;
        if (latestRssi.find(nodeId) != latestRssi.end())
            rssi = latestRssi[nodeId];

        double delay = 0.2; // Dummy delay for now

        double reliability = CalculateFuzzyScore(speed, rssi, delay, neighborCount);

        NS_LOG_UNCOND ("Node " << nodeId
            << " | Speed: " << speed
            << " | RSSI: " << rssi
            << " | Delay: " << delay
            << " | Neighbors: " << neighborCount
            << " | Reliability: " << reliability);
    }

    // --- Density detection
    double avgNeighbors = totalNeighbors / (double) nodes.GetN ();
    NS_LOG_UNCOND ("[Density] Average Neighbors per Node = " << avgNeighbors);

    // --- Adjust AODV RREQ parameters
    if (avgNeighbors > 5)
    {
        // Dense network: reduce RREQ
        Config::Set("/NodeList/*/$ns3::aodv::AodvRoutingProtocol/RequestRetries", UintegerValue (1));
    }
    else
    {
        // Sparse network: increase RREQ
        Config::Set("/NodeList/*/$ns3::aodv::AodvRoutingProtocol/RequestRetries", UintegerValue (3));
    }

    Simulator::Schedule (Seconds (1.0), &MonitorNodes);
}

int main (int argc, char *argv[])
{
    std::string mobilityFile = "/home/prajesh/sumo/tools/Raopura/vanetmobility.tcl"; // Path to your SUMO-exported .tcl

    CommandLine cmd;
    cmd.AddValue ("mobility", "Ns2 mobility trace file path", mobilityFile);
    cmd.Parse (argc, argv);

    LogComponentEnable ("VANET_FuzzySUMO", LOG_LEVEL_INFO);

    nodes.Create (50); // 50 vehicles

    // Load mobility from SUMO .tcl
    Ns2MobilityHelper ns2 = Ns2MobilityHelper (mobilityFile);
    ns2.Install ();

    // Wifi settings
    WifiHelper wifi;
    wifi.SetStandard (WIFI_STANDARD_80211p);

    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
    wifiPhy.SetChannel (wifiChannel.Create ());

    WifiMacHelper wifiMac;
    wifiMac.SetType ("ns3::AdhocWifiMac");

    devices = wifi.Install (wifiPhy, wifiMac, nodes);

    // Internet + AODV
    InternetStackHelper stack;
    AodvHelper aodv;
    stack.SetRoutingHelper (aodv);
    stack.Install (nodes);

    Ipv4AddressHelper address;
    address.SetBase ("10.1.0.0", "255.255.255.0");
    interfaces = address.Assign (devices);

    // Capture RSSI
    Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx", MakeCallback (&PhyRxTrace));

    // Ping Setup (node 0 to 1)
    V4PingHelper ping (interfaces.GetAddress (1));
    ping.SetAttribute ("Interval", TimeValue (Seconds (1.0)));
    ping.SetAttribute ("Verbose", BooleanValue (false));
    ApplicationContainer app = ping.Install (nodes.Get (0));
    app.Start (Seconds (2.0));
    app.Stop (Seconds (25.0));

    // Start Monitoring
    Simulator::Schedule (Seconds (1.0), &MonitorNodes);

    Simulator::Stop (Seconds (30.0));
    Simulator::Run ();
    Simulator::Destroy ();
}
