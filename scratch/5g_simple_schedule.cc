#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store.h"
#include "ns3/mmwave-helper.h"
#include <ns3/buildings-helper.h>
#include "ns3/global-route-manager.h"
#include "ns3/applications-module.h"
#include "ns3/mmwave-net-device.h"
#include "ns3/ipv4-global-routing-helper.h"
#include <ns3/mmwave-beamforming-model.h>
#include "ns3/mmwave-eesm-ir-t1.h"
#include "ns3/mmwave-eesm-cc-t1.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include <ns3/eps-bearer.h>
#include <ns3/packet.h>
#include "ns3/mmwave-enb-net-device.h"
#include "ns3/three-gpp-antenna-model.h"
#include "ns3/simulator.h"
#include "ns3/flow-monitor-module.h"
//#include "ns3/test.h"
#include "ns3/log.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/point-to-point-helper.h"
#include <map>

using namespace ns3;
using namespace mmwave;

NS_LOG_COMPONENT_DEFINE ("5g_simple_schedule");


int main(){

    Time simTime = Seconds(5); // Define o tempo de simulação em milissegundos 
    Time udpAppStartTimeDl = Seconds(0.5); // Define o tempo que a aplicação de Dl deverá começar 
    Time udpAppStartTimeUl = Seconds(0.5); // Define o tempo que a aplicação de Ul deverá começar 
    Time udpAppStopTimeDl = Seconds(5);  // Define um tempo 1 segundo de tráfego de Dl 
    Time udpAppStopTimeUl = Seconds(5);  // Define um tempo 1 segundo de tráfego de Ul 

  
    // uint32_t interPacketInterval = 1e3 // App inter packet arrival [us]
    uint32_t packetSize = 500;        // tamanho dos pacotes gerados  [B] 
    uint32_t maxPackets = 1000;        // the maximum number of packets the application will send (zero means infinite)
   
    
    bool m_isDownlink = true;         // generate the downlink traffic
    bool m_isUplink = true;           // generate the uplink traffic
    
    bool harqEnabled = true;

    double maxXAxis = 1000;
    double maxYAxis = 1000;

    UintegerValue uintegerValue;
    BooleanValue booleanValue;
    StringValue stringValue;
    DoubleValue doubleValue;


    Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();


    Config::SetDefault ("ns3::MmWaveHelper::UseIdealRrc", BooleanValue (true));
    Config::SetDefault ("ns3::MmWavePhyMacCommon::NumHarqProcess", UintegerValue (100));
    //Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::EpochDuration", TimeValue (MilliSeconds (10.0)));

    // set to false to use the 3GPP radiation pattern (proper configuration of the bearing and downtilt angles is needed)
    // Config::SetDefault ("ns3::ThreeGppAntennaArrayModel::IsotropicElements", BooleanValue (true));
    Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (100.0)));
    Config::SetDefault ("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
                        TimeValue (MilliSeconds (100)));

    //Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::EpochDuration", TimeValue (MilliSeconds (10.0)));

    mmwaveHelper ->SetBeamformingModelType("ns3::MmWaveSvdBeamforming"); // Set Beamforming 
    mmwaveHelper->SetSchedulerType ("ns3::MmWaveFlexTtiPfMacScheduler"); // Set Scheduler 
    Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (harqEnabled)); // Enable Hybrid Automatic Repeat Request (HARQ) 
    std::cout << "Scheduler usado --> " << mmwaveHelper->GetSchedulerType() << std::endl;

    // Carrier bandwidth in Hz
    double bandwidth = 20e6; // 20MHz 
    // Center frequency in Hz
    double centerFrequency = 3.5e9; //3.5GHz 

    Config::SetDefault ("ns3::MmWavePhyMacCommon::Bandwidth", DoubleValue (bandwidth));
    Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (centerFrequency));


    mmwaveHelper->SetPathlossModelType ("ns3::ThreeGppUmiStreetCanyonPropagationLossModel");
    mmwaveHelper->SetChannelConditionModelType ("ns3::ThreeGppUmiStreetCanyonChannelConditionModel");

    Ptr<MmWavePointToPointEpcHelper> epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
    mmwaveHelper->SetEpcHelper (epcHelper);


    // Create base stations and mobile terminals 
    uint16_t gNbNum = 1;
    uint16_t ueNum = 3;
    double gNbHeight = 3;
    // double ueHeight = 1.5;
    double d = 500;


    NodeContainer gNbNodes;
    NodeContainer ueNodes;

    gNbNodes.Create(gNbNum);
    ueNodes.Create( gNbNum * ueNum); // usuários por beam * gNB

    Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator>();  // gNB
    Ptr<UniformDiscPositionAllocator> uePositionAlloc = CreateObject<UniformDiscPositionAllocator> ();

    Vector centerPosition = Vector (maxXAxis / 2, maxYAxis / 2, gNbHeight); 

    // double x = d * cos ((2 * M_PI )); 
    // double y = d * sin ((2 * M_PI ));
    
    gnbPositionAlloc->Add (centerPosition); 
    

    MobilityHelper gnbmobility;
    // Definindo Mobilidade 
    gnbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    gnbmobility.SetPositionAllocator (gnbPositionAlloc);
    gnbmobility.Install (gNbNodes);
    gnbmobility.Install (ueNodes);


    MobilityHelper uemobility;

    uePositionAlloc->SetX (centerPosition.x);
    uePositionAlloc->SetY (centerPosition.y);
    //uePositionAlloc->SetZ (centerPosition.z);
    uePositionAlloc->SetRho (d);
    Ptr<UniformRandomVariable> speed = CreateObject<UniformRandomVariable> ();
    speed->SetAttribute ("Min", DoubleValue (2.0));
    speed->SetAttribute ("Max", DoubleValue (4.0));

    uemobility.SetMobilityModel ("ns3::RandomWalk2dOutdoorMobilityModel", "Speed",
                                PointerValue (speed), "Bounds",
                                RectangleValue (Rectangle (0, maxXAxis, 0, maxYAxis)));
    uemobility.SetPositionAllocator (uePositionAlloc);
    uemobility.Install (ueNodes);

    // Install mmWave, lte, mc Devices to the nodes
    NetDeviceContainer gNbNetDevs = mmwaveHelper->InstallEnbDevice(gNbNodes);
    std::cout << "gNB installed!!" << std::endl;
    NetDeviceContainer ueNetDevs = mmwaveHelper->InstallUeDevice(ueNodes);
    std::cout << "UE installed!!" << std::endl;

     
    std::cout << "---- Positions ---- " << std::endl;
    for (uint32_t j = 0; j < gNbNodes.GetN (); ++j) {

        std::cout << "gNB: "<< j << "   Position: " << gNbNodes.Get (j)->GetObject<MobilityModel> ()->GetPosition () << std::endl;
    }

    for (uint32_t j = 0; j < ueNodes.GetN (); ++j) {

        std::cout << "UE: "<< j << "   Position: " << ueNodes.Get (j)->GetObject<MobilityModel> ()->GetPosition () << std::endl;
    }


    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    // create the internet and install the IP stack on the UEs
    // get SGW/PGW and create a single RemoteHost
    Ptr<Node> pgw = epcHelper->GetPgwNode ();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create (1);
    Ptr<Node> remoteHost = remoteHostContainer.Get (0);
    InternetStackHelper internet;
    internet.Install (remoteHostContainer);
    // connect a remoteHost to pgw. Setup routing too
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
    p2ph.SetDeviceAttribute ("Mtu", UintegerValue (2500));
    p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
    NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
    // in this container, interface 0 is the pgw, 1 is the remoteHost
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);
    internet.Install (ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueNetDevs));


    // Set the default gateway for the UEs
    for (uint32_t j = 0; j < ueNodes.GetN (); ++j)
      {
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNodes.Get(j)->GetObject<Ipv4> ());
        ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
      }

    // attach UEs to the closest eNB
    mmwaveHelper->AttachToClosestEnb (ueNetDevs, gNbNetDevs);

    // assign IP address to UEs, and install UDP downlink applications
    uint16_t dlPort = 1234;
    uint16_t ulPort = 2000;
    ApplicationContainer clientAppsDl;
    ApplicationContainer serverAppsDl;
    ApplicationContainer clientAppsUl;
    ApplicationContainer serverAppsUl;
    

    //ObjectMapValue objectMapValue;
    Time udpInterval = NanoSeconds(1); // The time to wait between packets

    if (m_isUplink)
      {
        UdpServerHelper ulPacketSinkHelper (ulPort);
        serverAppsUl.Add (ulPacketSinkHelper.Install (remoteHost));

        // configure here UDP traffic flows
        for (uint32_t j = 0; j < ueNodes.GetN (); ++j)
          {

            UdpClientHelper ulClient (remoteHostAddr, ulPort);
            ulClient.SetAttribute ("MaxPackets", UintegerValue(maxPackets)); // 800
            ulClient.SetAttribute("PacketSize", UintegerValue(packetSize));  // 200
            ulClient.SetAttribute ("Interval", TimeValue (NanoSeconds(1)));
            clientAppsUl.Add (ulClient.Install (ueNodes.Get(j)));

            Ptr<EpcTft> tft = Create<EpcTft> ();
            EpcTft::PacketFilter ulpf;
            ulpf.remotePortStart = ulPort;
            ulpf.remotePortEnd = ulPort;
            ulpf.direction = EpcTft::UPLINK;
            tft->Add (ulpf);

          }
; 
        serverAppsUl.Stop(udpAppStopTimeUl);
        clientAppsUl.Stop(udpAppStopTimeUl);
      }


    if (m_isDownlink)
      {
        UdpServerHelper dlPacketSinkHelper (dlPort);
        serverAppsDl.Add (dlPacketSinkHelper.Install (ueNodes));

        // configure here UDP traffic flows
        for (uint32_t j = 0; j < ueNodes.GetN (); ++j)
          {
            UdpClientHelper dlClient (ueIpIface.GetAddress (j), dlPort);
            dlClient.SetAttribute ("MaxPackets", UintegerValue(maxPackets)); // 800
            dlClient.SetAttribute("PacketSize", UintegerValue(packetSize));  // 200 
            dlClient.SetAttribute ("Interval", TimeValue (NanoSeconds(1)));
            clientAppsDl.Add (dlClient.Install (remoteHost));

            Ptr<EpcTft> tft = Create<EpcTft> ();
            EpcTft::PacketFilter dlpf;
            dlpf.localPortStart = dlPort;
            dlpf.localPortEnd = dlPort;
            dlpf.direction = EpcTft::DOWNLINK;
            tft->Add (dlpf);

          }
        // start UDP server and client apps
       serverAppsDl.Start(udpAppStartTimeDl);
       clientAppsDl.Start(udpAppStartTimeDl);
       serverAppsDl.Stop(udpAppStopTimeDl);
       clientAppsDl.Stop(udpAppStopTimeDl);
     }

    Simulator::Stop (Seconds(5));
    
    mmwaveHelper->EnableTraces();

    Simulator::Run ();

    monitor->CheckForLostPackets();

    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
    
    double TotaldataRecvDl = 0;
    double TotaldataRateDl = 0;
    double TotaldataRecvUl = 0;
    double TotaldataRateUl = 0;
    double dataRate = 0;
    // double dataRecvUl = 0;

    if (m_isDownlink)
      {
        for ( uint32_t i = 0; i < serverAppsDl.GetN (); i++)
          {
            Ptr<UdpServer> serverApp = serverAppsDl.Get (i)->GetObject<UdpServer> ();
            double data = (serverApp->GetReceived () * packetSize * 8);
            TotaldataRecvDl += data;

            dataRate = data / simTime.GetSeconds();
            TotaldataRateDl = TotaldataRecvDl / simTime.GetSeconds();

            uint32_t UEId = i + 1;

            std::cout << " UE = " << UEId << "    Data Rate: " << dataRate << " bytes/s" << " Number of Received Packets (Dl): "<< data <<  std::endl;
            dataRate = 0 ;
          }
      }

    std::cout << " Total Data Rate (Downlink) "  << TotaldataRateDl << " bytes/s" << " Total Number of Received Packets (Dl):  "<< TotaldataRecvDl <<  std::endl;


    if (m_isUplink)
      {
        for ( uint32_t i = 0; i < serverAppsUl.GetN (); i++)
          {
            Ptr<UdpServer> serverApp = serverAppsUl.Get (i)->GetObject<UdpServer> ();
            double data = (serverApp->GetReceived () * packetSize * 8);
            TotaldataRecvUl += data;
            TotaldataRateUl = TotaldataRecvUl / simTime.GetSeconds();
          }
      }

    std::cout << " Total Data Rate (Uplink) "  << TotaldataRateUl << " bytes/s" << " Total Number of Received Packets (Ul):  "<< TotaldataRecvUl <<  std::endl;

   
    Simulator::Destroy ();
}

