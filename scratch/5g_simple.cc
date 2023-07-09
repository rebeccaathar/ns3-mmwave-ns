
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store.h"
#include "ns3/mmwave-helper.h"
#include <ns3/buildings-helper.h>
#include "ns3/global-route-manager.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/log.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
#include "ns3/point-to-point-helper.h"
#include <map>

using namespace ns3;
using namespace mmwave;

/* 
 *In this example, a single UE is connected with a single MmWave BS. The
 * UE is randomly placed inside a circle with radius of 150m and it moves with
 * random velocity between 2m/s and 4m/s. The BS is placed in the center with
 * height 15m. The system bandwidth is fixed at 1GHz. If CA is enabled, 2 CCs
 * are used and the total bandwidth is divided among the two according to
 * the bandDiv parameter, which is equal to the ratio between the badwidth
 * allocated to CC0 and the bandwidth allocated to CC1.
*/

NS_LOG_COMPONENT_DEFINE ("5gNetwork");

void
PrintPosition (Ptr<Node> node)
{
  Ptr<MobilityModel> model = node->GetObject<MobilityModel> ();
  NS_LOG_UNCOND ("Position +****************************** " << model->GetPosition () << " at time "
                                                             << Simulator::Now ().GetSeconds ());
}


static ns3::GlobalValue g_interPckInterval ("interPckInterval", "Interarrival time of UDP packets (us)",
                                            ns3::UintegerValue (20), ns3::MakeUintegerChecker<uint32_t> ());
                                          

static ns3::GlobalValue g_rlcAmEnabled ("rlcAmEnabled", "If true, use RLC AM, else use RLC UM",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_outPath ("outPath",
                                   "The path of output log files",
                                   ns3::StringValue ("./"), ns3::MakeStringChecker ());

static ns3::GlobalValue g_noiseAndFilter ("noiseAndFilter", "If true, use noisy SINR samples, filtered. If false, just use the SINR measure",
                                          ns3::BooleanValue (false), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_reportTablePeriodicity ("reportTablePeriodicity", "Periodicity of RTs",
                                                  ns3::UintegerValue (1600), ns3::MakeUintegerChecker<uint32_t> ());


static ns3::GlobalValue g_lteUplink ("lteUplink", "If true, always use LTE for uplink signalling",
                                     ns3::BooleanValue (false), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_simTime ("simTime", "Simulation time in seconds", ns3::DoubleValue (5),
                                   ns3::MakeDoubleChecker<double> (0.1, 100.0));



int main (int argc, char *argv[])
{  
   // Command line arguments
  CommandLine cmd;
  cmd.Parse (argc, argv);

  UintegerValue uintegerValue;
  BooleanValue booleanValue;
  StringValue stringValue;
  DoubleValue doubleValue; 

  bool harqEnabled = false;
  // bool fixedTti = false;

  GlobalValue::GetValueByName ("rlcAmEnabled", booleanValue);
  bool rlcAmEnabled = booleanValue.Get ();
  // GlobalValue::GetValueByName ("bufferSize", uintegerValue);
  // uint32_t bufferSize = uintegerValue.Get ();
  GlobalValue::GetValueByName ("interPckInterval", uintegerValue);
  uint32_t interPacketInterval = uintegerValue.Get ();
 
  Ptr<MmWaveHelper> helper = CreateObject<MmWaveHelper> ();
  // helper->SetSchedulerType ("ns3::MmWaveFlexTtiMacScheduler");
  // Config::SetDefault ("ns3::MmWaveUeMac::UpdateUeSinrEstimatePeriod", DoubleValue (0));

  Config::SetDefault ("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue (rlcAmEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::HarqEnabled", BooleanValue (harqEnabled));
  // Config::SetDefault ("ns3::MmWaveFlexTtiPfMacScheduler::HarqEnabled", BooleanValue (harqEnabled));
  // Config::SetDefault ("ns3::MmWavePhyMacCommon::NumHarqProcess", UintegerValue (100));
  // Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (harqEnabled));
  // Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::FixedTti", BooleanValue (fixedTti));
  // Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::SymPerSlot", UintegerValue (6));
  // Config::SetDefault ("ns3::MmWavePhyMacCommon::TbDecodeLatency", UintegerValue (500.0));
  // Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (100)));


  double bandwidth = 20e6; //20MHz 
  double centerFrequency = 3.5e9;
  // int numAntennasMcUe = 4;
  // int numAntennasMmWave = 4;


  // NS_LOG_INFO ("Bandwidth " << bandwidth << " centerFrequency " << double (centerFrequency)
  //                           << " numAntennasMcUe " << numAntennasMcUe
  //                           << " numAntennasMmWave " << numAntennasMmWave);


  // Ptr<MmWaveHelper> helper = CreateObject<MmWaveHelper> ();

  // // Set the number of antennas in the devices
  // Config::SetDefault ("ns3::McUeNetDevice::AntennaNum", UintegerValue (numAntennasMcUe));
  // Config::SetDefault ("ns3::MmWaveNetDevice::AntennaNum", UintegerValue (numAntennasMmWave));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::Bandwidth", DoubleValue (bandwidth));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (centerFrequency));


 // create the EPC
  Ipv4Address remoteHostAddr;
  Ptr<Node> remoteHost;
  InternetStackHelper internet;
  Ptr<MmWavePointToPointEpcHelper> epcHelper;
  // Ipv4StaticRoutingHelper ipv4RoutingHelper;


  epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
  helper->SetEpcHelper (epcHelper);

  // create the Internet by connecting remoteHost to pgw. Setup routing too
  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  // create remotehost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);

  internet.Install (remoteHostContainer);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ipv4InterfaceContainer internetIpIfaces;

  remoteHost = remoteHostContainer.Get (0);
  // create the Internet
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500)); // The MAC-level Maximum Transmission Unit
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.01)));

  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);

  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.255.0.0");
  internetIpIfaces = ipv4h.Assign (internetDevices);
  // interface 0 is localhost, 1 is the p2p device
  remoteHostAddr = internetIpIfaces.GetAddress (1);

  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.255.0.0"), 1);

   // create the enb node
  NodeContainer enbNodes;
  enbNodes.Create (1);

  // set mobility
  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
  enbPositionAlloc->Add (Vector (0.0, 0.0, 15.0));

  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator (enbPositionAlloc);
  enbmobility.Install (enbNodes);
  BuildingsHelper::Install (enbNodes);

  // install enb device
  NetDeviceContainer enbNetDevices = helper->InstallEnbDevice (enbNodes);
  std::cout << "eNB device installed" << std::endl;

  // create ue node
  NodeContainer ueNodes;
  ueNodes.Create (1);

  // set mobility
  MobilityHelper uemobility;
  enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  uemobility.SetPositionAllocator (enbPositionAlloc);
  uemobility.Install (ueNodes.Get(0));
  BuildingsHelper::Install (enbNodes);
  uemobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                               "Bounds", RectangleValue (Rectangle (-200, 200, -200, 200)));
  Config::SetDefault ("ns3::UniformDiscPositionAllocator::rho", DoubleValue (150));
  Config::SetDefault ("ns3::UniformDiscPositionAllocator::Z", DoubleValue (1.6));
  Ptr<UniformDiscPositionAllocator> uePositionAlloc = CreateObject<UniformDiscPositionAllocator> ();
  uemobility.SetPositionAllocator (uePositionAlloc);
  uemobility.Install (ueNodes.Get (0));


  BuildingsHelper::Install (ueNodes);
  std::cout << "UE initial position :" << ueNodes.Get (0)->GetObject<MobilityModel> ()->GetPosition () << std::endl;

  // install ue device
  NetDeviceContainer ueNetDevices = helper->InstallUeDevice (ueNodes);
  std::cout << "UE device installed" << std::endl;


  // install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (ueNetDevices);
  // assign IP address to UEs, and install applications
  // set the default gateway for the UE
  Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNodes.Get (0)->GetObject<Ipv4> ());
  ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

  helper->AttachToClosestEnb (ueNetDevices, enbNetDevices);

  // install and start applications on UEs and remote host
  uint16_t dlPort = 1234;
  uint16_t ulPort = 2000;
  ApplicationContainer clientApps;
  ApplicationContainer serverApps;

  // uint16_t interPacketInterval = 100;

  PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
  PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
  serverApps.Add (dlPacketSinkHelper.Install (ueNodes.Get (0)));
  serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

  UdpClientHelper dlClient (ueIpIface.GetAddress (0), dlPort);
  dlClient.SetAttribute ("Interval", TimeValue (MilliSeconds (interPacketInterval)));
  dlClient.SetAttribute ("MaxPackets", UintegerValue (1000000));

  UdpClientHelper ulClient (remoteHostAddr, ulPort);
  ulClient.SetAttribute ("Interval", TimeValue (MilliSeconds (interPacketInterval)));
  ulClient.SetAttribute ("MaxPackets", UintegerValue (1000000));

    
  clientApps.Add (dlClient.Install (remoteHost));
  clientApps.Add (ulClient.Install (ueNodes.Get (0)));

  helper->EnableTraces();

  // Start applications
  GlobalValue::GetValueByName ("simTime", doubleValue);
  double simTime = doubleValue.Get ();
  // sinkApps.Start (Seconds (0));

  clientApps.Start (MilliSeconds (100));
  clientApps.Stop (Seconds (simTime - 0.1));


  bool run = true;
  if (run)
    {
      NS_LOG_UNCOND ("Simulation time is " << simTime << " seconds! ");
      Simulator::Stop (Seconds (simTime));
      NS_LOG_INFO ("Run Simulation.");
      Simulator::Run ();
    }

//   NS_LOG_INFO (lteHelper);
  NS_LOG_INFO ("Done.");
  Simulator::Destroy ();
  return 0;
}
  
