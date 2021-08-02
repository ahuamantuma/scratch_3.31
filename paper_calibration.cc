#include <iostream>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/packet.h"

#define PI 3.14159265

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("PaperCalibration");

// Funcion principal
int 
main (int argc, char *argv[])
{
  // Simulation:
  int n_nodes = 2;
  double Duration = 3600;
  double Distance = 1;
  double PathLoss = 40;
  bool isTXRXLoggingEnabled = false;
  // Radio:
  double TxPower = 0;
  double RxSensitivity = -101;
  int HtMcsControl=7;
  int HtMcsData=7;
  int MaxSsrc = 0;
  int MaxSlrc = 0;
  double NoiseFigure = 0; // Lo dejamos sin ruido
  double RxGain = 0, TxGain=0;
  int RtsCtsThr = 65500; // 50: RTS activado

  // Commands
  CommandLine cmd (__FILE__);
  cmd.AddValue ("n_nodes", "Cantidad de nodos: default=", n_nodes);
  cmd.AddValue ("Duration", "Tiempo total de simulacion (segundos): default=", Duration);
  cmd.Parse (argc, argv);
  
  // Proces HtMcs
  std::string HtMcsControlMode = "HtMcs" + std::to_string(HtMcsControl);
  std::string HtMcsDataMode = "HtMcs" + std::to_string(HtMcsData);
    
  // Set Real time simulation 
  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  // Create nodes
  NodeContainer nodes;
  nodes.Create (n_nodes);
  
  // ======= Ubicacion de los nodos ======= 
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator> ();
  
  posAlloc->Add(Vector (0,0,0));
  posAlloc->Add(Vector (0, Distance, 0));

  mobility.SetPositionAllocator (posAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  // Configurar las ubicaciones en los nodos
  mobility.Install (nodes);
    
  // =====================================================
  
  // Seteo el estandar Wifi
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211n_2_4GHZ);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                  "DataMode",StringValue (HtMcsDataMode),
                                  "ControlMode",StringValue (HtMcsControlMode),
                                  "MaxSsrc", UintegerValue (MaxSsrc),
                                  "MaxSlrc", UintegerValue (MaxSlrc),
                                  "RtsCtsThreshold", UintegerValue (RtsCtsThr));
  
  // Creo y seteo el canal wifi
  Ptr<YansWifiChannel> wifiChannel = CreateObject <YansWifiChannel> ();
  wifiChannel->SetPropagationDelayModel (CreateObject <ConstantSpeedPropagationDelayModel> ());
  
  // Create propagation loss matrix
  // =========================================================================
  Ptr<MatrixPropagationLossModel> lossModel = CreateObject<MatrixPropagationLossModel> ();
  
  lossModel->SetDefaultLoss (PathLoss);
  
  wifiChannel->SetPropagationLossModel (lossModel);  
  // =========================================================================

  // Seteo los parametros de la capa fisica Wifi
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  wifiPhy.SetChannel (wifiChannel);
  wifiPhy.Set ("TxPowerStart",DoubleValue (TxPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (TxPower));
  wifiPhy.Set ("RxSensitivity", DoubleValue (RxSensitivity));
  wifiPhy.Set ("RxNoiseFigure", DoubleValue(NoiseFigure));
  wifiPhy.Set ("RxGain", DoubleValue(RxGain));
  wifiPhy.Set ("TxGain", DoubleValue(TxGain));
  
  WifiMacHelper wifiMac;
  NetDeviceContainer devices;

  // Seteo el tipo de conectividad Wifi como 'Ad Hoc'
  wifiMac.SetType ("ns3::AdhocWifiMac");
  // Creo las interfaces wifi Ad Hoc en los nodos
  devices = wifi.Install (wifiPhy, wifiMac, nodes);

  // Modificaciones especiales
  // =========================================================================
  Ptr<NetDevice> dev;
  Ptr<WifiNetDevice> wifi_dev;
  Ptr<AdhocWifiMac> adhoc_mac;
  
  // Nodos
  for (int i=0 ; i<n_nodes ; i++ ) {
    dev = nodes.Get(i)->GetDevice(0);
    wifi_dev = DynamicCast<WifiNetDevice>(dev);
      // Disable A-MPDU and A-MSDU
    wifi_dev->GetMac()->SetAttribute("BE_MaxAmpduSize", UintegerValue (0));
    wifi_dev->GetMac()->SetAttribute("BE_MaxAmsduSize", UintegerValue (0));
      // Set timeslot to 9 useg
    wifi_dev->GetMac()->SetSlot(MicroSeconds(9));
      // Activate RX Logging on Mac Low
    if (isTXRXLoggingEnabled) {
      adhoc_mac = DynamicCast<AdhocWifiMac> (wifi_dev->GetMac());
      adhoc_mac->EnableMacLowLogging();
    }
  }

  // Channel
  // Activate TX Logging on YansWifiChannel
  if (isTXRXLoggingEnabled) {
    Ptr<YansWifiChannel> yanschannel = DynamicCast<YansWifiChannel> (wifi_dev->GetChannel());
    yanschannel->SetLogActivate(true);
  }
  // =========================================================================

  // Asociacion con los containers
  TapBridgeHelper tapBridge;
  tapBridge.SetAttribute ("Mode", StringValue ("UseLocal")); // Nodo se conecta a un TAP existente
  
  for (int i=0; i < n_nodes; i++)
  {
    tapBridge.SetAttribute ("DeviceName", StringValue ("tap-" + std::to_string(i+1) ));
    tapBridge.Install (nodes.Get (i), devices.Get (i));
  }

  // Fin de los eventos de la simulacion
  Simulator::Stop (Seconds (Duration));

  //NS_LOG_UNCOND ("Simulation Started");
  std::clog << "Simulation Started" << std::endl;

  Simulator::Run ();
  Simulator::Destroy ();
  std::clog << "Simulation Finished" << std::endl;
  return 0;
}
