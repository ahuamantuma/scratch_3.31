#include <iostream>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/packet.h"

#define PI 3.14159265

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("AdhocTesting");

// Funcion principal
int 
main (int argc, char *argv[])
{
  // Simulation:
  int n_nodes = 2;
  double Duration = 3600;
  double Distance = 50;
  double PathLoss = 60;
  bool isTXRXLoggingEnabled = false;
  bool isHiddenTerminalEnabled = true;
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
  cmd.AddValue ("TxPower", "Potencia de transmision (dBm): default=", TxPower);
  cmd.AddValue ("HtMcsControl","Esquema de modulacion y codificacion para el trafico de control en ambos nodos: default=",HtMcsControl);
  cmd.AddValue ("HtMcsData","Esquema de modulacion y codificacion para el trafico de datos en ambos nodos: default=",HtMcsData);
  cmd.AddValue ("RtsCtsThr","Thereshold para RtsCts: default=", RtsCtsThr);
  cmd.AddValue ("MaxSsrc","Short Retry Limit: default=", MaxSsrc);
  cmd.AddValue ("MaxSlrc","Long Retry Limit: default=", MaxSlrc);
  cmd.AddValue ("HT_Enabled","Hidden Terminal enbled: default=", isHiddenTerminalEnabled);
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
  
  // ======= Logica para la ubicacion de los nodos ======= 
  MobilityHelper mobility;
  /*
  // nodos ubicados como un anillo, radio = Distance
  Ptr<ListPositionAllocator> posStas = CreateObject<ListPositionAllocator> ();
  
  int radio = Distance;
  double fase = 0;
  double xpos = 0;
  double ypos = 0;
  double separacion_angular = 2*PI / n_nodes;

  for (int i=0; i < n_nodes; i++)
  {
    xpos = radio * cos (fase);
    ypos = radio * sin (fase);
    //std::clog << "node-"<<(i+1)<<", xpos="<< xpos <<", ypos="<< ypos <<std::endl;
    posStas->Add (Vector (xpos, ypos, 0.0));
    fase += separacion_angular;
  }
  
  mobility.SetPositionAllocator (posStas);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  */
  // nodos ubicados estaticamente en grilla
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (Distance),
                                 "DeltaY", DoubleValue (Distance),
                                 "GridWidth", UintegerValue (3));
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
  
  lossModel->SetDefaultLoss (PathLoss); // 160 para HT
  //lossModel->SetLoss (nodes.Get (0)->GetObject<MobilityModel> (), nodes.Get (1)->GetObject<MobilityModel> (), PathLoss);
  //lossModel->SetLoss (nodes.Get (1)->GetObject<MobilityModel> (), nodes.Get (2)->GetObject<MobilityModel> (), PathLoss);
  //lossModel->SetLoss (nodes.Get (0)->GetObject<MobilityModel> (), nodes.Get (2)->GetObject<MobilityModel> (), PathLoss * 2);
  
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
  NetDeviceContainer staDevs;
  NetDeviceContainer apDev;

  // Seteo el tipo de conectividad Wifi como 'Ad Hoc'
  wifiMac.SetType ("ns3::AdhocWifiMac");
  // Creo las interfaces wifi Ad Hoc en los nodos
  staDevs = wifi.Install (wifiPhy, wifiMac, nodes);

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
    tapBridge.Install (nodes.Get (i), staDevs.Get (i));
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
