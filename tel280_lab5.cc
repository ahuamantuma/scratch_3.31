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
  int n_nodes = 12;
  double Duration = 3600;
  double PathLoss = 80;
  bool isTXRXLoggingEnabled = false;
  // Radio:
  double TxPower = 20;
  double RxSensitivity = -101;
  int HtMcsControl=0;
  int HtMcsData=5;
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
  // Topologia simple del paper del profe
  int n_nodes_col1 = 3;
  int n_nodes_col2 = 2;
  //int n_nodes_col3 = 1;
  //int n_nodes_col4 = 1;
  int n_nodes_col5 = 2;
  int n_nodes_col6 = 3;
  
  int radio_col1 = 30;
  int radio_col2 = 15;
  //int radio_col3 = 30;
  //int radio_col4 = 30;
  int radio_col5 = 15;
  int radio_col6 = 30;
  int altura = 20;
  
  double fase;
  double xpos;
  double ypos;
  double separacion_angular;

  // Col 1:
  fase = 0;
  xpos = 0;
  ypos = 0;
  separacion_angular = 2*PI / n_nodes_col1;
  
  Ptr<ListPositionAllocator> posNodes = CreateObject<ListPositionAllocator> ();
  
  for (int i=0; i < n_nodes_col1; i++)
  {
    xpos = radio_col1 * cos (fase);
    ypos = radio_col1 * sin (fase);
    //std::clog << "node-"<<(i+1)<<", x="<< xpos <<", y="<< ypos << ", z="<< 0 << std::endl;
    posNodes->Add (Vector (xpos, ypos, 0));
    fase += separacion_angular;
  }
  // Col 2:
  fase = 0;
  xpos = 0;
  ypos = 0;
  separacion_angular = 2*PI / n_nodes_col2;
  
  for (int i=0; i < n_nodes_col2; i++)
  {
    xpos = radio_col2 * cos (fase);
    ypos = radio_col2 * sin (fase);
    //std::clog << "node-"<<(i+1)<<", x="<< xpos <<", y="<< ypos << ", z="<< 0 << std::endl;
    posNodes->Add (Vector (xpos, ypos, altura));
    fase += separacion_angular;
  }

  // Col 3
  posNodes->Add (Vector (0, 0, altura*2));
  // Col 4
  posNodes->Add (Vector (0, 0, altura*3));
  
  // Col 5:
  fase = 0;
  xpos = 0;
  ypos = 0;
  separacion_angular = 2*PI / n_nodes_col5;
  
  for (int i=0; i < n_nodes_col5; i++)
  {
    xpos = radio_col5 * cos (fase);
    ypos = radio_col5 * sin (fase);
    //std::clog << "node-"<<(i+1)<<", xpos="<< xpos <<", ypos="<< ypos <<std::endl;
    posNodes->Add (Vector (xpos, ypos, altura*4));
    fase += separacion_angular;
  }
  // Col 6:
  fase = 0;
  xpos = 0;
  ypos = 0;
  separacion_angular = 2*PI / n_nodes_col6;
  
  for (int i=0; i < n_nodes_col6; i++)
  {
    xpos = radio_col6 * cos (fase);
    ypos = radio_col6 * sin (fase);
    //std::clog << "node-"<<(i+1)<<", xpos="<< xpos <<", ypos="<< ypos <<std::endl;
    posNodes->Add (Vector (xpos, ypos, altura*5));
    fase += separacion_angular;
  }

  // =====================================================
  
  MobilityHelper mobility;
  mobility.SetPositionAllocator (posNodes);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);
    
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
  
  lossModel->SetDefaultLoss (PathLoss * 2); // 160 para HT
  
  //columna 1
  lossModel->SetLoss (nodes.Get (0)->GetObject<MobilityModel> (), nodes.Get (1)->GetObject<MobilityModel> (), PathLoss);
  lossModel->SetLoss (nodes.Get (1)->GetObject<MobilityModel> (), nodes.Get (2)->GetObject<MobilityModel> (), PathLoss);
  lossModel->SetLoss (nodes.Get (2)->GetObject<MobilityModel> (), nodes.Get (0)->GetObject<MobilityModel> (), PathLoss);
  
  //columna 1 a 2
  lossModel->SetLoss (nodes.Get (0)->GetObject<MobilityModel> (), nodes.Get (3)->GetObject<MobilityModel> (), PathLoss);
  lossModel->SetLoss (nodes.Get (0)->GetObject<MobilityModel> (), nodes.Get (4)->GetObject<MobilityModel> (), PathLoss);
  lossModel->SetLoss (nodes.Get (1)->GetObject<MobilityModel> (), nodes.Get (3)->GetObject<MobilityModel> (), PathLoss);
  lossModel->SetLoss (nodes.Get (1)->GetObject<MobilityModel> (), nodes.Get (4)->GetObject<MobilityModel> (), PathLoss);
  lossModel->SetLoss (nodes.Get (2)->GetObject<MobilityModel> (), nodes.Get (3)->GetObject<MobilityModel> (), PathLoss);
  lossModel->SetLoss (nodes.Get (2)->GetObject<MobilityModel> (), nodes.Get (4)->GetObject<MobilityModel> (), PathLoss);

  //columna 2
  lossModel->SetLoss (nodes.Get (3)->GetObject<MobilityModel> (), nodes.Get (4)->GetObject<MobilityModel> (), PathLoss);

  //columna 2 a 3
  lossModel->SetLoss (nodes.Get (3)->GetObject<MobilityModel> (), nodes.Get (5)->GetObject<MobilityModel> (), PathLoss);
  lossModel->SetLoss (nodes.Get (4)->GetObject<MobilityModel> (), nodes.Get (5)->GetObject<MobilityModel> (), PathLoss);
  
  //columna 3 a 4
  lossModel->SetLoss (nodes.Get (5)->GetObject<MobilityModel> (), nodes.Get (6)->GetObject<MobilityModel> (), PathLoss);

  //columna 4 a 5
  lossModel->SetLoss (nodes.Get (6)->GetObject<MobilityModel> (), nodes.Get (7)->GetObject<MobilityModel> (), PathLoss);
  lossModel->SetLoss (nodes.Get (6)->GetObject<MobilityModel> (), nodes.Get (8)->GetObject<MobilityModel> (), PathLoss);

  //columna 5
  lossModel->SetLoss (nodes.Get (7)->GetObject<MobilityModel> (), nodes.Get (8)->GetObject<MobilityModel> (), PathLoss);

  //columna 5 a 6
  lossModel->SetLoss (nodes.Get (7)->GetObject<MobilityModel> (), nodes.Get (9)->GetObject<MobilityModel> (), PathLoss);
  lossModel->SetLoss (nodes.Get (7)->GetObject<MobilityModel> (), nodes.Get (10)->GetObject<MobilityModel> (), PathLoss);
  lossModel->SetLoss (nodes.Get (7)->GetObject<MobilityModel> (), nodes.Get (11)->GetObject<MobilityModel> (), PathLoss);
  lossModel->SetLoss (nodes.Get (8)->GetObject<MobilityModel> (), nodes.Get (9)->GetObject<MobilityModel> (), PathLoss);
  lossModel->SetLoss (nodes.Get (8)->GetObject<MobilityModel> (), nodes.Get (10)->GetObject<MobilityModel> (), PathLoss);
  lossModel->SetLoss (nodes.Get (8)->GetObject<MobilityModel> (), nodes.Get (11)->GetObject<MobilityModel> (), PathLoss);

  //columna 6
  lossModel->SetLoss (nodes.Get (9)->GetObject<MobilityModel> (), nodes.Get (10)->GetObject<MobilityModel> (), PathLoss);
  lossModel->SetLoss (nodes.Get (10)->GetObject<MobilityModel> (), nodes.Get (11)->GetObject<MobilityModel> (), PathLoss);
  lossModel->SetLoss (nodes.Get (11)->GetObject<MobilityModel> (), nodes.Get (9)->GetObject<MobilityModel> (), PathLoss);

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
  NetDeviceContainer nodeDevs;

  // Seteo el tipo de conectividad Wifi como 'Ad Hoc'
  wifiMac.SetType ("ns3::AdhocWifiMac");
  // Creo las interfaces wifi Ad Hoc en los nodos
  nodeDevs = wifi.Install (wifiPhy, wifiMac, nodes);

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
    tapBridge.Install (nodes.Get (i), nodeDevs.Get (i));
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
