#include <iostream>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/packet.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("TesisChannelModelTest");

// Funcion principal
int 
main (int argc, char *argv[])
{
  // ==== SIMULATION PARAMETERS ====
  
  // Simulation itself
  double Duration = 3600;
  bool isTXRXLoggingEnabled = false;
  
  // Topology
  int n_nodes = 2;
  double Distance = 165.84;
  
  // Radio
  double TxPower = 20;
  double RxSensitivity = -101; // Artificial Thershold
  double NoiseFigure = 4;
  double RxGain = 5, TxGain=5; // Ganancias de 5dBi
  
  // Medium Access Control (MAC)
  int McsControl=0; // 802.11n
  int McsData=7; // 802.11n
  int MaxSsrc = 0;
  int MaxSlrc = 0;
  
  // Channel Model: Large scale fading + Small scale fading
    // Log-distance Path Loss Model (Large scale fading):
  double PLE = 2.03; // Path Loss Exponent
  double ReferenceDistance = 50; // Del paper 115
  double ReferenceLoss = 90.42; // 90.42; // Del paper 115 !!!!!!!!!!!!!!!!!!!!!!!!!!
  double Shadowing_mu = 0; // Shadowing Mean (Lo pondre en 0 para considerar -90 dB de referencePathLoss)
  double Shadowing_sigma = 1.335; // Shadowing Standard Deviation
    // Rician Fading (Small scale fading)
  double Kfactor_mu = 2.63;
  double Kfactor_sigma = 3.82;

  // ====== SIMULATION COMMANDS ======
  CommandLine cmd (__FILE__);
  cmd.AddValue ("Duration", "Tiempo total de simulacion (segundos): default=", Duration);
  cmd.AddValue ("EnableMACLogs","Activar logs de paquetes en la capa MAC: default=", isTXRXLoggingEnabled);
  
  cmd.AddValue ("n_nodes", "Cantidad de nodos: default=", n_nodes);
  cmd.AddValue ("Distance", "Distancia entre nodos (metros): default=", Distance);
  
  cmd.AddValue ("TxPower", "Potencia de transmision (dBm): default=", TxPower);
  cmd.AddValue ("RxSensitivity", "Limite artificial para recepcionar un paquete (dBm): default=", RxSensitivity);
  
  cmd.AddValue ("PLE", " Path Loss Exponent: default=0",PLE);
  cmd.AddValue ("Shadowing_mu","Media de la variable normal de atenuacion : default=",Shadowing_mu);
  cmd.AddValue ("Shadowing_sigma","Desviacion estandar de la variable normal de atenuacion",Shadowing_sigma); 
  
  cmd.AddValue ("Kfactor_mu","Media de la variable normal de atenuacion : default=",Kfactor_mu);
  cmd.AddValue ("Kfactor_sigma","Desviacion estandar de la variable normal de atenuacion : default=",Kfactor_sigma);
  cmd.Parse (argc, argv);
  
  // ===== Set Real time simulation =====
  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  // ===== Create nodes =====
  NodeContainer nodes;
  nodes.Create (n_nodes);
    
  // Seteo el estandar Wifi
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211n_5GHZ);

  // ======= CONFIGURO EL REMOTE STATION MANAGER ======
  WifiMode nonunicastmode = WifiModeFactory::CreateWifiMcs ("HtMcs7", 7, WIFI_MOD_CLASS_HT);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                  "DataMode",StringValue ("HtMcs" + std::to_string(McsData)),
                                  "ControlMode",StringValue ("HtMcs" + std::to_string(McsControl)),
                                  "MaxSsrc", UintegerValue (MaxSsrc),
                                  "MaxSlrc", UintegerValue (MaxSlrc),
                                  "NonUnicastMode", WifiModeValue (nonunicastmode));

  // #============ CANAL AIRE AIRE ============
  // #Seteo los parametros del Radio
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");

  // #Large Scale Fading (Modelo Log Distance Path LossModel)
  wifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel",
                                "Exponent", DoubleValue(PLE),
                                "ReferenceDistance", DoubleValue(ReferenceDistance),
                                "ReferenceLoss", DoubleValue(ReferenceLoss));
  wifiChannel.AddPropagationLoss ("ns3::LogNormalShadowing",
                            "Mu", DoubleValue (Shadowing_mu),
                            "Sigma", DoubleValue (Shadowing_sigma));
  
  // #Small Scale Fading (Modelo Rician Fading)
  //wifiChannel.AddPropagationLoss ("ns3::RicianFading",
  //                          "KfactorMu", DoubleValue (Kfactor_mu),
  //                          "KfactorSigma", DoubleValue (Kfactor_sigma));
  
  //Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel> ();
  //channel->SetPropagationDelayModel (CreateObject<ConstantSpeedPropagationDelayModel> ());
  //Ptr<LogDistancePropagationLossModel> log = CreateObject<LogDistancePropagationLossModel> ();
  //channel->SetPropagationLossModel (log);

  // ==========================================

  // Configuro los parametros del Radio:
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ()); // En vez de wifiChannel.Create le paso un Ptr
  wifiPhy.Set ("TxPowerStart",DoubleValue (TxPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (TxPower));
  wifiPhy.Set ("RxSensitivity", DoubleValue (RxSensitivity));
  wifiPhy.Set ("RxNoiseFigure", DoubleValue(NoiseFigure));
  wifiPhy.Set ("RxGain", DoubleValue(RxGain));
  wifiPhy.Set ("TxGain", DoubleValue(TxGain));
//  wifiPhy.Set ("ShortGuardEnabled", BooleanValue(true));

  // Configuro el tipo de conectividad Wifi como 'Ad Hoc'
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");

  // Creo las interfaces wifi Ad Hoc en los nodos
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, nodes);

  // ============ MODIFICACIONES A LA INTERFAZ INALAMBRICA ===================
  Ptr<NetDevice> dev;
  Ptr<WifiNetDevice> wifi_dev;
  Ptr<AdhocWifiMac> adhoc_mac;

  // Nodos
  for (int i=0 ; i < n_nodes ; i++) {
    dev = nodes.Get(i)->GetDevice(0);
    wifi_dev = DynamicCast<WifiNetDevice>(dev);
    // Disable A-MPDU and A-MSDU
    wifi_dev->GetMac()->SetAttribute("VO_MaxAmpduSize", UintegerValue (0));
    wifi_dev->GetMac()->SetAttribute("VO_MaxAmsduSize", UintegerValue (0));
    // Set timeslot to 9 useg
    wifi_dev->GetMac()->SetSlot(MicroSeconds(9));
    // Activate RX Logging on Mac Low
    adhoc_mac = DynamicCast<AdhocWifiMac> (wifi_dev->GetMac());
    if (isTXRXLoggingEnabled) {
      adhoc_mac->EnableMacLowLogging();
    }
  }
    //Ptr<HtConfiguration> htconf = CreateObject<HtConfiguration> ();
    //htconf->SetShortGuardIntervalSupported (true);
    //wifi_dev->SetHtConfiguration(htconf);
  // ==============================================================
  
  // ============= Activate TX Logging on YansWifiChannel =================
  if (isTXRXLoggingEnabled) {
    Ptr<YansWifiChannel> yanschannel = DynamicCast<YansWifiChannel> (wifi_dev->GetChannel());
    yanschannel->SetLogActivate(true);
  }
  // =========================================================================

  // Ubicacion de los nodos
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (Distance, 0.0, 0.0));
    
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);

  // Asociacion con los containers
  TapBridgeHelper tapBridge;
  tapBridge.SetAttribute ("Mode", StringValue ("UseLocal")); // Nodo se conecta a un TAP existente
  tapBridge.SetAttribute ("DeviceName", StringValue ("tap-1"));
  tapBridge.Install (nodes.Get (0), devices.Get (0));
  tapBridge.SetAttribute ("DeviceName", StringValue ("tap-2"));
  tapBridge.Install (nodes.Get (1), devices.Get (1));

  // Fin de los eventos de la simulacion
  Simulator::Stop (Seconds (Duration));
  
  std::clog << "Simulation Started" << std::endl;
  Simulator::Run ();
  Simulator::Destroy ();
  std::clog << "Simulation Finished" << std::endl;
  return 0;
}
