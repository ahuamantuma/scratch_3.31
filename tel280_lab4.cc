#include <iostream>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/packet.h"

#define PI 3.14159265

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("Tel280Lab4");

// Funcion principal
int 
main (int argc, char *argv[])
{
  // Simulation:
  bool isInfra = false; // false: adhoc
  int n_stas = 4;
  int n_ap = 1;
  double Duration = 3600;
  double Distance = 100;
  double PathLoss = 80;
  //bool isTXRXLoggingEnabled = false;
  bool isHiddenTerminalEnabled = true;
  // Radio:
  double TxPower = 20;
  double RxSensitivity = -101;
  int HtMcsControl=0;
  int HtMcsData=7;
  int MaxSsrc = 0;
  int MaxSlrc = 0;
  double NoiseFigure = 0; // Lo dejamos sin ruido
  double RxGain = 0, TxGain=0;
  int RtsCtsThr = 65500; // 50: RTS activado

  // Commands
  CommandLine cmd (__FILE__);
  cmd.AddValue ("n_stas", "Cantidad de nodos: default=", n_stas);
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
  NodeContainer stas;
  NodeContainer ap;
  stas.Create (n_stas);
  ap.Create (n_ap);
  
  
  // ======= Logica para la ubicacion de los nodos ======= 
  // nodos ubicados como un anillo, radio = Distance
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> posStas = CreateObject<ListPositionAllocator> ();
  
  int radio = Distance;
  double fase = 0;
  double xpos = 0;
  double ypos = 0;
  double separacion_angular = 2*PI / n_stas;

  for (int i=0; i < n_stas; i++)
  {
    xpos = radio * cos (fase);
    ypos = radio * sin (fase);
    //std::clog << "node-"<<(i+1)<<", xpos="<< xpos <<", ypos="<< ypos <<std::endl;
    posStas->Add (Vector (xpos, ypos, 0.0));
    fase += separacion_angular;
  }
  // =====================================================

  mobility.SetPositionAllocator (posStas);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (stas);

  // Ubicacion del AP
  Ptr<ListPositionAllocator> posAp = CreateObject<ListPositionAllocator> ();
  posAp->Add (Vector (0.0, 0.0, 0.0));
  
  mobility.SetPositionAllocator (posAp);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (ap);
    
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
    Ptr<MatrixPropagationLossModel> lossModel = CreateObject<MatrixPropagationLossModel> ();
    if (isHiddenTerminalEnabled) {
      lossModel->SetDefaultLoss (PathLoss * 2); // 160 para HT
    }
    else {
      lossModel->SetDefaultLoss (PathLoss); // 80 para no HT
    }
      // Seteo todos los enlaces con el AP en 80
    for (int i=0; i < n_stas; i++)
    {
      lossModel->SetLoss (ap.Get (0)->GetObject<MobilityModel> (), stas.Get (i)->GetObject<MobilityModel> (), PathLoss);
    }
  
  wifiChannel->SetPropagationLossModel (lossModel);  
  
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

  if (isInfra) {
    // Create Ssid
    Ssid ssid = Ssid ("WifiAp");
    
    // Setup stas
    wifiMac.SetType ("ns3::StaWifiMac",
                     "ActiveProbing", BooleanValue (false),
                     "Ssid", SsidValue (ssid));
    staDevs = wifi.Install (wifiPhy, wifiMac, stas);
    
    // Setup ap
    wifiMac.SetType ("ns3::ApWifiMac",
                     "Ssid", SsidValue (ssid));
    apDev = wifi.Install (wifiPhy, wifiMac, ap);
  }
  else{
    // Seteo el tipo de conectividad Wifi como 'Ad Hoc'
    wifiMac.SetType ("ns3::AdhocWifiMac");
    // Creo las interfaces wifi Ad Hoc en los nodos
    staDevs = wifi.Install (wifiPhy, wifiMac, stas);
    apDev = wifi.Install (wifiPhy, wifiMac, ap);
  }

  // Modificaciones especiales
  // =========================================================================
  Ptr<NetDevice> dev;
  Ptr<WifiNetDevice> wifi_dev;
  Ptr<InfrastructureWifiMac> infra_mac;
  
  // Sta 1
  for (int i=0 ; i<n_stas ; i++ ) {
    dev = stas.Get(i)->GetDevice(0);
    wifi_dev = DynamicCast<WifiNetDevice>(dev);
      // Disable A-MPDU and A-MSDU
    wifi_dev->GetMac()->SetAttribute("BE_MaxAmpduSize", UintegerValue (0));
    wifi_dev->GetMac()->SetAttribute("BE_MaxAmsduSize", UintegerValue (0));
      // Set timeslot to 9 useg
    wifi_dev->GetMac()->SetSlot(MicroSeconds(9));
      // Activate RX Logging on Mac Low
    //if (isTXRXLoggingEnabled) {
    //  infra_mac = DynamicCast<InfrastructureWifiMac> (wifi_dev->GetMac());
    //  infra_mac->EnableMacLowLogging();
    //}
  }

  // AP
  dev = ap.Get(0)->GetDevice(0);
  wifi_dev = DynamicCast<WifiNetDevice>(dev);
      // Disable A-MPDU and A-MSDU
    wifi_dev->GetMac()->SetAttribute("BE_MaxAmpduSize", UintegerValue (0));
    wifi_dev->GetMac()->SetAttribute("BE_MaxAmsduSize", UintegerValue (0));
      // Set timeslot to 9 useg
    wifi_dev->GetMac()->SetSlot(MicroSeconds(9));
      // Activate RX Logging on Mac Low
    //Ptr<ApWifiMac> ap_mac = DynamicCast<ApWifiMac> (wifi_dev->GetMac());
    //if (isTXRXLoggingEnabled) {
    //  infra_mac = DynamicCast<InfrastructureWifiMac> (wifi_dev->GetMac());
    //  infra_mac->EnableMacLowLogging();
    //}

  // Channel
  // Activate TX Logging on YansWifiChannel
  //if (isTXRXLoggingEnabled) {
  //  Ptr<YansWifiChannel> yanschannel = DynamicCast<YansWifiChannel> (wifi_dev->GetChannel());
  //  yanschannel->SetLogActivate(true);
  //}
  // =========================================================================

  // Asociacion con los containers
  TapBridgeHelper tapBridge;
  tapBridge.SetAttribute ("Mode", StringValue ("UseLocal")); // Nodo se conecta a un TAP existente
  
  // Stations
  for (int i=0; i < n_stas; i++)
  {
    tapBridge.SetAttribute ("DeviceName", StringValue ("tap-" + std::to_string(i+1) ));
    tapBridge.Install (stas.Get (i), staDevs.Get (i));
  }
  
  // Ap:
  tapBridge.SetAttribute ("DeviceName", StringValue ("tap-" + std::to_string(n_stas + 1) )); // AP
  tapBridge.Install (ap.Get (0), apDev.Get (0));

  // Fin de los eventos de la simulacion
  Simulator::Stop (Seconds (Duration));

  //NS_LOG_UNCOND ("Simulation Started");
  std::clog << "Simulation Started" << std::endl;

  Simulator::Run ();
  Simulator::Destroy ();
  std::clog << "Simulation Finished" << std::endl;
  return 0;
}
