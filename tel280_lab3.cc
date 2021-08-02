#include <iostream>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/packet.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("tel280-lab3");

// Trace Sink for MonitorSnifferRx
void
SinkRxSnifferNode1 (std::string context, Ptr<const Packet> packet, 
    uint16_t FreqMhz, WifiTxVector txVector, MpduInfo ampdu, SignalNoiseDbm snr)
{ 
  NS_LOG_UNCOND ("Node 1 Rx: Signal="<<snr.signal<<"dBm, Noise="<<snr.noise
      <<"dBm, Freq="<< FreqMhz<<"Mhz, ChannelWidth="
      <<txVector.GetChannelWidth()<<"Mhz, GuardInterval="
      <<txVector.GetGuardInterval()<<"ns"); 
}
void
SinkRxSnifferNode2 (std::string context, Ptr<const Packet> packet, 
    uint16_t FreqMhz, WifiTxVector txVector, MpduInfo ampdu, SignalNoiseDbm snr)
{ 
  NS_LOG_UNCOND ("Node 2 Rx: Signal="<<snr.signal<<"dBm, Noise="<<snr.noise
      <<"dBm, Freq="<< FreqMhz<<"Mhz, ChannelWidth="
      <<txVector.GetChannelWidth()<<"Mhz, GuardInterval="
      <<txVector.GetGuardInterval()<<"ns"); 
}

// Funcion principal
int 
main (int argc, char *argv[])
{
  int n_nodes = 2;
  double Duration = 3600;
  double Distance = 30;
  double TxPower = 10;
  double RxSensitivity = -101;
  int HtMcsControl=0;
  int HtMcsData=0;
  int FadingModel=0;
  //double beta = 1.0;
  bool RxSnifferNode1 = false, RxSnifferNode2 = false;
  int PathLoss = 0;
  double FixedRssLoss = -60;
  //Parameters for Aironet 3600 Access Point
  double NoiseFigure = 0; // Lo dejamos sin ruido
  double RxGain = 2, TxGain=2;
  
  // Parameters
  CommandLine cmd (__FILE__);
  cmd.AddValue ("Duration", "Tiempo total de simulacion (segundos): default=600", Duration);
  cmd.AddValue ("Distance", "Distancia entre nodos (metros): default=10", Distance);
  cmd.AddValue ("TxPower", "Potencia de transmision (dBm): default=10", TxPower);
  cmd.AddValue ("RxSensitivity", "Sensibilidad de repecion (dBm): default=-101", RxSensitivity);
  cmd.AddValue ("PathLoss", "Modelo de PathLoss, [0:Fijo, 1:Friis]: default=0",PathLoss);
  cmd.AddValue ("FixedRss","Potencia de recepcion fija independiente del transmit power (solo si se eligio ese modelo): default=-60",FixedRssLoss);
  cmd.AddValue ("HtMcsControl","Esquema de modulacion y codificacion para el trafico de control en ambos nodos: default=0",HtMcsControl); 
  cmd.AddValue ("HtMcsData","Esquema de modulacion y codificacion para el trafico de datos en ambos nodos: default=0",HtMcsData);
  cmd.AddValue ("FadingModel","Modelo de Fading para el canal, [0:None, 1:Rayleigh]: default=0",FadingModel);
  cmd.AddValue ("RxSnifferNode1","Modo Sniffer para el nodo 1: default=false", RxSnifferNode1);
  cmd.AddValue ("RxSnifferNode2","Modo Sniffer para el nodo 2: default=false", RxSnifferNode2);
  cmd.Parse (argc, argv);
    
  std::string HtMcsControlMode = "HtMcs" + std::to_string(HtMcsControl);
  std::string HtMcsDataMode = "HtMcs" + std::to_string(HtMcsData);
    
  // Set Real time simulation 
  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  NodeContainer nodes;
  nodes.Create (n_nodes);
    
  // Seteo el estandar Wifi
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211n_2_4GHZ);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                  "DataMode",StringValue (HtMcsDataMode),
                                  "ControlMode",StringValue (HtMcsControlMode));

  // Seteo los parametros de la capa fisica Wifi y potencia de transmision
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  
  if (PathLoss==0){
    wifiChannel.AddPropagationLoss ("ns3::FixedRssLossModel",
                                "Rss", DoubleValue(FixedRssLoss));
  }
  else if (PathLoss==1){
    wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel",
                                "Frequency", DoubleValue(2.412e9));
  }
  else {
    NS_LOG_UNCOND ("Modelo de PathLos no valido");
    return 0;
  }

  if (FadingModel==0) {
    // Sin Fading
  }
  else if (FadingModel==1) {
    wifiChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel",
                                "Distance1", DoubleValue(1000),
                                "Distance2", DoubleValue(1001),
                                "m0", DoubleValue(1),
                                "GammaRv", StringValue("ns3::GammaRandomVariable") );
  }
  else {
    NS_LOG_UNCOND ("Modelo de fading no valido");
    return 0;
  }
  
  wifiPhy.SetChannel (wifiChannel.Create ());
  wifiPhy.Set ("TxPowerStart",DoubleValue (TxPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (TxPower));
  wifiPhy.Set ("RxSensitivity", DoubleValue (RxSensitivity));
  wifiPhy.Set ("RxNoiseFigure", DoubleValue(NoiseFigure));
  wifiPhy.Set ("RxGain", DoubleValue(RxGain));
  wifiPhy.Set ("TxGain", DoubleValue(TxGain));

  // Seteo el tipo de conectividad Wifi como 'Ad Hoc'
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");

  // Creo las interfaces wifi Ad Hoc en los nodos
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, nodes);

  // Deshabilitamos las agregaciones y seteamos el timeslot en 9useg
  // =========================================================================
  // Nodo 1
  Ptr<NetDevice> dev = nodes.Get(0)->GetDevice(0);
  Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice>(dev);
  // Disable A-MPDU and A-MSDU
  wifi_dev->GetMac()->SetAttribute("BE_MaxAmpduSize", UintegerValue (0));
  wifi_dev->GetMac()->SetAttribute("BE_MaxAmsduSize", UintegerValue (0));
  
  wifi_dev->GetMac()->SetSlot(MicroSeconds(9));

  // Nodo 2
  dev = nodes.Get(1)->GetDevice(0);
  wifi_dev = DynamicCast<WifiNetDevice>(dev);
  // Disable A-MPDU and A-MSDU
  wifi_dev->GetMac()->SetAttribute("BE_MaxAmpduSize", UintegerValue (0));
  wifi_dev->GetMac()->SetAttribute("BE_MaxAmsduSize", UintegerValue (0));
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

  NS_LOG_UNCOND ("Simulation Started");
    
  if (RxSnifferNode1) {
  // Trace Sink Callback MakeCallback
    std::ostringstream oss;
    oss << "/NodeList/"<< nodes.Get (0)->GetId()
         << "/DeviceList/*"
         << "/$ns3::WifiNetDevice/Phy"                                           
         << "/MonitorSnifferRx";

    Config::Connect (oss.str(), MakeCallback (&SinkRxSnifferNode1));
  }
  
  if (RxSnifferNode2) {
  // Trace Sink Callback MakeCallback
    std::ostringstream oss;
    oss << "/NodeList/"<< nodes.Get (1)->GetId()
         << "/DeviceList/*"
         << "/$ns3::WifiNetDevice/Phy"                                           
         << "/MonitorSnifferRx";

    Config::Connect (oss.str(), MakeCallback (&SinkRxSnifferNode2));
  }

  Simulator::Run ();
  Simulator::Destroy ();
  NS_LOG_UNCOND ("Simulation Finished");
  return 0;
}
