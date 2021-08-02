#include <iostream>
//#include <fstream>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/packet.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("tel280-emulation-lab2");

// Trace Sink for MonitorSnifferRx
void
SinkRxSnifferNode1 (std::string context, Ptr<const Packet> packet, 
    uint16_t FreqMhz, WifiTxVector txVector, MpduInfo ampdu, SignalNoiseDbm snr)
{ 
  NS_LOG_UNCOND ("Node 1 Rx: Signal="<<snr.signal<<"dBm, Noise="<<snr.noise
      <<"dBm, Freq="
      << FreqMhz<<"Mhz, ChannelWidth="
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
  double Duration = 600;
  double Distance = 290;
  double TxPower = 10;
  double RxSensitivity = -90;
  int HtMcsControl=0;
  int HtMcsData=0;
  int FadingModel=0;
  //double beta = 1.0;
  bool RxSnifferNode1 = false, RxSnifferNode2 = false;
  int PathLossModel = 0;
  double FixedRssLoss = -60;
  //Parameters for Aironet 3600 Access Point
  double NoiseFigure = 1; // Lo dejamos sin ruido
  double RxGain = 2, TxGain=2;
  
  // Parameters
  CommandLine cmd (__FILE__);
  cmd.AddValue ("Duration", "Tiempo total de simulacion (segundos): default=600", Duration);
  cmd.AddValue ("Distance", "Distancia entre nodos (metros): default=10", Distance);
  cmd.AddValue ("TxPower", "Potencia de transmision (dBm): default=10", TxPower);
  cmd.AddValue ("RxSensitivity", "Sensibilidad de repecion (dBm): default=-90", RxSensitivity);
  cmd.AddValue ("PathLossModel", "Modelo de PathLoss, [0:Fijo, 1:Friis]: default=0",PathLossModel);
  cmd.AddValue ("FixedRss","Potencia de recepcion fija independiente del transmit power (solo si se eligio ese modelo): default=-60",FixedRssLoss);
  cmd.AddValue ("HtMcsControl","Esquema de modulacion y codificacion para el trafico de control en ambos nodos: default=0",HtMcsControl); 
  cmd.AddValue ("HtMcsData","Esquema de modulacion y codificacion para el trafico de datos en ambos nodos: default=0",HtMcsData);
  //cmd.AddValue ("WithFading","Modelo de Fading habilitado: default=false",WithFading);
  cmd.AddValue ("FadingModel","Modelo de Fading para el canal, [0:None, 1:Rayleigh]: default=0",FadingModel);
  //cmd.AddValue ("StdDeviation","Desviacion estandar de la variable aleatoria del modelo de propagacion: default=1.0", beta);
  cmd.AddValue ("RxSnifferNode1","Modo Sniffer para el nodo 1: default=false", RxSnifferNode1);
  cmd.AddValue ("RxSnifferNode2","Modo Sniffer para el nodo 2: default=false", RxSnifferNode2);
  cmd.Parse (argc, argv);
    
  std::string HtMcsControlMode = "HtMcs" + std::to_string(HtMcsControl);
  std::string HtMcsDataMode = "HtMcs" + std::to_string(HtMcsData);
  //LogComponentEnable ("TapBridgeApiWifiNetDevice", LOG_LEVEL_ALL);

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
  
  if (PathLossModel==0){
    wifiChannel.AddPropagationLoss ("ns3::FixedRssLossModel",
                                "Rss", DoubleValue(FixedRssLoss));
  }
  else if (PathLossModel==1){
    wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
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

  // Para el comportamiento del Access Point Aironet en la experiencia de
  // inalambricas. La sensitividad de recepcion tambien varia con el MCS 
  // del DataMode
           
  // Config Path de WifiPhy
  std::ostringstream osconfphy;
  osconfphy << "/NodeList/*"
            << "/DeviceList/*"
            << "/$ns3::WifiNetDevice/Phy/RxSensitivity";
                                           
    if (HtMcsDataMode.compare("HtMcs0")==0) {
       ns3::Config::Set (osconfphy.str(), ns3::DoubleValue(-90));
    }
    else if (HtMcsDataMode.compare("HtMcs1")==0) {
       ns3::Config::Set (osconfphy.str(), ns3::DoubleValue(-87));
    }
    else if (HtMcsDataMode.compare("HtMcs2")==0) {
       ns3::Config::Set (osconfphy.str(), ns3::DoubleValue(-84));
    }
    else if (HtMcsDataMode.compare("HtMcs3")==0) {
       ns3::Config::Set (osconfphy.str(), ns3::DoubleValue(-82));
    }
    else if (HtMcsDataMode.compare("HtMcs4")==0) {
       ns3::Config::Set (osconfphy.str(), ns3::DoubleValue(-79));
    }
    else if (HtMcsDataMode.compare("HtMcs5")==0) {
       ns3::Config::Set (osconfphy.str(), ns3::DoubleValue(-74));
    }
    else if (HtMcsDataMode.compare("HtMcs6")==0) {
       ns3::Config::Set (osconfphy.str(), ns3::DoubleValue(-72));
    }
    else if (HtMcsDataMode.compare("HtMcs7")==0) {
       ns3::Config::Set (osconfphy.str(), ns3::DoubleValue(-71));
    }
    else {
    }
  
  NS_LOG_UNCOND ("Simulation Started");
    
  //wifiPhy.EnablePcap ("emulation_propagationloss", devices.Get(0));
  //wifiPhy.EnablePcap ("emulation_propagationloss", devices.Get(1));
    
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
