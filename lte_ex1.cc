/*
 * LTE simulator Ex1
 * a fixed 1 km 1-UE/1-eNB LTE scenario where connection starts at 1 second, runs until end of the simulation.
 * default to 10 seconds.
 * 
 * Output summarizes UL/DL data volume plus time/TTI.
 * 
 */

// #include "ns3/buildings-helper.h"
#include "ns3/core-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include <iomanip>
#include <iostream>

using namespace ns3;

int
main(int argc, char* argv[])
{
	Time simTime = Seconds(10);

	CommandLine cmd(__FILE__);
	cmd.AddValue("simTime", "Total duration of the simulation", simTime);
	cmd.Parse(argc, argv);

	Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();

	NodeContainer enbNodes;
	NodeContainer ueNodes;
	enbNodes.Create(1);
	ueNodes.Create(1);

	MobilityHelper mobility;
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(enbNodes);
	mobility.Install(ueNodes);

	// BuildingsHelper::Install(enbNodes);
	// BuildingsHelper::Install(ueNodes);

	Ptr<MobilityModel> enbMobility = enbNodes.Get(0)->GetObject<MobilityModel>();
	Ptr<MobilityModel> ueMobility = ueNodes.Get(0)->GetObject<MobilityModel>();
	enbMobility->SetPosition(Vector(0.0, 0.0, 0.0));
	ueMobility->SetPosition(Vector(1000.0, 0.0, 0.0)); // 1 km from eNB

	NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(enbNodes);
	NetDeviceContainer ueDevs = lteHelper->InstallUeDevice(ueNodes);

	EpsBearer bearer(EpsBearer::GBR_CONV_VOICE);
	Simulator::Schedule(Seconds(1.0),
	                    [lteHelper, ueDev = ueDevs.Get(0), enbDev = enbDevs.Get(0), bearer]() {
		lteHelper->Attach(ueDev, enbDev);
		lteHelper->ActivateDataRadioBearer(ueDev, bearer);
	});

	lteHelper->EnableRlcTraces();

	Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats();
	rlcStats->SetAttribute("StartTime", TimeValue(Seconds(0.0)));
	rlcStats->SetAttribute("EpochDuration", TimeValue(simTime + MilliSeconds(1)));

	Simulator::Stop(simTime);
	Simulator::Run();

	uint64_t totalDlBytes = 0;
	uint64_t totalUlBytes = 0;
	const uint8_t firstDataLcid = 3;
	const uint8_t lastDataLcid = 10;

	for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
	{
		Ptr<LteUeNetDevice> ueDev = ueDevs.Get(i)->GetObject<LteUeNetDevice>();
		uint64_t imsi = ueDev->GetImsi();
		for (uint8_t lcid = firstDataLcid; lcid <= lastDataLcid; ++lcid)
		{
			totalDlBytes += rlcStats->GetDlRxData(imsi, lcid);
			totalUlBytes += rlcStats->GetUlRxData(imsi, lcid);
		}
	}

	Time simulatedTime = Simulator::Now();
	uint64_t simulatedTti = static_cast<uint64_t>(simulatedTime.GetMilliSeconds());

	std::cout << "\n===== LTE Simulation Summary =====\n";
	std::cout << "Total downlinked data: " << totalDlBytes/1024/1024 << " Megabytes\n";
	std::cout << "Total uplinked data:   " << totalUlBytes/1024/1024 << " Megabytes\n";
	std::cout << std::fixed << std::setprecision(3)
	          << "Simulated time:        " << simulatedTime.GetSeconds() << " s\n";
	std::cout << "Simulated TTI:         " << simulatedTti << " (1 TTI = 1 ms)\n";
	std::cout << "==================================\n";

	Simulator::Destroy();

	return 0;
}


