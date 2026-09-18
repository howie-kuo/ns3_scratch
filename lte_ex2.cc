/*
 * LTE simulator Ex2
 * 
 * extending ex1 
 *   - add per second data volumn output to the simulator
 */

#include "ns3/buildings-helper.h"
#include "ns3/core-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include <iomanip>
#include <iostream>
#include <vector>

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

	BuildingsHelper::Install(enbNodes);
	BuildingsHelper::Install(ueNodes);

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

	const uint8_t firstDataLcid = 3;
	const uint8_t lastDataLcid = 10;
	const uint32_t sampleSeconds = static_cast<uint32_t>(simTime.GetSeconds());
	std::vector<uint64_t> dlCumulative(sampleSeconds + 1, 0);
	std::vector<uint64_t> ulCumulative(sampleSeconds + 1, 0);

	for (uint32_t sec = 1; sec <= sampleSeconds; ++sec)
	{
		Simulator::Schedule(Seconds(sec), [&, sec]() {
			uint64_t dlBytes = 0;
			uint64_t ulBytes = 0;
			for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
			{
				Ptr<LteUeNetDevice> ueDev = ueDevs.Get(i)->GetObject<LteUeNetDevice>();
				uint64_t imsi = ueDev->GetImsi();
				for (uint8_t lcid = firstDataLcid; lcid <= lastDataLcid; ++lcid)
				{
					dlBytes += rlcStats->GetDlRxData(imsi, lcid);
					ulBytes += rlcStats->GetUlRxData(imsi, lcid);
				}
			}
			dlCumulative[sec] = dlBytes;
			ulCumulative[sec] = ulBytes;
		});
	}

	Simulator::Stop(simTime);
	Simulator::Run();

	uint64_t totalDlBytes = dlCumulative[sampleSeconds];
	uint64_t totalUlBytes = ulCumulative[sampleSeconds];

	Time simulatedTime = Simulator::Now();
	uint64_t simulatedTti = static_cast<uint64_t>(simulatedTime.GetMilliSeconds());

	std::cout << "\n===== LTE Simulation Summary =====\n";
	std::cout << std::left << std::setw(10) << "Time(s)" << std::setw(22)
	          << "Downlink(MB,1s)" << std::setw(22) << "Uplink(MB,1s)" << "\n";
	std::cout << "------------------------------------------------------\n";
	for (uint32_t sec = 1; sec <= sampleSeconds; ++sec)
	{
		double dlIntervalMb = static_cast<double>(dlCumulative[sec] - dlCumulative[sec - 1]) /
		                     (1024.0 * 1024.0);
		double ulIntervalMb = static_cast<double>(ulCumulative[sec] - ulCumulative[sec - 1]) /
		                     (1024.0 * 1024.0);
		std::cout << std::left << std::setw(10) << sec << std::fixed << std::setprecision(3)
		          << std::setw(22) << dlIntervalMb << std::setw(22) << ulIntervalMb << "\n";
	}

	std::cout << "------------------------------------------------------\n";
	std::cout << std::fixed << std::setprecision(3)
	          << "Total downlinked data: "
	          << static_cast<double>(totalDlBytes) / (1024.0 * 1024.0) << " Megabytes\n";
	std::cout << "Total uplinked data:   "
	          << static_cast<double>(totalUlBytes) / (1024.0 * 1024.0) << " Megabytes\n";
	std::cout << std::fixed << std::setprecision(3)
	          << "Simulated time:        " << simulatedTime.GetSeconds() << " s\n";
	std::cout << "Simulated TTI:         " << simulatedTti << " (1 TTI = 1 ms)\n";
	std::cout << "==================================\n";

	Simulator::Destroy();

	return 0;
}


