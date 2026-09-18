/*
 * LTE simulator Ex2
 * 
 * extending ex1 
 *   - add per second data volumn output to the simulator
 *   - add activeDRB and ScheduledTTI to the per second trace
 *   - add a second UE and setup connect/disconnect events to observe the behaviour.
 */

// #include "ns3/buildings-helper.h"
#include "ns3/core-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include <iomanip>
#include <iostream>
#include <set>
#include <vector>

using namespace ns3;

static void
CountDlScheduledTti(std::vector<std::set<uint64_t>>* dlScheduledTtiBySecond,
                    uint32_t maxSecond,
                    std::string,
                    DlSchedulingCallbackInfo dlInfo)
{
	double nowSeconds = Simulator::Now().GetSeconds();
	uint32_t intervalSec = static_cast<uint32_t>(nowSeconds) + 1;
	if (intervalSec >= 1 && intervalSec <= maxSecond)
	{
		uint64_t ttiId = static_cast<uint64_t>(dlInfo.frameNo) * 10 +
		                 static_cast<uint64_t>(dlInfo.subframeNo);
		(*dlScheduledTtiBySecond)[intervalSec].insert(ttiId);
	}
}

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
	ueNodes.Create(2);

	MobilityHelper mobility;
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(enbNodes);
	mobility.Install(ueNodes);

	// BuildingsHelper::Install(enbNodes);
	// BuildingsHelper::Install(ueNodes);

	Ptr<MobilityModel> enbMobility = enbNodes.Get(0)->GetObject<MobilityModel>();
	Ptr<MobilityModel> ueMobility0 = ueNodes.Get(0)->GetObject<MobilityModel>();
	Ptr<MobilityModel> ueMobility1 = ueNodes.Get(1)->GetObject<MobilityModel>();
	enbMobility->SetPosition(Vector(0.0, 0.0, 0.0));
	ueMobility0->SetPosition(Vector(1000.0, 0.0, 0.0)); // 1 km from eNB
	ueMobility1->SetPosition(Vector(1000.0, 0.0, 0.0)); // 1 km from eNB

	NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(enbNodes);
	NetDeviceContainer ueDevs = lteHelper->InstallUeDevice(ueNodes);

	EpsBearer bearer(EpsBearer::GBR_CONV_VOICE);
	Simulator::Schedule(Seconds(1.0),
	                    [lteHelper, ueDev = ueDevs.Get(0), enbDev = enbDevs.Get(0), bearer]() {
		lteHelper->Attach(ueDev, enbDev);
		lteHelper->ActivateDataRadioBearer(ueDev, bearer);
	});
	Simulator::Schedule(Seconds(4.0),
	                    [lteHelper, ueDev = ueDevs.Get(1), enbDev = enbDevs.Get(0), bearer]() {
		lteHelper->Attach(ueDev, enbDev);
		lteHelper->ActivateDataRadioBearer(ueDev, bearer);
	});

	Simulator::Schedule(Seconds(8.0), [ueDev = ueDevs.Get(0)]() {
		ueDev->GetObject<LteUeNetDevice>()->GetNas()->Disconnect();
	});

	Simulator::Schedule(Seconds(9.0), [ueDev = ueDevs.Get(1)]() {
		ueDev->GetObject<LteUeNetDevice>()->GetNas()->Disconnect();
	});

	lteHelper->EnableRlcTraces();

	Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats();
	rlcStats->SetAttribute("StartTime", TimeValue(Seconds(0.0)));
	rlcStats->SetAttribute("EpochDuration", TimeValue(simTime + MilliSeconds(1)));

	const uint8_t firstDataLcid = 3;
	const uint8_t lastDataLcid = 10;
	const uint32_t numDataLcids = lastDataLcid - firstDataLcid + 1;
	const uint32_t sampleSeconds = static_cast<uint32_t>(simTime.GetSeconds());
	std::vector<uint64_t> dlCumulative(sampleSeconds + 1, 0);
	std::vector<uint64_t> ulCumulative(sampleSeconds + 1, 0);
	std::vector<uint32_t> dlActiveDrbEnb(sampleSeconds + 1, 0);
	std::vector<uint64_t> prevDlRxBytes(ueDevs.GetN() * numDataLcids, 0);
	std::vector<std::set<uint64_t>> dlScheduledTtiBySecond(sampleSeconds + 1);

	Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMap/*/LteEnbMac/DlScheduling",
	                MakeBoundCallback(&CountDlScheduledTti, &dlScheduledTtiBySecond, sampleSeconds));

	for (uint32_t sec = 1; sec <= sampleSeconds; ++sec)
	{
		Simulator::Schedule(Seconds(sec), [&, sec]() {
			uint64_t dlBytes = 0;
			uint64_t ulBytes = 0;
			uint32_t activeDrbCount = 0;
			for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
			{
				Ptr<LteUeNetDevice> ueDev = ueDevs.Get(i)->GetObject<LteUeNetDevice>();
				uint64_t imsi = ueDev->GetImsi();
				uint32_t activeDrbForUe = 0;
				for (uint8_t lcid = firstDataLcid; lcid <= lastDataLcid; ++lcid)
				{
					uint64_t currDlRxBytes = rlcStats->GetDlRxData(imsi, lcid);
					dlBytes += currDlRxBytes;
					ulBytes += rlcStats->GetUlRxData(imsi, lcid);
					size_t flowIndex = i * numDataLcids + (lcid - firstDataLcid);
					if (currDlRxBytes > prevDlRxBytes[flowIndex])
					{
						++activeDrbForUe;
					}
					prevDlRxBytes[flowIndex] = currDlRxBytes;
				}
				activeDrbCount += activeDrbForUe;
			}
			dlCumulative[sec] = dlBytes;
			ulCumulative[sec] = ulBytes;
			dlActiveDrbEnb[sec] = activeDrbCount;
		});
	}

	Simulator::Stop(simTime);
	Simulator::Run();

	uint64_t totalDlBytes = dlCumulative[sampleSeconds];
	uint64_t totalUlBytes = ulCumulative[sampleSeconds];

	Time simulatedTime = Simulator::Now();
	uint64_t simulatedTti = static_cast<uint64_t>(simulatedTime.GetMilliSeconds());

	std::cout << "\n===== LTE Simulation Summary =====\n";
	std::cout << std::left << std::setw(10) << "Time(s)" << std::setw(20) << "Downlink(MB,1s)"
	          << std::setw(18) << "Uplink(MB,1s)" << std::setw(20) << "DL Active DRB(eNB)"
	          << std::setw(18) << "Scheduled TTI" << "\n";
	std::cout << "--------------------------------------------------------------------------------\n";
	for (uint32_t sec = 1; sec <= sampleSeconds; ++sec)
	{
		double dlIntervalMb = static_cast<double>(dlCumulative[sec] - dlCumulative[sec - 1]) /
		                     (1024.0 * 1024.0);
		double ulIntervalMb = static_cast<double>(ulCumulative[sec] - ulCumulative[sec - 1]) /
		                     (1024.0 * 1024.0);
		uint32_t scheduledTtiCount = dlScheduledTtiBySecond[sec].size();
		std::cout << std::left << std::setw(10) << sec << std::fixed << std::setprecision(3)
		          << std::setw(20) << dlIntervalMb << std::setw(18) << ulIntervalMb
		          << std::setw(20) << dlActiveDrbEnb[sec] << std::setw(18) << scheduledTtiCount
		          << "\n";
	}

	std::cout << "--------------------------------------------------------------------------------\n";
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


