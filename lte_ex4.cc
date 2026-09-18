/*
 * LTE simulator Ex4
 * 
 * extending ex3
 *   - modify to collect per-TTI statistics using the tracing API
 */

// #include "ns3/buildings-helper.h"
#include "ns3/core-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <set>
#include <vector>

using namespace ns3;

class LtePerTtiTraceCollector
{
  public:
	enum class OutputFormat
	{
		TABLE,
		CSV
	};

	class FormattedView
	{
	  public:
		FormattedView(const LtePerTtiTraceCollector& collector, OutputFormat format)
			: m_collector(collector),
			  m_format(format)
		{
		}

		const LtePerTtiTraceCollector& GetCollector() const
		{
			return m_collector;
		}

		OutputFormat GetFormat() const
		{
			return m_format;
		}

	  private:
		const LtePerTtiTraceCollector& m_collector;
		OutputFormat m_format;
	};

	struct PerTtiStats
	{
		bool isScheduledTti = false;
		uint64_t dlRxBytes = 0;
		uint64_t ulRxBytes = 0;
		std::set<uint64_t> dlActiveDrbKeys;
		uint64_t mcsSum = 0;
		uint64_t mcsCount = 0;
	};

	explicit LtePerTtiTraceCollector(uint64_t maxTti)
		: m_maxTti(maxTti),
		  m_perTtiStats(maxTti + 1)
	{
	}

	void Connect()
	{
		Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMap/*/LteEnbMac/DlScheduling",
						MakeCallback(&LtePerTtiTraceCollector::CollectDlSchedulingPerTti, this));
		Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/DrbCreated",
						MakeCallback(&LtePerTtiTraceCollector::ConnectDlRxTraceOnDrbCreated, this));
	}

	const std::vector<PerTtiStats>& GetPerTtiStats() const
	{
		return m_perTtiStats;
	}

	uint64_t GetTotalDlBytes() const
	{
		uint64_t totalDlBytes = 0;
		for (uint64_t tti = 1; tti <= m_maxTti; ++tti)
		{
			totalDlBytes += m_perTtiStats[tti].dlRxBytes;
		}
		return totalDlBytes;
	}

	uint64_t GetTotalUlBytes() const
	{
		uint64_t totalUlBytes = 0;
		for (uint64_t tti = 1; tti <= m_maxTti; ++tti)
		{
			totalUlBytes += m_perTtiStats[tti].ulRxBytes;
		}
		return totalUlBytes;
	}

	FormattedView Format(OutputFormat format) const
	{
		return FormattedView(*this, format);
	}

	std::string PrintPerTtiTable() const
	{
		return ToString(OutputFormat::TABLE);
	}

	std::string PrintPerTtiCsv() const
	{
		return ToString(OutputFormat::CSV);
	}

	std::string ToString(OutputFormat format) const
	{
		if (format == OutputFormat::CSV)
		{
			return BuildCsvString();
		}
		return BuildTableString();
	}

	friend std::ostream& operator<<(std::ostream& os, const FormattedView& view)
	{
		os << view.GetCollector().ToString(view.GetFormat());
		return os;
	}

	friend std::ostream& operator<<(std::ostream& os, const LtePerTtiTraceCollector& collector)
	{
		os << collector.ToString(OutputFormat::TABLE);
		return os;
	}

  private:
	std::string BuildTableString() const
	{
		std::ostringstream os;
		os << std::left << std::setw(8) << "TTI" << std::setw(14) << "DL MB/TTI"
		   << std::setw(14) << "UL MB/TTI" << std::setw(16) << "DL DRB(eNB)"
		   << std::setw(12) << "AvgDL MCS" << std::setw(14) << "Sched TTI" << "\n";
		os << "------------------------------------------------------------------------------\n";
		for (uint64_t tti = 1; tti <= m_maxTti; ++tti)
		{
			double dlIntervalMb = static_cast<double>(m_perTtiStats[tti].dlRxBytes) /
			                     (1024.0 * 1024.0);
			double ulIntervalMb = static_cast<double>(m_perTtiStats[tti].ulRxBytes) /
			                     (1024.0 * 1024.0);
			uint32_t activeDrbCount = m_perTtiStats[tti].dlActiveDrbKeys.size();
			uint32_t scheduledTtiCount = m_perTtiStats[tti].isScheduledTti ? 1 : 0;
			double avgDlMcs =
			    m_perTtiStats[tti].mcsCount > 0
			        ? static_cast<double>(m_perTtiStats[tti].mcsSum) / m_perTtiStats[tti].mcsCount
			        : 0.0;
			os << std::left << std::setw(8) << tti << std::fixed << std::setprecision(3)
			   << std::setw(14) << dlIntervalMb << std::setw(14) << ulIntervalMb
			   << std::setw(16) << activeDrbCount << std::setw(12) << avgDlMcs
			   << std::setw(14) << scheduledTtiCount << "\n";
		}
		return os.str();
	}

	std::string BuildCsvString() const
	{
		std::ostringstream os;
		os << "TTI,DL_MB_PER_TTI,UL_MB_PER_TTI,DL_DRB_ENB,AVG_DL_MCS,SCHED_TTI\n";
		for (uint64_t tti = 1; tti <= m_maxTti; ++tti)
		{
			double dlIntervalMb = static_cast<double>(m_perTtiStats[tti].dlRxBytes) /
			                     (1024.0 * 1024.0);
			double ulIntervalMb = static_cast<double>(m_perTtiStats[tti].ulRxBytes) /
			                     (1024.0 * 1024.0);
			uint32_t activeDrbCount = m_perTtiStats[tti].dlActiveDrbKeys.size();
			uint32_t scheduledTtiCount = m_perTtiStats[tti].isScheduledTti ? 1 : 0;
			double avgDlMcs =
			    m_perTtiStats[tti].mcsCount > 0
			        ? static_cast<double>(m_perTtiStats[tti].mcsSum) / m_perTtiStats[tti].mcsCount
			        : 0.0;
			os << tti << ',' << std::fixed << std::setprecision(6) << dlIntervalMb << ','
			   << ulIntervalMb << ',' << activeDrbCount << ',' << avgDlMcs << ','
			   << scheduledTtiCount << "\n";
		}
		return os.str();
	}

	bool IsTrackedTti(uint64_t tti) const
	{
		return tti >= 1 && tti <= m_maxTti;
	}

	void CollectDlSchedulingPerTti(std::string, DlSchedulingCallbackInfo dlInfo)
	{
		uint64_t tti = static_cast<uint64_t>(Simulator::Now().GetMilliSeconds());
		if (!IsTrackedTti(tti))
		{
			return;
		}

		PerTtiStats& stats = m_perTtiStats[tti];
		stats.isScheduledTti = true;
		if (dlInfo.sizeTb1 > 0)
		{
			stats.mcsSum += dlInfo.mcsTb1;
			++stats.mcsCount;
		}
		if (dlInfo.sizeTb2 > 0)
		{
			stats.mcsSum += dlInfo.mcsTb2;
			++stats.mcsCount;
		}
	}

	void CollectDlRxPerTti(std::string, uint16_t rnti, uint8_t lcid, uint32_t packetSize, uint64_t)
	{
		uint64_t tti = static_cast<uint64_t>(Simulator::Now().GetMilliSeconds());
		if (!IsTrackedTti(tti))
		{
			return;
		}

		PerTtiStats& stats = m_perTtiStats[tti];
		stats.dlRxBytes += packetSize;
		uint64_t drbKey = (static_cast<uint64_t>(rnti) << 8) | lcid;
		stats.dlActiveDrbKeys.insert(drbKey);
	}

	void CollectUlTxPerTti(std::string, uint16_t, uint8_t, uint32_t packetSize)
	{
		uint64_t tti = static_cast<uint64_t>(Simulator::Now().GetMilliSeconds());
		if (!IsTrackedTti(tti))
		{
			return;
		}

		m_perTtiStats[tti].ulRxBytes += packetSize;
	}

	void ConnectDlRxTraceOnDrbCreated(std::string context,
									  uint64_t,
									  uint16_t,
									  uint16_t,
									  uint8_t lcid)
	{
		std::string basePath = context.substr(0, context.rfind('/')) + "/DataRadioBearerMap/" +
							   std::to_string(lcid);
		Config::Connect(basePath + "/LteRlc/RxPDU",
						MakeCallback(&LtePerTtiTraceCollector::CollectDlRxPerTti, this));
		Config::Connect(basePath + "/LteRlc/TxPDU",
						MakeCallback(&LtePerTtiTraceCollector::CollectUlTxPerTti, this));
	}

	uint64_t m_maxTti;
	std::vector<PerTtiStats> m_perTtiStats;
};

int
main(int argc, char* argv[])
{
	Time simTime = Seconds(10);
	std::string csvFile = "lte_ex4.csv";
	bool printTable = false;

	CommandLine cmd(__FILE__);
	cmd.AddValue("simTime", "Total duration of the simulation", simTime);
	cmd.AddValue("csvFile", "Output CSV filename for per-TTI report", csvFile);
	cmd.AddValue("printTable", "Print per-TTI table to console", printTable);
	cmd.Parse(argc, argv);

	Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
	lteHelper->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(100));
	lteHelper->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(100));

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
	ueMobility1->SetPosition(Vector(1500.0, 0.0, 0.0)); // 1.5 km from eNB

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

	const uint64_t maxTti = static_cast<uint64_t>(simTime.GetMilliSeconds());
	LtePerTtiTraceCollector traceCollector(maxTti);
	traceCollector.Connect();

	Simulator::Stop(simTime);
	Simulator::Run();

	uint64_t totalDlBytes = traceCollector.GetTotalDlBytes();
	uint64_t totalUlBytes = traceCollector.GetTotalUlBytes();

	Time simulatedTime = Simulator::Now();
	uint64_t simulatedTti = static_cast<uint64_t>(simulatedTime.GetMilliSeconds());
	std::ofstream csvOutput(csvFile, std::ios::out | std::ios::trunc);
	if (!csvOutput.is_open())
	{
		NS_FATAL_ERROR("Could not open CSV output file: " << csvFile);
	}
	csvOutput << traceCollector.Format(LtePerTtiTraceCollector::OutputFormat::CSV);
	csvOutput.close();

	std::cout << "\n===== LTE Simulation Summary =====\n";
	if (printTable)
	{
		std::cout << traceCollector;
	}
	std::cout << "Per-TTI CSV written to: " << csvFile << "\n";

	std::cout << "------------------------------------------------------------------------------\n";
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


