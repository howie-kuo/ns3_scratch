/*
 * LTE simulator Ex4
 * 
 * extending ex3
 *   - modify to collect per-TTI statistics using the tracing API
 */

// #include "ns3/buildings-helper.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <set>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

std::string
ToLowerCopy(std::string value)
{
	std::transform(value.begin(),
				   value.end(),
				   value.begin(),
				   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	return value;
}

uint64_t
MbpsToBps(double mbps)
{
	return static_cast<uint64_t>(std::max(1.0, mbps * 1000000.0));
}

class UeThroughputTimeSeriesCollector
{
  public:
	struct ThroughputSummary
	{
		double meanMbps = 0.0;
		double minMbps = 0.0;
		double maxMbps = 0.0;
		double p95Mbps = 0.0;
	};

	UeThroughputTimeSeriesCollector(Time start, Time stop, Time samplePeriod)
		: m_start(start),
		  m_stop(stop),
		  m_samplePeriod(samplePeriod),
		  m_samplePeriodNs(samplePeriod.GetNanoSeconds())
	{
		NS_ABORT_MSG_IF(m_stop <= m_start,
						"Throughput time-series stop must be greater than start");
		NS_ABORT_MSG_IF(m_samplePeriodNs <= 0,
						"Throughput time-series sample period must be positive");

		const int64_t durationNs = (m_stop - m_start).GetNanoSeconds();
		const uint32_t bins = static_cast<uint32_t>(durationNs / m_samplePeriodNs +
											 (durationNs % m_samplePeriodNs == 0 ? 0 : 1));
		m_binRxBytes.assign(bins, 0);
	}

	std::vector<double> GetIntervalThroughputMbps() const
	{
		std::vector<double> samples;
		samples.reserve(m_binRxBytes.size());

		for (uint32_t i = 0; i < m_binRxBytes.size(); ++i)
		{
			const Time intervalStart = m_start + NanoSeconds(static_cast<int64_t>(i) * m_samplePeriodNs);
			const Time intervalEnd = std::min(m_stop, intervalStart + m_samplePeriod);
			const double intervalDurationSec = (intervalEnd - intervalStart).GetSeconds();
			const double throughputMbps = intervalDurationSec > 0.0
											 ? (m_binRxBytes[i] * 8.0) /
											   (intervalDurationSec * 1000000.0)
											 : 0.0;
			samples.push_back(throughputMbps);
		}

		return samples;
	}

	ThroughputSummary GetSummary() const
	{
		ThroughputSummary summary;
		const std::vector<double> samples = GetIntervalThroughputMbps();
		if (samples.empty())
		{
			return summary;
		}

		double sumMbps = 0.0;
		summary.minMbps = samples.front();
		summary.maxMbps = samples.front();
		for (double value : samples)
		{
			sumMbps += value;
			summary.minMbps = std::min(summary.minMbps, value);
			summary.maxMbps = std::max(summary.maxMbps, value);
		}
		summary.meanMbps = sumMbps / static_cast<double>(samples.size());

		std::vector<double> sortedSamples = samples;
		std::sort(sortedSamples.begin(), sortedSamples.end());
		const uint32_t percentileIndex =
			static_cast<uint32_t>(std::ceil(0.95 * sortedSamples.size()) - 1);
		summary.p95Mbps = sortedSamples[std::min(percentileIndex,
									 static_cast<uint32_t>(sortedSamples.size() - 1))];

		return summary;
	}

	void Connect(Ptr<PacketSink> sink)
	{
		const bool connected = sink->TraceConnectWithoutContext(
			"Rx",
			MakeCallback(&UeThroughputTimeSeriesCollector::OnRx, this));
		NS_ABORT_MSG_IF(!connected,
						"Failed to connect UE1 sink Rx trace for throughput time series");
	}

	uint64_t GetTotalWindowRxBytes() const
	{
		uint64_t totalBytes = 0;
		for (uint64_t bytes : m_binRxBytes)
		{
			totalBytes += bytes;
		}
		return totalBytes;
	}

	void WriteCsv(const std::string& outputFile, const std::string& jobType) const
	{
		std::ofstream output(outputFile, std::ios::out | std::ios::trunc);
		if (!output.is_open())
		{
			NS_FATAL_ERROR("Could not open UE1 throughput time-series file: " << outputFile);
		}

		output << "job_type,interval_start_s,interval_end_s,rx_bytes,throughput_mbps\n";
		output << std::fixed << std::setprecision(6);

		for (uint32_t i = 0; i < m_binRxBytes.size(); ++i)
		{
			const Time intervalStart = m_start + NanoSeconds(static_cast<int64_t>(i) * m_samplePeriodNs);
			const Time intervalEnd = std::min(m_stop, intervalStart + m_samplePeriod);
			const double intervalDurationSec = (intervalEnd - intervalStart).GetSeconds();
			const double throughputMbps = intervalDurationSec > 0.0
											 ? (m_binRxBytes[i] * 8.0) /
											   (intervalDurationSec * 1000000.0)
											 : 0.0;

			output << jobType << ',' << intervalStart.GetSeconds() << ','
				   << intervalEnd.GetSeconds() << ',' << m_binRxBytes[i] << ','
				   << throughputMbps << "\n";
		}
	}

  private:
	void OnRx(Ptr<const Packet> packet, const Address&)
	{
		const Time now = Simulator::Now();
		if (now < m_start || now >= m_stop)
		{
			return;
		}

		const int64_t elapsedNs = (now - m_start).GetNanoSeconds();
		const uint32_t bin = static_cast<uint32_t>(elapsedNs / m_samplePeriodNs);
		if (bin < m_binRxBytes.size())
		{
			m_binRxBytes[bin] += packet->GetSize();
		}
	}

	Time m_start;
	Time m_stop;
	Time m_samplePeriod;
	int64_t m_samplePeriodNs;
	std::vector<uint64_t> m_binRxBytes;
};

} // namespace

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
	std::string csvFile = "lte_ex5.csv";
	std::string ue1ThroughputReportFile = "lte_ex5_ue1_throughput.csv";
	std::string ue1ThroughputTimeSeriesFile = "lte_ex5_ue1_throughput_timeseries.csv";
	std::string ue1ThroughputCombinedReportFile = "lte_ex5_ue1_throughput_combined.csv";
	bool printTable = false;

	double ue0BackgroundLoadMbps = 6.0;
	Time ue0BackgroundStart = Seconds(1.5);

	std::string ue1JobType = "video";
	double ue1JobRateMbps = 4.0;
	Time ue1JobStart = Seconds(3.0);
	Time ue1JobStop = Seconds(8.0);
	Time ue1ThroughputSamplePeriod = Seconds(1.0);

	CommandLine cmd(__FILE__);
	cmd.AddValue("simTime", "Total duration of the simulation", simTime);
	cmd.AddValue("csvFile", "Output CSV filename for per-TTI report", csvFile);
	cmd.AddValue("ue1ThroughputReportFile",
				 "Output CSV filename for UE1 throughput report",
				 ue1ThroughputReportFile);
	cmd.AddValue("ue1ThroughputTimeSeriesFile",
				 "Output CSV filename for UE1 throughput time series",
				 ue1ThroughputTimeSeriesFile);
	cmd.AddValue("ue1ThroughputCombinedReportFile",
				 "Output CSV filename for UE1 combined throughput summary",
				 ue1ThroughputCombinedReportFile);
	cmd.AddValue("printTable", "Print per-TTI table to console", printTable);
	cmd.AddValue("ue0BackgroundLoadMbps",
				 "Total UE0 background load [Mbps] split as 40% video, 30% web, 20% telemetry, 10% VoIP",
				 ue0BackgroundLoadMbps);
	cmd.AddValue("ue0BackgroundStart", "Time when UE0 background traffic starts", ue0BackgroundStart);
	cmd.AddValue("ue1JobType",
				 "UE1 job type: video | web | telemetry | voip",
				 ue1JobType);
	cmd.AddValue("ue1JobRateMbps", "UE1 job target sending rate [Mbps]", ue1JobRateMbps);
	cmd.AddValue("ue1JobStart", "UE1 job start time", ue1JobStart);
	cmd.AddValue("ue1JobStop", "UE1 job stop time", ue1JobStop);
	cmd.AddValue("ue1ThroughputSamplePeriod",
				 "UE1 throughput time-series sampling period",
				 ue1ThroughputSamplePeriod);
	cmd.Parse(argc, argv);

	NS_ABORT_MSG_IF(ue1JobStop <= ue1JobStart,
				"ue1JobStop must be greater than ue1JobStart");
	NS_ABORT_MSG_IF(simTime <= ue1JobStop,
				"simTime must be greater than ue1JobStop to measure UE1 throughput");
	NS_ABORT_MSG_IF(ue1ThroughputSamplePeriod <= Seconds(0),
				"ue1ThroughputSamplePeriod must be positive");

	Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
	Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
	lteHelper->SetEpcHelper(epcHelper);
	lteHelper->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(100));
	lteHelper->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(100));
	Ptr<Node> pgw = epcHelper->GetPgwNode();

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

	NodeContainer remoteHostContainer;
	remoteHostContainer.Create(1);
	Ptr<Node> remoteHost = remoteHostContainer.Get(0);

	InternetStackHelper internet;
	internet.Install(remoteHostContainer);

	PointToPointHelper p2ph;
	p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
	p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
	p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(10)));
	NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

	Ipv4AddressHelper ipv4h;
	ipv4h.SetBase("1.0.0.0", "255.0.0.0");
	Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
	Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);

	Ipv4StaticRoutingHelper ipv4RoutingHelper;
	Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
		ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
	remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
									  Ipv4Mask("255.0.0.0"),
									  1);

	internet.Install(ueNodes);
	Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
	for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
	{
		Ptr<Node> ueNode = ueNodes.Get(u);
		Ptr<Ipv4StaticRouting> ueStaticRouting =
			ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
		ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
	}

	Simulator::Schedule(Seconds(1.0),
	                    [lteHelper, ueDev = ueDevs.Get(0), enbDev = enbDevs.Get(0)]() {
		lteHelper->Attach(ueDev, enbDev);
	});
	Simulator::Schedule(Seconds(1.2),
	                    [lteHelper, ueDev = ueDevs.Get(1), enbDev = enbDevs.Get(0)]() {
		lteHelper->Attach(ueDev, enbDev);
	});

	const double ue0VideoLoadMbps = ue0BackgroundLoadMbps * 0.40;
	const double ue0WebLoadMbps = ue0BackgroundLoadMbps * 0.30;
	const double ue0TelemetryLoadMbps = ue0BackgroundLoadMbps * 0.20;
	const double ue0VoipLoadMbps = ue0BackgroundLoadMbps * 0.10;

	const uint16_t ue0VideoPort = 12000;
	const uint16_t ue0WebPort = 12001;
	const uint16_t ue0TelemetryPort = 12002;
	const uint16_t ue0VoipPort = 12003;
	const uint16_t ue1JobPort = 13000;

	PacketSinkHelper ue0VideoSinkHelper("ns3::UdpSocketFactory",
								InetSocketAddress(Ipv4Address::GetAny(), ue0VideoPort));
	PacketSinkHelper ue0WebSinkHelper("ns3::TcpSocketFactory",
							  InetSocketAddress(Ipv4Address::GetAny(), ue0WebPort));
	PacketSinkHelper ue0TelemetrySinkHelper("ns3::UdpSocketFactory",
									InetSocketAddress(Ipv4Address::GetAny(), ue0TelemetryPort));
	PacketSinkHelper ue0VoipSinkHelper("ns3::UdpSocketFactory",
								InetSocketAddress(Ipv4Address::GetAny(), ue0VoipPort));

	ApplicationContainer ue0VideoSink = ue0VideoSinkHelper.Install(remoteHost);
	ApplicationContainer ue0WebSink = ue0WebSinkHelper.Install(remoteHost);
	ApplicationContainer ue0TelemetrySink = ue0TelemetrySinkHelper.Install(remoteHost);
	ApplicationContainer ue0VoipSink = ue0VoipSinkHelper.Install(remoteHost);
	ue0VideoSink.Start(Seconds(0.5));
	ue0WebSink.Start(Seconds(0.5));
	ue0TelemetrySink.Start(Seconds(0.5));
	ue0VoipSink.Start(Seconds(0.5));
	ue0VideoSink.Stop(simTime);
	ue0WebSink.Stop(simTime);
	ue0TelemetrySink.Stop(simTime);
	ue0VoipSink.Stop(simTime);

	OnOffHelper ue0VideoBg("ns3::UdpSocketFactory", InetSocketAddress(remoteHostAddr, ue0VideoPort));
	ue0VideoBg.SetAttribute("DataRate", DataRateValue(DataRate(MbpsToBps(ue0VideoLoadMbps))));
	ue0VideoBg.SetAttribute("PacketSize", UintegerValue(1200));
	ue0VideoBg.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
	ue0VideoBg.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

	OnOffHelper ue0WebBg("ns3::TcpSocketFactory", InetSocketAddress(remoteHostAddr, ue0WebPort));
	ue0WebBg.SetAttribute(
		"DataRate",
		DataRateValue(DataRate(MbpsToBps(std::max(ue0WebLoadMbps * 2.0, 1.0)))));
	ue0WebBg.SetAttribute("PacketSize", UintegerValue(1440));
	ue0WebBg.SetAttribute("OnTime", StringValue("ns3::ExponentialRandomVariable[Mean=0.2]"));
	ue0WebBg.SetAttribute("OffTime", StringValue("ns3::ExponentialRandomVariable[Mean=0.2]"));

	OnOffHelper ue0TelemetryBg("ns3::UdpSocketFactory",
							 InetSocketAddress(remoteHostAddr, ue0TelemetryPort));
	ue0TelemetryBg.SetAttribute("DataRate",
						DataRateValue(DataRate(MbpsToBps(ue0TelemetryLoadMbps))));
	ue0TelemetryBg.SetAttribute("PacketSize", UintegerValue(256));
	ue0TelemetryBg.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
	ue0TelemetryBg.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

	OnOffHelper ue0VoipBg("ns3::UdpSocketFactory", InetSocketAddress(remoteHostAddr, ue0VoipPort));
	ue0VoipBg.SetAttribute("DataRate", DataRateValue(DataRate(MbpsToBps(ue0VoipLoadMbps))));
	ue0VoipBg.SetAttribute("PacketSize", UintegerValue(160));
	ue0VoipBg.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
	ue0VoipBg.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

	ApplicationContainer ue0VideoApp = ue0VideoBg.Install(ueNodes.Get(0));
	ApplicationContainer ue0WebApp = ue0WebBg.Install(ueNodes.Get(0));
	ApplicationContainer ue0TelemetryApp = ue0TelemetryBg.Install(ueNodes.Get(0));
	ApplicationContainer ue0VoipApp = ue0VoipBg.Install(ueNodes.Get(0));
	ue0VideoApp.Start(ue0BackgroundStart);
	ue0WebApp.Start(ue0BackgroundStart);
	ue0TelemetryApp.Start(ue0BackgroundStart);
	ue0VoipApp.Start(ue0BackgroundStart);
	ue0VideoApp.Stop(simTime);
	ue0WebApp.Stop(simTime);
	ue0TelemetryApp.Stop(simTime);
	ue0VoipApp.Stop(simTime);

	std::string ue1JobTypeNormalized = ToLowerCopy(ue1JobType);
	std::string ue1SocketFactory;
	uint32_t ue1PacketSize = 1200;
	double ue1ConfiguredRateMbps = ue1JobRateMbps;
	bool ue1BurstyWeb = false;

	if (ue1JobTypeNormalized == "video")
	{
		ue1SocketFactory = "ns3::UdpSocketFactory";
		ue1PacketSize = 1200;
	}
	else if (ue1JobTypeNormalized == "web")
	{
		ue1SocketFactory = "ns3::TcpSocketFactory";
		ue1PacketSize = 1440;
		ue1BurstyWeb = true;
		ue1ConfiguredRateMbps = std::max(ue1JobRateMbps * 2.0, 1.0);
	}
	else if (ue1JobTypeNormalized == "telemetry")
	{
		ue1SocketFactory = "ns3::UdpSocketFactory";
		ue1PacketSize = 256;
	}
	else if (ue1JobTypeNormalized == "voip")
	{
		ue1SocketFactory = "ns3::UdpSocketFactory";
		ue1PacketSize = 160;
	}
	else
	{
		NS_FATAL_ERROR("Unsupported ue1JobType='"
					   << ue1JobType
					   << "'. Supported types: video | web | telemetry | voip");
	}

	PacketSinkHelper ue1SinkHelper(ue1SocketFactory,
							 InetSocketAddress(Ipv4Address::GetAny(), ue1JobPort));
	ApplicationContainer ue1SinkApps = ue1SinkHelper.Install(remoteHost);
	ue1SinkApps.Start(Seconds(0.5));
	ue1SinkApps.Stop(simTime);
	Ptr<PacketSink> ue1Sink = DynamicCast<PacketSink>(ue1SinkApps.Get(0));
	UeThroughputTimeSeriesCollector ue1ThroughputSeries(ue1JobStart,
									ue1JobStop,
									ue1ThroughputSamplePeriod);
	NS_ABORT_MSG_IF(!ue1Sink, "Failed to create UE1 PacketSink for throughput reporting");
	ue1ThroughputSeries.Connect(ue1Sink);

	OnOffHelper ue1JobHelper(ue1SocketFactory, InetSocketAddress(remoteHostAddr, ue1JobPort));
	ue1JobHelper.SetAttribute("DataRate",
						 DataRateValue(DataRate(MbpsToBps(ue1ConfiguredRateMbps))));
	ue1JobHelper.SetAttribute("PacketSize", UintegerValue(ue1PacketSize));
	if (ue1BurstyWeb)
	{
		ue1JobHelper.SetAttribute("OnTime",
						 StringValue("ns3::ExponentialRandomVariable[Mean=0.2]"));
		ue1JobHelper.SetAttribute("OffTime",
						  StringValue("ns3::ExponentialRandomVariable[Mean=0.2]"));
	}
	else
	{
		ue1JobHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
		ue1JobHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
	}

	ApplicationContainer ue1JobApp = ue1JobHelper.Install(ueNodes.Get(1));
	ue1JobApp.Start(ue1JobStart);
	ue1JobApp.Stop(ue1JobStop);

	std::cout << "Configured UE0 background load [Mbps]: video=" << ue0VideoLoadMbps
			  << ", web=" << ue0WebLoadMbps << ", telemetry=" << ue0TelemetryLoadMbps
			  << ", voip=" << ue0VoipLoadMbps << " (total=" << ue0BackgroundLoadMbps
			  << ")" << std::endl;
	std::cout << "Configured UE1 job: type=" << ue1JobTypeNormalized << ", rate="
			  << ue1JobRateMbps << " Mbps, window=[" << ue1JobStart.GetSeconds() << "s,"
			  << ue1JobStop.GetSeconds() << "s]" << std::endl;

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

	const uint64_t ue1RxBytes = ue1ThroughputSeries.GetTotalWindowRxBytes();
	const double ue1ActiveSeconds = (ue1JobStop - ue1JobStart).GetSeconds();
	const double ue1AvgThroughputMbps =
		ue1ActiveSeconds > 0.0 ? (ue1RxBytes * 8.0) / (ue1ActiveSeconds * 1000000.0) : 0.0;
	const auto ue1SeriesSummary = ue1ThroughputSeries.GetSummary();

	std::ofstream ue1ReportOutput(ue1ThroughputReportFile, std::ios::out | std::ios::trunc);
	if (!ue1ReportOutput.is_open())
	{
		NS_FATAL_ERROR("Could not open UE1 throughput report file: " << ue1ThroughputReportFile);
	}
	ue1ReportOutput << "job_type,start_s,stop_s,duration_s,rx_bytes,avg_throughput_mbps\n";
	ue1ReportOutput << ue1JobTypeNormalized << ',' << ue1JobStart.GetSeconds() << ','
					<< ue1JobStop.GetSeconds() << ',' << ue1ActiveSeconds << ',' << ue1RxBytes
					<< ',' << std::fixed << std::setprecision(6) << ue1AvgThroughputMbps
					<< "\n";
	ue1ReportOutput.close();

	std::ofstream ue1CombinedOutput(ue1ThroughputCombinedReportFile,
								 std::ios::out | std::ios::trunc);
	if (!ue1CombinedOutput.is_open())
	{
		NS_FATAL_ERROR("Could not open UE1 combined throughput file: "
					   << ue1ThroughputCombinedReportFile);
	}
	ue1CombinedOutput << "job_type,start_s,stop_s,duration_s,rx_bytes,avg_throughput_mbps,"
				  << "mean_sample_throughput_mbps,min_sample_throughput_mbps,"
				  << "max_sample_throughput_mbps,p95_sample_throughput_mbps\n";
	ue1CombinedOutput << std::fixed << std::setprecision(6) << ue1JobTypeNormalized << ','
					  << ue1JobStart.GetSeconds() << ',' << ue1JobStop.GetSeconds()
					  << ',' << ue1ActiveSeconds << ',' << ue1RxBytes << ','
					  << ue1AvgThroughputMbps << ',' << ue1SeriesSummary.meanMbps
					  << ',' << ue1SeriesSummary.minMbps << ',' << ue1SeriesSummary.maxMbps
					  << ',' << ue1SeriesSummary.p95Mbps << "\n";
	ue1CombinedOutput.close();
	ue1ThroughputSeries.WriteCsv(ue1ThroughputTimeSeriesFile, ue1JobTypeNormalized);

	std::cout << "\n===== LTE Simulation Summary =====\n";
	if (printTable)
	{
		std::cout << traceCollector;
	}
	std::cout << "Per-TTI CSV written to: " << csvFile << "\n";
	std::cout << "UE1 throughput report written to: " << ue1ThroughputReportFile << "\n";
	std::cout << "UE1 throughput time series written to: " << ue1ThroughputTimeSeriesFile
			  << "\n";
	std::cout << "UE1 combined throughput report written to: "
			  << ue1ThroughputCombinedReportFile << "\n";
	std::cout << "UE1 throughput report: type=" << ue1JobTypeNormalized
			  << ", rxBytes=" << ue1RxBytes << ", avgThroughput=" << std::fixed
			  << std::setprecision(3) << ue1AvgThroughputMbps << " Mbps\n";
	std::cout << "UE1 sample throughput stats [Mbps]: mean=" << ue1SeriesSummary.meanMbps
			  << ", min=" << ue1SeriesSummary.minMbps
			  << ", max=" << ue1SeriesSummary.maxMbps
			  << ", p95=" << ue1SeriesSummary.p95Mbps << "\n";

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


