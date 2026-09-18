#include <ns3/core-module.h>        // DE simulator core
#include <ns3/lte-module.h>         // LTE
#include <ns3/internet-module.h>    // IP stack to support UDP traffic Application
#include <ns3/applications-module.h>
#include <ns3/network-module.h>
#include <ns3/point-to-point-module.h>
#include <ns3/mobility-module.h>    // Mobility
#include <ns3/netanim-module.h>     // Visualisation

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace ns3;

namespace
{
struct JobRecord
{
    uint32_t jobId;
    uint16_t ueId;
    bool isDownlink;
    double arrivalTimeSec;
    uint32_t jobSizeBytes;
    uint64_t rxBytes;
    double completionTimeSec;
    bool completed;
};

std::vector<JobRecord> g_jobRecords;
Ptr<LteAmc> g_ulAmc;
Ptr<LteAmc> g_dlAmc;
uint16_t g_ulBandwidthRb = 100;
uint16_t g_dlBandwidthRb = 100;
uint64_t g_totalUlRbUsed = 0;
uint64_t g_totalDlRbUsed = 0;
uint64_t g_totalUlScheduledTbBytes = 0;
uint64_t g_totalDlScheduledTbBytes = 0;
uint64_t g_totalUlScheduledTti = 0;
uint64_t g_totalDlScheduledTti = 0;
uint64_t g_currentUlTti = 0;
bool g_hasCurrentUlTti = false;
bool g_currentUlTtiScheduled = false;
uint64_t g_currentDlTti = 0;
bool g_hasCurrentDlTti = false;
bool g_currentDlTtiScheduled = false;
std::map<uint64_t, std::set<uint8_t>> g_establishedDrbsByImsi;
uint32_t g_currentEstablishedDrbCount = 0;
double g_establishedDrbIntegral = 0.0;
double g_establishedDrbLastUpdateSec = 0.0;
double g_statsStartSec = 0.0;
bool g_progressBarEnabled = true;
double g_progressTotalSec = 0.0;
double g_progressUpdateSec = 0.5;
uint32_t g_progressBarWidth = 40;

bool
IsConnectedUeRrcState(LteUeRrc::State state)
{
    return state == LteUeRrc::CONNECTED_NORMALLY || state == LteUeRrc::CONNECTED_HANDOVER ||
           state == LteUeRrc::CONNECTED_PHY_PROBLEM ||
           state == LteUeRrc::CONNECTED_REESTABLISHING;
}

void
AccumulateEstablishedDrbIntegral(double nowSec)
{
    const double clampedNowSec = std::max(0.0, nowSec);
    const double integrateFromSec = std::max(g_establishedDrbLastUpdateSec, g_statsStartSec);
    if (clampedNowSec > integrateFromSec)
    {
        g_establishedDrbIntegral +=
            (clampedNowSec - integrateFromSec) * static_cast<double>(g_currentEstablishedDrbCount);
    }
    g_establishedDrbLastUpdateSec = clampedNowSec;
}

void
DrbCreatedTrace(uint64_t imsi, uint16_t, uint16_t, uint8_t lcid)
{
    AccumulateEstablishedDrbIntegral(Simulator::Now().GetSeconds());

    auto& lcidSet = g_establishedDrbsByImsi[imsi];
    if (lcidSet.insert(lcid).second)
    {
        ++g_currentEstablishedDrbCount;
    }
}

void
UeStateTransitionTrace(uint64_t imsi,
                       uint16_t,
                       uint16_t,
                       LteUeRrc::State oldState,
                       LteUeRrc::State newState)
{
    if (!IsConnectedUeRrcState(oldState) || IsConnectedUeRrcState(newState))
    {
        return;
    }

    AccumulateEstablishedDrbIntegral(Simulator::Now().GetSeconds());

    auto it = g_establishedDrbsByImsi.find(imsi);
    if (it == g_establishedDrbsByImsi.end())
    {
        return;
    }

    const uint32_t ueDrbCount = static_cast<uint32_t>(it->second.size());
    g_currentEstablishedDrbCount =
        (g_currentEstablishedDrbCount >= ueDrbCount) ? (g_currentEstablishedDrbCount - ueDrbCount) : 0;
    g_establishedDrbsByImsi.erase(it);
}

void
RenderProgressBar(double ratio)
{
    ratio = std::clamp(ratio, 0.0, 1.0);
    const uint32_t filled = static_cast<uint32_t>(ratio * g_progressBarWidth);

    std::cout << "\rSimulation progress [";
    for (uint32_t i = 0; i < g_progressBarWidth; ++i)
    {
        std::cout << (i < filled ? '=' : ' ');
    }
    std::cout << "] " << std::setw(6) << std::fixed << std::setprecision(2) << (100.0 * ratio) << "%"
              << std::flush;
}

void
ProgressBarTick()
{
    if (!g_progressBarEnabled || g_progressTotalSec <= 0.0)
    {
        return;
    }

    const double nowSec = Simulator::Now().GetSeconds();
    RenderProgressBar(nowSec / g_progressTotalSec);

    if (nowSec < g_progressTotalSec)
    {
        Simulator::Schedule(Seconds(g_progressUpdateSec), &ProgressBarTick);
    }
}

void
FlushCurrentUlTti()
{
    if (!g_hasCurrentUlTti)
    {
        return;
    }

    if (g_currentUlTtiScheduled)
    {
        ++g_totalUlScheduledTti;
    }

    g_hasCurrentUlTti = false;
    g_currentUlTtiScheduled = false;
}

void
FlushCurrentDlTti()
{
    if (!g_hasCurrentDlTti)
    {
        return;
    }
    if (g_currentDlTtiScheduled)
    {
        ++g_totalDlScheduledTti;
    }
    g_hasCurrentDlTti = false;
    g_currentDlTtiScheduled = false;
}

void
JobSinkRxTrace(uint32_t jobId, Ptr<const Packet> packet, const Address&, const Address&)
{
    if (jobId >= g_jobRecords.size())
    {
        return;
    }

    JobRecord& rec = g_jobRecords[jobId];
    rec.rxBytes += packet->GetSize();
    if (!rec.completed && rec.rxBytes >= rec.jobSizeBytes)
    {
        rec.completed = true;
        rec.completionTimeSec = Simulator::Now().GetSeconds();
    }
}

void
UlSchedulingTrace(uint16_t ulBandwidthRb,
                 uint32_t,
                 uint32_t,
                 uint16_t,
                 uint8_t mcs,
                 uint16_t tbsSize,
                 uint8_t)
{
    if (Simulator::Now().GetSeconds() < g_statsStartSec)
    {
        return;
    }

    if (!g_ulAmc || ulBandwidthRb == 0)
    {
        return;
    }

    const uint64_t nowTti = static_cast<uint64_t>(Simulator::Now().GetMicroSeconds() / 1000);
    if (!g_hasCurrentUlTti)
    {
        g_currentUlTti = nowTti;
        g_hasCurrentUlTti = true;
        g_currentUlTtiScheduled = false;
    }
    else if (nowTti != g_currentUlTti)
    {
        if (g_currentUlTtiScheduled)
        {
            ++g_totalUlScheduledTti;
        }
        g_currentUlTti = nowTti;
        g_currentUlTtiScheduled = false;
    }

    if (tbsSize > 0)
    {
        g_currentUlTtiScheduled = true;
    }

    uint16_t matchedRb = 0;
    uint32_t bestError = std::numeric_limits<uint32_t>::max();

    // Inverse-map RB count from (MCS, TB size). UL trace reports TB size in bytes.
    for (uint16_t rb = 1; rb <= ulBandwidthRb; ++rb)
    {
        uint16_t predictedTbBytes = static_cast<uint16_t>(g_ulAmc->GetUlTbSizeFromMcs(mcs, rb) / 8);
        if (predictedTbBytes == tbsSize)
        {
            matchedRb = rb;
            break;
        }

        uint32_t err = (predictedTbBytes > tbsSize) ? (predictedTbBytes - tbsSize)
                                                    : (tbsSize - predictedTbBytes);
        if (err < bestError)
        {
            bestError = err;
            matchedRb = rb;
        }
    }

    g_totalUlRbUsed += matchedRb;
    g_totalUlScheduledTbBytes += tbsSize;
}

void
DlSchedulingTrace(uint16_t dlBandwidthRb, DlSchedulingCallbackInfo dlInfo)
{
    if (Simulator::Now().GetSeconds() < g_statsStartSec)
    {
        return;
    }

    if (!g_dlAmc || dlBandwidthRb == 0)
    {
        return;
    }

    const uint64_t nowTti = static_cast<uint64_t>(Simulator::Now().GetMicroSeconds() / 1000);
    if (!g_hasCurrentDlTti)
    {
        g_currentDlTti = nowTti;
        g_hasCurrentDlTti = true;
        g_currentDlTtiScheduled = false;
    }
    else if (nowTti != g_currentDlTti)
    {
        if (g_currentDlTtiScheduled)
        {
            ++g_totalDlScheduledTti;
        }
        g_currentDlTti = nowTti;
        g_currentDlTtiScheduled = false;
    }

    if (dlInfo.sizeTb1 > 0 || dlInfo.sizeTb2 > 0)
    {
        g_currentDlTtiScheduled = true;
    }

    uint16_t matchedRb = 0;
    uint32_t bestError = std::numeric_limits<uint32_t>::max();

    // Inverse-map RB count from DL (MCS, TB size). DL trace reports TB sizes in bytes.
    for (uint16_t rb = 1; rb <= dlBandwidthRb; ++rb)
    {
        uint16_t predictedTb1Bytes = static_cast<uint16_t>(g_dlAmc->GetDlTbSizeFromMcs(dlInfo.mcsTb1, rb) / 8);
        uint16_t predictedTb2Bytes = 0;
        if (dlInfo.sizeTb2 > 0)
        {
            predictedTb2Bytes = static_cast<uint16_t>(g_dlAmc->GetDlTbSizeFromMcs(dlInfo.mcsTb2, rb) / 8);
        }

        if (predictedTb1Bytes == dlInfo.sizeTb1 && predictedTb2Bytes == dlInfo.sizeTb2)
        {
            matchedRb = rb;
            break;
        }

        uint32_t err1 = (predictedTb1Bytes > dlInfo.sizeTb1) ? (predictedTb1Bytes - dlInfo.sizeTb1)
                                                              : (dlInfo.sizeTb1 - predictedTb1Bytes);
        uint32_t err2 = (predictedTb2Bytes > dlInfo.sizeTb2) ? (predictedTb2Bytes - dlInfo.sizeTb2)
                                                              : (dlInfo.sizeTb2 - predictedTb2Bytes);
        uint32_t totalErr = err1 + err2;
        if (totalErr < bestError)
        {
            bestError = totalErr;
            matchedRb = rb;
        }
    }

    g_totalDlRbUsed += matchedRb;
    g_totalDlScheduledTbBytes += static_cast<uint64_t>(dlInfo.sizeTb1) + dlInfo.sizeTb2;
}
} // namespace

int
main(int argc, char* argv[])
{
    uint16_t numOfUe = 50;
    uint16_t numOfEnbs = 1;
    bool useCa = false;
    uint16_t activeUes = 22;
    double ulArrivalRate = 0.01; // jobs/s per active UE for uplink jobs
    uint32_t ulJobSizeMeanBytes = 50000000;
    double dlArrivalRate = 0.02; // jobs/s per active UE for downlink jobs
    uint32_t dlJobSizeMeanBytes = 150000000;  
    Time trafficStart = Seconds(1.0);
    Time statsWarmup = Seconds(2.0);
    Time simTime = Seconds(60) + statsWarmup;   // 10 seconds
    bool enableProgressBar = true;
    double progressUpdateSec = 0.5;
    std::string jobCsvFile = "lte_jobs.csv";

    CommandLine cmd(__FILE__);
    cmd.AddValue("numOfUe", "Number of UEs", numOfUe);
    cmd.AddValue("numOfEnbs", "Number of eNodeBs", numOfEnbs);
    cmd.AddValue("simTime", "Total duration of the simulation", simTime);
    cmd.AddValue("useCa", "Whether to use carrier aggregation.", useCa);
    cmd.AddValue("activeUes", "Number of UEs generating jobs", activeUes);
    cmd.AddValue("ulArrivalRate", "Per-UE uplink job arrival rate [jobs/s]", ulArrivalRate);
    cmd.AddValue("ulJobSizeMeanBytes", "Mean uplink job size [bytes]", ulJobSizeMeanBytes);
    cmd.AddValue("dlArrivalRate", "Per-UE downlink job arrival rate [jobs/s]", dlArrivalRate);
    cmd.AddValue("dlJobSizeMeanBytes", "Mean downlink job size [bytes]", dlJobSizeMeanBytes);
    cmd.AddValue("trafficStart", "Time when job-generation process starts", trafficStart);
    cmd.AddValue("statsWarmup",
                 "Warm-up duration after trafficStart before collecting statistics",
                 statsWarmup);
    cmd.AddValue("enableProgressBar", "Enable in-run simulation progress bar", enableProgressBar);
    cmd.AddValue("progressUpdateSec", "Progress bar update period in seconds", progressUpdateSec);
    cmd.AddValue("jobCsvFile", "CSV output path for per-job metrics", jobCsvFile);
    cmd.Parse(argc, argv);

    g_jobRecords.clear();
    g_totalUlRbUsed = 0;
    g_totalDlRbUsed = 0;
    g_totalUlScheduledTbBytes = 0;
    g_totalDlScheduledTbBytes = 0;
    g_totalUlScheduledTti = 0;
    g_totalDlScheduledTti = 0;
    g_currentUlTti = 0;
    g_hasCurrentUlTti = false;
    g_currentUlTtiScheduled = false;
    g_currentDlTti = 0;
    g_hasCurrentDlTti = false;
    g_currentDlTtiScheduled = false;
    g_establishedDrbsByImsi.clear();
    g_currentEstablishedDrbCount = 0;
    g_establishedDrbIntegral = 0.0;
    g_establishedDrbLastUpdateSec = 0.0;
    g_statsStartSec = (trafficStart + statsWarmup).GetSeconds();
    g_progressBarEnabled = enableProgressBar;
    g_progressTotalSec = std::max(0.0, simTime.GetSeconds());
    g_progressUpdateSec = std::max(0.1, progressUpdateSec);

    // LTE module operates in FDD mode; set explicit 20 MHz bandwidth (100 RB) for UL and DL.
    Config::SetDefault("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(100));
    Config::SetDefault("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(100));
    // Keep CA configuration consistent if carrier aggregation is enabled.
    Config::SetDefault("ns3::CcHelper::DlBandwidth", UintegerValue(100));
    Config::SetDefault("ns3::CcHelper::UlBandwidth", UintegerValue(100));
    g_ulBandwidthRb = 100;
    g_dlBandwidthRb = 100;
    g_ulAmc = CreateObject<LteAmc>();
    g_dlAmc = CreateObject<LteAmc>();

    if (useCa)
    {
        Config::SetDefault("ns3::LteHelper::UseCa", BooleanValue(useCa));
        Config::SetDefault("ns3::LteHelper::NumberOfComponentCarriers", UintegerValue(2)); 
        Config::SetDefault("ns3::LteHelper::EnbComponentCarrierManager",
                           StringValue("ns3::RrComponentCarrierManager"));
    }

    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    lteHelper->SetEpcHelper(epcHelper);
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    
    // lteHelper->EnableLogComponents ();

    // Create Nodes: eNodeB and UE
    NodeContainer enbNodes;
    NodeContainer ueNodes;
    enbNodes.Create(numOfEnbs);
    ueNodes.Create(numOfUe);

    // Install Mobility Model
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(enbNodes);  // enb do not move

    mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                   "MinX", DoubleValue (3.0),
                                   "MinY", DoubleValue (3.0),
                                   "DeltaX", DoubleValue (5.0),
                                   "DeltaY", DoubleValue (10.0),
                                   "GridWidth", UintegerValue (3),
                                   "LayoutType", StringValue ("RowFirst"));

    mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                               "Bounds", RectangleValue (Rectangle (-2000, 2000, -2000, 2000)));
    mobility.Install(ueNodes);

    // Create Devices and install them in the Nodes (eNB and UE)
    NetDeviceContainer enbDevs;
    NetDeviceContainer ueDevs;
    // Default scheduler is PF, uncomment to use RR
    // lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");

    enbDevs = lteHelper->InstallEnbDevice(enbNodes);
    ueDevs = lteHelper->InstallUeDevice(ueNodes);

    // add a internet remote host
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create (1);
    Ptr<Node> remoteHost = remoteHostContainer.Get (0);
    InternetStackHelper internet;
    
    // remote host
    internet.Install (remoteHostContainer);

    // Connect remote host to PGW
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
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // UE IP stack
    internet.Install (ueNodes);
    Ipv4InterfaceContainer ueIpIfaces;
    ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Node> ueNode = ueNodes.Get(u);
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }
   

    // Attach a UE to a eNB
    lteHelper->Attach(ueDevs, enbDevs.Get(0));

    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/ComponentCarrierMap/*/LteEnbMac/UlScheduling",
                                  MakeBoundCallback(&UlSchedulingTrace, g_ulBandwidthRb));
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/ComponentCarrierMap/*/LteEnbMac/DlScheduling",
                                  MakeBoundCallback(&DlSchedulingTrace, g_dlBandwidthRb));
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/LteUeRrc/DrbCreated",
                                  MakeCallback(&DrbCreatedTrace));
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/LteUeRrc/StateTransition",
                                  MakeCallback(&UeStateTransitionTrace));

    // With EPC enabled, default EPS bearer is activated automatically at Attach.

    // Workload: Poisson arrivals + finite-size jobs (UL and DL)
    activeUes = std::min<uint16_t>(activeUes, ueNodes.GetN());
    const uint16_t basePort = 9000;
    uint32_t nextPort = basePort;
    ApplicationContainer sinkApps;

    uint32_t totalJobs = 0;
    uint32_t ulJobsScheduled = 0;
    uint32_t dlJobsScheduled = 0;

    // UL jobs: UE -> remote host
    if (ulArrivalRate > 0)
    {
        for (uint16_t u = 0; u < activeUes; ++u)
        {
            Ptr<ExponentialRandomVariable> interArrivalRv = CreateObject<ExponentialRandomVariable>();
            interArrivalRv->SetAttribute("Mean", DoubleValue(1.0 / ulArrivalRate));

            Ptr<ExponentialRandomVariable> jobSizeRv = CreateObject<ExponentialRandomVariable>();
            jobSizeRv->SetAttribute("Mean",
                                    DoubleValue(std::max(1.0, static_cast<double>(ulJobSizeMeanBytes))));

            double t = trafficStart.GetSeconds();
            while (true)
            {
                // Poisson process: i.i.d. exponential inter-arrival times.
                t += interArrivalRv->GetValue();
                if (t >= simTime.GetSeconds())
                {
                    break;
                }

                const uint32_t jobSize =
                    static_cast<uint32_t>(std::max(1.0, jobSizeRv->GetValue()));

                if (nextPort > std::numeric_limits<uint16_t>::max())
                {
                    std::cerr << "Ran out of destination ports while scheduling jobs." << std::endl;
                    break;
                }

                const uint16_t jobPort = static_cast<uint16_t>(nextPort++);
                const uint32_t jobId = static_cast<uint32_t>(g_jobRecords.size());

                JobRecord rec;
                rec.jobId = jobId;
                rec.ueId = u;
                rec.isDownlink = false;
                rec.arrivalTimeSec = t;
                rec.jobSizeBytes = jobSize;
                rec.rxBytes = 0;
                rec.completionTimeSec = -1.0;
                rec.completed = false;
                g_jobRecords.push_back(rec);

                PacketSinkHelper sinkHelper("ns3::TcpSocketFactory",
                                            InetSocketAddress(Ipv4Address::GetAny(), jobPort));
                ApplicationContainer thisSink = sinkHelper.Install(remoteHost);
                thisSink.Start(Seconds(0.5));
                thisSink.Stop(simTime);
                sinkApps.Add(thisSink);

                Ptr<PacketSink> sink = DynamicCast<PacketSink>(thisSink.Get(0));
                sink->TraceConnectWithoutContext("RxWithAddresses",
                                                 MakeBoundCallback(&JobSinkRxTrace, jobId));

                BulkSendHelper bulkSend("ns3::TcpSocketFactory",
                                        InetSocketAddress(remoteHostAddr, jobPort));
                bulkSend.SetAttribute("MaxBytes", UintegerValue(jobSize));
                bulkSend.SetAttribute("SendSize", UintegerValue(1400));

                ApplicationContainer app = bulkSend.Install(ueNodes.Get(u));
                app.Start(Seconds(t));
                app.Stop(simTime);

                ++totalJobs;
                ++ulJobsScheduled;
            }
        }
    }

    // DL jobs: remote host -> UE
    if (dlArrivalRate > 0)
    {
        for (uint16_t u = 0; u < activeUes; ++u)
        {
            Ptr<ExponentialRandomVariable> interArrivalRv = CreateObject<ExponentialRandomVariable>();
            interArrivalRv->SetAttribute("Mean", DoubleValue(1.0 / dlArrivalRate));

            Ptr<ExponentialRandomVariable> jobSizeRv = CreateObject<ExponentialRandomVariable>();
            jobSizeRv->SetAttribute("Mean",
                                    DoubleValue(std::max(1.0, static_cast<double>(dlJobSizeMeanBytes))));

            double t = trafficStart.GetSeconds();
            while (true)
            {
                // Poisson process: i.i.d. exponential inter-arrival times.
                t += interArrivalRv->GetValue();
                if (t >= simTime.GetSeconds())
                {
                    break;
                }

                const uint32_t jobSize =
                    static_cast<uint32_t>(std::max(1.0, jobSizeRv->GetValue()));

                if (nextPort > std::numeric_limits<uint16_t>::max())
                {
                    std::cerr << "Ran out of destination ports while scheduling jobs." << std::endl;
                    break;
                }

                const uint16_t jobPort = static_cast<uint16_t>(nextPort++);
                const uint32_t jobId = static_cast<uint32_t>(g_jobRecords.size());

                JobRecord rec;
                rec.jobId = jobId;
                rec.ueId = u;
                rec.isDownlink = true;
                rec.arrivalTimeSec = t;
                rec.jobSizeBytes = jobSize;
                rec.rxBytes = 0;
                rec.completionTimeSec = -1.0;
                rec.completed = false;
                g_jobRecords.push_back(rec);

                PacketSinkHelper sinkHelper("ns3::TcpSocketFactory",
                                            InetSocketAddress(Ipv4Address::GetAny(), jobPort));
                ApplicationContainer thisSink = sinkHelper.Install(ueNodes.Get(u));
                thisSink.Start(Seconds(0.5));
                thisSink.Stop(simTime);
                sinkApps.Add(thisSink);

                Ptr<PacketSink> sink = DynamicCast<PacketSink>(thisSink.Get(0));
                sink->TraceConnectWithoutContext("RxWithAddresses",
                                                 MakeBoundCallback(&JobSinkRxTrace, jobId));

                BulkSendHelper bulkSend("ns3::TcpSocketFactory",
                                        InetSocketAddress(ueIpIfaces.GetAddress(u), jobPort));
                bulkSend.SetAttribute("MaxBytes", UintegerValue(jobSize));
                bulkSend.SetAttribute("SendSize", UintegerValue(1400));

                ApplicationContainer app = bulkSend.Install(remoteHost);
                app.Start(Seconds(t));
                app.Stop(simTime);

                ++totalJobs;
                ++dlJobsScheduled;
            }
        }
    }

    std::cout << "Configured workload: activeUes=" << activeUes
              << " | UL[arrivalRate=" << ulArrivalRate << " jobs/s/UE, mean=" << ulJobSizeMeanBytes
              << " B, dist=Exponential"
              << "] | DL[arrivalRate=" << dlArrivalRate << " jobs/s/UE, mean=" << dlJobSizeMeanBytes
              << " B, dist=Exponential"
              << "] | statsStart=" << g_statsStartSec << "s"
              << "] | jobsScheduled(UL/DL/Total)=" << ulJobsScheduled << "/" << dlJobsScheduled
              << "/" << totalJobs << std::endl;

    lteHelper->EnableTraces();

    // Ensure every node has a mobility model (for NetAnim warnings suppression).
    for (auto it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Node> node = *it;
        if (!node->GetObject<MobilityModel>())
        {
            Ptr<ConstantPositionMobilityModel> mm = CreateObject<ConstantPositionMobilityModel>();
            node->AggregateObject(mm);
        }
    }

    AnimationInterface anim("animation.xml");
    anim.SetMobilityPollInterval(Seconds(1.00));
    anim.SetMaxPktsPerTraceFile (100000000000);

    Simulator::Stop(simTime);
    if (g_progressBarEnabled)
    {
        Simulator::ScheduleNow(&ProgressBarTick);
    }
    Simulator::Run();
    if (g_progressBarEnabled)
    {
        RenderProgressBar(1.0);
        std::cout << std::endl;
    }
    FlushCurrentUlTti();
    FlushCurrentDlTti();
    AccumulateEstablishedDrbIntegral(simTime.GetSeconds());

    std::ofstream jobCsv(jobCsvFile, std::ios::out | std::ios::trunc);
    if (!jobCsv.is_open())
    {
        std::cerr << "Failed to open CSV file: " << jobCsvFile << std::endl;
    }
    else
    {
        jobCsv << "jobId,direction,ueId,arrivalTimeSec,jobSizeBytes,completionTimeSec,flowCompletionTimeSec,throughputMbps,completed,inStatsWindow,rxBytes\n";
        uint32_t completedJobsInStats = 0;
        uint64_t ulPayloadBytes = 0;
        uint64_t dlPayloadBytes = 0;
        double ulActiveJobSeconds = 0.0;
        double dlActiveJobSeconds = 0.0;
        const double statsWindowEndSec = simTime.GetSeconds();
        for (const auto& rec : g_jobRecords)
        {
            const double fct = rec.completed ? (rec.completionTimeSec - rec.arrivalTimeSec) : -1.0;
            const double throughputMbps = (rec.completed && fct > 0.0)
                                             ? ((8.0 * static_cast<double>(rec.rxBytes)) / (fct * 1e6))
                                             : -1.0;
            const std::string direction = rec.isDownlink ? "DL" : "UL";
            const bool inStatsWindow = rec.arrivalTimeSec >= g_statsStartSec;
            jobCsv << rec.jobId << ',' << direction << ',' << rec.ueId << ',' << rec.arrivalTimeSec
                   << ',' << rec.jobSizeBytes << ',' << rec.completionTimeSec << ',' << fct << ','
                   << throughputMbps << ',' << (rec.completed ? 1 : 0) << ','
                   << (inStatsWindow ? 1 : 0) << ',' << rec.rxBytes << '\n';
            if (inStatsWindow && rec.isDownlink)
            {
                dlPayloadBytes += rec.rxBytes;
            }
            else if (inStatsWindow)
            {
                ulPayloadBytes += rec.rxBytes;
            }
            if (inStatsWindow && rec.completed)
            {
                ++completedJobsInStats;
            }

            const double jobActiveEndSec = rec.completed ? rec.completionTimeSec : statsWindowEndSec;
            const double overlapStartSec = std::max(rec.arrivalTimeSec, g_statsStartSec);
            const double overlapEndSec = std::min(jobActiveEndSec, statsWindowEndSec);
            const double activeOverlapSec = std::max(0.0, overlapEndSec - overlapStartSec);
            if (rec.isDownlink)
            {
                dlActiveJobSeconds += activeOverlapSec;
            }
            else
            {
                ulActiveJobSeconds += activeOverlapSec;
            }
        }

        const uint16_t nCc = useCa ? 2 : 1;
        const double statsWindowSec = std::max(0.0, simTime.GetSeconds() - g_statsStartSec);
        const uint64_t totalTti = static_cast<uint64_t>(statsWindowSec * 1000.0);
        const uint64_t totalUlRbAvailable =
            totalTti * static_cast<uint64_t>(numOfEnbs) * static_cast<uint64_t>(nCc) * g_ulBandwidthRb;
        const uint64_t totalDlRbAvailable =
            totalTti * static_cast<uint64_t>(numOfEnbs) * static_cast<uint64_t>(nCc) * g_dlBandwidthRb;
        const double ulRbUtilPct =
            (totalUlRbAvailable > 0)
                ? (100.0 * static_cast<double>(g_totalUlRbUsed) / static_cast<double>(totalUlRbAvailable))
                : 0.0;
        const double dlRbUtilPct =
            (totalDlRbAvailable > 0)
                ? (100.0 * static_cast<double>(g_totalDlRbUsed) / static_cast<double>(totalDlRbAvailable))
                : 0.0;

        const uint64_t totalPayloadBytes = ulPayloadBytes + dlPayloadBytes;
        const double ulPayloadMiB = static_cast<double>(ulPayloadBytes) / (1024.0 * 1024.0);
        const double dlPayloadMiB = static_cast<double>(dlPayloadBytes) / (1024.0 * 1024.0);
        const double totalPayloadMiB = static_cast<double>(totalPayloadBytes) / (1024.0 * 1024.0);
        const double ulAvgThroughputMbps =
            (statsWindowSec > 0.0)
                ? ((8.0 * static_cast<double>(ulPayloadBytes)) / (statsWindowSec * 1e6))
                : 0.0;
        const double dlAvgThroughputMbps =
            (statsWindowSec > 0.0)
                ? ((8.0 * static_cast<double>(dlPayloadBytes)) / (statsWindowSec * 1e6))
                : 0.0;
        const double totalAvgThroughputMbps =
            (statsWindowSec > 0.0)
                ? ((8.0 * static_cast<double>(totalPayloadBytes)) / (statsWindowSec * 1e6))
                : 0.0;
        const double ulScheduledTtiPct =
            (totalTti > 0)
                ? (100.0 * static_cast<double>(g_totalUlScheduledTti) / static_cast<double>(totalTti))
                : 0.0;
        const double dlScheduledTtiPct =
            (totalTti > 0)
                ? (100.0 * static_cast<double>(g_totalDlScheduledTti) / static_cast<double>(totalTti))
                : 0.0;
        const double avgEstablishedDrbsAlive =
            (statsWindowSec > 0.0)
                ? (g_establishedDrbIntegral / statsWindowSec)
                : 0.0;
        const double ulAvgJobsPerTti =
            (statsWindowSec > 0.0)
                ? (ulActiveJobSeconds / statsWindowSec)
                : 0.0;
        const double dlAvgJobsPerTti =
            (statsWindowSec > 0.0)
                ? (dlActiveJobSeconds / statsWindowSec)
                : 0.0;
        const double totalAvgJobsPerTti =
            (statsWindowSec > 0.0)
                ? ((ulActiveJobSeconds + dlActiveJobSeconds) / statsWindowSec)
                : 0.0;

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "RB usage (UL): " << g_totalUlRbUsed << " / " << totalUlRbAvailable
                  << " (" << ulRbUtilPct << "%)" << std::endl;
        std::cout << "RB usage (DL): " << g_totalDlRbUsed << " / " << totalDlRbAvailable
                  << " (" << dlRbUtilPct << "%)" << std::endl;
          std::cout << "Data volume transferred (UL payload): " << ulPayloadMiB
              << " MiB, avg throughput=" << ulAvgThroughputMbps << " Mbps"
              << std::endl;
          std::cout << "Data volume transferred (DL payload): " << dlPayloadMiB
              << " MiB, avg throughput=" << dlAvgThroughputMbps << " Mbps"
              << std::endl;
          std::cout << "Data volume transferred (UL+DL payload): " << totalPayloadMiB
                << " MiB, avg throughput=" << totalAvgThroughputMbps << " Mbps"
                << std::endl;
        std::cout << "Statistics window: [" << g_statsStartSec << "s, " << simTime.GetSeconds()
                  << "s], duration=" << statsWindowSec << "s" << std::endl;
        std::cout << "Scheduled TTIs (UL): " << g_totalUlScheduledTti << " / " << totalTti
              << " (" << ulScheduledTtiPct << "%)" << std::endl;
        std::cout << "Scheduled TTIs (DL): " << g_totalDlScheduledTti << " / " << totalTti
              << " (" << dlScheduledTtiPct << "%)" << std::endl;
        std::cout << "Scheduled TB volume (UL/DL): " << g_totalUlScheduledTbBytes << " / "
                  << g_totalDlScheduledTbBytes << " bytes" << std::endl;
          std::cout << "Average established DRBs alive (RRC/EPS): " << avgEstablishedDrbsAlive
                << std::endl;
          std::cout << "Average active jobs per TTI (UL/DL/Total): " << ulAvgJobsPerTti << " / "
                << dlAvgJobsPerTti << " / " << totalAvgJobsPerTti << std::endl;
        std::cout << "Per-job metrics written to " << jobCsvFile
                  << " (completedJobsInStatsWindow=" << completedJobsInStats
                  << "/" << g_jobRecords.size() << ")" << std::endl;
    }

    Simulator::Destroy();
    return 0;
}
