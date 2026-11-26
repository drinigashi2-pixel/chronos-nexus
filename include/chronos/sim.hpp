// sim.hpp - per-cycle pipeline (one cycle = one day):
//   apply due events -> predictor -> min-cost flow -> merge/chunk missions -> meta layer
//   -> scheduling (ALNS + ALT A*) -> infrastructure -> validate -> execute through the day
//   (events trigger incremental re-planning at their time) -> advance planets.
// Writes the 10 required outputs + statistics JSON.
#pragma once
#include <string>
#include <vector>

#include "chronos/infra.hpp"
#include "chronos/meta.hpp"
#include "chronos/replan.hpp"
#include "chronos/score.hpp"

namespace chronos {

struct SimConfig {
  std::string datasetDir, outDir = "results/run";
  double timeMs = 30000, memMB = 2048;
  uint64_t seed = 1;
  int days = -1;                         // -1: use dataset config (default 14)
  ReplanMode replanMode = ReplanMode::Incremental;
  SchedMethod schedForced = SchedMethod::Auto;
  int allocForced = -1;                  // -1 auto, else AllocMethod
  RouteAlgo routing = RouteAlgo::ALT;
  bool useRegret = true, useALNS = true, useLocalSearch = true, useRepair = true, usePruning = true;
  bool quiet = false;
  bool writeFiles = true;
  double loadMs = 0;                     // time already spent loading (counted in the budget)
};

struct Statistics {
  // the 19 required statistics
  double totalScore = 0, populationSurvivalRate = 0, resourcesDelivered = 0, resourceShortages = 0,
         resourceWaste = 0;
  int missionsCompleted = 0, missionsFailed = 0;
  double totalDistance = 0, totalFuel = 0, totalTransportCost = 0, totalRisk = 0;
  uint64_t nodesExplored = 0, candidatesEvaluated = 0, statesExplored = 0;
  double searchSpaceReductionPct = 0;
  int eventsHandled = 0;
  double avgReplanMs = 0, totalExecMs = 0, peakMemoryMB = 0;
  // extras
  double naiveStates = 0;
  int missionsTotal = 0, missionsOnTime = 0, missionsLate = 0, missionsExpired = 0, missionsCancelled = 0;
  int missionsPendingAtEnd = 0, proactiveMissions = 0, eventsSkipped = 0;
  double maxReplanMs = 0, loadMs = 0, landmarkMs = 0;
  int validationHardViolations = 0, facilitiesBuilt = 0;
  double budgetSpent = 0;
  uint64_t routeQueries = 0, cacheHits = 0, cacheInvalidations = 0, labelsPruned = 0, candidatesPruned = 0,
           dominatedShips = 0, bnbNodes = 0, altRebuilds = 0;
  double naiveRoutingRatio = 0;  // sampled plain-Dijkstra settles / ALT settles
  int64_t alnsIterations = 0;
  int days = 0;
  double probeAltSettled = 0, probeBidirSettled = 0;  // meta-layer routing race (deterministic)
  std::string routingChosen;
  std::string replanMode, schedulerUsed;
};

struct CycleRecord {
  int day = 0;
  std::string strategy, reason;
  int active = 0, created = 0, assigned = 0, shipments = 0, predicted = 0;
  int64_t alnsIters = 0;
  double allocMs = 0, schedMs = 0, infraMs = 0, cost = 0, allocObjective = 0;
  int hardViolations = 0, late = 0;
};

struct InfraRecord {
  int option = -1;
  double start = 0, complete = 0, value = 0;
  std::string method, trigger;
};

struct PredictionRecord {
  int day = 0;
  ShortagePrediction p;
  bool covered = false;
};

class Simulator {
 public:
  Simulator(World& w, const SimConfig& cfg);
  Statistics run();
  void writeOutputs(const std::string& dir) const;
  const Statistics& stats() const { return stats_; }
  const ScoreBreakdown& score() const { return score_; }
  Router& router() { return router_; }
  Scheduler& scheduler() { return sched_; }

 private:
  struct InFlight {
    int mission, ship;
    double arrive;
    bool failed;
  };
  void planCycle(int day);
  void runInfrastructure(const std::vector<NodeId>* planets, const std::string& trigger, double share);
  void execute(double until);
  void commitMission(int ship, int m);
  void processArrivals(double until);
  void advancePlanets(double fromH, double toH);
  void expireMissions(double now);
  void completeFacilities(double now);
  void handleShipLoss(int ship, double now, bool destroyed, double repairH);
  Features features() const;
  void finalizeStats();
  std::vector<double> incomingVector() const;
  std::vector<double> outgoingVector() const;
  double chunkCapacity() const;
  void reindexPlans(const std::vector<int>* ships);

  World& w_;
  SimConfig cfg_;
  BudgetManager budget_;
  Router router_;
  Scheduler sched_;
  ResourceAllocator alloc_;
  Predictor pred_;
  InfraPlanner infra_;
  double initialBudget_ = 0;
  std::vector<int> paretoFailDay_;
  double lastDayMs_ = 0;  // wall time of the previous simulated day (pace check)  // last day a mission's Pareto search failed
  DependencyIndex dep_;
  Replanner replan_;
  MetaSelector meta_;
  Rng rng_;
  Strategy strat_;
  Timer clock_;
  std::vector<InFlight> inFlight_;
  std::vector<Event> queue_;  // pending events (input + follow-ups), sorted by time
  size_t queuePos_ = 0;
  // records
  std::vector<CycleRecord> cycles_;
  std::vector<InfraRecord> infraLog_;
  std::vector<PredictionRecord> predictions_;
  std::vector<ReplanRecord> replans_;
  std::vector<Shipment> shipmentLog_;
  std::vector<int> shipmentDay_;
  std::vector<ConvergencePoint> convergence_;
  // accumulators
  double popExposed_ = 0, shortageW_ = 0, demandW_ = 0, wasteW_ = 0, producedW_ = 0;
  double shortageUnits_ = 0, wasteUnits_ = 0, delivered_ = 0;
  double transportCost_ = 0, naiveTransport_ = 0, fuelUsed_ = 0, naiveFuel_ = 0, distance_ = 0;
  double riskSum_ = 0;
  int riskCount_ = 0;
  double prodValueStart_ = 0, techStart_ = 0;
  double maxSeverity_ = 0;
  std::vector<double> lostPop_;
  Statistics stats_;
  ScoreBreakdown score_;
};

}  // namespace chronos
