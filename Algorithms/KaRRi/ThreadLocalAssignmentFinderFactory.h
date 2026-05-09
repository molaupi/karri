/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Moritz Laupichler <moritz.laupichler@kit.edu>
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
/// ******************************************************************************


#pragma once

#include <tbb/enumerable_thread_specific.h>
#include "DataStructures/Labels/BasicLabelSet.h"
#include "DataStructures/Labels/SimdLabelSet.h"


#include "Algorithms/KaRRi/InputConfig.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/KaRRi/PbnsAssignments/VehicleLocator.h"
#include "Algorithms/KaRRi/EllipticBCH/FeasibleEllipticDistances.h"
#include "Algorithms/KaRRi/EllipticBCH/BatchUpdatesEllipticBucketsEnvironment.h"
#include "Algorithms/KaRRi/EllipticBCH/EllipticBCHSearches.h"
#include "Algorithms/KaRRi/EllipticBCH/PDLocsAtExistingStopsFinder.h"
#include "Algorithms/KaRRi/CostCalculator.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/RequestState/RelevantPDLocs.h"
#include "Algorithms/KaRRi/PDDistanceQueries/PDDistances.h"
#include "Algorithms/KaRRi/RequestState/RelevantPDLocsFilter.h"
#include "Algorithms/KaRRi/OrdinaryAssignments/OrdinaryAssignmentsFinder.h"
#include "Algorithms/KaRRi/PbnsAssignments/PBNSAssignmentsFinder.h"
#include "Algorithms/KaRRi/PalsAssignments/PALSAssignmentsFinder.h"
#include "Algorithms/KaRRi/DalsAssignments/DALSAssignmentsFinder.h"
#include "Algorithms/KaRRi/RequestState/VehicleToPDLocQuery.h"
#include "Algorithms/KaRRi/RequestState/RequestStateInitializer.h"
#include "Algorithms/KaRRi/AssignmentFinder.h"
#include "Algorithms/KaRRi/PDDistanceQueries/BCHStrategy.h"
#include "Algorithms/KaRRi/PDDistanceQueries/CHStrategy.h"

#include "Algorithms/KaRRi/TransferPoints/OrdinaryTransfers/DirectTransferDistances/BCHDirectTransferDistancesFinder.h"
#include "Algorithms/KaRRi/TransferPoints/TransfersALS/TransfersALSPVeh/TransferALSPVehFinder.h"
#include "Algorithms/KaRRi/TransferPoints/TransfersALS/TransfersALSDVeh/TransferALSDVehFinder.h"


#if KARRI_PALS_STRATEGY == KARRI_COL

#include "Algorithms/KaRRi/PalsAssignments/CollectiveBCHStrategy.h"

#elif KARRI_PALS_STRATEGY == KARRI_IND

#include "Algorithms/KaRRi/PalsAssignments/IndividualBCHStrategy.h"

#else // KARRI_PALS_STRATEGY == KARRI_DIJ

#include "Algorithms/KaRRi/PalsAssignments/DijkstraStrategy.h"

#endif

#if KARRI_DALS_STRATEGY == KARRI_COL

#include "Algorithms/KaRRi/DalsAssignments/CollectiveBCHStrategy.h"
#include "Algorithms/KaRRi/RequestState/PDLocsFinder.h"

#elif KARRI_DALS_STRATEGY == KARRI_IND

#include "Algorithms/KaRRi/DalsAssignments//IndividualBCHStrategy.h"

#else // KARRI_DALS_STRATEGY == KARRI_DIJ

#include "Algorithms/KaRRi/DalsAssignments/DijkstraStrategy.h"

#endif


namespace karri {
    template<typename VehicleInputGraph,
        typename PsgInputGraph,
        typename VehCHEnv,
        typename PsgCHEnv,
        typename EllipticBucketsT,
        typename EllipticBucketsEnvT,
        bool AreEllipticBucketsSortedByRemainingLeeway,
        typename PDLocsAtExistingStopsFinderT,
        typename LastStopBucketsT,
        bool AreLastStopBucketsSorted,
        typename LastStopAtVerticesInfo>
    struct ThreadLocalAssignmentFinderFactory {
    private:
        const VehicleInputGraph &vehicleInputGraph;
        const VehicleInputGraph &revVehicleGraph;
        const PsgInputGraph &psgInputGraph;
        const PsgInputGraph &revPsgGraph;
        const VehCHEnv &vehChEnv;
        const PsgCHEnv &psgChEnv;
        const RPHASTEnvironment &vehRphastEnv;
        const EllipticBucketsT &ellipticSourceBuckets;
        const EllipticBucketsT &ellipticTargetBuckets;
        const EllipticBucketsEnvT &ellipticBucketsEnv;
        const PDLocsAtExistingStopsFinderT &pdLocsAtExistingStopsFinder;
        const LastStopBucketsT &lastStopBuckets;
        const LastStopAtVerticesInfo &lastStopAtVerticesInfo;
        const Fleet &fleet;
        const RouteState &routeState;

        using VehicleLocatorImpl = VehicleLocator<VehicleInputGraph, VehCHEnv>;
        tbb::enumerable_thread_specific<VehicleLocatorImpl> locator;

        using EllipticBCHLabelSet = std::conditional_t<KARRI_ELLIPTIC_BCH_USE_SIMD,
            SimdLabelSet<KARRI_ELLIPTIC_BCH_LOG_K, ParentInfo::NO_PARENT_INFO>,
            BasicLabelSet<KARRI_ELLIPTIC_BCH_LOG_K, ParentInfo::NO_PARENT_INFO> >;

        using FeasibleEllipticDistancesImpl = FeasibleEllipticDistances<EllipticBCHLabelSet>;
        tbb::enumerable_thread_specific<FeasibleEllipticDistancesImpl> feasibleEllipticPickups;
        tbb::enumerable_thread_specific<FeasibleEllipticDistancesImpl> feasibleEllipticDropoffs;

        using EllipticBCHSearchesImpl = EllipticBCHSearches<VehicleInputGraph, VehCHEnv, CostCalculator::CostFunction,
            EllipticBucketsT, AreEllipticBucketsSortedByRemainingLeeway, FeasibleEllipticDistancesImpl,
            EllipticBCHLabelSet>;
        tbb::enumerable_thread_specific<EllipticBCHSearchesImpl> ellipticSearches;

        using RelevantPDLocsFilterImpl = RelevantPDLocsFilter<FeasibleEllipticDistancesImpl, VehicleInputGraph,
            VehCHEnv>;
        tbb::enumerable_thread_specific<RelevantPDLocsFilterImpl> relevantPdLocsFilter;

        using VehicleToPDLocQueryImpl = VehicleToPDLocQuery<VehicleInputGraph>;
        tbb::enumerable_thread_specific<VehicleToPDLocQueryImpl> vehicleToPdLocQuery;

        using PDDistancesLabelSet = std::conditional_t<KARRI_PD_DISTANCES_USE_SIMD,
            SimdLabelSet<KARRI_PD_DISTANCES_LOG_K, ParentInfo::NO_PARENT_INFO>,
            BasicLabelSet<KARRI_PD_DISTANCES_LOG_K, ParentInfo::NO_PARENT_INFO> >;

        using FFPDDistanceQueryImpl = std::conditional_t<KARRI_PD_STRATEGY == KARRI_BCH_PD_STRAT,
            PDDistanceQueryStrategies::BCHStrategy<VehicleInputGraph, VehCHEnv, PDDistancesLabelSet>,
            PDDistanceQueryStrategies::CHStrategy<VehicleInputGraph, VehCHEnv, PDDistancesLabelSet> >;
        tbb::enumerable_thread_specific<FFPDDistanceQueryImpl> ffPDDistanceQuery;

        // todo: The OrdinaryAssignmentsFinder does not manage any large memory on its own, could be constructed
        //  on-the-fly for one less enumerable_thread_specific.
        using OrdinaryAssignmentsFinderImpl = OrdinaryAssignmentsFinder;
        tbb::enumerable_thread_specific<OrdinaryAssignmentsFinderImpl> ordinaryInsertionsFinder;

        using CurVehLocToPickupLabelSet = PDDistancesLabelSet;
        using CurVehLocToPickupSearchesImpl = CurVehLocToPickupSearches<VehicleInputGraph, VehicleLocatorImpl, VehCHEnv,
            CurVehLocToPickupLabelSet>;
        tbb::enumerable_thread_specific<CurVehLocToPickupSearchesImpl> curVehLocToPickupSearches;

        using PBNSInsertionsFinderImpl = PBNSAssignmentsFinder<VehicleInputGraph, CurVehLocToPickupSearchesImpl>;
        tbb::enumerable_thread_specific<PBNSInsertionsFinderImpl> pbnsInsertionsFinder;


        using PALSLabelSet = std::conditional_t<KARRI_PALS_USE_SIMD,
            SimdLabelSet<KARRI_PALS_LOG_K, ParentInfo::NO_PARENT_INFO>,
            BasicLabelSet<KARRI_PALS_LOG_K, ParentInfo::NO_PARENT_INFO> >;

#if KARRI_PALS_STRATEGY == KARRI_COL

        // Use Collective-BCH PALS Strategy
        using PALSStrategy = PickupAfterLastStopStrategies::CollectiveBCHStrategy<VehicleInputGraph, VehCHEnv,
            LastStopBucketsT, AreLastStopBucketsSorted, PDDistancesLabelSet, VehicleToPDLocQueryImpl, PALSLabelSet>;

        static PALSStrategy makePalsStrategy(const VehicleInputGraph &vehicleInputGraph, const VehicleInputGraph &,
                                             const Fleet &fleet, const VehCHEnv &vehChEnv,
                                             const LastStopBucketsT &lastStopBuckets,
                                             const RouteState &routeState,
                                             VehicleToPDLocQueryImpl &vehicleToPdLocQuery,
                                             const LastStopAtVerticesInfo &) {
            return PALSStrategy(vehicleInputGraph, fleet, vehChEnv, vehicleToPdLocQuery, lastStopBuckets,
                                routeState);
        }

#elif KARRI_PALS_STRATEGY == KARRI_IND

        // Use Individual-BCH PALS Strategy
        using PALSStrategy = PickupAfterLastStopStrategies::IndividualBCHStrategy<VehicleInputGraph, VehCHEnv,
            LastStopBucketsT, AreLastStopBucketsSorted, PDDistancesImpl, PALSLabelSet>;

        static PALSStrategy makePalsStrategy(const VehicleInputGraph &vehicleInputGraph, const VehicleInputGraph &,
                                             const Fleet &fleet, const VehCHEnv &vehChEnv,
                                             const LastStopBucketsT &lastStopBuckets,
                                             const RouteState &routeState,
                                             VehicleToPDLocQueryImpl &,
                                             const LastStopAtVerticesInfo &) {
            return PALSStrategy(vehicleInputGraph, fleet, vehChEnv, lastStopBuckets, routeState);
        }

#else // KARRI_PALS_STRATEGY == KARRI_DIJ

        // Use Dijkstra PALS Strategy
        using PALSStrategy = PickupAfterLastStopStrategies::DijkstraStrategy<VehicleInputGraph, LastStopAtVerticesInfo,
            PDDistancesImpl, PALSLabelSet>;
        static PALSStrategy makePalsStrategy(const VehicleInputGraph &vehicleInputGraph,
                                             const VehicleInputGraph &revVehicleGraph,
                                             const Fleet &fleet, const VehCHEnv &,
                                             const LastStopBucketsT &,
                                             const RouteState &routeState,
                                             VehicleToPDLocQueryImpl &,
                                             const LastStopAtVerticesInfo &lastStopAtVerticesInfo) {
            return PALSStrategy(vehicleInputGraph, revVehicleGraph, fleet, routeState, lastStopAtVerticesInfo);
        }
#endif
        tbb::enumerable_thread_specific<PALSStrategy> palsStrategy;

        // todo: The PALSAssignmentsFinder does not manage any large memory on its own, could be constructed
        //  on-the-fly for one less enumerable_thread_specific.
        using PALSInsertionsFinderImpl = PALSAssignmentsFinder<VehicleInputGraph, PALSStrategy, LastStopAtVerticesInfo>;
        tbb::enumerable_thread_specific<PALSInsertionsFinderImpl> palsInsertionsFinder;


        using DALSLabelSet = std::conditional_t<KARRI_DALS_USE_SIMD,
            SimdLabelSet<KARRI_DALS_LOG_K, ParentInfo::NO_PARENT_INFO>,
            BasicLabelSet<KARRI_DALS_LOG_K, ParentInfo::NO_PARENT_INFO> >;
#if KARRI_DALS_STRATEGY == KARRI_COL
        // Use Collective-BCH DALS Strategy
        using DALSStrategy = DropoffAfterLastStopStrategies::CollectiveBCHStrategy<VehicleInputGraph, VehCHEnv,
            LastStopBucketsT, AreLastStopBucketsSorted, CurVehLocToPickupSearchesImpl>;

        static DALSStrategy makeDalsStrategy(const VehicleInputGraph &vehicleInputGraph, const VehicleInputGraph &,
                                             const Fleet &fleet, const VehCHEnv &vehChEnv,
                                             const LastStopBucketsT &lastStopBuckets,
                                             const RouteState &routeState,
                                             CurVehLocToPickupSearchesImpl &curVehLocToPickupSearches,
                                             const LastStopAtVerticesInfo &) {
            return DALSStrategy(vehicleInputGraph, fleet, routeState, vehChEnv, lastStopBuckets,
                                curVehLocToPickupSearches);
        }
#elif KARRI_DALS_STRATEGY == KARRI_IND
        // Use Individual-BCH DALS Strategy
        using DALSStrategy = DropoffAfterLastStopStrategies::IndividualBCHStrategy<VehicleInputGraph, VehCHEnv,
            LastStopBucketsT, AreLastStopBucketsSorted, CurVehLocToPickupSearchesImpl, DALSLabelSet>;

        static DALSStrategy makeDalsStrategy(const VehicleInputGraph &vehicleInputGraph, const VehicleInputGraph &,
                                             const Fleet &fleet, const VehCHEnv &vehChEnv,
                                             const LastStopBucketsT &lastStopBuckets,
                                             const RouteState &routeState,
                                             CurVehLocToPickupSearchesImpl &curVehLocToPickupSearches,
                                             const LastStopAtVerticesInfo &) {
            return DALSStrategy(vehicleInputGraph, fleet, vehChEnv, lastStopBuckets, curVehLocToPickupSearches,
                                routeState);
        }
#else // KARRI_DALS_STRATEGY == KARRI_DIJ
        // Use Dijkstra DALS Strategy
        using DALSStrategy = DropoffAfterLastStopStrategies::DijkstraStrategy<VehicleInputGraph, LastStopAtVerticesInfo,
            CurVehLocToPickupSearchesImpl, DALSLabelSet>;
        static DALSStrategy makeDalsStrategy(const VehicleInputGraph &vehicleInputGraph,
                                             const VehicleInputGraph &revVehicleGraph,
                                             const Fleet &fleet, const VehCHEnv &,
                                             const LastStopBucketsT &,
                                             const RouteState &routeState,
                                             CurVehLocToPickupSearchesImpl &curVehLocToPickupSearches,
                                             const LastStopAtVerticesInfo &lastStopAtVerticesInfo) {
            return DALSStrategy(vehicleInputGraph, revVehicleGraph, fleet, curVehLocToPickupSearches, routeState,
                                lastStopAtVerticesInfo);
        }
#endif
        tbb::enumerable_thread_specific<DALSStrategy> dalsStrategy;

        // todo: The DALSAssignmentsFinder does not manage any large memory on its own, could be constructed
        //  on-the-fly for one less enumerable_thread_specific.
        using DALSInsertionsFinderImpl = DALSAssignmentsFinder<DALSStrategy>;
        tbb::enumerable_thread_specific<DALSInsertionsFinderImpl> dalsInsertionsFinder;

        using RequestStateInitializerImpl = RequestStateInitializer<VehicleInputGraph, PsgInputGraph, VehCHEnv,
            PsgCHEnv>;
        tbb::enumerable_thread_specific<RequestStateInitializerImpl> requestStateInitializer;

        using PDLocsFinderImpl = PDLocsFinder<VehicleInputGraph, PsgInputGraph, VehicleToPDLocQueryImpl>;
        tbb::enumerable_thread_specific<PDLocsFinderImpl> pdLocsFinder;


        using InsertionAsserterImpl = InsertionAsserter<VehicleInputGraph, VehCHEnv>;
        tbb::enumerable_thread_specific<InsertionAsserterImpl> insertionAsserter;

#if KARRI_TRANSFER_HEURISTIC_LEVEL < 2 || KARRI_TRANSFER_USE_DIJKSTRA_ELLIPSE_RECONSTRUCTION
        using EllipseReconstructorLabelSet = std::conditional_t<KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_USE_SIMD,
            SimdLabelSet<KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_LOG_K, ParentInfo::NO_PARENT_INFO>,
            BasicLabelSet<KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_LOG_K, ParentInfo::NO_PARENT_INFO> >;
#endif

#if KARRI_TRANSFER_USE_DIJKSTRA_ELLIPSE_RECONSTRUCTION
        using EllipseReconstructorImpl = DijkstraEllipseReconstructor<VehicleInputGraph, VehCHEnv, TravelTimeAttribute,
            EllipseReconstructorLabelSet>;
#else
#if KARRI_TRANSFER_HEURISTIC_LEVEL < 2

        static constexpr int ELLIPSES_TOP_VERTICES_DIVISOR =
                KARRI_TRANSFER_HEURISTIC_LEVEL == 0 ? 1 : KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_TOP_VERTICES_DIVISOR;

        static constexpr bool PARALLELIZE_PHAST_DETOUR_ELLIPSES = false;

        using EllipseReconstructorImpl = PHASTEllipseReconstructor<VehicleInputGraph, VehCHEnv, EllipticBucketsEnvT,
            PARALLELIZE_PHAST_DETOUR_ELLIPSES, ELLIPSES_TOP_VERTICES_DIVISOR, TraversalCostAttribute,
            EllipseReconstructorLabelSet>;
        tbb::enumerable_thread_specific<EllipseReconstructorImpl> ellipseReconstructor;
#else
        using EllipseReconstructorImpl = OnlyAtStopEllipseReconstructor;
        EllipseReconstructorImpl ellipseReconstructor(routeState);
        tbb::enumerable_thread_specific<EllipseReconstructorImpl> ellipseReconstructor;
#endif
#endif

        static constexpr bool KARRIT_USE_TP_PARETO_CHECKS = KARRI_TRANSFER_USE_TRANSFER_POINT_PARETO_CHECKS;
        static constexpr bool KARRIT_USE_COST_LOWER_BOUNDS = KARRI_TRANSFER_USE_COST_LOWER_BOUNDS;

#if KARRI_TRANSFER_HEURISTIC_LEVEL < 2
        using DirectTransferDistancesLabelSet = std::conditional_t<KARRI_TRANSFER_DIRECT_DISTANCES_USE_SIMD,
            SimdLabelSet<KARRI_TRANSFER_DIRECT_DISTANCES_LOG_K, ParentInfo::NO_PARENT_INFO>,
            BasicLabelSet<KARRI_TRANSFER_DIRECT_DISTANCES_LOG_K, ParentInfo::NO_PARENT_INFO> >;
        using DirectTransferDistancesFinder = BCHDirectTransferDistancesFinder<VehCHEnv,
            DirectTransferDistancesLabelSet>;
        tbb::enumerable_thread_specific<DirectTransferDistancesFinder> pickupToTransferDistancesFinder;
        tbb::enumerable_thread_specific<DirectTransferDistancesFinder> transferToDropoffDistancesFinder;

        using OrdinaryTransferInsertionsImpl = OrdinaryTransferFinder<VehicleInputGraph, VehCHEnv,
            CurVehLocToPickupSearchesImpl, KARRIT_USE_COST_LOWER_BOUNDS, KARRIT_USE_TP_PARETO_CHECKS,
            InsertionAsserterImpl, DirectTransferDistancesFinder>;
        tbb::enumerable_thread_specific<OrdinaryTransferInsertionsImpl> ordinaryTransferInsertions;
#else
        using OrdinaryTransferInsertionsImpl = NoOpOrdinaryTransferFinder;
        OrdinaryTransferInsertionsImpl ordinaryTransferInsertions;
        tbb::enumerable_thread_specific<OrdinaryTransferInsertionsImpl> ordinaryTransferInsertionsDummy;
#endif

        using TransfersPickupALSStrategy = Transfers::TransfersPickupALSBCHStrategy<VehicleInputGraph, VehCHEnv,
            LastStopBucketsT, AreLastStopBucketsSorted, LastStopAtVerticesInfo, PALSLabelSet>;
        tbb::enumerable_thread_specific<TransfersPickupALSStrategy> transfersPickupALSStrategy;

        using TransfersDropoffALSStrategy = Transfers::TransfersDropoffALSBCHStrategy<VehicleInputGraph, VehCHEnv,
            LastStopBucketsT, AreLastStopBucketsSorted, DALSLabelSet>;
        tbb::enumerable_thread_specific<TransfersDropoffALSStrategy> transfersDropoffALSStrategy;


        using TALSLabelSet = std::conditional_t<KARRI_TRANSFER_TALS_USE_SIMD,
            SimdLabelSet<KARRI_TRANSFER_TALS_LOG_K, ParentInfo::NO_PARENT_INFO>,
            BasicLabelSet<KARRI_TRANSFER_TALS_LOG_K, ParentInfo::NO_PARENT_INFO> >;

#if KARRI_TRANSFER_TALS_STRAT == KARRI_TRANSFER_TALS_PHAST

        using TransferStrategyALSImpl = PHASTStrategyALS<VehicleInputGraph, VehCHEnv, RPHASTEnvironment, TALSLabelSet,
            std::ofstream>;
        tbb::enumerable_thread_specific<TransferStrategyALSImpl> transferALSStrategy;
#else
        using TransferStrategyALSImpl = CHStrategyALS<VehicleInputGraph, VehCHEnv, TALSLabelSet>;
        TransferStrategyALSImpl transferALSStrategy(routeState, fleet, vehicleInputGraph, *vehChEnv);
        tbb::enumerable_thread_specific<TransferStrategyALSImpl> transferALSStrategy;

#endif

        using TransferALSPVehFinderImpl = TransferALSPVehFinder<VehicleInputGraph, VehCHEnv, TransferStrategyALSImpl,
            TransfersPickupALSStrategy, CurVehLocToPickupSearchesImpl, KARRIT_USE_COST_LOWER_BOUNDS,
            KARRIT_USE_TP_PARETO_CHECKS, InsertionAsserterImpl>;
        tbb::enumerable_thread_specific<TransferALSPVehFinderImpl> transferALSPVehFinder;

        using TransferALSDVehFinderImpl = TransferALSDVehFinder<VehicleInputGraph, VehCHEnv, TransferStrategyALSImpl,
            CurVehLocToPickupSearchesImpl, KARRIT_USE_COST_LOWER_BOUNDS, KARRIT_USE_TP_PARETO_CHECKS,
            InsertionAsserterImpl>;
        tbb::enumerable_thread_specific<TransferALSDVehFinderImpl> transferALSDVehFinder;

        using AssignmentsWithTransferFinderImpl = AssignmentsWithTransferFinder<OrdinaryTransferInsertionsImpl,
            TransferALSPVehFinderImpl,
            TransferALSDVehFinderImpl,
            TransfersDropoffALSStrategy,
            EllipseReconstructorImpl,
            InsertionAsserterImpl>;
        tbb::enumerable_thread_specific<AssignmentsWithTransferFinderImpl> assignmentsWithTransferFinder;

    public:
        ThreadLocalAssignmentFinderFactory(const VehicleInputGraph &vehicleInputGraph,
                                           const VehicleInputGraph &revVehicleGraph,
                                           const PsgInputGraph &psgInputGraph,
                                           const PsgInputGraph &revPsgGraph,
                                           const VehCHEnv &vehChEnv,
                                           const PsgCHEnv &psgChEnv,
                                           const RPHASTEnvironment &vehRphastEnv,
                                           const EllipticBucketsT &ellipticSourceBuckets,
                                           const EllipticBucketsT &ellipticTargetBuckets,
                                           const EllipticBucketsEnvT &ellipticBucketsEnv,
                                           const PDLocsAtExistingStopsFinderT &pdLocsAtExistingStopsFinder,
                                           const LastStopBucketsT &lastStopBuckets,
                                           const LastStopAtVerticesInfo &lastStopAtVerticesInfo,
                                           const Fleet &fleet,
                                           const RouteState &routeState)
            : vehicleInputGraph(vehicleInputGraph),
              revVehicleGraph(revVehicleGraph),
              psgInputGraph(psgInputGraph),
              revPsgGraph(revPsgGraph),
              vehChEnv(vehChEnv),
              psgChEnv(psgChEnv),
              vehRphastEnv(vehRphastEnv),
              ellipticSourceBuckets(ellipticSourceBuckets),
              ellipticTargetBuckets(ellipticTargetBuckets),
              ellipticBucketsEnv(ellipticBucketsEnv),
              pdLocsAtExistingStopsFinder(pdLocsAtExistingStopsFinder),
              lastStopBuckets(lastStopBuckets),
              lastStopAtVerticesInfo(lastStopAtVerticesInfo),
              fleet(fleet),
              routeState(routeState),
              locator([&]() {
                  return VehicleLocatorImpl(
                      vehicleInputGraph, vehChEnv, routeState);
              }),
              feasibleEllipticPickups([&]() {
                  return FeasibleEllipticDistancesImpl(fleet.size(), routeState);
              }),
              feasibleEllipticDropoffs([&]() {
                  return FeasibleEllipticDistancesImpl(fleet.size(), routeState);
              }),
              ellipticSearches([&]() {
                  return EllipticBCHSearchesImpl(
                      vehicleInputGraph, fleet, ellipticSourceBuckets, ellipticTargetBuckets, vehChEnv, routeState);
              }),
              relevantPdLocsFilter([&]() {
                  return RelevantPDLocsFilterImpl(fleet, vehicleInputGraph, vehChEnv, routeState);
              }),
              vehicleToPdLocQuery([&]() {
                  return VehicleToPDLocQueryImpl(vehicleInputGraph, revVehicleGraph);
              }),
              ffPDDistanceQuery([&]() {
                  return FFPDDistanceQueryImpl(vehicleInputGraph, vehChEnv);
              }),
              ordinaryInsertionsFinder([&]() {
                  return OrdinaryAssignmentsFinderImpl(fleet, routeState);
              }),
              curVehLocToPickupSearches([&]() {
                  return CurVehLocToPickupSearchesImpl(
                      vehicleInputGraph, locator.local(), vehChEnv, routeState, fleet.size());
              }),
              pbnsInsertionsFinder([&]() {
                  return PBNSInsertionsFinderImpl(vehicleInputGraph, curVehLocToPickupSearches.local(), fleet,
                                                  routeState);
              }),
              palsStrategy([&]() {
                  return makePalsStrategy(
                      vehicleInputGraph, revVehicleGraph, fleet, vehChEnv, lastStopBuckets, routeState,
                      vehicleToPdLocQuery.local(), lastStopAtVerticesInfo);
              }),
              palsInsertionsFinder([&]() {
                  return PALSInsertionsFinderImpl(
                      palsStrategy.local(), vehicleInputGraph, fleet, lastStopAtVerticesInfo, routeState);
              }),
              dalsStrategy([&]() {
                  return makeDalsStrategy(
                      vehicleInputGraph, revVehicleGraph, fleet, vehChEnv, lastStopBuckets, routeState,
                      curVehLocToPickupSearches.local(), lastStopAtVerticesInfo);
              }),
              dalsInsertionsFinder([&]() {
                  return DALSInsertionsFinderImpl(dalsStrategy.local());
              }),
              requestStateInitializer([&]() {
                  return RequestStateInitializerImpl(vehicleInputGraph, psgInputGraph, vehChEnv, psgChEnv);
              }),
              pdLocsFinder([&]() {
                  return PDLocsFinderImpl(vehicleInputGraph, psgInputGraph, revPsgGraph, vehicleToPdLocQuery.local());
              }),
              insertionAsserter([&]() {
                  return InsertionAsserterImpl(routeState, vehicleInputGraph, vehChEnv);
              }),
              ellipseReconstructor([&]() {
                  return EllipseReconstructorImpl(vehicleInputGraph, vehChEnv, fleet, ellipticBucketsEnv, routeState);
              }),
              pickupToTransferDistancesFinder([&]() {
                  return DirectTransferDistancesFinder(vehicleInputGraph.numVertices(), vehChEnv, PDLocType::PICKUP);
              }),
              transferToDropoffDistancesFinder([&]() {
                  return DirectTransferDistancesFinder(vehicleInputGraph.numVertices(), vehChEnv, PDLocType::DROPOFF);
              }),
              ordinaryTransferInsertions([&]() {
                  return OrdinaryTransferInsertionsImpl(vehicleInputGraph,
                                                        vehChEnv,
                                                        curVehLocToPickupSearches.local(),
                                                        pickupToTransferDistancesFinder.local(),
                                                        transferToDropoffDistancesFinder.local(),
                                                        fleet, routeState,
                                                        insertionAsserter.local());
              }),
              transfersPickupALSStrategy([&]() {
                  return TransfersPickupALSStrategy(vehicleInputGraph, fleet, vehChEnv, lastStopBuckets,
                                                    lastStopAtVerticesInfo, routeState);
              }),
              transfersDropoffALSStrategy([&]() {
                  return TransfersDropoffALSStrategy(vehicleInputGraph, fleet, vehChEnv, lastStopBuckets, routeState);
              }),
              transferALSStrategy([&]() {
                  return TransferStrategyALSImpl(routeState, fleet, vehicleInputGraph, vehChEnv, vehRphastEnv);
              }),
              transferALSPVehFinder([&]() {
                  return TransferALSPVehFinderImpl(vehicleInputGraph,
                                                   vehChEnv,
                                                   transferALSStrategy.local(),
                                                   transfersPickupALSStrategy.local(),
                                                   curVehLocToPickupSearches.local(),
                                                   fleet,
                                                   routeState,
                                                   insertionAsserter.local());
              }),
              transferALSDVehFinder([&]() {
                  return TransferALSDVehFinderImpl(vehicleInputGraph,
                                                   vehChEnv,
                                                   transferALSStrategy.local(),
                                                   curVehLocToPickupSearches.local(),
                                                   fleet,
                                                   routeState,
                                                   insertionAsserter.local());
              }),
        assignmentsWithTransferFinder([&]() {
            return AssignmentsWithTransferFinderImpl(ordinaryTransferInsertions.local(),
                                                     transferALSPVehFinder.local(),
                                                     transferALSDVehFinder.local(),
                                                     transfersDropoffALSStrategy.local(),
                                                     ellipseReconstructor.local(),
                                                     routeState,
                                                     insertionAsserter.local());
        })
         {
        }

        using InsertionFinderImpl = AssignmentFinder<
            VehicleInputGraph,
            FeasibleEllipticDistancesImpl,
            RequestStateInitializerImpl,
            PDLocsFinderImpl,
            PDLocsAtExistingStopsFinderT,
            EllipticBCHSearchesImpl,
            FFPDDistanceQueryImpl,
            OrdinaryAssignmentsFinderImpl,
            PBNSInsertionsFinderImpl,
            PALSInsertionsFinderImpl,
            DALSInsertionsFinderImpl,
            RelevantPDLocsFilterImpl,
            AssignmentsWithTransferFinderImpl,
            InsertionAsserterImpl>;

        InsertionFinderImpl getThreadLocalAssignmentFinder() {
            return InsertionFinderImpl(vehicleInputGraph,
                                       requestStateInitializer.local(),
                                       pdLocsFinder.local(),
                                       pdLocsAtExistingStopsFinder,
                                       feasibleEllipticPickups.local(),
                                       feasibleEllipticDropoffs.local(),
                                       ellipticSearches.local(),
                                       ffPDDistanceQuery.local(),
                                       ordinaryInsertionsFinder.local(),
                                       pbnsInsertionsFinder.local(),
                                       palsInsertionsFinder.local(),
                                       dalsInsertionsFinder.local(),
                                       relevantPdLocsFilter.local(),
                                       assignmentsWithTransferFinder.local(),
                                       insertionAsserter.local());
        }
    };
} // karri
