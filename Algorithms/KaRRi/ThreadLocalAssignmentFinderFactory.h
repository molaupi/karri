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
#include "EllipticBCH/StaticFeasibleEllipticDistances.h"

#if KARRI_PD_STRATEGY == KARRI_BCH_PD_STRAT

#include "Algorithms/KaRRi/PDDistanceQueries/BCHStrategy.h"

#else // KARRI_PD_STRATEGY == KARRI_CH_PD_STRAT
#include "Algorithms/KaRRi/PDDistanceQueries/CHStrategy.h"
#endif


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
        const EllipticBucketsT &ellipticSourceBuckets;
        const EllipticBucketsT &ellipticTargetBuckets;
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

        using FeasibleEllipticDistancesImpl = StaticFeasibleEllipticDistances<EllipticBCHLabelSet>;
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
        using PDDistancesImpl = PDDistances<PDDistancesLabelSet>;

        using FFPDDistanceQueryImpl = std::conditional_t<KARRI_PD_STRATEGY == KARRI_BCH_PD_STRAT,
            PDDistanceQueryStrategies::BCHStrategy<VehicleInputGraph, VehCHEnv, PDDistancesLabelSet>,
            PDDistanceQueryStrategies::BCHStrategy<VehicleInputGraph, VehCHEnv, PDDistancesLabelSet> >;
        tbb::enumerable_thread_specific<FFPDDistanceQueryImpl> ffPDDistanceQuery;

        // todo: The OrdinaryAssignmentsFinder does not manage any large memory on its own, could be constructed
        //  on-the-fly for one less enumerable_thread_specific.
        using OrdinaryAssignmentsFinderImpl = OrdinaryAssignmentsFinder<PDDistancesImpl>;
        tbb::enumerable_thread_specific<OrdinaryAssignmentsFinderImpl> ordinaryInsertionsFinder;

        using CurVehLocToPickupLabelSet = PDDistancesLabelSet;
        using CurVehLocToPickupSearchesImpl = CurVehLocToPickupSearches<VehicleInputGraph, VehicleLocatorImpl, VehCHEnv,
            CurVehLocToPickupLabelSet>;
        tbb::enumerable_thread_specific<CurVehLocToPickupSearchesImpl> curVehLocToPickupSearches;

        using PBNSInsertionsFinderImpl = PBNSAssignmentsFinder<PDDistancesImpl, CurVehLocToPickupSearchesImpl>;
        tbb::enumerable_thread_specific<PBNSInsertionsFinderImpl> pbnsInsertionsFinder;


        using PALSLabelSet = std::conditional_t<KARRI_PALS_USE_SIMD,
            SimdLabelSet<KARRI_PALS_LOG_K, ParentInfo::NO_PARENT_INFO>,
            BasicLabelSet<KARRI_PALS_LOG_K, ParentInfo::NO_PARENT_INFO> >;

#if KARRI_PALS_STRATEGY == KARRI_COL

        // Use Collective-BCH PALS Strategy
        using PALSStrategy = PickupAfterLastStopStrategies::CollectiveBCHStrategy<VehicleInputGraph, VehCHEnv,
            LastStopBucketsT, AreLastStopBucketsSorted, VehicleToPDLocQueryImpl, PDDistancesImpl, PALSLabelSet>;

        static PALSStrategy makePalsStrategy(const VehicleInputGraph &vehicleInputGraph, const VehicleInputGraph &,
                                             const Fleet &fleet, const VehCHEnv &vehChEnv,
                                             const LastStopBucketsT &lastStopBuckets,
                                             const RouteState &routeState,
                                             VehicleToPDLocQueryImpl &vehicleToPdLocQuery,
                                             const LastStopAtVerticesInfo&) {
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
                                             const LastStopAtVerticesInfo&) {
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
                                             const LastStopAtVerticesInfo& lastStopAtVerticesInfo) {
            return PALSStrategy(vehicleInputGraph, revVehicleGraph, fleet, routeState, lastStopAtVerticesInfo);
        }
#endif
        tbb::enumerable_thread_specific<PALSStrategy> palsStrategy;

        // todo: The PALSAssignmentsFinder does not manage any large memory on its own, could be constructed
        //  on-the-fly for one less enumerable_thread_specific.
        using PALSInsertionsFinderImpl = PALSAssignmentsFinder<VehicleInputGraph, PDDistancesImpl, PALSStrategy,
            LastStopAtVerticesInfo>;
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
                                             const LastStopAtVerticesInfo&) {
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
                                             const LastStopAtVerticesInfo&) {
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
                                             const LastStopAtVerticesInfo& lastStopAtVerticesInfo) {
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

    public:
        ThreadLocalAssignmentFinderFactory(const VehicleInputGraph &vehicleInputGraph,
                                           const VehicleInputGraph &revVehicleGraph,
                                           const PsgInputGraph &psgInputGraph,
                                           const PsgInputGraph &revPsgGraph,
                                           const VehCHEnv &vehChEnv,
                                           const PsgCHEnv &psgChEnv,
                                           const EllipticBucketsT &ellipticSourceBuckets,
                                           const EllipticBucketsT &ellipticTargetBuckets,
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
              ellipticSourceBuckets(ellipticSourceBuckets),
              ellipticTargetBuckets(ellipticTargetBuckets),
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
                  return PBNSInsertionsFinderImpl(curVehLocToPickupSearches.local(), fleet, routeState);
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
              }) {
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
            RelevantPDLocsFilterImpl>;

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
                                       relevantPdLocsFilter.local());
        }
    };
} // karri
