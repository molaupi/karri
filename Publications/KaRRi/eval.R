
# Converts number of seconds to MM:SS format.
convertToMMSS <- function(seconds) {
  seconds <- as.double(seconds)
  seconds <- round(seconds)
  minutes <- floor(seconds / 60)
  seconds <- seconds %% 60
  return(sprintf("%02d:%02d", minutes, seconds))
}

# Converts number of seconds to HH:MM format.
convertToHHMM <- function(seconds) {
  seconds <- as.double(seconds)
  seconds <- round(seconds)
  minutes <- floor(seconds / 60)
  hours <- floor(minutes / 60)
  minutes <- minutes %% 60
  return(sprintf("%02d:%02d", hours, minutes))
}


# Given the path to the result files of a KaRRi run (e.g. 
# "<output-dir>/Berlin-1pct_pedestrian/karri-col-simd_300_300"), 
# this function returns an overview over the solution quality of the assignments.
quality <- function(file_base, mode_choice=TRUE) {
  asgnstats <- fread(paste0(file_base, ".assignmentquality.csv"))
  setkey(asgnstats, request_id)
  # asgnstats <- asgnstats[order(asgnstats$request_id)]
  legstats <- fread(paste0(file_base, ".legstats.csv"))
  bestasgns <- fread(paste0(file_base, ".bestassignments.csv"))
  setkey(bestasgns, request_id)
  # bestasgns <- bestasgns[order(bestasgns$request_id)]
  ratioTaxi <- 1
  if (mode_choice) {
    modes <- fread(paste0(file_base, ".modechoice.csv"))
    setkey(modes, request_id)
    # modes <- modes[order(modes$request_id)]
    numTaxi <- sum(modes[["mode"]] == "Taxi")
    numTotal <- nrow(modes)
    ratioTaxi <- numTaxi / numTotal
    stopifnot(numTotal == nrow(asgnstats))
    stopifnot(numTotal == nrow(bestasgns))
    stopifnot(sum(asgnstats$request_id != modes$request_id) == 0)
    stopifnot(sum(bestasgns$request_id != modes$request_id) == 0)
    modes <- modes[mode == "Taxi"]
    setkey(modes, request_id)
    asgnstats <- asgnstats[modes, nomatch=0]
    bestasgns <- bestasgns[modes, nomatch=0]
  }
  
  eventsimstats <- fread(paste0(file_base, ".eventsimulationstats.csv"))
  num.Vehicles <- sum(eventsimstats$type == "VehicleStartup")
  
  df <- data.frame(
             wait_time_avg = c(mean(asgnstats$wait_time) / 10), # avg wait time for each request
             wait_time_q95 = c(quantile(asgnstats$wait_time, 0.95) / 10), # q95 wait time for each request
             ride_time_avg = c(mean(asgnstats$ride_time) / 10), # avg ride time for each request
             ride_time_q95 = c(quantile(asgnstats$ride_time, 0.95) / 10), # q95 ride time for each request
             trip_time_avg = c(mean(asgnstats$trip_time) / 10), # avg trip time for each request
             trip_time_q95 = c(quantile(asgnstats$trip_time, 0.95) / 10), # q95 trip time for each request
             direct_time_avg = c(mean(bestasgns$direct_od_dist) / 10), # avg direct vehicle time from origin to destination
             walk_to_pickup_avg=c(mean(asgnstats$walk_to_pickup_time) / 10), # avg walking time to pickup
             walk_to_pickup_q95=c(quantile(asgnstats$walk_to_pickup_time, 0.95) / 10), # q95 walking time to pickup
             walk_to_dropoff_avg=c(mean(asgnstats$walk_to_dropoff_time) / 10), # avg walking time to dropoff
             walk_to_dropoff_q95=c(quantile(asgnstats$walk_to_dropoff_time, 0.95) / 10), # q95 walking time to dropoff
             stop_time_avg = c(sum(legstats$stop_time) / num.Vehicles / 10), # avg total stop time for each vehicle
             empty_time_avg = c(sum(legstats[legstats$occupancy == 0, "drive_time"]) / num.Vehicles / 10), # avg time spent driving empty for each vehicle
             occ_time_avg = c(sum(legstats[legstats$occupancy > 0, "drive_time"]) / num.Vehicles / 10) # avg time spent driving occupied for each vehicle
             )
  
  df$op_time_avg <- df$stop_time_avg + df$empty_time_avg + df$occ_time_avg # avg total operation time for each vehicle
  df$occ_rate_avg <- sum(legstats$drive_time * legstats$occupancy) / sum(legstats$drive_time) # avg occupation rate while driving for each vehicle
  df$cost <- mean(asgnstats$cost) # avg cost (according to cost function used in KaRRi) for each vehicle
  
  # Reformat passenger times to MM:SS
  psg_time_cols <- c("wait_time_avg", "wait_time_q95", 
                     "ride_time_avg", "ride_time_q95", 
                     "trip_time_avg", "trip_time_q95",
                     "direct_time_avg",
                     "walk_to_pickup_avg", "walk_to_pickup_q95",
                     "walk_to_dropoff_avg", "walk_to_dropoff_q95"
                     )
  df[, colnames(df) %in% psg_time_cols] <- convertToMMSS(df[, colnames(df) %in% psg_time_cols])
  
  # Reformat vehicle times to HH:MM
  veh_time_cols <- c("stop_time_avg", "empty_time_avg", "occ_time_avg", "op_time_avg")
  df[, colnames(df) %in% veh_time_cols] <- convertToHHMM(df[, colnames(df) %in% veh_time_cols])
  
  df["service_rate"] <- c(ratioTaxi)
  
  print(df)
}

# Given the paths to the result files of two KaRRi runs, this functions checks
# whether all assignments are the same in both runs.
compareBestAssignments <- function(file1, file2) {
  bestins1 <- read.csv(paste0(file1, ".bestassignments.csv"))
  bestins2 <- read.csv(paste0(file2, ".bestassignments.csv"))
    
  bestins1 <- bestins1[order(bestins1$request_id),]
  bestins2 <- bestins2[order(bestins2$request_id),]
  
  relcols <- c("request_id", 
               "request_time", 
               "direct_od_dist", 
               "vehicle_id",
               "pickup_insertion_point",
               "dropoff_insertion_point",
               "dist_to_pickup",
               "dist_from_pickup",
               "dist_to_dropoff",
               "dist_from_dropoff",
               "pickup_id",
               "pickup_walking_dist",
               "dropoff_id",
               "dropoff_walking_dist",
               "num_stops",
               "pickup_location",
               "dropoff_location",
               "stop_loc_before_pickup",
               "stop_loc_before_dropoff")
  rel1 <- bestins1[, relcols]
  rel2 <- bestins2[, relcols]
  
  # Get smallest row index where at least one value differs
  idx <- match(TRUE, rowSums(rel1 != rel2) > 0)
  if (is.na(idx)) {
    print("All best insertions are equal.")
  } else {
    print(bestins1[idx, "request_id"])
    row1 <- bestins1[idx,]
    row2 <- bestins2[idx,]
    View(rbind(row1, row2))
  }
}

compareFiles <- function(file1, file2, ending, trim = TRUE, ignore_columns=c()) {
  df1 <- fread(paste0(file1, ending))
  df2 <- fread(paste0(file2, ending))
  
  for (col in ignore_columns) {
    if (col %in% names(df1))
      df1[, c(col) := NULL]
    if (col %in% names(df2))
      df2[, c(col) := NULL]
  }
  
  df1 <- df1[order(df1$request_id)]
  df2 <- df2[order(df2$request_id)]
  
  if (nrow(df1) > nrow(df2))
    df1 <- df1[1:nrow(df2)]
  if (nrow(df2) > nrow(df1))
    df2 <- df2[1:nrow(df1)]
  
  # Get smallest row index where at least one value differs
  idx <- match(TRUE, rowSums(df1 != df2) > 0)
  if (is.na(idx)) {
    print("All rows are equal in non-ignored columns.")
  } else {
    print(df1[idx, "request_id"])
    row1 <- df1[idx]
    row2 <- df2[idx]
    View(rbind(row1, row2), paste0("diff-", ending))
  }
}

compareModeChoice <- function(file1, file2, trim = TRUE) {
  compareFiles(file1, file2, ".modechoice.csv", trim)
}

library(data.table)
perfStats <- function(file_base, type_name, single_dispatched_subset=FALSE, n = NA) {
  stats <- fread(paste0(file_base, ".perf_", type_name, ".csv"))
  setkey(stats, request_id)
  if (single_dispatched_subset) {
    dfsubset <- fread(paste0(file_base, ".requestsdispatchedsingle.csv"))
    setkey(dfsubset, request_id)
    stats <- stats[dfsubset, nomatch=0L]
  }
  
  if (!is.na(n)) {
    if (n > nrow(stats))
      stop("n is greater than number of rows in stats file")
    stats <- stats[1:n, ]
  }
  
  stats <- stats[, request_id := NULL]
  stats <- stats[, lapply(.SD, mean)]
  stats <- round(stats, 2)
  return(stats)
}

getFirstDiffInEllipticBchStats <- function(file1, file2) {
  stats1 <- fread(paste0(file1, ".perf_ellipticbch.csv"))
  stats2 <- fread(paste0(file2, ".perf_ellipticbch.csv"))
  
  if (nrow(stats1) < nrow(stats2))
    stats2 <- stats2[1:nrow(stats1),]
  if (nrow(stats2) < nrow(stats1))
    stats1 <- stats1[1:nrow(stats2),]
  
  relcols <- c("request_id", 
               "pickup_num_edge_relaxations", 
               "pickup_num_vertices_settled", 
               "pickup_num_entries_scanned",
               "pickup_num_vehicles_seen",
               "pickup_num_stops_seen",
               "dropoff_num_edge_relaxations",
               "dropoff_num_edge_relaxations", 
               "dropoff_num_vertices_settled", 
               "dropoff_num_entries_scanned",
               "dropoff_num_vehicles_seen",
               "dropoff_num_stops_seen")
  rel1 <- stats1[, ..relcols]
  rel2 <- stats2[, ..relcols]
  
  # Get smallest row index where at least one value differs
  idx <- match(TRUE, rowSums(rel1 != rel2) > 0)
  if (is.na(idx)) {
    print("All non-time elliptic perf metrics match.")
  } else {
    print(stats1[idx, "request_id"])
    row1 <- stats1[idx,]
    row2 <- stats2[idx,]
    View(rbind(row1, row2))
  }
}

# Given the path to the result files of a KaRRi run, this function returns an 
# overview over the average runtime per request (in microseconds).
overallPerfStats <- function(file_base, single_dispatched_subset = FALSE) {
  perfStats(file_base, "overall", single_dispatched_subset)
}

# Given the path to the result files of a KaRRi run, this function returns 
# performance statistics for the initialization phase.
initreqPerfStats <- function(file_base, single_dispatched_subset = FALSE) {
  perfStats(file_base, "initreq", single_dispatched_subset)
}

# Given the path to the result files of a KaRRi run, this function returns 
# performance statistics for the elliptic BCH searches
ellipticBchPerfStats <- function(file_base, single_dispatched_subset = FALSE) {
  perfStats(file_base, "ellipticbch", single_dispatched_subset)
}

# Given the path to the result files of a KaRRi run, this function returns 
# performance statistics for the computation of PD-distances.
pdDistancesPerfStats <- function(file_base, single_dispatched_subset = FALSE) {
  perfStats(file_base, "pddistances", single_dispatched_subset)
}

# Given the path to the result files of a KaRRi run, this function returns 
# performance statistics for the ordinary insertions phase.
ordPerfStats <- function(file_base, single_dispatched_subset = FALSE) {
  perfStats(file_base, "ord", single_dispatched_subset)
}

# Given the path to the result files of a KaRRi run, this function returns 
# performance statistics for the PBNS phase.
pbnsPerfStats <- function(file_base, single_dispatched_subset = FALSE) {
  perfStats(file_base, "pbns", single_dispatched_subset)
}

# Given the path to the result files of a KaRRi run, this function returns 
# performance statistics for the PALS phase.
palsPerfStats <- function(file_base, single_dispatched_subset = FALSE) {
  perfStats(file_base, "pals", single_dispatched_subset)
}

# Given the path to the result files of a KaRRi run, this function returns 
# performance statistics for the DALS phase.
dalsPerfStats <- function(file_base, single_dispatched_subset = FALSE) {
  perfStats(file_base, "dals", single_dispatched_subset)
}

# Given the path to the result files of a KaRRi run, this function returns 
# performance statistics for updating the route state for each assignment.
updatePerfStats <- function(file_base, single_dispatched_subset=FALSE) {
  perfStats(file_base, "update", single_dispatched_subset)
}


# Given the path to the result files of a KaRRi run, this function returns
# performance statistics for the different types of events in the event 
# simulation. request_receipt_event_time contains the main work of dispatching
# requests while all other times consist only of advancing the event 
# simulation by logging events and introducing new events.
eventSimulationPerfStats <- function(file_base) {
  
  stats <- read.csv(paste0(file_base, ".eventsimulationstats.csv"))
  df <- data.frame(
    vehicle_startup_event_time = c(mean(stats[stats$type=="VehicleStartup", ]$running_time)),
    vehicle_arrival_event_time = c(mean(stats[stats$type=="VehicleArrival", ]$running_time)),
    vehicle_departure_event_time = c(mean(stats[stats$type=="VehicleDeparture", ]$running_time)),
    vehicle_shutdown_event_time = c(mean(stats[stats$type=="VehicleShutdown", ]$running_time)),
    request_receipt_event_time = c(mean(stats[stats$type=="RequestReceipt",]$running_time)),
    request_batch_dispatch_event_time = c(mean(stats[stats$type=="RequestBatchDispatch",]$running_time)),
    request_pure_walking_arrival_event_time = c(mean(stats[stats$type=="RequestWalkingArrival",]$running_time))
  )
  
  print(df)
  
  asgnstats <- read.csv(paste0(file_base, ".assignmentquality.csv"))
  num.requests <- nrow(asgnstats)
  print(paste0("Mean dispatch time per request: ", sum(stats[stats$type=="RequestBatchDispatch", ]$running_time) / num.requests))
  
}

# 
# library(data.table)
# batchDispatchPerfStats <- function(file_base, 
#                                    num_threads = 1, 
#                                    single_dispatch_subset=FALSE,
#                                    group_by_occurrence_time=FALSE) {
#   
#   stats <- fread(paste0(file_base, ".batchdispatchstats.csv"))
#   if (single_dispatch_subset) {
#     stats <- stats[is_single_request == "1"]
#   }
#   
#   print(paste0("Total time (batch dispatch): ", sum(stats$find_assignments_running_time + stats$choose_accepted_running_time + stats$update_system_state_running_time) / 1000000000, "s"))
#   
#   stats$num_batches <- as.integer(ceiling(stats$num_requests / num_threads))
#   
#   if (group_by_occurrence_time) {
#     stats <- stats[, .(
#       num_iterations = max(iteration),
#       assignments_work = sum(num_requests),
#       assignments_span = sum(num_batches),
#       num_requests = max(num_requests),
#       find_assignments_running_time = sum(find_assignments_running_time, na.rm=TRUE),
#       choose_accepted_running_time = sum(choose_accepted_running_time, na.rm=TRUE),
#       update_system_state_running_time = sum(update_system_state_running_time, na.rm=TRUE),
#       total_time = sum(find_assignments_running_time + choose_accepted_running_time + update_system_state_running_time, na.rm=TRUE),
#       num_elliptic_bucket_entry_deletions = sum(num_elliptic_bucket_entry_deletions, na.rm=TRUE)
#     ), by=.(occurence_time)
#     ]  
#   } else {
#   stats <- stats[, .(
#     occurence_time = max(occurence_time), # always equal for every member of group
#     num_iterations = max(iteration),
#     assignments_work = sum(num_requests),
#     assignments_span = sum(num_batches),
#     num_requests = max(num_requests),
#     find_assignments_running_time = sum(find_assignments_running_time, na.rm=TRUE),
#     choose_accepted_running_time = sum(choose_accepted_running_time, na.rm=TRUE),
#     update_system_state_running_time = sum(update_system_state_running_time, na.rm=TRUE),
#     total_time = sum(find_assignments_running_time + choose_accepted_running_time + update_system_state_running_time, na.rm=TRUE),
#     num_elliptic_bucket_entry_deletions = sum(num_elliptic_bucket_entry_deletions, na.rm=TRUE)
#   ), by=.(batch_id)
#     ]
#   }
#   
#   # print(paste0("Max num iterations: ", max(stats$num_iterations)))
#   # print(paste0("Max num iterations, occurence time: ", stats[stats$num_iterations == max(stats$num_iterations), ]$occurence_time))
#   # print(paste0("Max num iterations, num requests: ", stats[stats$num_iterations == max(stats$num_iterations), ]$num_requests))
#   # print(paste0("Max num requests: ", max(stats$num_requests)))
#   
#   means <- apply(stats,2,mean)
#   # means <- data.frame(lapply(means, function(x) format(x, nsmall=2)))
#   return(means)
# }
# 
# oldBatchDispatchPerfStats <- function(file_base) {
#   
#   stats <- read.csv(paste0(file_base, ".batchdispatchstats.csv"))
#   
#   stats <- stats %>% group_by(occurence_time) %>% summarise(
#     num_iterations = max(iteration),
#     num_requests = max(num_requests),
#     find_assignments_running_time = sum(find_assignments_running_time, na.rm=TRUE),
#     choose_accepted_running_time = sum(choose_accepted_running_time, na.rm=TRUE),
#     update_system_state_running_time = sum(update_system_state_running_time, na.rm=TRUE),
#     # num_elliptic_bucket_entry_deletions = sum(num_elliptic_bucket_entry_deletions, na.rm=TRUE)
#   )
#   
#   print(paste0("Max num iterations: ", max(stats$num_iterations)))
#   print(paste0("Max num iterations, occurence time: ", stats[stats$num_iterations == max(stats$num_iterations), ]$occurence_time))
#   print(paste0("Max num iterations, num requests: ", stats[stats$num_iterations == max(stats$num_iterations), ]$num_requests))
#   print(paste0("Max num requests: ", max(stats$num_requests)))
#   
#   means <- apply(stats,2,mean)
#   means <- data.frame(lapply(means, function(x) format(x, nsmall=2)))
#   print(means)
#   
#   
# }
# 
# 
# multiBatchDispatchPerfStats <- function(file_base, thread_nums = c(1,4,8,32,64,96), num_runs=1) {
#   
#   df <- NULL
#   for (t in thread_nums) {
#     dft <- NULL
#     for (r in 1:num_runs) {
#       dft <- rbind(dft, batchDispatchPerfStats(paste0(file_base,"_t",t,"_run",r),t))
#     }
#     dft <- apply(dft,2,mean)
#     df <- rbind(df, dft)
#   }
#   
#   df <- as.data.frame(df)
#   df[,"num_threads"]<-thread_nums
#   return(df)
# }
# 
# makeSpeedups <- function(df) {
#   num_threads <- df$num_threads
#   mat <- as.matrix(df)
#   first <- as.numeric(mat[1,])
#   mat_rel <- sweep(mat, 2, first, FUN = function(col, s) s / col)
#   dfrel <- as.data.frame(mat_rel)
#   colnames(dfrel) <- colnames(df)
#   rownames(dfrel) <- rownames(df)
#   dfrel$num_threads <- num_threads
#   return(dfrel)
# }
# 
# 
# batchUpdatePerfStats <- function(file_base) {
#   
#   stats <- read.csv(paste0(file_base, ".batch_insert_stats.csv"))
#   
#   print(paste0("Total time (batch update): ", sum(stats$total_time) / 1000000000, "s"))
#   
#   stats <- stats %>% group_by(occurence_time) %>% summarise(
#     num_iterations = max(iteration),
#     num_requests = max(num_requests),
#     update_route_state_time = sum(update_route_state_time, na.rm = TRUE),
#     ell_entry_insertions_find_insertions_time = sum(ell_entry_insertions_find_insertions_time, na.rm = TRUE),
#     ell_entry_insertions_num_insertions = sum(ell_entry_insertions_num_insertions, na.rm = TRUE),
#     ell_entry_insertions_perform_insertions_time = sum(ell_entry_insertions_perform_insertions_time, na.rm = TRUE),
#     last_stop_find_insertions_and_deletions_time = sum(last_stop_find_insertions_and_deletions_time, na.rm = TRUE),
#     last_stop_num_insertions_and_deletions = sum(last_stop_num_insertions_and_deletions, na.rm = TRUE),
#     last_stop_perform_insertions_and_deletions_time = sum(last_stop_perform_insertions_and_deletions_time, na.rm = TRUE),
#     # last_stop_update_prefix_sum_time = sum(last_stop_update_prefix_sum_time, na.rm = TRUE),
#     num_ell_source_entries_after_insertions = sum(num_ell_source_entries_after_insertions, na.rm = TRUE),
#     num_ell_target_entries_after_insertions = sum(num_ell_target_entries_after_insertions, na.rm = TRUE),
#     update_leeways_num_entries_scanned_source = sum(update_leeways_num_entries_scanned_source, na.rm = TRUE),
#     update_leeways_num_entries_scanned_target = sum(update_leeways_num_entries_scanned_target, na.rm = TRUE),
#     update_leeways_time = sum(update_leeways_time, na.rm = TRUE),
#     total_time = sum(total_time, na.rm = TRUE)
#   )
#   
#   means <- apply(stats,2,mean)
#   means <- data.frame(lapply(means, function(x) format(x, nsmall=2)))
#   print(means)
#   
# }

countBestAssignmentTypes <- function(file_base) {
  asgns <- fread(paste0(file_base, ".bestassignments.csv"))
  modes <- fread(paste0(file_base, ".modechoice.csv"))
  
    # --- taxi-only subsets
    taximodes     <- modes[mode == "Taxi"]
    setkey(taximodes, request_id)

    # restrict assignments to taxi request_ids (semi-join style)
    setkey(asgns, request_id)
    taxiasgns <- asgns[taximodes, nomatch = 0]
  
  # Let k = number of stops of vehicle before insertion, 
  #     i = index where pickup is to be inserted 
  #     j = index where dropoff is to be inserted
  
  # Ord if                0 < i <= j < k - 1
    # Ord-Ord if            0 < i < j < k - 1,
    # Ord-paired if         0 < i = j < k - 1,
  # PBNS if               0 = i <= j < k - 1,
    # PBNS-Ord if           0 = i < j < k - 1,
    # PBNS-paired if        0 = i = j < k - 1,
  # PALS if               0 <= i = j = k - 1,
  # DALS if               0 <= i < j = k - 1,
    # Ord-DALS if           0 < i < j = k - 1,
    # PBNS-DALS if          0 = i < j = k - 1
  
  # types <- apply(taxiasgns, 1, getTypeRow)
  indices <- taxiasgns[, .(i = pickup_insertion_point,
                           j = dropoff_insertion_point,
                           k = num_stops)]
  types <- indices[, .(ord = ((0 < i) & (i <= j) & (i < k - 1) & (j < k - 1)),
                       pbns = ((0 == i) & (i <= j) & (j < k - 1)),
                       pals = ((i == k - 1) & (j == k - 1)),
                       dals = ((0 <= i) & (i < j) & (j == k - 1)))]
  num_asgns <- nrow(taxiasgns)
  num_ord <- sum(types$ord)
  pct_ord <- format(num_ord / num_asgns * 100, nsmall=2)
  num_pbns <- sum(types$pbns)
  pct_pbns <- format(num_pbns / num_asgns * 100, nsmall=2)
  num_pals <- sum(types$pals)
  pct_pals <- format(num_pals / num_asgns * 100, nsmall=2)
  num_dals <- sum(types$dals)
  pct_dals <- format(num_dals / num_asgns * 100, nsmall=2)

  print(paste0("Total: ", num_asgns, ", ",
               "Ord: ", num_ord, " (", pct_ord, "%), ",
               "PBNS: ", num_pbns, " (", pct_pbns, "%), ",
               "PALS: ", num_pals, " (", pct_pals, "%), ",
               "DALS: ", num_dals, " (", pct_dals, "%)"))
  
}


tripTimeRelDirectDistanceDistributionPlot <- function(file_base,
                                                      time_var = "trip_time",
                                                      y_min=1,
                                                      bin_width = 600, # tenths of seconds,
                                                      min_num_per_bin = 1,
                                                      x_min=0 # tenths of seconds
                                                      ) {
  mc <- fread(paste0(file_base, ".modechoice.csv"))
  as <- fread(paste0(file_base, ".assignmentquality.csv"))
  ba <- fread(paste0(file_base, ".bestassignments.csv"))
  setkey(mc, request_id)
  setkey(as, request_id)
  setkey(ba, request_id)
  mc <- mc[mode == "Taxi"]
  as <- as[mc, nomatch=0]
  ba <- ba[mc, nomatch=0]
  
  df <- data.table(tt = ba$direct_od_dist, ratio = as[[time_var]] / ba$direct_od_dist)
  df[, bin_idx := floor(df$tt / bin_width)]
  agg <- df[, .(r = mean(ratio), n =.N), by = bin_idx]
  agg <- agg[order(bin_idx)]
  agg[, tt := bin_idx * bin_width]
  agg <- agg[tt >= x_min]
  agg <- agg[n >= min_num_per_bin]
  
  ggplot(agg, aes(x=tt,y=r)) + 
    geom_line() + 
    scale_x_continuous(name="direct dist", breaks=6000*(0:500), labels=function(x) convertToMMSS(x / 10)) + 
    scale_y_continuous(name=paste0(time_var, " / direct dist"), limits=c(y_min,NA), breaks=c(y_min,1,1.5,2,3,2*(2:50)))
}

library(ggplot2)
absTripStatRelDirectDistancePlot <- function(file_base,
                                                        time_var = "cost_no_trip",
                                                      y_min=0,
                                                      bin_width = 1, # tenths of seconds,
                                                      min_num_per_bin = 1,
                                                      x_min=0, # tenths of seconds
                                             x_max = NA, # tenths of seconds
                                             y_breaks=waiver(),
                                             y_label_func = waiver()
) {
  mc <- fread(paste0(file_base, ".modechoice.csv"))
  as <- fread(paste0(file_base, ".assignmentquality.csv"))
  ba <- fread(paste0(file_base, ".bestassignments.csv"))
  setkey(mc, request_id)
  setkey(as, request_id)
  setkey(ba, request_id)
  mc <- mc[mode == "Taxi"]
  as <- as[mc, nomatch=0]
  ba <- ba[mc, nomatch=0]
  as[, cost_no_trip := cost - trip_time]
  as[, walk_total_time := walk_to_pickup_time + walk_to_dropoff_time]
  # View(as)
  
  df <- data.table(tt = ba$direct_od_dist, var_vals = as[[time_var]])
  df <- df[tt <= as$ride_time]
  df <- df[order(tt)]
  df <- df[tt >= x_min]
  if (!is.na(x_max))
    df <- df[tt <= x_max]
  print(mean(df$var_vals))
  print(summary(lm(formula = var_vals ~ tt, data=df)))
  print(cor.test(df$tt, df$var_vals, method = "pearson"))
  print(cor.test(df$tt, df$var_vals, method = "spearman"))
  
  # df[, ratio := var_vals / tt]
  # df[, bin_idx := floor(df$tt / bin_width)]
  # agg <- df[, .(r = mean(ratio), n =.N, var_vals=mean(var_vals)), by = bin_idx]
  # agg <- agg[order(bin_idx)]
  # agg[, tt := bin_idx * bin_width]
  # agg <- agg[tt >= x_min]
  # if (!is.na(x_max))
  #   agg <- agg[tt <= x_max]
  # agg <- agg[n >= min_num_per_bin]
  # print(paste0("df num rows = ", nrow(df), ", agg num rows = ", nrow(agg)))
  
  # print(mean(agg$var_vals))
  # print(summary(lm(formula = var_vals ~ tt, data=agg)))
  
  ggplot(df, aes(x=tt,y=var_vals)) + 
    geom_point(size=0.02) + 
    geom_smooth(method='lm') +
    scale_x_continuous(name="direct dist", breaks=6000*(0:500), labels=function(x) convertToMMSS(x / 10)) +
    scale_y_continuous(name=time_var, limits=c(y_min, NA), breaks=y_breaks, labels=y_label_func)
    # scale_y_continuous(name="non-trip cost / direct dist", limits=c(y_min,NA), breaks=c(y_min,1,1.5,2,3,2*(2:50)))
}

histogramVar <- function(file_base,
                                             var = "direct_od_dist",
                                             y_min=0,
                                             bin_width = 600,
                                             min_num_per_bin = 1,
                                             x_min=0, # tenths of seconds
                                             x_max = NA, # tenths of seconds
                         x_breaks=waiver(),
                                              x_label_func=waiver()
) {
  mc <- fread(paste0(file_base, ".modechoice.csv"))
  as <- fread(paste0(file_base, ".assignmentquality.csv"))
  ba <- fread(paste0(file_base, ".bestassignments.csv"))
  setkey(mc, request_id)
  setkey(as, request_id)
  setkey(ba, request_id)
  mc <- mc[mode == "Taxi"]
  as <- as[mc, nomatch=0]
  ba <- ba[mc, nomatch=0]
  as[, cost_no_trip := cost - trip_time]
  
  df <- ba[as]
  df[, bin_idx := floor(df[[var]] / bin_width)]
  agg <- df[, .(n =.N), by = bin_idx]
  agg <- agg[order(bin_idx)]
  agg[[var]] <- agg$bin_idx * bin_width
  agg <- agg[get(var) >= x_min]
  if (!is.na(x_max))
    agg <- agg[get(var) <= x_max]
  agg <- agg[n >= min_num_per_bin]
  
  ggplot(agg, aes(x=agg[[var]],y=n)) + 
    geom_bar(stat = "identity", fill = "#4C72B0") +
    scale_x_continuous(breaks=x_breaks,labels=x_label_func)
  }