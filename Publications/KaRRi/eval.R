

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
quality <- function(file_base) {
  
  asgnstats <- read.csv(paste0(file_base, ".assignmentquality.csv"))
  legstats <- read.csv(paste0(file_base, ".legstats.csv"))
  bestasgns <- read.csv(paste0(file_base, ".bestassignments.csv"))
  
  eventsimstats <- read.csv(paste0(file_base, ".eventsimulationstats.csv"))
  num.Vehicles <- sum(eventsimstats$type == "VehicleStartup")
  
  print(num.Vehicles)
  
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
  
  print(df)
}

# Given the paths to the result files of two KaRRi runs, this functions checks
# whether all assignments are the same in both runs.
compareBestAssignments <- function(file1, file2) {
  bestins1 <- read.csv(paste0(file1, ".bestassignments.csv"))
  bestins2 <- read.csv(paste0(file2, ".bestassignments.csv"))
    
  bestins1 <- bestins1[order(bestins1$request_id),]
  bestins2 <- bestins2[order(bestins2$request_id),]
  
  # Get smallest row index where at least one value differs
  idx <- match(TRUE, rowSums(bestins1 != bestins2) > 0)
  if (is.na(idx)) {
    print("All best insertions are equal.")
  } else {
    print(bestins1[idx, "request_id"])
    row1 <- bestins1[idx,]
    row2 <- bestins2[idx,]
    View(rbind(row1, row2))
  }
}

perfStats <- function(file_base, type_name) {
  stats <- read.csv(paste0(file_base, ".perf_", type_name, ".csv"))
  stats <- stats[, ! colnames(stats) %in% c("request_id")]
  stats <- apply(stats, 2, mean)
  stats <- round(stats, 2)
  return(stats)
}

# Given the path to the result files of a KaRRi run, this function returns an 
# overview over the average runtime per request (in microseconds).
overallPerfStats <- function(file_base) {
  perfStats(file_base, "overall")
}

# Given the path to the result files of a KaRRi run, this function returns 
# performance statistics for the initialization phase.
initreqPerfStats <- function(file_base) {
  perfStats(file_base, "initreq")
}

# Given the path to the result files of a KaRRi run, this function returns 
# performance statistics for the elliptic BCH searches
ellipticBchPerfStats <- function(file_base) {
  perfStats(file_base, "ellipticbch")
}

# Given the path to the result files of a KaRRi run, this function returns 
# performance statistics for the computation of PD-distances.
pdDistancesPerfStats <- function(file_base) {
  perfStats(file_base, "pddistances")
}

# Given the path to the result files of a KaRRi run, this function returns 
# performance statistics for the ordinary insertions phase.
ordPerfStats <- function(file_base) {
  perfStats(file_base, "ord")
}

# Given the path to the result files of a KaRRi run, this function returns 
# performance statistics for the PBNS phase.
pbnsPerfStats <- function(file_base) {
  perfStats(file_base, "pbns")
}

# Given the path to the result files of a KaRRi run, this function returns 
# performance statistics for the PALS phase.
palsPerfStats <- function(file_base) {
  perfStats(file_base, "pals")
}

# Given the path to the result files of a KaRRi run, this function returns 
# performance statistics for the DALS phase.
dalsPerfStats <- function(file_base) {
  perfStats(file_base, "dals")
}

# Given the path to the result files of a KaRRi run, this function returns 
# performance statistics for updating the route state for each assignment.
updatePerfStats <- function(file_base) {
  perfStats(file_base, "update")
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


library(dplyr)
batchDispatchPerfStats <- function(file_base) {
  
  stats <- read.csv(paste0(file_base, ".batchdispatchstats.csv"))
  if(!("init_iteration_running_time" %in% colnames(stats)))
  {
    stats$init_iteration_running_time <- c(0)
  }
  
  
  print(paste0("Total time (batch dispatch): ", sum(stats$init_iteration_running_time + stats$find_assignments_running_time + stats$choose_accepted_running_time + stats$update_system_state_running_time) / 1000000000, "s"))
  
  stats <- stats %>% group_by(occurence_time) %>% summarise(
    num_iterations = max(iteration),
    num_requests = max(num_requests),
    init_iteration_running_time = sum(init_iteration_running_time, na.rm = TRUE),
    find_assignments_running_time = sum(find_assignments_running_time, na.rm=TRUE),
    choose_accepted_running_time = sum(choose_accepted_running_time, na.rm=TRUE),
    update_system_state_running_time = sum(update_system_state_running_time, na.rm=TRUE),
    num_elliptic_bucket_entry_deletions = sum(num_elliptic_bucket_entry_deletions, na.rm=TRUE)
  )
  
  print(paste0("Max num iterations: ", max(stats$num_iterations)))
  print(paste0("Max num iterations, occurence time: ", stats[stats$num_iterations == max(stats$num_iterations), ]$occurence_time))
  print(paste0("Max num iterations, num requests: ", stats[stats$num_iterations == max(stats$num_iterations), ]$num_requests))
  print(paste0("Max num requests: ", max(stats$num_requests)))
  
  means <- apply(stats,2,mean)
  means <- data.frame(lapply(means, function(x) format(x, nsmall=2)))
  print(means)

  
}

oldBatchDispatchPerfStats <- function(file_base) {
  
  stats <- read.csv(paste0(file_base, ".batchdispatchstats.csv"))
  
  stats <- stats %>% group_by(occurence_time) %>% summarise(
    num_iterations = max(iteration),
    num_requests = max(num_requests),
    find_assignments_running_time = sum(find_assignments_running_time, na.rm=TRUE),
    choose_accepted_running_time = sum(choose_accepted_running_time, na.rm=TRUE),
    update_system_state_running_time = sum(update_system_state_running_time, na.rm=TRUE),
    # num_elliptic_bucket_entry_deletions = sum(num_elliptic_bucket_entry_deletions, na.rm=TRUE)
  )
  
  print(paste0("Max num iterations: ", max(stats$num_iterations)))
  print(paste0("Max num iterations, occurence time: ", stats[stats$num_iterations == max(stats$num_iterations), ]$occurence_time))
  print(paste0("Max num iterations, num requests: ", stats[stats$num_iterations == max(stats$num_iterations), ]$num_requests))
  print(paste0("Max num requests: ", max(stats$num_requests)))
  
  means <- apply(stats,2,mean)
  means <- data.frame(lapply(means, function(x) format(x, nsmall=2)))
  print(means)
  
  
}


multiDispatchPerfStats <- function(file_base, num_runs=1) {
  
  stats <- read.csv(paste0(file_base, ".batchdispatchstats.csv"))
  
  print(paste0("Total time (batch dispatch): ", sum(stats$find_assignments_running_time + stats$choose_accepted_running_time + stats$update_system_state_running_time) / 1000000000, "s"))
  
  stats <- stats %>% group_by(occurence_time) %>% summarise(
    num_iterations = max(iteration),
    num_requests = max(num_requests),
    find_assignments_running_time = sum(find_assignments_running_time, na.rm=TRUE),
    choose_accepted_running_time = sum(choose_accepted_running_time, na.rm=TRUE),
    update_system_state_running_time = sum(update_system_state_running_time, na.rm=TRUE),
    num_elliptic_bucket_entry_deletions = sum(num_elliptic_bucket_entry_deletions, na.rm=TRUE)
  )
  
  print(paste0("Max num iterations: ", max(stats$num_iterations)))
  print(paste0("Max num iterations, occurence time: ", stats[stats$num_iterations == max(stats$num_iterations), ]$occurence_time))
  print(paste0("Max num iterations, num requests: ", stats[stats$num_iterations == max(stats$num_iterations), ]$num_requests))
  print(paste0("Max num requests: ", max(stats$num_requests)))
  
  means <- apply(stats,2,mean)
  means <- data.frame(lapply(means, function(x) format(x, nsmall=2)))
  print(means)
  
  
}


batchUpdatePerfStats <- function(file_base) {
  
  stats <- read.csv(paste0(file_base, ".batch_insert_stats.csv"))
  
  print(paste0("Total time (batch update): ", sum(stats$total_time) / 1000000000, "s"))
  
  stats <- stats %>% group_by(occurence_time) %>% summarise(
    num_iterations = max(iteration),
    num_requests = max(num_requests),
    update_route_state_time = sum(update_route_state_time, na.rm = TRUE),
    ell_entry_insertions_find_insertions_time = sum(ell_entry_insertions_find_insertions_time, na.rm = TRUE),
    ell_entry_insertions_num_insertions = sum(ell_entry_insertions_num_insertions, na.rm = TRUE),
    ell_entry_insertions_perform_insertions_time = sum(ell_entry_insertions_perform_insertions_time, na.rm = TRUE),
    last_stop_find_insertions_and_deletions_time = sum(last_stop_find_insertions_and_deletions_time, na.rm = TRUE),
    last_stop_num_insertions_and_deletions = sum(last_stop_num_insertions_and_deletions, na.rm = TRUE),
    last_stop_perform_insertions_and_deletions_time = sum(last_stop_perform_insertions_and_deletions_time, na.rm = TRUE),
    # last_stop_update_prefix_sum_time = sum(last_stop_update_prefix_sum_time, na.rm = TRUE),
    num_ell_source_entries_after_insertions = sum(num_ell_source_entries_after_insertions, na.rm = TRUE),
    num_ell_target_entries_after_insertions = sum(num_ell_target_entries_after_insertions, na.rm = TRUE),
    update_leeways_num_entries_scanned_source = sum(update_leeways_num_entries_scanned_source, na.rm = TRUE),
    update_leeways_num_entries_scanned_target = sum(update_leeways_num_entries_scanned_target, na.rm = TRUE),
    update_leeways_time = sum(update_leeways_time, na.rm = TRUE),
    total_time = sum(total_time, na.rm = TRUE)
  )
  
  means <- apply(stats,2,mean)
  means <- data.frame(lapply(means, function(x) format(x, nsmall=2)))
  print(means)
  
}

countBestAssignmentTypes <- function(file_base) {
  asgns <- read.csv(paste0(file_base, ".bestassignments.csv"))
  
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
  getType <- function(i, j, k) {
    i <- as.integer(i)
    j <- as.integer(j)
    k <- as.integer(k)
    if (0 < i && i <= j && i < k - 1)
      return("ord")
    if (0 == i && i <= j && j < k - 1)
      return("pbns")
    if (i == j && j == k - 1)
      return("pals")
    if (0 <= i && i < j && j == k - 1)
      return("dals")
    exit("wtf")
    return("error")
  }
  
  getTypeRow <- function(row) {
    return(getType(row[["pickup_insertion_point"]], row[["dropoff_insertion_point"]], row["num_stops"]))
  }
  
  types <- apply(asgns, 1, getTypeRow)
  
  num_asgns <- nrow(asgns)
  num_ord <- sum(types == "ord")
  pct_ord <- format(num_ord / num_asgns * 100, nsmall=2)
  num_pbns <- sum(types == "pbns")
  pct_pbns <- format(num_pbns / num_asgns * 100, nsmall=2)
  num_pals <- sum(types == "pals")
  pct_pals <- format(num_pals / num_asgns * 100, nsmall=2)
  num_dals <- sum(types == "dals")
  pct_dals <- format(num_dals / num_asgns * 100, nsmall=2)

  print(paste0("Total: ", num_asgns, ", ",
               "Ord: ", num_ord, " (", pct_ord, "%), ",
               "PBNS: ", num_pbns, " (", pct_pbns, "%), ",
               "PALS: ", num_pals, " (", pct_pals, "%), ",
               "DALS: ", num_dals, " (", pct_dals, "%)"))
  
}
