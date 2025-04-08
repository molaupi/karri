

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
quality <- function(fileBase, numVehicles=NULL, reqIdSubset=NULL) {
  
  asgnstats <- read.csv(paste0(fileBase, ".assignmentquality.csv"))
  asgnstats <- asgnstats[order(asgnstats$request_id),]
  legstats <- read.csv(paste0(fileBase, ".legstats.csv"))
  bestasgns <- read.csv(paste0(fileBase, ".bestassignments.csv"))
  bestasgns <- bestasgns[order(bestasgns$request_id),]
  
  if (is.null(numVehicles)) {
    eventsimstats <- read.csv(paste0(fileBase, ".eventsimulationstats.csv"))
    numVehicles <- sum(eventsimstats$type == "VehicleStartup")
  }
  
  if (!is.null(reqIdSubset)) {
    asgnstats <- asgnstats[asgnstats$request_id %in% reqIdSubset,]
    bestasgns <- bestasgns[bestasgns$request_id %in% reqIdSubset,]
  }
  
  df <- data.frame(
             wait_time_avg = c(mean(asgnstats$wait_time) / 10), # avg wait time for each request
             wait_time_q95 = c(quantile(asgnstats$wait_time, 0.95) / 10), # q95 wait time for each request
             ride_time_avg = c(mean(asgnstats$ride_time) / 10), # avg ride time for each request
             ride_time_q95 = c(quantile(asgnstats$ride_time, 0.95) / 10), # q95 ride time for each request
             trip_time_avg = c(mean(asgnstats$trip_time) / 10), # avg trip time for each request
             trip_time_q95 = c(quantile(asgnstats$trip_time, 0.95) / 10), # q95 trip time for each request
             direct_time_avg = c(mean(bestasgns$direct_od_dist) / 10), # avg direct drive time from origin to destination
             trip_delay_avg = c(mean(ifelse(bestasgns$direct_od_dist != 0, asgnstats$trip_time / bestasgns$direct_od_dist, 1))), # avg relative delay for whole trip
             ride_delay_avg = c(mean(ifelse(bestasgns$direct_od_dist != 0, asgnstats$ride_time / bestasgns$direct_od_dist, 1))), # avg relative delay for only ride
             walk_to_pickup_avg=c(mean(asgnstats$walk_to_pickup_time) / 10), # avg walking time to pickup
             walk_to_pickup_q95=c(quantile(asgnstats$walk_to_pickup_time, 0.95) / 10), # q95 walking time to pickup
             walk_to_dropoff_avg=c(mean(asgnstats$walk_to_dropoff_time) / 10), # avg walking time to dropoff
             walk_to_dropoff_q95=c(quantile(asgnstats$walk_to_dropoff_time, 0.95) / 10), # q95 walking time to dropoff
             stop_time_avg = c(sum(legstats$stop_time) / numVehicles / 10), # avg total stop time for each vehicle
             empty_time_avg = c(sum(legstats[legstats$occupancy == 0, "drive_time"]) / numVehicles / 10), # avg time spent driving empty for each vehicle
             occ_time_avg = c(sum(legstats[legstats$occupancy > 0, "drive_time"]) / numVehicles / 10) # avg time spent driving occupied for each vehicle
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
    vehicle_startup_event_time = c(mean(stats[stats$type=="VehicleStartupEvent", ]$running_time)),
    vehicle_arrival_event_time = c(mean(stats[stats$type=="VehicleArrivalEvent", ]$running_time)),
    vehicle_departure_event_time = c(mean(stats[stats$type=="VehicleDepartureEvent", ]$running_time)),
    vehicle_shutdown_event_time = c(mean(stats[stats$type=="VehicleShutdownEvent", ]$running_time)),
    request_receipt_event_time = c(mean(stats[stats$type=="RequestReceiptEvent",]$running_time)),
    request_pure_walking_arrival_event_time = c(mean(stats[stats$type=="PureWalkingArrivalEvent",]$running_time))
  )
  
  print(df)
}
