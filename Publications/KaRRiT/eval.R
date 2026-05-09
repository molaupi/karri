

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


# Given the path to the result files of a KaRRi run (e.g. "<output-dir>/karrit-exact_0_600_0"),
# this function returns an overview over the solution quality of the assignments.
quality <- function(file_base, notransfer = FALSE) {
  
  asgnstats <- read.csv(paste0(file_base, ".assignmentquality.csv"))
  legstats <- read.csv(paste0(file_base, ".legstats.csv"))
  bestasgn <- read.csv(paste0(file_base, if (notransfer) ".bestassignments.csv" else ".bestassignmentsoverall.csv"))
  numAccepted <- sum(bestasgn["accepted"] == 1)
  numTotal <- nrow(bestasgn)
  bestasgn <- bestasgn[bestasgn["accepted"] == 1,]
  
  eventsimstats <- read.csv(paste0(file_base, ".eventsimulationstats.csv"))
  num.Vehicles <- sum(eventsimstats$type == "VehicleStartup")
  
  asgnstats <- asgnstats[order(asgnstats$request_id),]
  bestasgn <- head(bestasgn, nrow(asgnstats))
  relride <- asgnstats$ride_time / bestasgn$direct_od_dist
  reltrip <- asgnstats$trip_time / bestasgn$direct_od_dist
  
  df <- data.frame(
             wait_time_avg = c(mean(asgnstats$wait_time) / 10), # avg wait time for each request
             wait_time_q95 = c(quantile(asgnstats$wait_time, 0.95) / 10), # q95 wait time for each request
             ride_time_avg = c(mean(asgnstats$ride_time) / 10), # avg ride time for each request
             ride_time_q95 = c(quantile(asgnstats$ride_time, 0.95) / 10), # q95 ride time for each request
             rel_ride_time_avg = c(mean(relride)), # avg ride time relative to direct od dist
             trip_time_avg = c(mean(asgnstats$trip_time) / 10), # avg trip time for each request
             trip_time_q95 = c(quantile(asgnstats$trip_time, 0.95) / 10), # q95 trip time for each request
             rel_trip_time_avg = c(mean(reltrip)), # avg trip time relative to direct od dist
             walk_to_pickup_avg=c(mean(asgnstats$walk_to_pickup_time) / 10), # avg walking time to pickup
             walk_to_pickup_q95=c(quantile(asgnstats$walk_to_pickup_time, 0.95) / 10), # q95 walking time to pickup
             walk_to_dropoff_avg=c(mean(asgnstats$walk_to_dropoff_time) / 10), # avg walking time to dropoff
             walk_to_dropoff_q95=c(quantile(asgnstats$walk_to_dropoff_time, 0.95) / 10), # q95 walking time to dropoff
             stop_time_avg = c(sum(legstats$stop_time) / num.Vehicles / 10), # avg total stop time for each vehicle
             empty_time_avg = c(sum(legstats[legstats$occupancy == 0, "drive_time"]) / num.Vehicles / 10), # avg time spent driving empty for each vehicle
             occ_time_avg = c(sum(legstats[legstats$occupancy > 0, "drive_time"]) / num.Vehicles / 10), # avg time spent driving occupied for each vehicle
             drive_time_avg =c(sum(legstats$drive_time) / num.Vehicles / 10),
             op_time_avg = c(sum(legstats$drive_time + legstats$stop_time) / num.Vehicles / 10)
             )
  
  # df$op_time_avg <- df$stop_time_avg + df$empty_time_avg + df$occ_time_avg # avg total operation time for each vehicle
  df$occ_rate_avg <- sum(legstats$drive_time * legstats$occupancy) / sum(legstats$drive_time) # avg occupation rate while driving for each vehicle
  df$cost <- mean(asgnstats$cost) # avg cost (according to cost function used in KaRRi) for each vehicle
  
  # Reformat passenger times to MM:SS
  psg_time_cols <- c("wait_time_avg", "wait_time_q95", 
                     "ride_time_avg", "ride_time_q95", 
                     "trip_time_avg", "trip_time_q95",
                     "walk_to_pickup_avg", "walk_to_pickup_q95",
                     "walk_to_dropoff_avg", "walk_to_dropoff_q95"
                     )
  df[, colnames(df) %in% psg_time_cols] <- convertToMMSS(df[, colnames(df) %in% psg_time_cols])
  
  # Reformat vehicle times to HH:MM
  veh_time_cols <- c("stop_time_avg", "empty_time_avg", "occ_time_avg", "drive_time_avg", "op_time_avg")
  df[, colnames(df) %in% veh_time_cols] <- convertToHHMM(df[, colnames(df) %in% veh_time_cols])
  df["service_rate"] <- c(numAccepted / numTotal)

  print(df)
}

# Prints statistics for type of assignments for run without transfers.
asgnTypeStatsNoTransfer <- function(file) {
  df <- read.csv(paste0(file, ".bestassignments.csv"))
  
  numnoveh <- sum(df$not_using_vehicle == " true")
  numord <- sum((df$pickup_insertion_point > 0) & (df$pickup_insertion_point < df$num_stops - 1) & (df$dropoff_insertion_point < df$num_stops - 1))
  numpbns <- sum((df$pickup_insertion_point == 0) & (df$pickup_insertion_point < df$num_stops - 1) & (df$dropoff_insertion_point < df$num_stops - 1))
  numpals <- sum(df$pickup_insertion_point == df$num_stops - 1)
  numdals <- sum((df$pickup_insertion_point < df$num_stops - 1) & (df$dropoff_insertion_point == df$num_stops - 1))
  
  print(paste0("NoVeh: ", numnoveh))
  print(paste0("ORD: ", numord))
  print(paste0("PBNS: ", numpbns))
  print(paste0("PALS: ", numpals))
  print(paste0("DALS: ", numdals))
  print(paste0("Sum: ", numnoveh + numord + numpbns + numpals + numdals))
}

# Prints statistics for number of legs and type of no-transfer (!) assignments for run with transfers.
asgnTypeStats <- function(file) {
  dfoverall <- read.csv(paste0(file, ".bestassignmentsoverall.csv"))
  numnoveh <- sum(dfoverall$number_of_legs == 0)
  numoneleg <- sum(dfoverall$number_of_legs == 1)
  numtwolegs <- sum(dfoverall$number_of_legs == 2)
  
  df <- read.csv(paste0(file, ".bestassignmentswithouttransfer.csv"))
  
  numord <- sum((df$pickup_insertion_point > 0) & (df$pickup_insertion_point < df$num_stops - 1) & (df$dropoff_insertion_point < df$num_stops - 1))
  numpbns <- sum((df$pickup_insertion_point == 0) & (df$pickup_insertion_point < df$num_stops - 1) & (df$dropoff_insertion_point < df$num_stops - 1))
  numpals <- sum(df$pickup_insertion_point == df$num_stops - 1)
  numdals <- sum((df$pickup_insertion_point < df$num_stops - 1) & (df$dropoff_insertion_point == df$num_stops - 1))
  
  print(paste0("NoVeh: ", numnoveh))
  print(paste0("One leg: ", numoneleg))
  print(paste0("Two legs:", numtwolegs))
  print(paste0("ORD: ", numord))
  print(paste0("PBNS: ", numpbns))
  print(paste0("PALS: ", numpals))
  print(paste0("DALS: ", numdals))
  print(paste0("Sum: ", numnoveh + numord + numpbns + numpals + numdals))
}

# Given the paths to the result files of two KaRRi runs, this functions checks
# whether all assignments are the same in both runs.
compareBestAssignments <- function(file1, file2) {
  overall1 <- read.csv(paste0(file1, ".bestassignmentsoverall.csv"))
  overall2 <- read.csv(paste0(file2, ".bestassignmentsoverall.csv"))
  
  zerolegs1 <- read.csv(paste0(file1, ".bestassignmentswithoutvehicle.csv"))
  zerolegs2 <- read.csv(paste0(file2, ".bestassignmentswithoutvehicle.csv"))
  
  oneleg1 <- read.csv(paste0(file1, ".bestassignmentswithouttransfer.csv"))
  oneleg2 <- read.csv(paste0(file2, ".bestassignmentswithouttransfer.csv"))
  
  twolegs1 <- read.csv(paste0(file1, ".bestassignmentswithtransfer.csv"))
  twolegs2 <- read.csv(paste0(file2, ".bestassignmentswithtransfer.csv"))
  
  for (i in 1:(nrow(overall1))) {
    numlegs1 <- overall1$number_of_legs[i]
    numlegs2 <- overall2$number_of_legs[i]
    
    if (numlegs1 != numlegs2) {
      print(paste0(i, ": Number of legs 1: ", numlegs1, ", Number of legs 2: ", numlegs2))
      if (numlegs1 == 0)
        View(zerolegs1[i, ])
      else if (numlegs1 == 1)
        View(oneleg1[i,  ])
      else
        View(twolegs1[i, ])
      if (numlegs2 == 0)
        View(zerolegs2[i, ])
      else if (numlegs2 == 1)
        View(oneleg2[i, ])
      else
        View(twolegs2[i, ])
      return()
    }
    
    if (numlegs1 == 0) {
      row1 <- zerolegs1[i,]
      row2 <- zerolegs2[i,]
      if (rowSums(row1 != row2) > 0) {
        print(zerolegs1[i, "request_id"])
        View(rbind(row1, row2))
        return()
      }
    }
    
    if (numlegs1 == 1) {
      row1 <- oneleg1[i,]
      row2 <- oneleg2[i,]
      if (rowSums(row1 != row2) > 0) {
        print(oneleg1[i, "request_id"])
        View(rbind(row1, row2))
        return()
      }
    }
    
    if (numlegs1 == 2) {
      row1 <- twolegs1[i,]
      row2 <- twolegs2[i,]
      if (rowSums(row1 != row2) > 0) {
        print(twolegs1[i, "request_id"])
        View(rbind(row1, row2))
        return()
      }
    }
  }
  print("All best insertions are equal.")
}

perfStats <- function(file_base, type_name, remove_last_row = FALSE) {
  stats <- read.csv(paste0(file_base, ".perf_", type_name, ".csv"))
  if (remove_last_row) {
    stats <- head(stats, -1)
  }
  stats <- stats[, ! colnames(stats) %in% c("request_id")]
  stats <- apply(stats, 2, mean)
  stats <- round(stats, 2)
  return(stats)
}

# Given the path to the result files of a KaRRiT run, this function returns an
# overview over the average runtime per request (in microseconds).
overallPerfStats <- function(file_base) {
  perfStats(file_base, "overall")
}

# Given the path to the result files of a KaRRiT run, this function returns
# performance statistics for the initialization phase.
initreqPerfStats <- function(file_base) {
  perfStats(file_base, "initreq")
}

# Given the path to the result files of a KaRRiT run, this function returns
# performance statistics for the elliptic BCH searches in the no-transfer case.
ellipticBchPerfStats <- function(file_base) {
  perfStats(file_base, "ellipticbch")
}

# Given the path to the result files of a KaRRiT run, this function returns
# performance statistics for the computation of PD-distances in the no-transfer case.
pdDistancesPerfStats <- function(file_base) {
  perfStats(file_base, "pddistances")
}

# Given the path to the result files of a KaRRiT run, this function returns
# performance statistics for the no-transfer ordinary insertions phase.
ordPerfStats <- function(file_base) {
  perfStats(file_base, "ord")
}

# Given the path to the result files of a KaRRiT run, this function returns
# performance statistics for the no-transfer PBNS phase.
pbnsPerfStats <- function(file_base) {
  perfStats(file_base, "pbns")
}

# Given the path to the result files of a KaRRiT run, this function returns
# performance statistics for the no-transfer PALS phase.
palsPerfStats <- function(file_base) {
  perfStats(file_base, "pals")
}

# Given the path to the result files of a KaRRiT run, this function returns
# performance statistics for the no-transfer DALS phase.
dalsPerfStats <- function(file_base) {
  perfStats(file_base, "dals")
}

# Given the path to the result files of a KaRRiT run, this function returns
# performance statistics for updating the route state for each assignment.
updatePerfStats <- function(file_base) {
  perfStats(file_base, "update")
}

# Given the path to the result files of a KaRRiT run, this function returns
# performance statistics for reconstructing detour ellipses for transfer assignments.
ellipseReconstructionPerfStats <- function(file_base) {
  perfStats(file_base, "ellipse_reconstruction")
}

# Given the path to the result files of a KaRRiT run, this function returns
# performance statistics for finding ordinary transfer assignments.
ordinaryTransferPerfStats <- function(file_base) {
  perfStats(file_base, "transf_ord")
}

# Given the path to the result files of a KaRRiT run, this function returns
# performance statistics for finding after-last-stop transfer assignments for
# the pickup vehicle.
afterLastStopTransferPVehPerfStats <- function(file_base) {
  perfStats(file_base, "transf_als_pveh")
}

# Given the path to the result files of a KaRRiT run, this function returns
# performance statistics for finding after-last-stop transfer assignments for
# the dropoff vehicle.
afterLastStopTransferDVehPerfStats <- function(file_base) {
  perfStats(file_base, "transf_als_dveh")
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
