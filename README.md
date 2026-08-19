# dd_carla_platooning
Deliberately Decentralized implementation/modification of carla_platooning by Jacob Sanders. Converts centralized into decentralized protocol by having every vehicle be a leader (unless they're joining as a joiner) and only be concerned with its immediate front and back neighbors, if applicable. Join candidates are selected via a cost function defined by J. Heinovsk, et. al. in "Platoon Formation: Optimized Car to Platoon Assignment Strategies and Protocols"



# Modifications
## examples/joinAtBack
### logs
* extract_join_semantics:
  * Extra step to check if a platoon candidate was selected
  * Altered extract_leader_id() to extract the candidate instead of the assumed single leader

### CarlaPlatooningNetwork.ned
* CarlaPlatooningNetwork.ned
  * Added an AnnotationManager

### omnetpp.ini
* omnetpp.ini
  * Added DecentralizedJoinAtBack Configuration
    * 4 vehicle starting platoon, 1 late joiner
  * Added DecentralizedBrakeTest Configuration
    * 4 vehicle starting platoon, brake after 50 seconds
  * Added DecentralizedBaselineFourCar Configuration
    * 4 vehicle starting platoon (to test functionality)
  * Added DecentralizedMergeAtBack Configuration
    * 2 separate 4 vehicle starting platoons. Merge after 15 seconds

## src/carla/apps
* CarlaGeneralPlatooningApp.cc
  * Included heuristic and brake timer
  * Edited getTargetDistance() to use relative distance along the track
  * Added parameters to compute cost function as specified in the paper:
    * alpha
    * range
    * desired speed
  * Redefined initial formation to understand described initial formations
    * Init formation: {front neighbor, self, back neighbor}
  * Redefined roles to be exclusively about JOINERs and LEADERs
  * Lead cars to CC trailing cars to use CACC
  * startManeuverIfConfigured() to wait for heuristic timer to fire before committing to a maneuver
  * Added evaluatePlatoonCandidates()
    * Uses the cost function by iterating over all candidates and selecting the best one
  * Altered non-maneuver phase to use CACC (fix follower -> leader change)
* CarlaGeneralPlatooningApp.h
  * Added PlatoonCandidate struct
    * bool valid
    * int vehicleId
    * double cost
  * Added decentralized parameters
  * Included the evaluatePlatoonCandidates() helper function
* CarlaGeneralPlatooningApp.ned
  * Included the decentralized parameters to broadcast

## src/carla/platooning/maneuver
* CarlaJoinAtBack.cc
  * Deleted leader block for processJoinRequest()
  * Set role to LEADER in handleJoinFormation()
  * Set role to LEADER in handleJoinFormationAck()

## src/carla/platooning/state
* PlatoonNeighborState.h
  * Added desired speed

## src
* Makefile
  * Added INET compatibility

## carla_platooning
* .platooning
  * Added INET4.4

# Additional Files
## examples/joinAtBack/logs
* analyze_network_overhead
  * Plots and turns into csv's the network overhead of a single log
* compare_protocols.py
  * Takes two logs and plots:
    * Total Bandwidth over time
    * Maneuver Bandwidth over time 
    * Total Message Count over time
    * Maneuver Messages over time
    * Total Maneuver Message Counts and their types
    * Join Timeline
* plot_platoon_dynamics.py
  * plots and saves IEEE compliant graphs for bumper gap and speed of each vehicle as pdfs

# carla_platooning
* manager.controlln++
  * Don't actually know why this exists. inet4.4 thing?
