# Lane Change design

The Lane Change module is activated when lane change is needed and can be safely executed.

## Lane Change Requirement

- As the prerequisite, the type of lane boundary in the HD map has to be one of the following:
  - Dashed lane marking: Lane changes are permitted in both directions.
  - Dashed marking on the left and solid on the right: Lane changes are allowed from left to right.
  - Dashed marking on the right and solid on the left: Lane changes are allowed from right to left.
  - `allow_lane_change` tags is set as `true`
- During lane change request condition
  - The ego-vehicle isn’t on a `preferred_lane`.
  - The ego-vehicle isn't approaching a traffic light. (condition parameterized)
  - The ego-vehicle isn't approaching a crosswalk. (condition parameterized)
  - The ego-vehicle isn't approaching an intersection. (condition parameterized)
- lane change ready condition
  - Path of the lane change does not collide with other dynamic objects (see the figure below)
  - Lane change candidate path is approved by an operator.

## Generating Lane Change Candidate Path

The lane change candidate path is divided into two phases: preparation and lane-changing. The following figure illustrates each phase of the lane change candidate path.

![lane-change-phases](./images/lane_change-lane_change_phases.png)

The following chart illustrates the process of sampling candidate paths for lane change.

```plantuml
@startuml
skinparam defaultTextAlignment center
skinparam backgroundColor #WHITE

start

if (Is current lanes, target lanes OR target neighbor lanes polygon empty?) then (yes)
  #LightPink:Return prev approved path;
  stop
else (no)
endif

:Get target objects;

:Calculate prepare phase metrics;

:Start loop through prepare phase metrics;

repeat
  :Get prepare segment;

  if (Is LC start point outside target lanes range) then (yes)
    if (Is found a valid path) then (yes)
      #Orange:Return first valid path;
      stop
    else (no)
      #LightPink:Return prev approved path;
      stop
    endif
  else (no)
  endif

  :Calculate shift phase metrics;

  :Start loop through shift phase metrics;
  repeat
    :Get candidate path;
    note left: Details shown in the next chart
    if (Is valid candidate path?) then (yes)
      :Store candidate path;
      if (Is candidate path safe?) then (yes)
        #LightGreen:Return candidate path;
        stop
      else (no)
      endif
    else (no)
    endif
    repeat while (Finished looping shift phase metrics) is (FALSE)
  repeat while (Is finished looping prepare phase metrics) is (FALSE)

if (Is found a valid path) then (yes)
  #Orange:Return first valid path;
  stop
else (no)
endif

#LightPink:Return prev approved path;
stop

@enduml
```

While the following chart demonstrates the process of generating a valid candidate path.

```plantuml
@startuml
skinparam defaultTextAlignment center
skinparam backgroundColor #WHITE

start

:Calculate resample interval;

:Get reference path from target lanes;

if (Is reference path empty?) then (yes)
  #LightPink:Return;
  stop
else (no)
endif

:Get LC shift line;

:Set lane change Info;
note left: set information from sampled prepare phase and shift phase metrics\n<color:red>(duration, length, velocity, and acceleration)

:Construct candidate path;

if (Candidate path has enough length?) then (yes)
  #LightGreen:Return valid candidate path;
  stop
else (no)
  #LightPink:Return;
  stop
endif

@enduml
```

### Preparation phase

The preparation trajectory is the candidate path's first and the straight portion generated along the ego vehicle's current lane. The length of the preparation trajectory is computed as follows.

```C++
lane_change_prepare_distance = current_speed * lane_change_prepare_duration + 0.5 * deceleration * lane_change_prepare_duration^2
```

During the preparation phase, the turn signal will be activated when the remaining distance is equal to or less than `lane_change_search_distance`.

### Lane-changing phase

The lane-changing phase consist of the shifted path that moves ego from current lane to the target lane. Total distance of lane-changing phase is as follows. Note that during the lane changing phase, the ego vehicle travels at a constant speed.

```C++
lane_change_prepare_velocity = std::max(current_speed + deceleration * lane_change_prepare_duration, minimum_lane_changing_velocity)
lane_changing_distance = lane_change_prepare_velocity * lane_changing_duration
```

The `backward_length_buffer_for_end_of_lane` is added to allow some window for any possible delay, such as control or mechanical delay during brake lag.

#### Multiple candidate path samples (longitudinal acceleration)

Lane change velocity is affected by the ego vehicle's current velocity. High velocity requires longer preparation and lane changing distance. However we also need to plan lane changing trajectories in case ego vehicle slows down.
Computing candidate paths that assumes ego vehicle's slows down is performed by substituting predetermined deceleration value into `prepare_length`, `prepare_velocity` and `lane_changing_length` equation.

The predetermined longitudinal acceleration values are a set of value that starts from `longitudinal_acceleration = maximum_longitudinal_acceleration`, and decrease by `longitudinal_acceleration_resolution` until it reaches `longitudinal_acceleration = -maximum_longitudinal_deceleration`. Both `maximum_longitudinal_acceleration` and `maximum_longitudinal_deceleration` are calculated as: defined in the `common.param` file as `normal.min_acc`.

```C++
maximum_longitudinal_acceleration = min(common_param.max_acc, lane_change_param.max_acc)
maximum_longitudinal_deceleration = max(common_param.min_acc, lane_change_param.min_acc)
```

where `common_param` is vehicle common parameter, which defines vehicle common maximum longitudinal acceleration and deceleration. Whereas, `lane_change_param` has maximum longitudinal acceleration and deceleration for the lane change module. For example, if a user set and `common_param.max_acc=1.0` and `lane_change_param.max_acc=0.0`, `maximum_longitudinal_acceleration` becomes `0.0`, and the lane change does not accelerate in the lane change phase.

The `longitudinal_acceleration_resolution` is determine by the following

```C++
longitudinal_acceleration_resolution = (maximum_longitudinal_acceleration - minimum_longitudinal_acceleration) / longitudinal_acceleration_sampling_num
```

Note that when the `current_velocity` is lower than `minimum_lane_changing_velocity`, the vehicle needs to accelerate its velocity to `minimum_lane_changing_velocity`. Therefore, longitudinal acceleration becomes positive value (not decelerate).

The chart illustrates the conditions under which longitudinal acceleration values are sampled.

```plantuml
@startuml
skinparam defaultTextAlignment center
skinparam backgroundColor #WHITE

start

if (prev_module_path is empty?) then (yes)
  :Return empty list;
  stop
else (no)
endif

if (max_acc <= 0.0) then (yes)
  :Return **sampled acceleration values**;
  note left: Calculated sampled acceleration using\n<color:red>getAccelerationValues(min_acc, max_acc, longitudinal_acc_sampling_num)</color>
  stop
endif

if (max_lane_change_length >  ego's distance to the end of the current lanes.) then (yes)
  :Return **sampled acceleration values**;
  stop
endif

if (ego is stuck in the current lanes) then (yes)
  :Return **sampled acceleration values**;
  stop
else (no)
endif

if (is goal is in target lanes) then (yes)
  if (max_lane_change_length < ego's distance to the goal along the target lanes) then (yes)
    :Return {max_acc};
    stop
  else (no)
  endif
else (no)
  if (max_lane_change_length < ego's distance to the end of the target lanes.) then (yes)
    :Return {max_acc};
    stop
  else (no)
  endif
endif

:Return **sampled acceleration values**;
stop

@enduml

```

while the following describes the process by which longitudinal accelerations are sampled.

```plantuml
@startuml
start
:Initialize sampled_values with min_acc;

if (min_acc > max_acc) then (yes)
  :Return empty list;
  stop
elseif (max_acc - min_acc < epsilon) then (yes)
  :Return {0.0};
  stop
else (no)
  :Calculate resolution;
endif

:Start loop from min_acc to max_acc with resolution step;
repeat
  if (sampled_values.back() < -epsilon AND next_value > epsilon) then (yes)
    :Insert 0.0 into sampled_values;
  endif
  :Add sampled_acc to sampled_values;
  repeat while (sampled_acc < max_acc + epsilon) is (TRUE)

:Return sampled_values;
stop
@enduml
```

The following figure illustrates when `longitudinal_acceleration_sampling_num = 4`. Assuming that `maximum_deceleration = 1.0` then `a0 == 0.0 == no deceleration`, `a1 == 0.25`, `a2 == 0.5`, `a3 == 0.75` and `a4 == 1.0 == maximum_deceleration`. `a0` is the expected lane change trajectories should ego vehicle do not decelerate, and `a1`'s path is the expected lane change trajectories should ego vehicle decelerate at `0.25 m/s^2`.

![path_samples](./images/lane_change-candidate_path_samples.png)

Which path will be chosen will depend on validity and collision check.

#### Multiple candidate path samples (lateral acceleration)

In addition to sampling longitudinal acceleration, we also sample lane change paths by adjusting the value of lateral acceleration. Since lateral acceleration influences the duration of a lane change, a lower lateral acceleration value results in a longer lane change path, while a higher lateral acceleration value leads to a shorter lane change path. This allows the lane change module to generate a shorter lane change path by increasing the lateral acceleration when there is limited space for the lane change.

The maximum and minimum lateral accelerations are defined in the lane change parameter file as a map. The range of lateral acceleration is determined for each velocity by linearly interpolating the values in the map. Let's assume we have the following map

| Ego Velocity | Minimum lateral acceleration | Maximum lateral acceleration |
| :----------- | ---------------------------- | ---------------------------- |
| 0.0          | 0.2                          | 0.3                          |
| 2.0          | 0.2                          | 0.4                          |
| 4.0          | 0.3                          | 0.4                          |
| 6.0          | 0.3                          | 0.5                          |

In this case, when the current velocity of the ego vehicle is 3.0, the minimum and maximum lateral accelerations are 0.25 and 0.4 respectively. These values are obtained by linearly interpolating the second and third rows of the map, which provide the minimum and maximum lateral acceleration values.

Within this range, we sample the lateral acceleration for the ego vehicle. Similar to the method used for sampling longitudinal acceleration, the resolution of lateral acceleration (lateral_acceleration_resolution) is determined by the following:

```C++
lateral_acceleration_resolution = (maximum_lateral_acceleration - minimum_lateral_acceleration) / lateral_acceleration_sampling_num
```

#### Candidate Path's validity check

A candidate path is considered valid if it meets the following criteria:

1. The distance from the ego vehicle's current position to the end of the current lanes is sufficient to perform a single lane change.
2. The distance from the ego vehicle's current position to the goal along the current lanes is adequate to complete multiple lane changes.
3. The distance from the ego vehicle's current position to the end of the target lanes is adequate for completing multiple lane changes.
4. The distance from the ego vehicle's current position to the next regulatory element is adequate to perform a single lane change.
5. The lane change can be completed after passing a parked vehicle.
6. The lane change is deemed safe to execute.

The following flow chart illustrates the validity check.

```plantuml
@startuml
skinparam defaultTextAlignment center
skinparam backgroundColor #White

start
if (Check if start point is valid by check if it is covered by neighbour lanes polygon) then (not covered)
  #LightPink:Reject path;
  stop
else (covered)
endif

group check for distance #LightYellow
    :Calculate total length and goal related distances;
    if (total lane change length considering single lane change > distance from current pose to end of current lanes) then (yes)
      #LightPink:Reject path;
      stop
    else (no)
    endif

    if (goal is in current lanes) then (yes)
      if (total lane change length considering multiple lane changes > distance from ego to goal along current lanes) then (yes)
         #LightPink:Reject path;
        stop
      else (no)
      endif
    else (no)
    endif

    if (target lanes is empty) then (yes)
      #LightPink:Reject path;
      stop
    else (no)
    endif
    if (total lane change length considering multiple lane changes  > distance from ego to the end of target lanes) then (yes)
      #LightPink:Reject path;
      stop
    else (no)
    endif
end group

if (ego is not stuck but parked vehicle exists in target lane) then (yes)
  #LightPink:Reject path;
  stop
else (no)
endif

if (is safe to perform lane change) then (yes)
  #Cyan:Return candidate path list;
  stop
else (no)
  #LightPink:Reject path;
endif

stop

@enduml
```

#### Delay Lane Change Check

In certain situations, when there are stopped vehicles along the target lane ahead of Ego vehicle, to avoid getting stuck, it is desired to perform the lane change maneuver after the stopped vehicle.
To do so, all static objects ahead of ego along the target lane are checked in order from closest to furthest, if any object satisfies the following conditions, lane change will be delayed and candidate path will be rejected.

1. The distance from object to terminal end is sufficient to perform lane change
2. The distance to object is less than the lane changing length
3. The distance from object to next object is sufficient to perform lane change

If the parameter `check_only_parked_vehicle` is set to `true`, the check will only consider objects which are determined as parked.

The following flow chart illustrates the delay lane change check.

```plantuml
@startuml
skinparam defaultTextAlignment center
skinparam backgroundColor #White

start
if (Is target objects, candidate path, OR current lane path empty?) then (yes)
  #LightPink:Return false;
  stop
else (no)
endif

:Start checking objects from closest to furthest;
repeat
  if (Is distance from object to terminal sufficient) then (yes)
  else (no)
    #LightPink:Return false;
    stop
  endif

  if (Is distance to object less than lane changing length) then (yes)
  else (no)
    if (Is only check parked vehicles and vehicle is not parked) then (yes)
    else (no)
      if(Is last object OR distance to next object is sufficient) then (yes)
        #LightGreen: Return true;
        stop
      else (no)
      endif
    endif
  endif
  repeat while (Is finished checking all objects) is (FALSE)

#LightPink: Return false;
stop

@enduml
```

The following figures demonstrate different situations under which will or will not be triggered:

1. Delay lane change will be triggered as there is sufficient distance ahead.
   ![delay lane change 1](./images/delay_lane_change_1.drawio.svg)
2. Delay lane change will NOT be triggered as there is no sufficient distance ahead
   ![delay lane change 2](./images/delay_lane_change_2.drawio.svg)
3. Delay lane change will be triggered by fist NPC as there is sufficient distance ahead.
   ![delay lane change 3](./images/delay_lane_change_3.drawio.svg)
4. Delay lane change will be triggered by second NPC as there is sufficient distance ahead
   ![delay lane change 4](./images/delay_lane_change_4.drawio.svg)
5. Delay lane change will NOT be triggered as there is no sufficient distance ahead.
   ![delay lane change 5](./images/delay_lane_change_5.drawio.svg)

#### Candidate Path's Safety check

See [safety check utils explanation](../autoware_behavior_path_planner_common/docs/behavior_path_planner_safety_check.md)

#### Objects selection and classification

First, we divide the target objects into obstacles in the target lane, obstacles in the current lane, and obstacles in other lanes. Target lane indicates the lane that the ego vehicle is going to reach after the lane change and current lane mean the current lane where the ego vehicle is following before the lane change. Other lanes are lanes that do not belong to the target and current lanes. The following picture describes objects on each lane. Note that users can remove objects either on current and other lanes from safety check by changing the flag, which are `check_objects_on_current_lanes` and `check_objects_on_other_lanes`.

![object lanes](./images/lane_objects.drawio.svg)

Furthermore, to change lanes behind a vehicle waiting at a traffic light, we skip the safety check for the stopping vehicles near the traffic light. The explanation for parked car detection is written in [documentation for avoidance module](../autoware_behavior_path_static_obstacle_avoidance_module/README.md).

The detection area for the target lane can be expanded beyond its original boundaries to enable detection of objects that are outside the target lane's limits.

<div align="center">
  <table>
    <tr>
      <td>
        <div style="text-align: center;">
          <div style="color: black; font-size: 20px; margin-bottom: 10px;">Without Lane Expansion</div>
          <img src="./images/lane_change-lane_expansion-without.png" alt="Without lane expansion">
        </div>
      </td>
      <td>
        <div style="text-align: center;">
          <div style="color: black; font-size: 20px; margin-bottom: 10px;">With Lane Expansion</div>
          <img src="./images/lane_change-lane_expansion-with.png" alt="With lane expansion">
        </div>
      </td>
    </tr>
  </table>
</div>

##### Object filtering

```plantuml
@startuml
skinparam defaultTextAlignment center
skinparam backgroundColor #WHITE

title NormalLaneChange::filterObjects Method Execution Flow

start

group "Filter Objects by Class" {
while (has not finished iterating through predicted object list) is (TRUE)
  if (current object type != param.object_types_to_check?) then (TRUE)
  #LightPink:Remove current object;
else (FALSE)
  :Keep current object;
endif
end while
end group

if (predicted object list is empty?) then (TRUE)
  :Return empty result;
  stop
else (FALSE)
endif

group "Filter Oncoming Objects" #PowderBlue {
while (has not finished iterating through predicted object list?) is (TRUE)
if (object's yaw with reference to ego's yaw difference < 90 degree?) then (TRUE)
  :Keep current object;
else (FALSE)
if (object is stopping?) then (TRUE)
  :Keep current object;
else (FALSE)
  #LightPink:Remove current object;
endif
endif
endwhile
end group

if (predicted object list is empty?) then (TRUE)
  :Return empty result;
  stop
else (FALSE)
endif

group "Filter Objects By Lanelets" #LightGreen {

while (has not finished iterating through predicted object list) is (TRUE)
  :Calculate lateral distance diff;
  if (Object in target lane polygon?) then (TRUE)
    if (lateral distance diff > half of ego's width?) then (TRUE)
      if (Object's physical position is before terminal point?) then (TRUE)
        :Add to target_lane_objects;
      else (FALSE)
      endif
    else (FALSE)
    endif
  else (FALSE)
  endif

  if (Object overlaps with backward target lanes?) then (TRUE)
    :Add to target_lane_objects;
  else (FALSE)
    if (Object in current lane polygon?) then (TRUE)
      :Add to current_lane_objects;
    else (FALSE)
      :Add to other_lane_objects;
    endif
  endif
end while

:Return target lanes object,  current lanes object and other lanes object;
end group

:Generate path from current lanes;

if (path empty?) then (TRUE)
  :Return empty result;
  stop
else (FALSE)
endif

group "Filter Target Lanes' objects" #LightCyan {

while (has not finished iterating through target lanes' object list) is (TRUE)
  if(velocity is within threshold?) then (TRUE)
  :Keep current object;
  else (FALSE)
    if(object is ahead of ego?) then (TRUE)
      :keep current object;
    else (FALSE)
      #LightPink:remove current object;
    endif
   endif
endwhile
end group

group "Filter Current Lanes' objects"  #LightYellow {

while (has not finished iterating through current lanes' object list) is (TRUE)
  if(velocity is within threshold?) then (TRUE)
    if(object is ahead of ego?) then (TRUE)
      :keep current object;
    else (FALSE)
      #LightPink:remove current object;
    endif
    else (FALSE)
      #LightPink:remove current object;
   endif
endwhile
end group

group "Filter Other Lanes' objects"  #Lavender {

while (has not finished iterating through other lanes' object list) is (TRUE)
  if(velocity is within threshold?) then (TRUE)
    if(object is ahead of ego?) then (TRUE)
      :keep current object;
    else (FALSE)
      #LightPink:remove current object;
    endif
    else (FALSE)
      #LightPink:remove current object;
   endif
endwhile
end group

:Transform the objects into extended predicted object and return them as lane_change_target_objects;
stop

@enduml
```

##### Collision check in prepare phase

The ego vehicle may need to secure ample inter-vehicle distance ahead of the target vehicle before attempting a lane change. The flag `enable_collision_check_at_prepare_phase` can be enabled to gain this behavior. The following image illustrates the differences between the `false` and `true` cases.

![enable collision check at prepare phase](./images/lane_change-enable_collision_check_at_prepare_phase.png)

#### If the lane is blocked and multiple lane changes

When driving on the public road with other vehicles, there exist scenarios where lane changes cannot be executed. Suppose the candidate path is evaluated as unsafe, for example, due to incoming vehicles in the adjacent lane. In that case, the ego vehicle can't change lanes, and it is impossible to reach the goal. Therefore, the ego vehicle must stop earlier at a certain distance and wait for the adjacent lane to be evaluated as safe. The minimum stopping distance can be computed from shift length and minimum lane changing velocity.

```C++
lane_changing_time = f(shift_length, lat_acceleration, lat_jerk)
minimum_lane_change_distance = minimum_prepare_length + minimum_lane_changing_velocity * lane_changing_time + lane_change_finish_judge_buffer
```

The following figure illustrates when the lane is blocked in multiple lane changes cases.

![multiple-lane-changes](./images/lane_change-when_cannot_change_lanes.png)

### Stopping behavior

The stopping behavior of the ego vehicle is determined based on various factors, such as the number of lane changes required, the presence of obstacles, and the position of blocking objects in relation to the lane change plan. The objective is to choose a suitable stopping point that allows for a safe and effective lane change while adapting to different traffic scenarios.

The following flowchart and subsections explain the conditions for deciding where to insert a stop point when an obstacle is ahead.

```plantuml
@startuml
skinparam defaultTextAlignment center
skinparam backgroundColor #WHITE

title Inserting Stop Point

start
if (number of lane changes is zero?) then (<color:green><b>YES</b></color>)
stop
else (<color:red><b>NO</b></color>)
endif

if (do we want to insert stop point in current lanes?) then (<color:red><b>NO</b></color>)
#LightPink:Insert stop point at next lane change terminal start.;
stop
else (<color:green><b>YES</b></color>)
endif

if (Is there leading object in the current lanes that blocks ego's path?) then (<color:red><b>NO</b></color>)
#LightPink:Insert stop point at terminal stop.;
stop
else (<color:green><b>YES</b></color>)
endif

if (Blocking object's position is after target lane's start position?) then (<color:red><b>NO</b></color>)
#LightPink:Insert stop at the target lane's start position.;
stop
else (<color:green><b>YES</b></color>)
endif

if (Blocking object's position is before terminal stop position?) then (<color:red><b>NO</b></color>)
#LightPink:Insert stop at the terminal stop position;
stop
else (<color:green><b>YES</b></color>)
endif

if (Are there target lane objects between the ego and the blocking object?) then (<color:red><b>NO</b></color>)
#LightGreen:Insert stop behind the blocking object;
stop
else (<color:green><b>YES</b></color>)
#LightPink:Insert stop at terminal stop;
stop
@enduml
```

#### Ego vehicle's stopping position when an object exists ahead

When the ego vehicle encounters an obstacle ahead, it stops while maintaining a safe distance to prepare for a possible lane change. The exact stopping position depends on factors like whether the target lane is clear or if the lane change needs to be delayed. The following explains how different stopping scenarios are handled:

##### When the near the end of the lane change

Whether the target lane has obstacles or is clear, the ego vehicle stops while keeping a safe distance from the obstacle ahead, ensuring there is enough room for the lane change.

![stop_at_terminal_no_block](./images/lane_change-stop_at_terminal_no_block.drawio.svg)

![stop_at_terminal](./images/lane_change-stop_at_terminal.drawio.svg)

##### When the ego vehicle is not near the end of the lane change

The ego vehicle stops while maintaining a safe distance from the obstacle ahead, ensuring there is enough space for a lane change.

![stop_not_at_terminal_no_blocking_object](./images/lane_change-stop_not_at_terminal_no_blocking_object.drawio.svg)

#### Ego vehicle's stopping position when an object exists in the lane changing section

If there are objects within the lane change section of the target lane, the ego vehicle stops closer to the obstacle ahead, without maintaining the usual distance for a lane change.

##### When near the end of the lane change

Regardless of whether there are obstacles in the target lane, the ego vehicle stops while keeping a safe distance from the obstacle ahead, allowing for the lane change.

![stop_at_terminal_no_block](./images/lane_change-stop_at_terminal_no_block.drawio.svg)

![stop_at_terminal](./images/lane_change-stop_at_terminal.drawio.svg)

##### When not near the end of the lane change

If there are no obstacles in the lane change section of the target lane, the ego vehicle stops while keeping a safe distance from the obstacle ahead to accommodate the lane change.

![stop_not_at_terminal_no_blocking_object](./images/lane_change-stop_not_at_terminal_no_blocking_object.drawio.svg)

If there are obstacles within the lane change section of the target lane, the ego vehicle stops closer to the obstacle ahead, without keeping the usual distance needed for a lane change.

![stop_not_at_terminal](./images/lane_change-stop_not_at_terminal.drawio.svg)

#### When the target lane is far away

If the target lane for the lane change is far away and not next to the current lane, the ego vehicle stops closer to the obstacle ahead, as maintaining the usual distance for a lane change is not necessary.

![stop_far_from_target_lane](./images/lane_change-stop_far_from_target_lane.drawio.svg)

![stop_not_at_terminal](./images/lane_change-stop_not_at_terminal.drawio.svg)

### Lane Change When Stuck

The ego vehicle is considered stuck if it is stopped and meets any of the following conditions:

- There is an obstacle in front of the current lane
- The ego vehicle is at the end of the current lane

In this case, the safety check for lane change is relaxed compared to normal times.
Please refer to the 'stuck' section under the 'Collision checks during lane change' for more details.
The function to stop by keeping a margin against forward obstacle in the previous section is being performed to achieve this feature.

### Lane change regulations

If you want to regulate lane change on crosswalks, intersections, or traffic lights, the lane change module is disabled near any of them.
To regulate lane change on crosswalks, intersections, or traffic lights, set `regulation.crosswalk`, `regulation.intersection` or `regulation.traffic_light` to `true`.
If the ego vehicle gets stuck, to avoid stuck, it enables lane change in crosswalk/intersection.
If the ego vehicle stops more than `stuck_detection.stop_time` seconds, it is regarded as a stuck.
If the ego vehicle velocity is smaller than `stuck_detection.velocity`, it is regarded as stopping.

## Lane change completion checks

To determine if the ego vehicle has successfully changed lanes, one of two criteria must be met: either the longitudinal or the lateral criteria.

For the longitudinal criteria, the ego vehicle must pass the lane-changing end pose and be within the `finish_judge_buffer` distance from it. The module then checks if the ego vehicle is in the target lane. If true, the module returns success. This check ensures that the planner manager updates the root lanelet correctly based on the ego vehicle's current pose. Without this check, if the ego vehicle is changing lanes while avoiding an obstacle and its current pose is in the original lane, the planner manager might set the root lanelet as the original lane. This would force the ego vehicle to perform the lane change again. With the target lane check, the ego vehicle is confirmed to be in the target lane, and the planner manager can correctly update the root lanelets.

If the longitudinal criteria are not met, the module evaluates the lateral criteria. For the lateral criteria, the ego vehicle must be within `finish_judge_lateral_threshold` distance from the target lane's centerline, and the angle deviation must be within `finish_judge_lateral_angle_deviation` degrees. The angle deviation check ensures there is no sudden steering. If the angle deviation is set too high, the ego vehicle's orientation could deviate significantly from the centerline, causing the trajectory follower to aggressively correct the steering to return to the centerline. Keeping the angle deviation value as small as possible avoids this issue.

The process of determining lane change completion is shown in the following diagram.

```plantuml
@startuml
skinparam defaultTextAlignment center
skinparam backgroundColor #WHITE

title Lane change completion judge

start

:Calculate distance from current ego pose to lane change end pose;

if (Is ego velocity < 1.0?) then (<color:green><b>YES</b></color>)
  :Set <b>finish_judge_buffer</b> to 0.0;
else (<color:red><b>NO</b></color>)
  :Set <b>finish_judge_buffer</b> to lane_change_finish_judge_buffer;
endif

if (ego has passed the end_pose and ego is <b>finish_judge_buffer</b> meters away from end_pose?) then (<color:green><b>YES</b></color>)
  if (Current ego pose is in target lanes' polygon?) then (<color:green><b>YES</b></color>)
    :Lane change is <color:green><b>completed</b></color>;
    stop
  else (<color:red><b>NO</b></color>)
:Lane change is <color:red><b>NOT</b></color> completed;
stop
  endif
else (<color:red><b>NO</b></color>)
endif

if (ego's yaw deviation to centerline exceeds finish_judge_lateral_angle_deviation?) then (<color:red><b>YES</b></color>)
  :Lane change is <color:red><b>NOT</b></color> completed;
  stop
else (<color:green><b>NO</b></color>)
  :Calculate distance to the target lanes' centerline;
  if (abs(distance to the target lanes' centerline) is less than finish_judge_lateral_threshold?) then (<color:green><b>YES</b></color>)
    :Lane change is <color:green><b>completed</b></color>;
    stop
  else (<color:red><b>NO</b></color>)
    :Lane change is <color:red><b>NOT</b></color> completed;
    stop
  endif
endif

@enduml
```

## Terminal Lane Change Path

Depending on the space configuration around the Ego vehicle, it is possible that a valid LC path cannot be generated. If that happens, then Ego will get stuck at `terminal_start` and not be able to proceed. Therefore we introduced the terminal LC path feature; when ego gets near to the terminal point (dist to `terminal_start` is less than the maximum lane change length) a terminal lane changing path will be computed starting from the terminal start point on the current lane and connects to the target lane. The terminal path only needs to be computed once in each execution of LC module. If no valid candidate paths are found in the path generation process, then the terminal path will be used as a fallback candidate path, the safety of the terminal path is not ensured and therefore it can only be force approved. The following images illustrate the expected behavior without and with the terminal path feature respectively:

![no terminal path](./images/lane_change-no_terminal_path.png)

![terminal path](./images/lane_change-terminal_path.png)

Additionally if terminal path feature is enabled and path is computed, stop point placement can be configured to be at the edge of the current lane instead of at the `terminal_start` position, as indicated by the dashed red line in the image above.

## Generating Path Using Frenet Planner

!!! warning

    Generating path using Frenet planner applies only when ego is near terminal start

If the ego vehicle is far from the terminal, the lane change module defaults to using the [path shifter](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/autoware_behavior_path_planner_common/docs/behavior_path_planner_path_generation_design/). This ensures that the lane change is completed while the target lane remains a neighbor of the current lane. However, this approach may result in high curvature paths near the terminal, potentially causing long vehicles to deviate from the lane.

To address this, the lane change module provides an option to choose between the path shifter and the [Frenet planner](https://autowarefoundation.github.io/autoware.universe/main/planning/sampling_based_planner/autoware_frenet_planner/). The Frenet planner allows for some flexibility in the lane change endpoint, extending the lane changing end point slightly beyond the current lane's neighbors.

The following table provides comparisons between the planners

<div align="center">
  <table>
    <tr>
      <td align="center">With Path Shifter</td>
      <td align="center">With Frenet Planner</td>
    </tr>
    <tr>
      <td><img src="./images/terminal_straight_path_shifter.png" alt="Path shifter result at straight lanelets" width="450"></a></td>
      <td><img src="./images/terminal_straight_frenet.png" alt="Frenet planner result at straight lanelets" width="450"></a></td>
    </tr>
    <tr>
      <td><img src="./images/terminal_branched_path_shifter.png" alt="Path shifter result at branching lanelets" width="450"></a></td>
      <td><img src="./images/terminal_branched_frenet.png" alt="Frenet planner result at branching lanelets" width="450"></a></td>
    </tr>
    <tr>
      <td><img src="./images/terminal_curved_path_shifter.png" alt="Path shifter result at curved lanelets" width="450"></a></td>
      <td><img src="./images/terminal_curved_frenet.png" alt="Frenet planner result at curved lanelets" width="450"></a></td>
    </tr>
  </table>
</div>

!!! note

    The planner can be enabled or disabled using the `frenet.enable` flag.

!!! note

    Since only a segment of the target lane is used as input to generate the lane change path, the end pose of the lane change segment may not smoothly connect to the target lane centerline. To address this, increase the value of `frenet.th_curvature_smoothing` to improve the smoothness.

!!! note

    The yaw difference threshold (`frenet.th_yaw_diff`) limits the maximum curvature difference between the end of the prepare segment and the lane change segment. This threshold might prevent the generation of a lane change path when the lane curvature is high. In such cases, you can increase the frenet.th_yaw_diff value. However, note that if the prepare path was initially shifted by other modules, the resultant steering may not be continuous.

## Aborting a Previously Approved Lane Change

Once the lane change path is approved, there are several situations where we may need to abort the maneuver. The abort process is triggered when any of the following conditions is met

1. The ego vehicle is near a traffic light, crosswalk, or intersection, and it is possible to complete the lane change after the ego vehicle passes these areas.
2. The target object list is updated, requiring us to [delay lane change](#delay-lane-change-check)
3. The lane change is forcefully canceled via [RTC](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/cooperation/).
4. The path has become unsafe.

Furthermore, if the path has become unsafe, there are three possible outcomes for the maneuver:

1. **CANCEL**: The approved lane change path is canceled, and the ego vehicle resumes its previous maneuver.
2. **ABORT**: The lane change module generates a return path to bring the ego vehicle back to its current lane.
3. **CRUISE** or **STOP**: If aborting is not feasible, the ego vehicle continues with the lane change. [Another module](https://autowarefoundation.github.io/autoware.universe/main/planning/autoware_obstacle_cruise_planner/) should decide whether the ego vehicle should cruise or stop in this scenario.

**CANCEL** can be enabled by setting the `cancel.enable_on_prepare_phase` flag to `true`, and **ABORT** can be enabled by setting the `cancel.enable_on_lane_changing_phase` flag to true.

!!! warning

    Enabling **CANCEL** is a prerequisite for enabling **ABORT**.

!!! warning

    When **CANCEL** is disabled, all maneuvers will default to either **CRUISE** or **STOP**.

The chart shows the high level flow of the lane change abort process.

```plantuml
@startuml
skinparam defaultTextAlignment center
skinparam backgroundColor #WHITE

title High-Level Flow of Lane Change Abort Process

while(Lane Following)
  if (Lane Change required) then (**YES**)
    if (Safe to change lane) then (<color:green><b>SAFE</b></color>)
      :Approve safe path;
      while(Lane change maneuver is completed?) is (**NO**)
          if (Is cancel/abort Condition satisfied) then (**NO**)
          else (**YES**)
            if (Is Enough margin to retry lane change) then (**YES**)
              if (Ego is preparing to change lane) then (**YES**)
              #LightPink:CANCEL;
              break
              else (**NO**)
              if (Overhang from current lanes is less than threshold?) then (**YES**)
              #Cyan:ABORT;
              break
              else (NO)
              endif
            endif
          else (**NO**)
          endif
          #Yellow:CRUISE/STOP;
        endif
      endwhile (**YES**)
    else (<color:red><b>UNSAFE</b></color>)
    endif
  else (**NO**)
  endif
endwhile
-[hidden]->
detach
@enduml
```

### Preventing Oscillating Paths When Unsafe

Lane change paths can oscillate when conditions switch between safe and unsafe. To address this, a hysteresis count check is added before executing an abort maneuver. When the path is unsafe, the `unsafe_hysteresis_count_` increases. If it exceeds the `unsafe_hysteresis_threshold`, an abort condition check is triggered. This logic stabilizes the path approval process and prevents abrupt changes caused by temporary unsafe conditions.

```plantuml
@startuml
skinparam defaultTextAlignment center
skinparam backgroundColor #WHITE

title Hysteresis count flow for oscillation prevention

while (lane changing completed?) is (FALSE)
if (Perform collision check?) then (<color:green><b>SAFE</b></color>)
  :Reset unsafe_hysteresis_count_;
else (<color:red><b>UNSAFE</b></color>)
  :Increase unsafe_hysteresis_count_;
  if (unsafe_hysteresis_count_ is more than\n unsafe_hysteresis_threshold?) then (<color:green><b>FALSE</b></color>)
  else (<color:red><b>TRUE</b></color>)
    #LightPink:Check abort condition;
  -[hidden]->
  detach
  endif
endif
endwhile (TRUE)
stop
@enduml
```

### Evaluating Ego Vehicle's Position to Prevent Abrupt Maneuvers

To avoid abrupt maneuvers during **CANCEL** or **ABORT**, the lane change module ensures the ego vehicle can safely return to the original lane. This is done through geometric checks that verify whether the ego vehicle remains within the lane boundaries.

The edges of the ego vehicle’s footprint are compared against the boundary of the current lane to determine if they exceed the overhang tolerance, `cancel.overhang_tolerance`. If the distance from any edge of the footprint to the boundary exceeds this threshold, the vehicle is considered to be diverging.

The footprints checked against the lane boundary include:

1. Current Footprint: Based on the ego vehicle's current position.
2. Future Footprint: Based on the ego vehicle's estimated position after traveling a distance, calculated as $𝑑_{est}=𝑣_{ego} \cdot \Delta_{𝑡}$, where

   - $v_{ego}$ is ego vehicle's current velocity
   - $\Delta_{t}$ is parameterized time constant value, `cancel.delta_time`.

   as depicted in the following diagram

   ![can_return](./images/check_able_to_return.png)

!!! note

    The ego vehicle is considered capable of safely returning to the current lane only if **BOTH** the current and future footprint checks are `true`.

### Checking Approved Path Safety

The lane change module samples accelerations along the path and recalculates velocity to perform safety checks. The motivation for this feature is explained in the [Limitation](#limitation) section.

The computation of sampled accelerations is as follows:

Let

$$
\text{resolution} = \frac{a_{\text{min}} - a_{\text{LC}}}{N}
$$

The sequence of sampled accelerations is then given as

$$
\text{acc} = a_{\text{LC}} + k \cdot \text{resolution}, \quad k = [0, N]
$$

where

- $a_{\text{min}}$, is the minimum of the parameterized [global acceleration constant](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/common/common.param.yaml) `normal.min_acc` or the [parameterized constant](#essential-lane-change-parameters) `trajectory.min_longitudinal_acceleration`.
- $a_{\text{LC}}$ is the acceleration used to generate the approved path.
- $N$ is the parameterized constant `cancel.deceleration_sampling`

If none of the sampled accelerations pass the safety check, the lane change path will be canceled, subject to the [hysteresis check](#preventing-oscillating-paths-when-unsafe).

### Cancel

When lane change is canceled, the approved path is reset. After the reset, the ego vehicle will return to following the original reference path (the last approved path before the lane change started), as illustrated in the following image

![cancel](./images/lane_change-cancel.png)

The following parameters can be configured to tune the behavior of the cancel process:

1. [Safety constraints](#safety-constraints-to-cancel-lane-change-path) for cancel.
2. [Safety constraints](#safety-constraints-specifically-for-stopped-or-parked-vehicles) for parked vehicle.

!!! note

    To ensure feasible behavior, all safety constraint values must be equal to or less than their corresponding parameters in the [execution](#safety-constraints-during-lane-change-path-is-computed) settings.

    - The closer the values, the more conservative the lane change behavior will be. This means it will be easier to cancel the lane change but harder for the ego vehicle to complete a lane change.
    - The larger the difference, the more aggressive the lane change behavior will be. This makes it harder to cancel the lane change but easier for the ego vehicle to change lanes.

### Abort

During the prepare phase, the ego vehicle follows the previously approved path. However, once the ego vehicle begins the lane change, its heading starts to diverge from this path. Resetting to the previously approved path in this situation would cause abrupt steering, as the controller would attempt to rapidly realign the vehicle with the reference trajectory.

Instead, the lane change module generates an abort path. This return path is specifically designed to guide the ego vehicle back to the current lane, avoiding any sudden maneuvers. The following image provides an illustration of the abort process.

![abort](./images/lane_change-abort.png)

The abort path is generated by shifting the approved lane change path using the path shifter. This ensures the continuity in lateral velocity, and prevents abrupt changes in the vehicle’s movement. The abort start shift and abort end shift are computed as follows:

1. Start Shift: $d_{start}^{abort} = v_{ego} \cdot \Delta_{t}$
2. End Shift: $d_{end}^{abort} = v_{ego} \cdot ( \Delta_{t} + t_{end} )$

- $v_{ego}$ is ego vehicle's current velocity
- $\Delta_{t}$ is parameterized time constant value, `cancel.delta_time`.
- $t_{end}$ is the parameterized time constant value, `cancel.duration`.

as depicted in the following diagram

![abort_computation](./images/lane_change-abort_computation.png)

!!! note

    When executing the abort process, comfort is not a primary concern. However, due to safety considerations, limited real-world testing has been conducted to tune or validate this parameter. Currently, the maximum lateral jerk is set to an arbitrary value. To avoid generating a path with excessive lateral jerk, this value can be configured using `cancel.max_lateral_jerk`.

!!! note

    Lane change module returns `ModuleStatus::FAILURE` once abort is completed.

### Stop/Cruise

Once canceling or aborting the lane change is no longer an option, the ego vehicle will proceed with the lane change. This can happen in the following situations:

- The ego vehicle is performing a lane change near a terminal or dead-end, making it impossible to return to the original lane. In such cases, completing the lane change is necessary.
- If safety parameters are tuned too aggressively, it becomes harder to cancel or abort the lane change. This reduces tolerance for unexpected behaviors from surrounding vehicles, such as a trailing vehicle in the target lane suddenly accelerating or a leading vehicle suddenly decelerating. Aggressive settings leave less room for error during the maneuver.

![stop](./images/lane_change-cant_cancel_no_abort.png)

## Parameters

### Essential lane change parameters

The following parameters are configurable in [lane_change.param.yaml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/lane_change/lane_change.param.yaml)

| Name                                         | Unit   | Type   | Description                                                                                                            | Default value      |
| :------------------------------------------- | ------ | ------ | ---------------------------------------------------------------------------------------------------------------------- | ------------------ |
| `time_limit`                                 | [ms]   | double | Time limit for lane change candidate path generation                                                                   | 50.0               |
| `backward_lane_length`                       | [m]    | double | The backward length to check incoming objects in lane change target lane.                                              | 200.0              |
| `backward_length_buffer_for_end_of_lane`     | [m]    | double | The end of lane buffer to ensure ego vehicle has enough distance to start lane change                                  | 3.0                |
| `backward_length_buffer_for_blocking_object` | [m]    | double | The end of lane buffer to ensure ego vehicle has enough distance to start lane change when there is an object in front | 3.0                |
| `backward_length_from_intersection`          | [m]    | double | Distance threshold from the last intersection to invalidate or cancel the lane change path                             | 5.0                |
| `enable_stopped_vehicle_buffer`              | [-]    | bool   | If true, will keep enough distance from current lane front stopped object to perform lane change when possible         | true               |
| `trajectory.max_prepare_duration`            | [s]    | double | The maximum preparation time for the ego vehicle to be ready to perform lane change.                                   | 4.0                |
| `trajectory.min_prepare_duration`            | [s]    | double | The minimum preparation time for the ego vehicle to be ready to perform lane change.                                   | 2.0                |
| `trajectory.lateral_jerk`                    | [m/s3] | double | Lateral jerk value for lane change path generation                                                                     | 0.5                |
| `trajectory.minimum_lane_changing_velocity`  | [m/s]  | double | Minimum speed during lane changing process.                                                                            | 2.78               |
| `trajectory.lon_acc_sampling_num`            | [-]    | int    | Number of possible lane-changing trajectories that are being influenced by longitudinal acceleration                   | 3                  |
| `trajectory.lat_acc_sampling_num`            | [-]    | int    | Number of possible lane-changing trajectories that are being influenced by lateral acceleration                        | 3                  |
| `trajectory.max_longitudinal_acc`            | [m/s2] | double | maximum longitudinal acceleration for lane change                                                                      | 1.0                |
| `trajectory.min_longitudinal_acc`            | [m/s2] | double | maximum longitudinal deceleration for lane change                                                                      | -1.0               |
| `trajectory.lane_changing_decel_factor`      | [-]    | double | longitudinal deceleration factor during lane changing phase                                                            | 0.5                |
| `trajectory.th_prepare_curvature`            | [-]    | double | If the maximum curvature of the prepare segment exceeds the threshold, the prepare segment is invalid.                 | 0.03               |
| `min_length_for_turn_signal_activation`      | [m]    | double | Turn signal will be activated if the ego vehicle approaches to this length from minimum lane change length             | 10.0               |
| `lateral_acceleration.velocity`              | [m/s]  | double | Reference velocity for lateral acceleration calculation (look up table)                                                | [0.0, 4.0, 10.0]   |
| `lateral_acceleration.min_values`            | [m/s2] | double | Min lateral acceleration values corresponding to velocity (look up table)                                              | [0.4, 0.4, 0.4]    |
| `lateral_acceleration.max_values`            | [m/s2] | double | Max lateral acceleration values corresponding to velocity (look up table)                                              | [0.65, 0.65, 0.65] |

### Parameter to judge if lane change is completed

The following parameters are used to judge lane change completion.

| Name                                   | Unit  | Type   | Description                                                                                                            | Default value |
| :------------------------------------- | ----- | ------ | ---------------------------------------------------------------------------------------------------------------------- | ------------- |
| `lane_change_finish_judge_buffer`      | [m]   | double | The longitudinal distance starting from the lane change end pose.                                                      | 2.0           |
| `finish_judge_lateral_threshold`       | [m]   | double | The lateral distance from targets lanes' centerline. Used in addition with `finish_judge_lateral_angle_deviation`      | 0.1           |
| `finish_judge_lateral_angle_deviation` | [deg] | double | Ego angle deviation with reference to target lanes' centerline. Used in addition with `finish_judge_lateral_threshold` | 2.0           |

### Lane change regulations

| Name                       | Unit | Type    | Description                                                | Default value |
| :------------------------- | ---- | ------- | ---------------------------------------------------------- | ------------- |
| `regulation.crosswalk`     | [-]  | boolean | Allow lane change in between crosswalks                    | true          |
| `regulation.intersection`  | [-]  | boolean | Allow lane change in between intersections                 | true          |
| `regulation.traffic_light` | [-]  | boolean | Allow lane change to be performed in between traffic light | true          |

### Ego vehicle stuck detection

| Name                        | Unit  | Type   | Description                                         | Default value |
| :-------------------------- | ----- | ------ | --------------------------------------------------- | ------------- |
| `stuck_detection.velocity`  | [m/s] | double | Velocity threshold for ego vehicle stuck detection  | 0.1           |
| `stuck_detection.stop_time` | [s]   | double | Stop time threshold for ego vehicle stuck detection | 3.0           |

### Delay Lane Change

| Name                                              | Unit | Type   | Description                                                                                           | Default value |
| :------------------------------------------------ | ---- | ------ | ----------------------------------------------------------------------------------------------------- | ------------- |
| `delay_lane_change.enable`                        | [-]  | bool   | Flag to enable/disable lane change delay feature                                                      | true          |
| `delay_lane_change.check_only_parked_vehicle`     | [-]  | bool   | Flag to limit delay feature for only parked vehicles                                                  | false         |
| `delay_lane_change.min_road_shoulder_width`       | [m]  | double | Width considered as road shoulder if lane doesn't have road shoulder when checking for parked vehicle | 0.5           |
| `delay_lane_change.th_parked_vehicle_shift_ratio` | [-]  | double | Stopped vehicles beyond this distance ratio from center line will be considered as parked             | 0.6           |

### Terminal Lane Change Path

The following parameters are used to configure terminal lane change path feature.

| Name                              | Unit | Type | Description                                                               | Default value |
| :-------------------------------- | ---- | ---- | ------------------------------------------------------------------------- | ------------- |
| `terminal_path.enable`            | [-]  | bool | Flag to enable/disable terminal path feature                              | true          |
| `terminal_path.disable_near_goal` | [-]  | bool | Flag to disable terminal path feature if ego is near goal                 | true          |
| `terminal_path.stop_at_boundary`  | [-]  | bool | If true, ego will stop at current lane boundary instead of middle of lane | false         |

### Generating Lane Changing Path using Frenet Planner

!!! warning

    Only applicable when ego is near terminal start

| Name                            | Unit  | Type   | Description                                                                                                                                         | Default value |
| :------------------------------ | ----- | ------ | --------------------------------------------------------------------------------------------------------------------------------------------------- | ------------- |
| `frenet.enable`                 | [-]   | bool   | Flag to enable/disable frenet planner when ego is near terminal start.                                                                              | true          |
| `frenet.th_yaw_diff`            | [deg] | double | If the yaw diff between of the prepare segment's end and lane changing segment's start exceed the threshold , the lane changing segment is invalid. | 10.0          |
| `frenet.th_curvature_smoothing` | [-]   | double | Filters and appends target path points with curvature below the threshold to candidate path.                                                        | 0.1           |

### Collision checks

#### Target Objects

| Name                       | Unit | Type    | Description                                 | Default value |
| :------------------------- | ---- | ------- | ------------------------------------------- | ------------- |
| `target_object.car`        | [-]  | boolean | Include car objects for safety check        | true          |
| `target_object.truck`      | [-]  | boolean | Include truck objects for safety check      | true          |
| `target_object.bus`        | [-]  | boolean | Include bus objects for safety check        | true          |
| `target_object.trailer`    | [-]  | boolean | Include trailer objects for safety check    | true          |
| `target_object.unknown`    | [-]  | boolean | Include unknown objects for safety check    | true          |
| `target_object.bicycle`    | [-]  | boolean | Include bicycle objects for safety check    | true          |
| `target_object.motorcycle` | [-]  | boolean | Include motorcycle objects for safety check | true          |
| `target_object.pedestrian` | [-]  | boolean | Include pedestrian objects for safety check | true          |

#### common

| Name                                       | Unit | Type   | Description                                                                                                                                 | Default value |
| :----------------------------------------- | ---- | ------ | ------------------------------------------------------------------------------------------------------------------------------------------- | ------------- |
| `safety_check.lane_expansion.left_offset`  | [m]  | double | Expand the left boundary of the detection area, allowing objects previously outside on the left to be detected and registered as targets.   | 0.0           |
| `safety_check.lane_expansion.right_offset` | [m]  | double | Expand the right boundary of the detection area, allowing objects previously outside on the right to be detected and registered as targets. | 0.0           |

#### Additional parameters

| Name                                                     | Unit  | Type    | Description                                                                                                                                                                                                | Default value |
| :------------------------------------------------------- | ----- | ------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------- |
| `collision_check.enable_for_prepare_phase.general_lanes` | [-]   | boolean | Perform collision check starting from the prepare phase for situations not explicitly covered by other settings (e.g., intersections). If `false`, collision check only evaluated for lane changing phase. | false         |
| `collision_check.enable_for_prepare_phase.intersection`  | [-]   | boolean | Perform collision check starting from prepare phase when ego is in intersection. If `false`, collision check only evaluated for lane changing phase.                                                       | true          |
| `collision_check.enable_for_prepare_phase.turns`         | [-]   | boolean | Perform collision check starting from prepare phase when ego is in lanelet with turn direction tags. If `false`, collision check only evaluated for lane changing phase.                                   | true          |
| `collision_check.check_current_lanes`                    | [-]   | boolean | If true, the lane change module always checks objects in the current lanes for collision assessment. If false, it only checks objects in the current lanes when the ego vehicle is stuck.                  | false         |
| `collision_check.check_other_lanes`                      | [-]   | boolean | If true, the lane change module includes objects in other lanes when performing collision assessment.                                                                                                      | false         |
| `collision_check.use_all_predicted_paths`                | [-]   | boolean | If false, use only the predicted path that has the maximum confidence.                                                                                                                                     | true          |
| `collision_check.prediction_time_resolution`             | [s]   | double  | Time resolution for object's path interpolation and collision check.                                                                                                                                       | 0.5           |
| `collision_check.yaw_diff_threshold`                     | [rad] | double  | Maximum yaw difference between ego and object when executing rss-based collision checking                                                                                                                  | 3.1416        |

#### safety constraints during lane change path is computed

| Name                                                         | Unit    | Type   | Description                                                                                                                                                    | Default value |
| :----------------------------------------------------------- | ------- | ------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------- |
| `safety_check.execution.expected_front_deceleration`         | [m/s^2] | double | The front object's maximum deceleration when the front vehicle perform sudden braking. (\*1)                                                                   | -1.0          |
| `safety_check.execution.expected_rear_deceleration`          | [m/s^2] | double | The rear object's maximum deceleration when the rear vehicle perform sudden braking. (\*1)                                                                     | -1.0          |
| `safety_check.execution.rear_vehicle_reaction_time`          | [s]     | double | The reaction time of the rear vehicle driver which starts from the driver noticing the sudden braking of the front vehicle until the driver step on the brake. | 2.0           |
| `safety_check.execution.rear_vehicle_safety_time_margin`     | [s]     | double | The time buffer for the rear vehicle to come into complete stop when its driver perform sudden braking.                                                        | 1.0           |
| `safety_check.execution.lateral_distance_max_threshold`      | [m]     | double | The lateral distance threshold that is used to determine whether lateral distance between two object is enough and whether lane change is safe.                | 2.0           |
| `safety_check.execution.longitudinal_distance_min_threshold` | [m]     | double | The longitudinal distance threshold that is used to determine whether longitudinal distance between two object is enough and whether lane change is safe.      | 3.0           |
| `safety_check.execution.longitudinal_velocity_delta_time`    | [m]     | double | The time multiplier that is used to compute the actual gap between vehicle at each predicted points (not RSS distance)                                         | 0.8           |

#### safety constraints specifically for stopped or parked vehicles

| Name                                                      | Unit    | Type   | Description                                                                                                                                                    | Default value |
| :-------------------------------------------------------- | ------- | ------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------- |
| `safety_check.parked.expected_front_deceleration`         | [m/s^2] | double | The front object's maximum deceleration when the front vehicle perform sudden braking. (\*1)                                                                   | -1.0          |
| `safety_check.parked.expected_rear_deceleration`          | [m/s^2] | double | The rear object's maximum deceleration when the rear vehicle perform sudden braking. (\*1)                                                                     | -2.0          |
| `safety_check.parked.rear_vehicle_reaction_time`          | [s]     | double | The reaction time of the rear vehicle driver which starts from the driver noticing the sudden braking of the front vehicle until the driver step on the brake. | 1.0           |
| `safety_check.parked.rear_vehicle_safety_time_margin`     | [s]     | double | The time buffer for the rear vehicle to come into complete stop when its driver perform sudden braking.                                                        | 0.8           |
| `safety_check.parked.lateral_distance_max_threshold`      | [m]     | double | The lateral distance threshold that is used to determine whether lateral distance between two object is enough and whether lane change is safe.                | 1.0           |
| `safety_check.parked.longitudinal_distance_min_threshold` | [m]     | double | The longitudinal distance threshold that is used to determine whether longitudinal distance between two object is enough and whether lane change is safe.      | 3.0           |
| `safety_check.parked.longitudinal_velocity_delta_time`    | [m]     | double | The time multiplier that is used to compute the actual gap between vehicle at each predicted points (not RSS distance)                                         | 0.8           |

##### safety constraints to cancel lane change path

| Name                                                      | Unit    | Type   | Description                                                                                                                                                    | Default value |
| :-------------------------------------------------------- | ------- | ------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------- |
| `safety_check.cancel.expected_front_deceleration`         | [m/s^2] | double | The front object's maximum deceleration when the front vehicle perform sudden braking. (\*1)                                                                   | -1.0          |
| `safety_check.cancel.expected_rear_deceleration`          | [m/s^2] | double | The rear object's maximum deceleration when the rear vehicle perform sudden braking. (\*1)                                                                     | -2.0          |
| `safety_check.cancel.rear_vehicle_reaction_time`          | [s]     | double | The reaction time of the rear vehicle driver which starts from the driver noticing the sudden braking of the front vehicle until the driver step on the brake. | 1.5           |
| `safety_check.cancel.rear_vehicle_safety_time_margin`     | [s]     | double | The time buffer for the rear vehicle to come into complete stop when its driver perform sudden braking.                                                        | 0.8           |
| `safety_check.cancel.lateral_distance_max_threshold`      | [m]     | double | The lateral distance threshold that is used to determine whether lateral distance between two object is enough and whether lane change is safe.                | 1.0           |
| `safety_check.cancel.longitudinal_distance_min_threshold` | [m]     | double | The longitudinal distance threshold that is used to determine whether longitudinal distance between two object is enough and whether lane change is safe.      | 2.5           |
| `safety_check.cancel.longitudinal_velocity_delta_time`    | [m]     | double | The time multiplier that is used to compute the actual gap between vehicle at each predicted points (not RSS distance)                                         | 0.6           |

##### safety constraints used during lane change path is computed when ego is stuck

| Name                                                     | Unit    | Type   | Description                                                                                                                                                    | Default value |
| :------------------------------------------------------- | ------- | ------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------- |
| `safety_check.stuck.expected_front_deceleration`         | [m/s^2] | double | The front object's maximum deceleration when the front vehicle perform sudden braking. (\*1)                                                                   | -1.0          |
| `safety_check.stuck.expected_rear_deceleration`          | [m/s^2] | double | The rear object's maximum deceleration when the rear vehicle perform sudden braking. (\*1)                                                                     | -1.0          |
| `safety_check.stuck.rear_vehicle_reaction_time`          | [s]     | double | The reaction time of the rear vehicle driver which starts from the driver noticing the sudden braking of the front vehicle until the driver step on the brake. | 2.0           |
| `safety_check.stuck.rear_vehicle_safety_time_margin`     | [s]     | double | The time buffer for the rear vehicle to come into complete stop when its driver perform sudden braking.                                                        | 1.0           |
| `safety_check.stuck.lateral_distance_max_threshold`      | [m]     | double | The lateral distance threshold that is used to determine whether lateral distance between two object is enough and whether lane change is safe.                | 2.0           |
| `safety_check.stuck.longitudinal_distance_min_threshold` | [m]     | double | The longitudinal distance threshold that is used to determine whether longitudinal distance between two object is enough and whether lane change is safe.      | 3.0           |
| `safety_check.stuck.longitudinal_velocity_delta_time`    | [m]     | double | The time multiplier that is used to compute the actual gap between vehicle at each predicted points (not RSS distance)                                         | 0.8           |

(\*1) the value must be negative.

### Abort lane change

The following parameters are configurable in `lane_change.param.yaml`.

| Name                                   | Unit    | Type    | Description                                                                                                      | Default value |
| :------------------------------------- | ------- | ------- | ---------------------------------------------------------------------------------------------------------------- | ------------- |
| `cancel.enable_on_prepare_phase`       | [-]     | boolean | Enable cancel lane change                                                                                        | true          |
| `cancel.enable_on_lane_changing_phase` | [-]     | boolean | Enable abort lane change.                                                                                        | false         |
| `cancel.delta_time`                    | [s]     | double  | The time taken to start steering to return to the center line.                                                   | 3.0           |
| `cancel.duration`                      | [s]     | double  | The time taken to complete returning to the center line.                                                         | 3.0           |
| `cancel.max_lateral_jerk`              | [m/sss] | double  | The maximum lateral jerk for abort path                                                                          | 1000.0        |
| `cancel.overhang_tolerance`            | [m]     | double  | Lane change cancel is prohibited if the vehicle head exceeds the lane boundary more than this tolerance distance | 0.0           |
| `cancel.unsafe_hysteresis_threshold`   | [-]     | int     | threshold that helps prevent frequent switching between safe and unsafe decisions                                | 10            |
| `cancel.deceleration_sampling_num`     | [-]     | int     | Number of deceleration patterns to check safety to cancel lane change                                            | 5             |

### Debug

The following parameters are configurable in `lane_change.param.yaml`.

| Name                   | Unit | Type    | Description                  | Default value |
| :--------------------- | ---- | ------- | ---------------------------- | ------------- |
| `publish_debug_marker` | [-]  | boolean | Flag to publish debug marker | false         |

## Debug Marker & Visualization

To enable the debug marker, execute (no restart is needed)

```shell
ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner lane_change.publish_debug_marker true

```

or simply set the `publish_debug_marker` to `true` in the `lane_change.param.yaml` for permanent effect (restart is needed).

Then add the marker

```shell
/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/lane_change_left
```

in `rviz2`.

![debug](./images/lane_change-debug-1.png)

![debug2](./images/lane_change-debug-2.png)

![debug3](./images/lane_change-debug-3.png)

Available information

1. Ego to object relation, plus safety check information
2. Ego vehicle interpolated pose up to the latest safety check position.
3. Object is safe or not, shown by the color of the polygon (Green = Safe, Red = unsafe)
4. Valid candidate paths.
5. Position when lane changing start and end.

## Limitation

1. When a lane change is canceled, the lane change module returns `ModuleStatus::FAILURE`. As the module is removed from the approved module stack (see [Failure modules](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/autoware_behavior_path_planner/docs/behavior_path_planner_manager_design/#failure-modules)), a new instance of the lane change module is initiated. Due to this, any information stored prior to the reset is lost. For example, the `lane_change_prepare_duration` in the `TransientData` is reset to its maximum value.
2. The lane change module has no knowledge of any velocity modifications introduced to the path after it is approved. This is because other modules may add deceleration points after subscribing to the behavior path planner output, and the final velocity is managed by the [velocity smoother](https://autowarefoundation.github.io/autoware.universe/main/planning/autoware_velocity_smoother/). Since this limitation affects **CANCEL**, the lane change module mitigates it by [sampling accelerations along the approved lane change path](#checking-approved-path-safety). These sampled accelerations are used during safety checks to estimate the velocity that might occur if the ego vehicle decelerates.
3. Ideally, the abort path should account for whether its execution would affect trailing vehicles in the current lane. However, the lane change module does not evaluate such interactions or assess whether the abort path is safe. As a result, **the abort path is not guaranteed to be safe**. To minimize the risk of unsafe situations, the abort maneuver is only permitted if the ego vehicle has not yet diverged from the current lane.
4. Due to limited resources, the abort path logic is not fully optimized. The generated path may overshoot, causing the return trajectory to slightly shift toward the opposite lane. This can be dangerous, especially if the opposite lane has traffic moving in the opposite direction. Furthermore, the logic does not account for different vehicle types, which can lead to varying effects. For instance, the behavior might differ significantly between a bus and a small passenger car.
