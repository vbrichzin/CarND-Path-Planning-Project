# Reflection on project

Since the very good and detailed Q&A session started with the trajectory generation path I am also going to reflect on that first.

## Trajectory generation

The actual path planning resp. trajectory generation goes from [line 210 to 319](./src/main.cpp#L210).

As the whole code was taken from the Q&A session I am going to repeat the most important steps.
First the last two points of the previous trajectory, or the actual car position is taken if there is no previous trajectory, see lines 221-250. Furthermore three points in the distance (30m, 60m and 90m) are taken, see lines 252-262. These 5 points comprise in total the "anker" points through which a spline will be calculated. Yet before the spline is intialized, the points are transformed from global map coordinates to local car coordinates, see lines 264-272. Then the spline is created with these five "anker" points, see line 274-278.
In order to ensure a more steady change in the trajectory, the past trajectory that is not traversed during the simulator timestep is onto the next trajectory path for the next timestep, `next_x_vals` and `next_y_vals`, see lines 284-289. The additional points to append to the next timestep trajectory list of points are calculated from the generated spline, through the linearization assumption that Aaron described in the Q&A, see lines 291-304. Before appending them to reach the 50 trajectory points again each point pair is converted back from local car coordinates to global map coordinates, see lines 309-317. With that the trajectory generation is complete.

## Other car predictions

The prediction part goes from [line 106 to 170](./src/main.cpp#L106).

As was mentioned in the Q&A as a hint for implementation piece is predicting outwards whether there is a car ahead in the own vehicles lane within a certain distance or a car on the left or right side of the own vehicle lane also within a certain distance, but this time in front and behind the own vehicles expected position.
After defining the relevant flags in lines 106-111, first it is determined in a loop over all vehicles from the sensor fusion detection, in which lane the respective other vehicle is, see lines 117-130. By calculating the speed of that vehicle, lines 131-135, the distance to that other vehicle can be projected out, see line 138. Then the comparison is done that calculates if the other vehicle is in the vehicles own lane and the gap is small (<30m in front), resp. for vehicles on the left and right side the same is done, but looking for a gap <30m in front and <30m also from behind. If the conditions are true then the flags are set, see lines 141-168.
In effect, the flags will then be used in the behavior part to see if a lane change can be performed or not. As such, the flags act like a binary cost function.

## Behavior planning

The behavior planning part goes from [line 172 to 208](./src/main.cpp#L172).

With the flags generated whether there is a vehicle ahead to react to or there are vehicles to the left or right so that it is not safe to change lane the implementation is fairly straight forward.
First it is checked if there is a vehicle ahead (flag `other_car_ahead` is set), and if so whether there is a left or right lane to change into from the current position **and** whether it is safe to change the lane left or right (by evaluating the flags `other_car_right` and `other_car_left`). If the conditions are met the lanes are resp. changed one to the left or one to the right, see lines 173 to 189.
If there is no vehicle ahead, then it is first checked if the own vehicle is in the middle lane and if not a change from the left or right lane to the middle lane is initiated, all the while respecting whether it is safe to change the lane (again by evaluating the flags `other_car_right` and `other_car_left`), see lines 192-201. Lastly, still in the branch where it was determined that there was no vehicle ahead the speed of the car is increased up to the speed limit, see lines 204-207.
The actual lane change will then be performed in the trajectory generation where the lane number is an input parameter, see lines 253-255.
In effect the conditional checking if a lane change is possible in the lines 175 and 180 is like setting states for **LCL** resp. **LCR** without explicitly using a state machine logic.

# Further considerations

The implementation is quite simple, but the car is not violating any of the dynamic thresholds or colliding with other cars and even changing lanes when appropriate. The first is ensured through the slow change of the trajectory by using the previous path that was not used in the simulator timestep and only adding onto it in a smooth way by using the spline interpolation. Thanks, Aaron, for showing how this is done in the Q&A! The latter is ensured by rigidly checking if it is safe to change lane left or right, again leaving the dynamic part to the path planner which stays below the thresholds.
This implementation is surely not finding the quickest way through the traffic as with the flags only a very, very simple "cost function" is considered. Yet I observed the car going through the traffic around the track reliably. But the effectiveness depends also a little bit on the other traffic as no complicated maneuvers for the planning are considered.
