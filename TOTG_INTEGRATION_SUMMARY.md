# TOTG Integration Summary

## Overview

Time Optimal Trajectory Generation (TOTG) has been successfully integrated into the velocity_monitor.py application. This enhancement applies time-optimal parameterization to Cartesian paths computed by MoveIt2, resulting in smoother and faster trajectory execution.

---

## What Was Integrated

### 1. IPP Service Client (Line 47)
```python
self.ipp_client = self.create_client(ApplyIPP, '/apply_ipp')
```
- Creates a ROS2 service client for the `/apply_ipp` service
- This service applies Iterative Parabolic Planning (IPP) for time-optimal trajectory generation

### 2. TOTG Application Method (Lines 380-401)
```python
def apply_ipp_totg(self, trajectory, vel_scaling=0.6, acc_scaling=0.4):
    """Call the IPP service to apply TOTG (Time Optimal Trajectory Generation)."""
```

**Features:**
- Takes a MoveIt2-computed trajectory as input
- Applies time-optimal parameterization with velocity and acceleration scaling
- Gracefully falls back to original trajectory if IPP service is unavailable
- Configurable velocity scaling (default: 0.6 = 60% of max velocity)
- Configurable acceleration scaling (default: 0.4 = 40% of max acceleration)
- 5-second timeout for service calls
- Comprehensive logging for debugging

### 3. Integration Point (Lines 417-425)
The TOTG is applied in the Cartesian path execution workflow:
```python
# Get the computed trajectory from MoveIt
trajectory = response.solution

# Log original trajectory size
self.get_logger().info(f'[Cartesian Path] Original trajectory has {len(trajectory.joint_trajectory.points)} points')

# Apply TOTG via IPP service
trajectory = self.apply_ipp_totg(trajectory, vel_scaling, acc_scaling)

# Log optimized trajectory size
self.get_logger().info(f'[Cartesian Path] Final trajectory has {len(trajectory.joint_trajectory.points)} points')

# Execute the time-optimized trajectory
execute_goal = ExecuteTrajectory.Goal()
execute_goal.trajectory = trajectory
```

### 4. GUI Update (Line 746)
The planning method dropdown now shows:
```
"Cartesian Path (MoveIt) + TOTG"
```
This clearly indicates that TOTG optimization is applied to Cartesian paths.

---

## How It Works

### Workflow:
1. **User adds waypoints** in the GUI (X, Y, Z positions)
2. **User sets velocity/acceleration scaling** (sliders in GUI)
3. **User selects "Cartesian Path (MoveIt) + TOTG"** from planner dropdown
4. **User clicks "Execute Path"**
5. **MoveIt2 computes Cartesian path** connecting waypoints with straight lines
6. **IPP service applies TOTG** to optimize trajectory timing
7. **Optimized trajectory is executed** on the robot

### Benefits:
- **Smoother motion**: Time-optimal parameterization reduces jerky movements
- **Faster execution**: Trajectories are optimized for speed while respecting velocity/acceleration limits
- **Better performance**: IPP algorithm ensures joint limits are respected
- **Configurable safety**: Velocity and acceleration scaling provide safety margins

---

## How to Test

### Step 1: Launch the System
```bash
cd /home/ilv/ros2_ws
./launch_robot.sh
```

### Step 2: Verify IPP Service is Available
In another terminal:
```bash
source /home/ilv/ros2_ws/install/setup.bash
ros2 service list | grep apply_ipp
```

You should see: `/apply_ipp`

### Step 3: Use the GUI
1. Open the velocity monitor GUI (should launch automatically)
2. Add waypoints using the "Add Waypoint" button or by entering X, Y, Z coordinates
3. Set velocity scaling (0.0-1.0, default 0.6)
4. Set acceleration scaling (0.0-1.0, default 0.4)
5. Select "Cartesian Path (MoveIt) + TOTG" from the planning method dropdown
6. Click "Execute Path"

### Step 4: Monitor the Logs
Watch for these log messages:
```
[Cartesian Path] Path computed: 100.0% successful
[Cartesian Path] Original trajectory has X points
[TOTG] Applying time-optimal parameterization with vel=0.6, acc=0.4
[TOTG] Time-optimal trajectory generated successfully
[Cartesian Path] Final trajectory has Y points
```

---

## Configuration

### Velocity Scaling (vel_scaling)
- **Range**: 0.0 to 1.0
- **Default**: 0.6 (60% of maximum velocity)
- **Purpose**: Safety margin - prevents robot from moving at maximum speed
- **Recommendation**: Start with 0.6 and increase gradually if needed

### Acceleration Scaling (acc_scaling)
- **Range**: 0.0 to 1.0
- **Default**: 0.4 (40% of maximum acceleration)
- **Purpose**: Safety margin - ensures smooth acceleration/deceleration
- **Recommendation**: Keep at 0.4 for safety, increase to 0.6 for faster motion

---

## Troubleshooting

### IPP Service Not Available
**Symptom:**
```
[TOTG] IPP service not available, using original trajectory
```

**Solution:**
The IPP service may not be running. Check if it's included in your launch file:
```bash
grep -r "apply_ipp" /home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/fairino5_v6_moveit2_config/launch/
```

If not available, the system will gracefully fall back to the original MoveIt trajectory.

### Service Call Timeout
**Symptom:**
```
[TOTG] IPP service call failed, using original trajectory
```

**Possible Causes:**
1. IPP service is processing but taking too long (>5 seconds)
2. Service crashed during computation
3. Trajectory is too complex

**Solution:**
- Check IPP service logs for errors
- Try with fewer waypoints
- Increase timeout in code if needed (currently 5.0 seconds)

### Trajectory Execution Fails
**Symptom:**
Robot doesn't move after "Execute Path"

**Check:**
1. Hardware interface is connected: `ros2 control list_hardware_components`
2. Controllers are active: `ros2 control list_controllers`
3. Joint limits are not violated
4. No collision detected by MoveIt

---

## Technical Details

### IPP (Iterative Parabolic Planning)
- Algorithm for time-optimal trajectory generation
- Respects velocity and acceleration constraints
- Iteratively refines trajectory timing
- Produces smooth, time-optimal motion

### Service Interface
**Service Name:** `/apply_ipp`
**Service Type:** `fairino5_v6_moveit2_config/ApplyIPP`

**Request:**
```
moveit_msgs/RobotTrajectory trajectory
float64 max_velocity_scaling
float64 max_acceleration_scaling
```

**Response:**
```
moveit_msgs/RobotTrajectory trajectory
```

### Integration Architecture
```
User Input → MoveIt2 Cartesian Planner → Raw Trajectory
                                              ↓
                                         IPP Service (TOTG)
                                              ↓
                                    Time-Optimized Trajectory
                                              ↓
                                      Execute on Robot
```

---

## Comparison: With vs Without TOTG

### Without TOTG (Standard MoveIt)
- Trajectory points have even time spacing
- May not utilize full robot capabilities
- Conservative velocity/acceleration profiles
- Longer execution times

### With TOTG (IPP Optimization)
- Time-optimal spacing between points
- Maximizes robot performance within safety limits
- Smooth velocity/acceleration profiles
- Faster execution while respecting limits
- Better motion quality

---

## Files Modified

### `/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/fairino5_v6_moveit2_config/scripts/velocity_monitor.py`

**Changes:**
1. Line 47: Added IPP service client initialization
2. Lines 380-401: Added `apply_ipp_totg()` method
3. Lines 417-425: Integrated TOTG into Cartesian path execution
4. Line 746: Updated GUI label to indicate TOTG feature

**Total Lines Modified/Added:** ~30 lines

---

## Next Steps

### Recommended Testing Sequence:

1. **Test with simple path**
   - 2 waypoints
   - Small distance (e.g., 10cm)
   - Velocity scaling: 0.3
   - Acceleration scaling: 0.3

2. **Test with complex path**
   - 5+ waypoints
   - Larger distances
   - Velocity scaling: 0.6
   - Acceleration scaling: 0.4

3. **Compare performance**
   - Execute same path with and without TOTG
   - Measure execution time
   - Observe motion smoothness

4. **Tune parameters**
   - Gradually increase velocity/acceleration scaling
   - Find optimal values for your application
   - Document preferred settings

---

## Success Criteria

Your TOTG integration is working correctly if:

- ✅ No build errors
- ✅ IPP service is available at launch
- ✅ Logs show TOTG application messages
- ✅ Robot executes Cartesian paths smoothly
- ✅ Execution time is reasonable
- ✅ No joint limit violations
- ✅ Graceful fallback if IPP service unavailable

---

## References

- **MoveIt2 Time Parameterization**: https://moveit.picknik.ai/main/doc/examples/time_parameterization/time_parameterization_tutorial.html
- **IPP Algorithm**: Iterative Parabolic Planning for time-optimal trajectory generation
- **ros2_control Documentation**: https://control.ros.org/

---

Generated: 2025-12-17
Status: ✅ TOTG Integration Complete
Build: Successful
Ready for Testing: Yes