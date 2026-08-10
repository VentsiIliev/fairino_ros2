#!/usr/bin/env python3
"""Narrow planner-facing context that replaces direct god-node access."""


class PlannerContext:
    def __init__(
        self,
        node,
        state_store,
        motion_coordinator,
        motion_queue,
        safety_manager,
        cart_path_client,
        sequence_client,
        ipp_client,
        trajectory_executor,
        planner_support,
        trajectory_optimizer,
    ):
        self._node = node
        self._state_store = state_store
        self._motion = motion_coordinator
        self.motion_queue = motion_queue
        self.safety_manager = safety_manager
        self.cart_path_client = cart_path_client
        self.sequence_client = sequence_client
        self.ipp_client = ipp_client
        self.trajectory_executor = trajectory_executor
        self.trajectory_optimizer = trajectory_optimizer
        self._planner_support = planner_support
        self.T_tool = None
        self._last_requested_delta_mm = 0.0
        self._last_full_waypoints = None
        self._pending_path_trajectory = None
        self._pending_path_vel_scaling = None
        self._pending_path_acc_scaling = None
        self._pending_path_trajectory_optimizer = None

    def get_logger(self):
        return self._node.get_logger()

    def get_clock(self):
        return self._node.get_clock()

    def create_client(self, *args, **kwargs):
        return self._node.create_client(*args, **kwargs)

    def create_timer(self, *args, **kwargs):
        return self._node.create_timer(*args, **kwargs)

    def destroy_timer(self, *args, **kwargs):
        return self._node.destroy_timer(*args, **kwargs)

    def request_cartesian_path(self, request):
        return self.cart_path_client.call_async(request)

    def wait_for_cartesian_path_service(self, timeout_sec=1.0):
        return self.cart_path_client.wait_for_service(timeout_sec=timeout_sec)

    def request_motion_sequence(self, request):
        return self.sequence_client.call_async(request)

    def wait_for_motion_sequence_service(self, timeout_sec=1.0):
        return self.sequence_client.wait_for_service(timeout_sec=timeout_sec)

    def force_safety_update(self):
        return self.safety_manager.force_update(force=False)

    def check_position_safety(self, *args, **kwargs):
        return self.safety_manager.check_position_safety(*args, **kwargs)

    def submit_motion_task(self, task_function, task_args=None):
        self.motion_queue.submit(task_function=task_function, task_args=task_args)

    def mark_current_motion_complete(self, result):
        self.motion_queue.mark_current_complete(result)

    def clear_motion_queue(self):
        self.motion_queue.clear()

    @property
    def lock(self):
        return self._motion.lock

    @property
    def execution_lock(self):
        return self._motion.execution_lock

    @property
    def is_executing(self):
        return self._motion.is_executing

    @is_executing.setter
    def is_executing(self, value):
        self._motion.is_executing = value

    @property
    def plan_generation(self):
        return self._motion.plan_generation

    @plan_generation.setter
    def plan_generation(self, value):
        self._motion.plan_generation = value

    @property
    def last_move_result(self):
        return self._motion.last_move_result

    @last_move_result.setter
    def last_move_result(self, value):
        self._motion.last_move_result = value

    @property
    def active_execute_send_future(self):
        return self._motion.active_execute_send_future

    @active_execute_send_future.setter
    def active_execute_send_future(self, value):
        self._motion.active_execute_send_future = value

    @property
    def active_controller_goal(self):
        return self._motion.active_controller_goal

    @active_controller_goal.setter
    def active_controller_goal(self, value):
        self._motion.active_controller_goal = value

    @property
    def prev_cartesian(self):
        value = self._state_store.get_prev_cartesian()
        if value is not None:
            return value
        return getattr(self._node, "prev_cartesian", None)

    @prev_cartesian.setter
    def prev_cartesian(self, value):
        self._state_store.set_prev_cartesian(value)

    @property
    def current_joint_state(self):
        state = self._state_store.get_current_joint_state()
        if state is not None:
            return state
        return getattr(self._node, "current_joint_state", None)

    @current_joint_state.setter
    def current_joint_state(self, value):
        self._state_store.set_current_joint_state(value)

    def get_latest_data(self):
        return self._state_store.get_latest_data()

    def set_last_requested_delta_mm(self, value):
        self._last_requested_delta_mm = value

    def get_last_requested_delta_mm(self):
        return self._last_requested_delta_mm

    def set_last_full_waypoints(self, value):
        self._last_full_waypoints = value

    def get_last_full_waypoints(self):
        return self._last_full_waypoints

    def stage_pending_path(self, trajectory, vel_scaling, acc_scaling, trajectory_optimizer_name=None):
        self._pending_path_trajectory = trajectory
        self._pending_path_vel_scaling = vel_scaling
        self._pending_path_acc_scaling = acc_scaling
        self._pending_path_trajectory_optimizer = trajectory_optimizer_name

    def consume_pending_path(self):
        result = (
            self._pending_path_trajectory,
            self._pending_path_vel_scaling,
            self._pending_path_acc_scaling,
            self._pending_path_trajectory_optimizer,
        )
        self.clear_pending_path()
        return result

    def clear_pending_path(self):
        self._pending_path_trajectory = None
        self._pending_path_vel_scaling = None
        self._pending_path_acc_scaling = None
        self._pending_path_trajectory_optimizer = None

    def get_fk_client(self):
        return self._planner_support.get_fk_client()

    def get_ik_client(self):
        return self._planner_support.get_ik_client()

    def get_contour_ik_client(self):
        return self._planner_support.get_contour_ik_client()

    def get_linked_lin_client(self):
        return self._planner_support.get_linked_lin_client()

    def get_state_validity_client(self):
        return self._planner_support.get_state_validity_client()

    def get_ptp_client(self):
        return self._planner_support.get_ptp_client()

    def is_motion_stack_ready(self) -> bool:
        return self._node.is_motion_stack_ready()

    def get_motion_stack_fault_reason(self) -> str:
        return self._node.get_motion_stack_fault_reason()
