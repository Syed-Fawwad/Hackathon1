#!/usr/bin/env python3

"""
Behavior Manager for State Machine Control
This module implements a sophisticated behavior manager with state machine control for the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import threading
import json
from typing import Dict, Any, Optional, Callable, List, Tuple
from dataclasses import dataclass, field
from enum import Enum
import uuid
from collections import OrderedDict


class State(Enum):
    """Enumeration for behavior states"""
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class BehaviorStatus(Enum):
    """Enumeration for behavior status"""
    INITIALIZED = "initialized"
    STARTED = "started"
    EXECUTING = "executing"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass
class Behavior:
    """Data class representing a robot behavior"""
    name: str
    behavior_func: Callable
    parameters: Dict[str, Any] = field(default_factory=dict)
    priority: int = 0
    timeout: float = 30.0
    state: State = State.IDLE
    status: BehaviorStatus = BehaviorStatus.INITIALIZED
    start_time: Optional[float] = None
    end_time: Optional[float] = None
    result: Optional[Dict[str, Any]] = None
    error_message: Optional[str] = None


class StateMachine:
    """
    State machine for managing behavior execution states
    """
    def __init__(self, name: str, initial_state: State = State.IDLE):
        self.name = name
        self.current_state = initial_state
        self.previous_state = None
        self.state_callbacks: Dict[State, List[Callable]] = {}
        self.state_lock = threading.Lock()

    def transition_to(self, new_state: State) -> bool:
        """Transition to a new state"""
        with self.state_lock:
            self.previous_state = self.current_state
            self.current_state = new_state

            # Execute callbacks for the new state if any
            if new_state in self.state_callbacks:
                for callback in self.state_callbacks[new_state]:
                    try:
                        callback(self.previous_state, new_state)
                    except Exception as e:
                        print(f"Error in state transition callback: {e}")

            return True

    def add_state_callback(self, state: State, callback: Callable):
        """Add a callback function to execute when entering a state"""
        if state not in self.state_callbacks:
            self.state_callbacks[state] = []
        self.state_callbacks[state].append(callback)

    def is_in_state(self, state: State) -> bool:
        """Check if the state machine is in a specific state"""
        return self.current_state == state


class BehaviorManager(Node):
    """
    Behavior Manager for State Machine Control
    Manages robot behaviors with sophisticated state machine control and execution.
    """

    def __init__(self):
        super().__init__('behavior_manager')

        # Initialize behavior storage
        self.behaviors: Dict[str, Behavior] = {}
        self.active_behaviors: OrderedDict[str, str] = OrderedDict()  # behavior_name -> behavior_id
        self.state_machines: Dict[str, StateMachine] = {}
        self.behavior_lock = threading.Lock()

        # Initialize priority queue for behavior execution
        self.execution_queue: List[str] = []
        self.queue_lock = threading.Lock()

        # Publishers for behavior status
        self.behavior_status_pub = self.create_publisher(String, '/behavior_status', 10)
        self.state_machine_status_pub = self.create_publisher(String, '/state_machine_status', 10)

        # Timer for periodic status updates
        self.status_timer = self.create_timer(0.5, self._publish_status)

        # Parameter declarations
        self.declare_parameter('max_active_behaviors', 3)
        self.declare_parameter('enable_behavior_logging', True)
        self.declare_parameter('default_behavior_timeout', 30.0)

        self.max_active_behaviors = self.get_parameter('max_active_behaviors').value
        self.enable_logging = self.get_parameter('enable_behavior_logging').value
        self.default_timeout = self.get_parameter('default_behavior_timeout').value

        # Thread for behavior execution
        self.execution_thread = threading.Thread(target=self._behavior_execution_loop, daemon=True)
        self.execution_thread.start()

        self.get_logger().info('Behavior Manager initialized with state machine control')

    def _publish_status(self):
        """Publish behavior status periodically"""
        if not self.behaviors:
            return

        status_msg = String()
        status_data = {
            'timestamp': time.time(),
            'active_behaviors_count': len(self.active_behaviors),
            'total_behaviors': len(self.behaviors),
            'behaviors': {}
        }

        for name, behavior in self.behaviors.items():
            status_data['behaviors'][name] = {
                'state': behavior.state.value,
                'status': behavior.status.value,
                'priority': behavior.priority
            }

        status_msg.data = json.dumps(status_data)
        self.behavior_status_pub.publish(status_msg)

        # Also publish state machine status
        sm_status_msg = String()
        sm_status_data = {
            'timestamp': time.time(),
            'state_machines': {}
        }
        for name, sm in self.state_machines.items():
            sm_status_data['state_machines'][name] = {
                'current_state': sm.current_state.value,
                'previous_state': sm.previous_state.value if sm.previous_state else None
            }

        sm_status_msg.data = json.dumps(sm_status_data)
        self.state_machine_status_pub.publish(sm_status_msg)

    def register_behavior(self, name: str, behavior_func: Callable, parameters: Dict[str, Any] = None, priority: int = 0, timeout: float = None):
        """Register a new behavior with the manager"""
        if parameters is None:
            parameters = {}

        if timeout is None:
            timeout = self.default_timeout

        behavior = Behavior(
            name=name,
            behavior_func=behavior_func,
            parameters=parameters,
            priority=priority,
            timeout=timeout
        )

        with self.behavior_lock:
            self.behaviors[name] = behavior

        # Create a state machine for this behavior
        state_machine = StateMachine(name, State.IDLE)
        self.state_machines[name] = state_machine

        if self.enable_logging:
            self.get_logger().info(f'Registered behavior: {name} with priority {priority}')

    def execute_behavior(self, name: str, **kwargs) -> Optional[Dict[str, Any]]:
        """Execute a registered behavior"""
        if name not in self.behaviors:
            self.get_logger().error(f'Behavior {name} not registered')
            return None

        behavior = self.behaviors[name]

        with self.behavior_lock:
            # Check if we're at max active behaviors
            if len(self.active_behaviors) >= self.max_active_behaviors:
                self.get_logger().warn(f'Max active behaviors reached ({self.max_active_behaviors}), cannot start {name}')
                return {'success': False, 'message': f'Max active behaviors reached, cannot start {name}'}

            # Check if behavior is already running
            if behavior.state == State.RUNNING:
                self.get_logger().warn(f'Behavior {name} is already running')
                return {'success': False, 'message': f'Behavior {name} is already running'}

            # Update behavior state
            behavior.state = State.RUNNING
            behavior.status = BehaviorStatus.STARTED
            behavior.start_time = time.time()

            # Add to active behaviors
            behavior_id = str(uuid.uuid4())
            self.active_behaviors[behavior_id] = name

            # Transition state machine
            sm = self.state_machines[name]
            sm.transition_to(State.RUNNING)

        # Execute the behavior function
        try:
            result = behavior.behavior_func(**kwargs)
            with self.behavior_lock:
                behavior.state = State.COMPLETED
                behavior.status = BehaviorStatus.SUCCEEDED
                behavior.end_time = time.time()
                behavior.result = result

                # Remove from active behaviors
                for bid, bname in list(self.active_behaviors.items()):
                    if bname == name:
                        del self.active_behaviors[bid]
                        break

                # Update state machine
                self.state_machines[name].transition_to(State.COMPLETED)

            if self.enable_logging:
                self.get_logger().info(f'Behavior {name} completed successfully')

            return result
        except Exception as e:
            with self.behavior_lock:
                behavior.state = State.FAILED
                behavior.status = BehaviorStatus.FAILED
                behavior.end_time = time.time()
                behavior.error_message = str(e)
                behavior.result = {'success': False, 'error': str(e)}

                # Remove from active behaviors
                for bid, bname in list(self.active_behaviors.items()):
                    if bname == name:
                        del self.active_behaviors[bid]
                        break

                # Update state machine
                self.state_machines[name].transition_to(State.FAILED)

            self.get_logger().error(f'Behavior {name} failed: {e}')
            return {'success': False, 'error': str(e)}

    def cancel_behavior(self, name: str) -> bool:
        """Cancel a running behavior"""
        with self.behavior_lock:
            # Find the behavior in active behaviors
            behavior_id = None
            for bid, bname in self.active_behaviors.items():
                if bname == name:
                    behavior_id = bid
                    break

            if behavior_id is None:
                self.get_logger().warn(f'Behavior {name} is not currently active')
                return False

            # Get the behavior
            behavior = self.behaviors[name]

            # Update behavior state
            behavior.state = State.CANCELLED
            behavior.status = BehaviorStatus.CANCELLED
            behavior.end_time = time.time()

            # Remove from active behaviors
            del self.active_behaviors[behavior_id]

            # Update state machine
            self.state_machines[name].transition_to(State.CANCELLED)

            self.get_logger().info(f'Behavior {name} was cancelled')
            return True

    def pause_behavior(self, name: str) -> bool:
        """Pause a running behavior"""
        with self.behavior_lock:
            if name not in self.behaviors or self.behaviors[name].state != State.RUNNING:
                return False

            self.behaviors[name].state = State.PAUSED
            self.state_machines[name].transition_to(State.PAUSED)

            self.get_logger().info(f'Behavior {name} was paused')
            return True

    def resume_behavior(self, name: str) -> bool:
        """Resume a paused behavior"""
        with self.behavior_lock:
            if name not in self.behaviors or self.behaviors[name].state != State.PAUSED:
                return False

            self.behaviors[name].state = State.RUNNING
            self.state_machines[name].transition_to(State.RUNNING)

            self.get_logger().info(f'Behavior {name} was resumed')
            return True

    def get_behavior_status(self, name: str) -> Optional[Dict[str, Any]]:
        """Get the status of a specific behavior"""
        if name not in self.behaviors:
            return None

        behavior = self.behaviors[name]
        return {
            'name': behavior.name,
            'state': behavior.state.value,
            'status': behavior.status.value,
            'priority': behavior.priority,
            'start_time': behavior.start_time,
            'end_time': behavior.end_time,
            'result': behavior.result,
            'error_message': behavior.error_message
        }

    def get_active_behaviors(self) -> List[str]:
        """Get list of currently active behaviors"""
        with self.behavior_lock:
            return [self.active_behaviors[bid] for bid in self.active_behaviors.keys()]

    def get_all_behaviors(self) -> List[str]:
        """Get list of all registered behaviors"""
        return list(self.behaviors.keys())

    def _behavior_execution_loop(self):
        """Background thread for managing behavior execution"""
        while rclpy.ok():
            # Process execution queue
            self._process_execution_queue()

            time.sleep(0.1)  # Sleep to prevent busy waiting

    def _process_execution_queue(self):
        """Process the behavior execution queue"""
        with self.queue_lock:
            if not self.execution_queue:
                return

            # For now, just execute the first behavior in the queue
            # In a more sophisticated implementation, this would handle priorities and concurrency
            behavior_name = self.execution_queue.pop(0)

            if behavior_name in self.behaviors:
                # Execute the behavior in a separate thread
                thread = threading.Thread(target=self.execute_behavior, args=(behavior_name,))
                thread.daemon = True
                thread.start()

    def queue_behavior(self, name: str):
        """Queue a behavior for execution"""
        if name not in self.behaviors:
            self.get_logger().error(f'Cannot queue unknown behavior: {name}')
            return

        with self.queue_lock:
            self.execution_queue.append(name)
            self.get_logger().info(f'Queued behavior: {name}')

    def add_state_transition_callback(self, behavior_name: str, state: State, callback: Callable):
        """Add a callback for state transitions of a specific behavior"""
        if behavior_name in self.state_machines:
            self.state_machines[behavior_name].add_state_callback(state, callback)
        else:
            self.get_logger().warn(f'No state machine for behavior: {behavior_name}')


def main(args=None):
    rclpy.init(args=args)
    behavior_manager = BehaviorManager()

    # Example: Register some sample behaviors
    def sample_behavior(**kwargs):
        """Sample behavior function for testing"""
        behavior_manager.get_logger().info(f'Executing sample behavior with args: {kwargs}')
        time.sleep(2)  # Simulate work
        return {'success': True, 'message': 'Sample behavior completed', 'data': kwargs}

    def navigation_behavior(**kwargs):
        """Navigation behavior function for testing"""
        behavior_manager.get_logger().info(f'Executing navigation behavior to: {kwargs.get("target_pose", "unknown")}')
        time.sleep(3)  # Simulate work
        return {'success': True, 'message': 'Navigation completed', 'path_length': 10.5}

    def manipulation_behavior(**kwargs):
        """Manipulation behavior function for testing"""
        behavior_manager.get_logger().info(f'Executing manipulation behavior: {kwargs.get("action", "unknown")}')
        time.sleep(2.5)  # Simulate work
        return {'success': True, 'message': 'Manipulation completed', 'object': kwargs.get('object', 'unknown')}

    # Register behaviors
    behavior_manager.register_behavior('sample_behavior', sample_behavior, priority=1)
    behavior_manager.register_behavior('navigate', navigation_behavior, priority=2)
    behavior_manager.register_behavior('manipulate', manipulation_behavior, priority=3)

    # Add a state transition callback for demonstration
    def on_navigation_complete(from_state, to_state):
        behavior_manager.get_logger().info(f'Navigation state transition: {from_state.value} -> {to_state.value}')

    behavior_manager.add_state_transition_callback('navigate', State.COMPLETED, on_navigation_complete)

    try:
        rclpy.spin(behavior_manager)
    except KeyboardInterrupt:
        pass
    finally:
        behavior_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()