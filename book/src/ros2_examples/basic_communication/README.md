# Basic ROS 2 Communication Patterns

This directory contains basic examples of ROS 2 communication patterns:

## Publisher/Subscriber (Topics)
- **File**: `publisher_subscriber_example.py`
- **Pattern**: Asynchronous, one-way communication
- **Use Case**: Continuous data streams like sensor readings, robot state
- **Example Topic**: `/topic` with `std_msgs/String` messages

## Services (Request/Response)
- **File**: `service_example.py`
- **Pattern**: Synchronous, request/response communication
- **Use Case**: Operations that require confirmation or results
- **Example Service**: `/add_two_ints` with `example_interfaces/AddTwoInts`

## Actions (Goal-Based)
- **File**: `action_example.py`
- **Pattern**: Goal-based communication with feedback
- **Use Case**: Long-running operations with progress tracking
- **Example Action**: `/fibonacci` with `example_interfaces/Fibonacci`

## How to Run

### Publisher/Subscriber
```bash
# Terminal 1
python3 publisher_subscriber_example.py

# In another terminal, the same command will create both publisher and subscriber
```

### Service
```bash
# Terminal 1 - Start the service server
python3 service_example.py

# The client call is made automatically in the same script
```

### Action
```bash
# Terminal 1 - Start the action server
python3 action_example.py

# In another terminal, use the action client (not included in this basic example)
```

These examples demonstrate the three fundamental communication patterns in ROS 2 that form the basis of robot communication architecture.