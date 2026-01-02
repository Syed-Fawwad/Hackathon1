using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;

public class ROSConnector : MonoBehaviour
{
    [Header("ROS Connection Settings")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 9090;

    [Header("Topics")]
    public string jointStatesTopic = "/joint_states";
    public string robotStateTopic = "/unity_robot_state";
    public string cameraFeedTopic = "/unity_camera_feed";
    public string userCommandsTopic = "/user_commands";

    private ROSConnection ros;
    private Dictionary<string, double[]> jointPositions = new Dictionary<string, double[]>();
    private Dictionary<string, double[]> jointVelocities = new Dictionary<string, double[]>();
    private Dictionary<string, double[]> jointEfforts = new Dictionary<string, double[]>();

    // Robot joint transforms
    public Dictionary<string, Transform> jointTransforms = new Dictionary<string, Transform>();

    void Start()
    {
        // Get the ROS connection instance
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisteredAsROSGeometryHandler = true;
        ros.ConnectToROS(rosIPAddress, rosPort);

        // Subscribe to joint states topic
        ros.Subscribe<sensor_msgs_JointState>(jointStatesTopic, JointStateCallback);

        // Publish robot state
        InvokeRepeating("PublishRobotState", 0.1f, 0.1f); // 10 Hz
    }

    void JointStateCallback(sensor_msgs_JointState jointState)
    {
        // Update joint positions from ROS
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            double position = jointState.position[i];

            if (jointTransforms.ContainsKey(jointName))
            {
                // Update the joint transform based on the received position
                UpdateJointTransform(jointName, position);
            }
        }
    }

    void UpdateJointTransform(string jointName, double position)
    {
        Transform jointTransform = jointTransforms[jointName];
        if (jointTransform != null)
        {
            // For simplicity, assume all joints are rotation joints around Y-axis
            // In a real implementation, you would need to handle different joint types
            jointTransform.localRotation = Quaternion.Euler(0, (float)position * Mathf.Rad2Deg, 0);
        }
    }

    void PublishRobotState()
    {
        // Create and publish robot state message
        std_msgs_Float32MultiArray robotState = new std_msgs_Float32MultiArray();
        robotState.data = new float[3]; // x, y, z position of robot
        robotState.data[0] = transform.position.x;
        robotState.data[1] = transform.position.y;
        robotState.data[2] = transform.position.z;

        ros.Publish(robotStateTopic, robotState);
    }

    public void SendUserCommand(string command)
    {
        std_msgs_String userCommand = new std_msgs_String();
        userCommand.data = command;

        ros.Publish(userCommandsTopic, userCommand);
    }

    public void SetJointPositions(Dictionary<string, float> positions)
    {
        sensor_msgs_JointState jointState = new sensor_msgs_JointState();
        jointState.name = new List<string>();
        jointState.position = new List<double>();

        foreach (var kvp in positions)
        {
            jointState.name.Add(kvp.Key);
            jointState.position.Add(kvp.Value);
        }

        jointState.header = new std_msgs_Header();
        jointState.header.stamp = new builtin_interfaces_Time();
        jointState.header.stamp.sec = (int)Time.time;
        jointState.header.stamp.nanosec = (uint)((Time.time - Mathf.Floor(Time.time)) * 1000000000);

        ros.Publish(jointStatesTopic, jointState);
    }

    void OnApplicationQuit()
    {
        if (ros != null)
        {
            ros.Close();
        }
    }
}