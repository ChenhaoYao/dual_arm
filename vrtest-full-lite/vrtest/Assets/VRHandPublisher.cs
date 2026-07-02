using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using UnityEngine.XR;
using System.Collections.Generic;

public class VRHandPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    public string leftHandTopic = "/vr/left_hand/pose";
    public string rightHandTopic = "/vr/right_hand/pose";
    public string statusTopic = "/vr/status";
    public float publishFrequency = 50f;

    ROSConnection ros;
    float publishTimer;
    bool leftHandConnected;
    bool rightHandConnected;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(leftHandTopic);
        ros.RegisterPublisher<PoseStampedMsg>(rightHandTopic);
        ros.RegisterPublisher<StringMsg>(statusTopic);

        Debug.Log("[VRHandPublisher] 已启动，正在发布到 ROS");
    }

    void Update()
    {
        CheckVRDevices();

        publishTimer += Time.deltaTime;
        if (publishTimer >= 1f / publishFrequency)
        {
            publishTimer = 0f;
            PublishHandPose(XRNode.LeftHand, leftHandTopic);
            PublishHandPose(XRNode.RightHand, rightHandTopic);
            PublishStatus();
        }
    }

    void CheckVRDevices()
    {
        InputDevice leftHand = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
        InputDevice rightHand = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);

        bool leftNow = leftHand.isValid;
        bool rightNow = rightHand.isValid;

        if (leftNow != leftHandConnected)
        {
            leftHandConnected = leftNow;
            Debug.Log(leftNow ? "[VRHandPublisher] 左手已连接" : "[VRHandPublisher] 左手已断开");
        }

        if (rightNow != rightHandConnected)
        {
            rightHandConnected = rightNow;
            Debug.Log(rightNow ? "[VRHandPublisher] 右手已连接" : "[VRHandPublisher] 右手已断开");
        }
    }

    void PublishStatus()
    {
        StringMsg msg = new StringMsg();
        msg.data = "left=" + (leftHandConnected ? "connected" : "disconnected")
                 + " right=" + (rightHandConnected ? "connected" : "disconnected");
        ros.Publish(statusTopic, msg);
    }

    void PublishHandPose(XRNode node, string topic)
    {
        InputDevice device = InputDevices.GetDeviceAtXRNode(node);
        if (!device.isValid) return;

        if (device.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 position) &&
            device.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rotation))
        {
            PoseStampedMsg msg = new PoseStampedMsg();
            msg.header.frame_id = node == XRNode.LeftHand ? "left_hand" : "right_hand";
            msg.header.stamp = new TimeMsg
            {
                sec = (int)Time.realtimeSinceStartup,
                nanosec = (uint)((Time.realtimeSinceStartup % 1) * 1e9)
            };

            msg.pose.position = position.To<FLU>();
            msg.pose.orientation = rotation.To<FLU>();

            ros.Publish(topic, msg);
        }
    }
}
