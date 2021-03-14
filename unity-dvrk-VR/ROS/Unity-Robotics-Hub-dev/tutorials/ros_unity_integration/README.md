# ROS–Unity Integration

## ROS–Unity Communication
![](images/unity_ros.png)

A TCP endpoint running as a ROS node, which facilitates message passing to and from Unity and ROS.

The messages being passed between Unity and ROS are expected to be serialized as ROS would internally serialize them. To achieve this, the `MessageGeneration` plugin (from the [ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) repo) can generate C# classes, including serialization and deserialization functions, from ROS `.msg` files.

The `ROSConnection` plugin (also from [ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)) provides the Unity scripts necessary to publish, subscribe, or call a service.


## Tutorials
- [ROS–Unity Integration: Publisher](publisher.md) - Adding a Publisher to a Unity Scene
- [ROS–Unity Integration: Subscriber](subscriber.md) - Adding a Subscriber to a Unity Scene
- [ROS–Unity Integration: Service](service.md) - Adding a Service call to a Unity Scene
- [ROS–Unity Integration: Server Endpoint](server_endpoint.md) - How to write a Server Endpoint

## Example Unity Scripts

Example scripts implemented in tutorials:

- `unity_scripts/RosPublisherExample.cs`
	- Publishes the position of a GameObject every 0.5 seconds.

- `unity_scripts/RosServiceExample.cs`
	- Returns a destination position for a GameObject to move towards each time the service is called.

- `unity_scripts/RosSubscriberExample.cs`
	- Subscribes to a topic that accepts color messages and uses them to change the color of a GameObject in the Unity scene.