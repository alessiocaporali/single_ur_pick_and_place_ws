import agx
import agxCable
import numpy as np
from scipy.interpolate import CubicSpline

import agxSDK
import agxROS2    

import struct


class Cable():
    def __init__(self, radius, length, density):
        self.radius = radius
        self.length = length
        self.density = density

        self.num_nodes = int(self.length / self.density) + 1
        print("Number of nodes:", self.num_nodes)


    def initialize_straight_cable(self, direction='x'):

        values = [i * (self.length / (self.num_nodes - 1)) for i in range(self.num_nodes)]
        if direction == 'x':
            path = [agx.Vec3(x, 0, 0) for x in values]
        elif direction == 'y':
            path = [agx.Vec3(0, y, 0) for y in values]
        elif direction == 'z':
            path = [agx.Vec3(0, 0, z) for z in values]
        else:
            raise ValueError("Invalid direction. Choose from 'x', 'y', or 'z'.")
        
        route = agxCable.SegmentingRoute(0.01)
        cable = agxCable.Cable(self.radius, route)

        for pt in path:
            cable.add(agxCable.FreeNode(pt[0], pt[1], pt[2]))

        route.initialize()
        return cable

    def initialize_from_list_cable(self, points_list):
        route = agxCable.SegmentingRoute(0.01)
        cable = agxCable.Cable(self.radius, route)

        for pt in points_list:
            cable.add(agxCable.FreeNode(pt[0], pt[1], pt[2]))

        route.initialize()
        return cable
    

    def set_cable_properties(self, cable, bend_youngs_modulus, twist_youngs_modulus):
        properties = cable.getCableProperties()
        properties.setYoungsModulus(bend_youngs_modulus, agxCable.BEND)
        properties.setYoungsModulus(twist_youngs_modulus, agxCable.TWIST)


    def get_state(self, cable):
        positions = []
        rotations = []
        for segment in cable.segments:
            pos = segment.getRigidBody().getPosition()
            rot = segment.getRigidBody().getRotation()
            positions.append((pos[0], pos[1], pos[2]))
            rotations.append((rot[0], rot[1], rot[2], rot[3]))
            
        positions = np.array(positions)
        rotations = np.array(rotations)

        full_state = np.hstack([positions, rotations])
        return full_state


        # interpolate as spline
        diffs = np.diff(positions, axis=0)
        distances = np.sqrt((diffs ** 2).sum(axis=1))
        cumulative = np.insert(np.cumsum(distances), 0, 0.0)

        # Create splines for x, y, z
        cs_x = CubicSpline(cumulative, positions[:, 0])
        cs_y = CubicSpline(cumulative, positions[:, 1])
        cs_z = CubicSpline(cumulative, positions[:, 2])

        # Sample the spline at evenly spaced points along the cable
        total_length = cumulative[-1]
        s_sample = np.linspace(0, total_length, self.num_nodes)
        return np.vstack([cs_x(s_sample), cs_y(s_sample), cs_z(s_sample)]).T







class CableRos2Publisher(agxSDK.StepEventListener):
    """
    Publishes cable state both as a Float32MultiArray and a PointCloud2 over ROS2.

    Attributes:
        ROS_TOPIC: topic name for Float32MultiArray
        ROS_TOPIC_CLOUD: topic name for PointCloud2
    """

    ROS_TOPIC = "cable_sim_state"
    ROS_TOPIC_CLOUD = "cable_sim_state_cloud"

    def __init__(self, cable, cable_interface, cloud_frame_id: str = "world"):
        """
        Args:
            cable: The AGX cable object.
            cable_interface: Cable interface to retrieve the cable state.
            cloud_frame_id: Reference frame for PointCloud2 messages.
        """
        super().__init__()

        # Float32MultiArray publisher
        self.pub_state = agxROS2.PublisherStdMsgsFloat32MultiArray(self.ROS_TOPIC)
        self.msg_state = agxROS2.StdMsgsFloat32MultiArray()

        # PointCloud2 publisher
        self.pub_state_cloud = agxROS2.PublisherSensorMsgsPointCloud2(self.ROS_TOPIC_CLOUD)
        self.cloud_frame_id = cloud_frame_id

        self.cable = cable
        self.cable_interface = cable_interface

    def post(self, time: float):
        """
        Called at each simulation step after dynamics are integrated.
        Publishes both PointCloud2 and Float32MultiArray.
        """
        state = self.cable_interface.get_state(self.cable)

        self.publish_pointcloud(state)
        self.publish_multiarray(state)

    def publish_pointcloud(self, state: np.ndarray):
        """
        Publishes cable state as a PointCloud2 message.
        """
        N = state.shape[0]
        cloud = agxROS2.SensorMsgsPointCloud2()

        # Header
        cloud.header.frame_id = self.cloud_frame_id
        sim_time = self.cable.getSimulation().getTimeStamp()
        cloud.header.stamp.sec = int(sim_time)
        cloud.header.stamp.nanosec = int((sim_time % 1) * 1e9)

        # Geometry
        cloud.height = 1
        cloud.width = N
        cloud.is_dense = False
        cloud.point_step = 12  # 3 floats x 4 bytes
        cloud.row_step = cloud.point_step * cloud.width

        # Fields
        fields = agxROS2.SensorMsgsPointFieldVector()
        for name, offset in [("x", 0), ("y", 4), ("z", 8)]:
            f = agxROS2.SensorMsgsPointField()
            f.name = name
            f.offset = offset
            f.datatype = 7  # FLOAT32
            f.count = 1
            fields.append(f)
        cloud.fields = fields

        # Pack points as bytes
        state_pos = state[:, :3]
        cloud.data = bytearray(struct.pack('<' + 'fff'*N, *state_pos.flatten()))

        # Publish
        self.pub_state_cloud.sendMessage(cloud)

    def publish_multiarray(self, state: np.ndarray):
        """
        Publishes cable state as a Float32MultiArray message.
        """
        
        self.msg_state.data = state.flatten().tolist()
        self.pub_state.sendMessage(self.msg_state)