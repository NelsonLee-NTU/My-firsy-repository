#! /usr/bin/python
import numpy as np
from matplotlib.cm import get_cmap

import rospy
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs


def point_cloud(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx7 array of xyz positions (m) and rgba colors (0..1)
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    """
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyzrgba')]

    header = std_msgs.Header(frame_id=parent_frame, stamp=rospy.Time.now())

    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 7),
        row_step=(itemsize * 7 * points.shape[0]),
        data=data
    )


if __name__ == '__main__':
    rospy.init_node('my_point_cloud')
    pub_points = rospy.Publisher('my_point_cloud', sensor_msgs.PointCloud2,
                                 queue_size=1)
    rate = rospy.timer.Rate(10)

    print('loading points')
    raw_points = np.transpose(np.load('pointcloud.npy'))
    num_points = raw_points.shape[0]

    colors = np.hstack((np.ones((num_points, 1)), np.zeros((num_points, 2)), np.ones((num_points, 1))))

    #colors = np.array(get_cmap('cubehelix')(
        #np.cos(2 * np.pi / period * np.arange(curve.shape[0])) / 2 + 0.5))

    points = np.hstack((raw_points, colors))

    print('Publishing points now')
    while not rospy.is_shutdown():
        pub_points.publish(point_cloud(points, '/map'))
        rate.sleep()