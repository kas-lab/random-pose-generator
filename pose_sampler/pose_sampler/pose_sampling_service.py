# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pose_sampler_srv.srv import SamplePoses
import argparse
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
from pose_sampler.pose_sampler import PoseSampler
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion, Pose, Point


class PoseSamplerService(Node):

    def __init__(self, args):
        super().__init__('pose_sampler')
        self.srv = self.create_service(SamplePoses, 'sample_poses', self.srv_callback)
        self.args = args

    def pose_from_dict(self, pose_dict):
        point_msg = Point(x=pose_dict["x"], y=pose_dict["y"])
        q = quaternion_from_euler(0.0, 0.0, pose_dict["yaw"])
        quat_msg = Quaternion(
            x=q[0],
            y=q[1],
            z=q[2],
            w=q[3],
        )
        return Pose(position=point_msg, orientation=quat_msg)

    def srv_callback(self, request, response):
        pkg_path = get_package_share_directory(self.args.map_ros_path.split(os.path.sep)[0])
        path = os.path.join(pkg_path, *self.args.map_ros_path.split(os.path.sep)[1:])
        sampler = PoseSampler(
            map_yaml_path=path,
            obstacle_clearance_m=request.clearance,
            min_goal_distance=self.args.min_distance,
            max_goal_distance=self.args.max_distance,
            seed=self.args.seed,
            initial_pose=[request.initial_pose.position.x,
                          request.initial_pose.position.y],
            sequence=self.args.sequence
        )

        sampler.load_map()

        pose_dicts = sampler.generate_and_save(request.number, "", False)

        # if self.args.visualize_only:
        #     vis_path = os.path.join(self.args.output, "sampling_area.png")

        #     sampler.visualize_sampling_area(vis_path, presampled_poses=pose_dicts)
        #     return response

        for pose_dict in pose_dicts:
            response.poses.append(self.pose_from_dict(pose_dict["goal"]))

        return response


def parse_args():
    p = argparse.ArgumentParser(description="RCT Data Collector for Nav2")
    p.add_argument("--map-absolute-path", type=str, help="Path to map YAML file")
    p.add_argument("--map-ros-path", type=str, help="Path within share directory of given package")
    p.add_argument("--trials", type=int, default=3000)
    p.add_argument("--output", type=str, default="./rct_data")
    p.add_argument("--timeout", type=float, default=180.0)
    p.add_argument("--cooldown", type=float, default=3.0)
    p.add_argument("--clearance", type=float, default=0.5)
    p.add_argument("--min-distance", type=float, default=3.0)
    p.add_argument("--max-distance", type=float, default=15.0)
    p.add_argument("--config", type=str, help="YAML config file")
    p.add_argument("--resume", action="store_true")
    p.add_argument("--visualize-only", action="store_true")
    p.add_argument("--n-samples", type=int, default=20,
                   help="Number of sample pairs to show in visualization (default: 20)")
    p.add_argument("--collect-risk", action="store_true")
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--initial-pose", nargs=2, type=float, default=[])
    p.add_argument("--sequence", action="store_true", help="Enable sequential sampling")
    p.add_argument("--ros-args", nargs=argparse.REMAINDER, help="ROS-specific arguments")

    return p.parse_args()


def main(args=None):
    if args is None:
        args = parse_args()

    rclpy.init(args=None)

    minimal_service = PoseSamplerService(args)

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
