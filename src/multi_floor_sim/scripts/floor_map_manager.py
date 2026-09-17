#!/usr/bin/env python3

import math
import os
import time
from enum import Enum
from pathlib import Path

import yaml

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray
from nav2_msgs.srv import LoadMap
from std_msgs.msg import Int32, String


class SwitchState(Enum):
    IDLE = 'IDLE'
    SWITCHING_NAV_MAP = 'SWITCHING_NAV_MAP'
    SWITCHING_NDT_MAP = 'SWITCHING_NDT_MAP'
    PUBLISH_INITIAL_POSE = 'PUBLISH_INITIAL_POSE'
    WAITING_NDT_READY = 'WAITING_NDT_READY'
    READY = 'READY'


class FloorMapManager(Node):

    def __init__(self):
        super().__init__('floor_map_manager')

        package_share = Path(
            get_package_share_directory('multi_floor_sim')
        )

        self.config_file = (
            package_share / 'config' / 'floor_maps.yaml'
        )

        self.floors = {}

        self.current_floor_id = None
        self.pending_floor_id = None
        self.pending_cfg = None
        self.state = SwitchState.IDLE
        self.state_deadline = None

        # 新楼层定位确认状态
        self.waiting_for_ndt = False
        self.ndt_accept_streak = 0
        self.required_ndt_accepts = max(
            1,
            self.declare_parameter(
                'required_ndt_accepts', 2
            ).value
        )
        self.operation_timeout_sec = max(
            1.0,
            self.declare_parameter(
                'operation_timeout_sec', 10.0
            ).value
        )
        self.ndt_ready_timeout_sec = max(
            1.0,
            self.declare_parameter(
                'ndt_ready_timeout_sec', 30.0
            ).value
        )

        self.load_config()

        # --------------------------------------------------
        # 输入：当前楼层
        # --------------------------------------------------
        self.floor_sub = self.create_subscription(
            Int32,
            '/current_floor',
            self.floor_callback,
            10
        )

        # --------------------------------------------------
        # 状态输出
        # --------------------------------------------------
        self.floor_name_pub = self.create_publisher(
            String,
            '/floor_manager/floor_name',
            10
        )

        self.nav_map_pub = self.create_publisher(
            String,
            '/floor_manager/nav_map_path',
            10
        )

        self.ndt_map_pub = self.create_publisher(
            String,
            '/floor_manager/ndt_map_path',
            10
        )

        # --------------------------------------------------
        # Floor切换后自动给NDT新的初始位姿
        # --------------------------------------------------
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # NDT是否已经在新楼层重新稳定定位
        self.ndt_status_sub = self.create_subscription(
            DiagnosticArray,
            '/localization/status',
            self.ndt_status_callback,
            1
        )

        # --------------------------------------------------
        # Nav2二维地图切换
        # --------------------------------------------------
        self.load_map_client = self.create_client(
            LoadMap,
            '/map_server/load_map'
        )

        # --------------------------------------------------
        # NDT三维地图切换
        # --------------------------------------------------
        self.ndt_param_client = self.create_client(
            SetParameters,
            '/ndt_localizer/set_parameters'
        )

        # 防止服务响应或 NDT 恢复状态永久缺失时卡在切换中。
        self.timeout_timer = self.create_timer(
            0.5,
            self.check_state_timeout
        )

        self.get_logger().info(
            'Floor Map Manager started.'
        )

        self.get_logger().info(
            f'Loaded config: {self.config_file}'
        )

        for floor_id, cfg in self.floors.items():
            self.get_logger().info(
                f'Floor {floor_id}: '
                f'{cfg["name"]}, '
                f'nav={cfg["nav_map"]}, '
                f'ndt={cfg["ndt_map"]}'
            )

        self.get_logger().info(
            'Waiting for /current_floor '
            '(std_msgs/msg/Int32, reliable/volatile) ...'
        )


    def load_config(self):

        with open(self.config_file, 'r') as f:
            data = yaml.safe_load(f)

        for floor_name, cfg in data['floors'].items():

            floor_id = int(cfg['id'])

            self.floors[floor_id] = {
                'name': floor_name,
                'nav_map': cfg['nav_map']['yaml'],
                'ndt_map': cfg['ndt_map']['pcd'],
                'initial_pose': cfg.get(
                    'default_initial_pose',
                    {
                        'x': 0.0,
                        'y': 0.0,
                        'yaw': 0.0
                    }
                )
            }


    # ======================================================
    # 收到楼层变化
    # ======================================================
    def floor_callback(self, msg):

        requested_floor = int(msg.data)

        self.get_logger().info(
            f'Received floor request: {requested_floor}'
        )

        try:
            self.handle_floor_request(requested_floor)
        except Exception as exc:
            # 输入回调中的意外异常不能让长期运行的管理器退出。
            if self.pending_floor_id is not None:
                self.fail_switch(
                    f'Unhandled floor request exception: {exc}'
                )
            else:
                self.get_logger().error(
                    f'Floor request failed before switching: {exc}'
                )


    def handle_floor_request(self, requested_floor):

        if requested_floor not in self.floors:

            self.get_logger().error(
                f'Unknown floor id: {requested_floor}'
            )
            return

        if requested_floor == self.current_floor_id:

            self.get_logger().info(
                f'Floor {requested_floor} is already active.'
            )
            return

        if self.pending_floor_id is not None:

            self.get_logger().warning(
                f'Floor switch already in progress: '
                f'{self.pending_floor_id}'
            )
            return

        cfg = self.floors[requested_floor]

        if not os.path.isfile(cfg['nav_map']):

            self.get_logger().error(
                f'Nav map not found: {cfg["nav_map"]}'
            )
            return

        if not os.path.isfile(cfg['ndt_map']):

            self.get_logger().error(
                f'NDT map not found: {cfg["ndt_map"]}'
            )
            return

        self.pending_floor_id = requested_floor
        self.pending_cfg = cfg
        self.waiting_for_ndt = False
        self.ndt_accept_streak = 0

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'START FLOOR SWITCH -> '
            f'{cfg["name"]} '
            f'(id={requested_floor})'
        )

        # 第一步：切Nav2二维地图
        self.set_state(
            SwitchState.SWITCHING_NAV_MAP,
            self.operation_timeout_sec
        )
        self.switch_nav_map()


    # ======================================================
    # Step 1：Nav2 map.yaml
    # ======================================================
    def switch_nav_map(self):

        if not self.load_map_client.wait_for_service(
            timeout_sec=2.0
        ):
            self.fail_switch(
                '/map_server/load_map unavailable'
            )
            return

        request = LoadMap.Request()
        request.map_url = self.pending_cfg['nav_map']

        self.get_logger().info(
            f'[1/3] Switching Nav2 map: '
            f'{request.map_url}'
        )

        future = self.load_map_client.call_async(request)

        future.add_done_callback(
            self.nav_map_done
        )


    def nav_map_done(self, future):

        if self.state != SwitchState.SWITCHING_NAV_MAP:
            self.get_logger().warning(
                'Ignoring stale Nav2 map service response.'
            )
            return

        try:
            response = future.result()
        except Exception as exc:
            self.fail_switch(
                f'Nav2 LoadMap exception: {exc}'
            )
            return

        if response is None or response.result != 0:

            result = (
                response.result
                if response is not None
                else 'None'
            )

            self.fail_switch(
                f'Nav2 LoadMap failed: {result}'
            )
            return

        self.get_logger().info(
            '[1/3] Nav2 map switch SUCCESS.'
        )

        # 第二步
        self.set_state(
            SwitchState.SWITCHING_NDT_MAP,
            self.operation_timeout_sec
        )
        self.switch_ndt_map()


    # ======================================================
    # Step 2：NDT GlobalMap.pcd
    # ======================================================
    def switch_ndt_map(self):

        if not self.ndt_param_client.wait_for_service(
            timeout_sec=2.0
        ):
            self.fail_switch(
                '/ndt_localizer parameter service unavailable'
            )
            return

        ndt_path = self.pending_cfg['ndt_map']

        self.get_logger().info(
            f'[2/3] Switching NDT map: {ndt_path}'
        )

        parameter = Parameter(
            'map_path',
            Parameter.Type.STRING,
            ndt_path
        )

        request = SetParameters.Request()
        request.parameters = [
            parameter.to_parameter_msg()
        ]

        future = self.ndt_param_client.call_async(
            request
        )

        future.add_done_callback(
            self.ndt_map_done
        )


    def ndt_map_done(self, future):

        if self.state != SwitchState.SWITCHING_NDT_MAP:
            self.get_logger().warning(
                'Ignoring stale NDT map service response.'
            )
            return

        try:
            response = future.result()
        except Exception as exc:
            self.fail_switch(
                f'NDT parameter exception: {exc}'
            )
            return

        if response is None or not response.results:

            self.fail_switch(
                'NDT parameter returned no result'
            )
            return

        result = response.results[0]

        if not result.successful:

            self.fail_switch(
                f'NDT map switch failed: '
                f'{result.reason}'
            )
            return

        self.get_logger().info(
            '[2/3] NDT map switch SUCCESS.'
        )

        # 第三步
        self.set_state(
            SwitchState.PUBLISH_INITIAL_POSE,
            self.operation_timeout_sec
        )
        self.publish_initial_pose()


    # ======================================================
    # Step 3：新楼层初始位姿
    # ======================================================
    def publish_initial_pose(self):

        if self.pending_cfg is None:
            self.fail_switch(
                'No pending floor configuration for /initialpose'
            )
            return

        pose_cfg = self.pending_cfg['initial_pose']

        x = float(pose_cfg['x'])
        y = float(pose_cfg['y'])
        yaw = float(pose_cfg['yaw'])

        msg = PoseWithCovarianceStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.z = math.sin(
            yaw / 2.0
        )

        msg.pose.pose.orientation.w = math.cos(
            yaw / 2.0
        )

        # 从此刻开始，只接受新楼层重新产生的 NDT 状态
        self.waiting_for_ndt = True
        self.ndt_accept_streak = 0
        self.set_state(
            SwitchState.WAITING_NDT_READY,
            self.ndt_ready_timeout_sec
        )

        self.initial_pose_pub.publish(msg)

        self.get_logger().info(
            '[3/3] Published new /initialpose: '
            f'x={x:.3f}, y={y:.3f}, '
            f'yaw={yaw:.3f}'
        )

        self.get_logger().info(
            f'Waiting for {self.required_ndt_accepts} '
            'consecutive NDT ACCEPT messages ...'
        )


    # ======================================================
    # 等待新楼层 NDT 真正恢复
    # ======================================================
    def ndt_status_callback(self, msg):

        if (
            not self.waiting_for_ndt or
            self.state != SwitchState.WAITING_NDT_READY
        ):
            return

        if self.pending_floor_id is None:
            return

        gate_status = None

        for status in msg.status:
            if status.name == 'ndt_localizer/quality_gate':
                gate_status = status
                break

        if gate_status is None:
            return

        if gate_status.message == 'NDT ACCEPT':

            self.ndt_accept_streak += 1

            self.get_logger().info(
                f'NDT ACCEPT on new floor '
                f'({self.ndt_accept_streak}/'
                f'{self.required_ndt_accepts})'
            )

            if self.ndt_accept_streak >= self.required_ndt_accepts:

                self.waiting_for_ndt = False
                self.finish_switch()

        else:

            self.ndt_accept_streak = 0
            rejection_reason = next(
                (
                    item.value
                    for item in gate_status.values
                    if item.key == 'rejection_reason'
                ),
                'unspecified'
            )
            self.get_logger().warning(
                'NDT not ready on new floor; '
                f'accept streak reset (reason={rejection_reason}).'
            )


    # ======================================================
    # 新楼层真正 READY
    # ======================================================
    def finish_switch(self):

        if self.pending_floor_id is None or self.pending_cfg is None:
            self.fail_switch(
                'Cannot finish without a pending floor'
            )
            return

        floor_id = self.pending_floor_id
        cfg = self.pending_cfg

        self.current_floor_id = floor_id

        name_msg = String()
        name_msg.data = cfg['name']
        self.floor_name_pub.publish(name_msg)

        nav_msg = String()
        nav_msg.data = cfg['nav_map']
        self.nav_map_pub.publish(nav_msg)

        ndt_msg = String()
        ndt_msg.data = cfg['ndt_map']
        self.ndt_map_pub.publish(ndt_msg)

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'FLOOR READY: '
            f'{cfg["name"]} '
            f'(id={floor_id})'
        )

        self.get_logger().info(
            f'Nav2 map: {cfg["nav_map"]}'
        )

        self.get_logger().info(
            f'NDT map:  {cfg["ndt_map"]}'
        )

        self.get_logger().info(
            '========================================'
        )

        self.pending_floor_id = None
        self.pending_cfg = None
        self.waiting_for_ndt = False
        self.ndt_accept_streak = 0
        self.set_state(SwitchState.READY)


    def fail_switch(self, reason):

        self.get_logger().error(
            f'FLOOR SWITCH FAILED: {reason}'
        )

        self.waiting_for_ndt = False
        self.ndt_accept_streak = 0

        self.pending_floor_id = None
        self.pending_cfg = None
        fallback_state = (
            SwitchState.READY
            if self.current_floor_id is not None
            else SwitchState.IDLE
        )
        self.set_state(fallback_state)


    def set_state(self, new_state, timeout_sec=None):

        old_state = self.state
        self.state = new_state
        self.state_deadline = (
            time.monotonic() + timeout_sec
            if timeout_sec is not None
            else None
        )

        if old_state != new_state:
            self.get_logger().info(
                f'Floor switch state: '
                f'{old_state.value} -> {new_state.value}'
            )


    def check_state_timeout(self):

        if self.state_deadline is None:
            return

        if time.monotonic() < self.state_deadline:
            return

        timed_out_state = self.state.value
        self.state_deadline = None
        self.fail_switch(
            f'timeout while in state {timed_out_state}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = FloorMapManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
