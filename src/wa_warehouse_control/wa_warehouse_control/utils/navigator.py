from __future__ import annotations

from typing import ClassVar

from geometry_msgs import msg as geometry_msgs
from nav2_msgs.action import (
    AssistedTeleop,
    BackUp,
    ComputePathThroughPoses,
    ComputePathToPose,
    FollowPath,
    FollowWaypoints,
    NavigateThroughPoses,
    NavigateToPose,
    SmoothPath,
    Spin,
)
from nav2_msgs.srv import (
    ClearEntireCostmap,
    GetCostmap,
    LoadMap,
)
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


# HACK: BasicNavigator uses global arguments, so, if we remap the name
# of a node that uses it, the navigator will be remapped as well, breaking
# the mapping of topics.
class Navigator(BasicNavigator):
    """Override the initialization of the nav2 navigator."""

    name: ClassVar[str] = "basic_navigator"
    """Default node name."""

    namespace: ClassVar[str] = "/wa/mobilebot_1"
    """Default node namespace."""

    def __init__(
        self,
        name: str | None = None,
        namespace: str | None = None,
        use_sim_time: bool = True,
    ) -> None:
        """Override the initialization of the BasicNavigator node."""
        super(BasicNavigator, self).__init__(
            node_name=name if name is not None else self.name,
            namespace=namespace if namespace is not None else self.namespace,
            use_global_arguments=False,
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time),
            ],
        )

        self.initial_pose = geometry_msgs.PoseStamped()
        self.initial_pose.header.frame_id = "map"
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.initial_pose_received = False
        self.nav_through_poses_client = ActionClient(
            self,
            NavigateThroughPoses,
            "navigate_through_poses",
        )
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
        )
        self.follow_waypoints_client = ActionClient(
            self,
            FollowWaypoints,
            "follow_waypoints",
        )
        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")
        self.compute_path_to_pose_client = ActionClient(
            self,
            ComputePathToPose,
            "compute_path_to_pose",
        )
        self.compute_path_through_poses_client = ActionClient(
            self,
            ComputePathThroughPoses,
            "compute_path_through_poses",
        )
        self.smoother_client = ActionClient(self, SmoothPath, "smooth_path")
        self.spin_client = ActionClient(self, Spin, "spin")
        self.backup_client = ActionClient(self, BackUp, "backup")
        self.assisted_teleop_client = ActionClient(
            self,
            AssistedTeleop,
            "assisted_teleop",
        )
        self.localization_pose_sub = self.create_subscription(
            geometry_msgs.PoseWithCovarianceStamped,
            "amcl_pose",
            self._amclPoseCallback,
            amcl_pose_qos,
        )
        self.initial_pose_pub = self.create_publisher(
            geometry_msgs.PoseWithCovarianceStamped,
            "initialpose",
            10,
        )
        self.change_maps_srv = self.create_client(
            LoadMap,
            "map_server/load_map",
        )
        self.clear_costmap_global_srv = self.create_client(
            ClearEntireCostmap,
            "global_costmap/clear_entirely_global_costmap",
        )
        self.clear_costmap_local_srv = self.create_client(
            ClearEntireCostmap,
            "local_costmap/clear_entirely_local_costmap",
        )
        self.get_costmap_global_srv = self.create_client(
            GetCostmap,
            "global_costmap/get_costmap",
        )
        self.get_costmap_local_srv = self.create_client(
            GetCostmap,
            "local_costmap/get_costmap",
        )
