"""旧路径兼容:canonical 实现已移至 middleware.backends.piper.piper_arm_backend。"""

from middleware.backends.piper.piper_arm_backend import (  # noqa: F401
    PIPER_GRIPPER_RANGE_MM,
    PIPER_JOINT_LIMITS_DEG,
    PiperBackendBase,
    PiperMujocoBackend,
    PiperSdkBackend,
    clamp_gripper_mm,
    clamp_joints_deg,
)
