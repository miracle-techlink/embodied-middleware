#!/usr/bin/env python
# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from dataclasses import dataclass, field

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("starai_violin_leader")
@dataclass
class StaraiViolinLeaderConfig(TeleoperatorConfig):
    """StarAI Violin leader arm (Fashionstar UART bus servos).

    6 arm joints (servo IDs 0-5) + 1 gripper (servo ID 6). Angles are read in
    degrees over a UART bus at 1 Mbps via the `uservo` SDK.
    """

    # Serial port for the CH340 adapter (note: NOT ttyUSB0 on this machine).
    port: str = "/dev/ttyCH341USB0"
    baudrate: int = 1_000_000
    # Servo IDs, arm joints in order base->wrist, then the gripper.
    arm_servo_ids: list[int] = field(default_factory=lambda: [0, 1, 2, 3, 4, 5])
    gripper_servo_id: int = 6
