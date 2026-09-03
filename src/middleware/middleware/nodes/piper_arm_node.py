"""旧路径兼容:canonical 实现已移至 middleware.nodes.arms.piper_arm_node。"""

from middleware.nodes.arms.piper_arm_node import *  # noqa: F401,F403

if __name__ == "__main__":
    from middleware.nodes.arms.piper_arm_node import main

    main()
