"""旧路径兼容:canonical 实现已移至 middleware.nodes.arms.rebot_arm_node。

`python -m middleware.nodes.rebot_arm_node` 等历史启动方式不破;
新代码/脚本请用 canonical 路径。
"""

from middleware.nodes.arms.rebot_arm_node import *  # noqa: F401,F403

if __name__ == "__main__":
    from middleware.nodes.arms.rebot_arm_node import main

    main()
