"""旧路径兼容:canonical 实现已移至 middleware.nodes.control.teleop_map_node。"""

from middleware.nodes.control.teleop_map_node import *  # noqa: F401,F403

if __name__ == "__main__":
    from middleware.nodes.control.teleop_map_node import main

    main()
