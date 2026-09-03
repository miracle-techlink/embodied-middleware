"""旧路径兼容:canonical 实现已移至 middleware.nodes.leaders.starai_leader_node。"""

from middleware.nodes.leaders.starai_leader_node import *  # noqa: F401,F403

if __name__ == "__main__":
    from middleware.nodes.leaders.starai_leader_node import main

    main()
