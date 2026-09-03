"""旧路径兼容:canonical 实现已移至 middleware.nodes.benchmarks.msg_center_bench。"""

from middleware.nodes.benchmarks.msg_center_bench import *  # noqa: F401,F403

if __name__ == "__main__":
    from middleware.nodes.benchmarks.msg_center_bench import main

    main()
