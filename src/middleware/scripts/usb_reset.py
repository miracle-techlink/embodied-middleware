"""旧路径兼容:canonical 实现位于 middleware.maintenance.usb_reset。"""

from middleware.maintenance.usb_reset import *  # noqa: F401,F403

if __name__ == "__main__":
    from middleware.maintenance.usb_reset import main

    main()
