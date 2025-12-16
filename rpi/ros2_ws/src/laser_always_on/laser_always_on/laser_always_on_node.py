import rclpy
from rclpy.node import Node
import pigpio


class LaserAlwaysOn(Node):
    def __init__(self):
        super().__init__('laser_always_on')

        self.declare_parameter('gpio_pin', 23)
        self.declare_parameter('active_low', True)

        self.pin = int(self.get_parameter('gpio_pin').value)
        self.active_low = bool(self.get_parameter('active_low').value)

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError(
                "pigpiod에 연결 실패. 먼저 `sudo pigpiod` 실행 여부 확인!"
            )

        self.pi.set_mode(self.pin, pigpio.OUTPUT)

        self.turn_on()
        self.get_logger().info(
            f'Laser ALWAYS ON started. BCM={self.pin}, active_low={self.active_low}'
        )

    def turn_on(self):
        on_level = 1 if self.active_low else 0
        self.pi.write(self.pin, on_level)

    def turn_off(self):
        off_level = 0 if self.active_low else 1
        self.pi.write(self.pin, off_level)

    def destroy_node(self):
        try:
            self.turn_off()
            self.pi.stop()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = LaserAlwaysOn()
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

