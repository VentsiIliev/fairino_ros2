from motion.servo.cartesian_servo.i_cartesian_servo import CartesianServo


class MoveItCartesianServo(CartesianServo):

    def _on_start(self, *, frame, tool) -> bool:
        # configure TCP
        # configure command frame
        # activate/start MoveIt Servo
        # start internal high-rate publisher
        raise NotImplementedError


    def _on_update(self, command) -> bool:
        # Replace command consumed by high-rate publisher.
        raise NotImplementedError

    def _on_stop(self) -> bool:
        # Set zero velocity.
        # Stop publisher.
        # Stop/release MoveIt Servo session.
        raise NotImplementedError