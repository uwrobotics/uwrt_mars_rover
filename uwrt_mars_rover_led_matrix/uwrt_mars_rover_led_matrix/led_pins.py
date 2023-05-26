#import python Node libraries
import rclpy
from rclpy.node import Node
#import ColorRGBA
from std_msgs.msg import ColorRGBA
#imports for PWM
import RPi.GPIO as GPIO
import time

#Being susbcription
class MinimalSubscriber(Node):

#Pin definitions
    output_pins = {
        'JETSON_XAVIER': 18,
        'JETSON_NANO': 33,
        'JETSON_NX': 33,
        'CLARA_AGX_XAVIER': 18,
        'JETSON_TX2_NX': 32,
        'JETSON_ORIN': 18,
        'JETSON_ORIN_NX': 33,
        'JETSON_ORIN_NANO': 33
    }

    check_pwm = output_pins.get(GPIO.model, None)
    if check_pwm is None:
        # add error is pins are none (ROS2 error)
        raise Exception('PWM not supported on this board')
    

    #Initialize node rgb_pin_controller
    def __init__(self):
        super().__init__('rgb_pin_controller')

        #Frequency to reresent the # of cycles/sec
        PWM_FREQUENCY = 500

         #add pins (check colour)
        red_pin_channel = 13
        green_pin_channel = 15
        blue_pin_channel = 18

        # now we define the board and our pins
        GPIO.setmode(GPIO.BOARD)
        #add pins: output, set off
        GPIO.setup(red_pin_channel, GPIO.OUT, initial = GPIO.LOW)
        GPIO.setup(green_pin_channel, GPIO.OUT, initial = GPIO.LOW)
        GPIO.setup(blue_pin_channel, GPIO.OUT, initial = GPIO.LOW)

        #ensuring channels move at PWM frequency
        self.red_pin = GPIO.PWM(red_pin_channel, PWM_FREQUENCY)
        self.green_pin = GPIO.PWM(green_pin_channel, PWM_FREQUENCY)
        self.blue_pin = GPIO.PWM(blue_pin_channel, PWM_FREQUENCY)

        # Pins should be off (0) at the beginning
        self.red_pin.start(99)
        self.green_pin.start(99)
        self.blue_pin.start(99)

        #subscription part
        self.subscription = self.create_subscription(
            ColorRGBA,
            'led_matrix/color',
            self.listener_callback,
            10
        )
        #self.subscription


    #Callback and logging: extract and recieve RGBA and sauce them to PWM
    def listener_callback(self, msg):
        red = msg.r    
        green = msg.g
        blue = msg.b
        self.get_logger().info('RED: %d BLUE: %d GREEN: %d' % (red, blue, green))
        self.red_pin.ChangeDutyCycle(int(red))
        self.green_pin.ChangeDutyCycle(int(green))
        self.blue_pin.ChangeDutyCycle(int(blue))

#define main
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    GPIO.cleanup()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()      