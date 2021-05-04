#!/usr/bin/python3

import rospy, time
from i2cpwm_board.msg import Servo, ServoArray
from sensor_msgs.msg import Joy

class ServoConvert():
    """
    Class for controlling the servos
    """
    def __init__(self, id=1, center_value_throttle=333, center_value_steer=300, range=100, direction=1):
        self.value = 0.0
        self.id = id
        self._center_throttle = center_value_throttle
        self._center_steer = center_value_steer
        self._range = range
        self._half_range = 0.5 * range
        self._dir = direction

    def get_value_out(self, value_in, type):
        #--- value is in [-1, 1]
        self.value = value_in
        if type == "steer":
            self.value_out = int(self._dir*value_in*self._half_range + self._center_steer)
        else:
            self.value_out = int(self._dir*value_in*self._half_range + self._center_throttle)
        return(self.value_out)

class ROSDonkey():
    def __init__(self):
        rospy.loginfo("Setting Up the Node...")

        rospy.init_node('dk_llc')

        # Create actuator dictionary
        self.actuators = {}
        self.actuators['throttle'] = ServoConvert(id=1)
        self.actuators['steering'] = ServoConvert(id=2, direction=1) #-- positive left
        rospy.loginfo("> Actuators corrrectly initialized")

        # Create servo array
        self._servo_msg = ServoArray()
        for i in range(2): self._servo_msg.servos.append(Servo())

        # Create the servo array publisher
        self.ros_pub_servo_array = rospy.Publisher("/servos_absolute", ServoArray, queue_size=1)
        rospy.loginfo("> Publisher corrrectly initialized")

        # Create the Subscriber to Joystick commands
        self.ros_sub_twist = rospy.Subscriber("/joy", Joy, self.set_actuators_from_joystick)
        rospy.loginfo("> Subscriber corrrectly initialized")

        #--- Get the last time e got a commands
        self._last_time_cmd_rcv = time.time()
        self._timeout_s = 5

        rospy.loginfo("Initialization complete")

    def set_actuators_from_joystick(self, message):
        """
        Get a Joystick message from joy, set actuators based on message.
        Using Xbox controller - left stick for steer, right stick for throttle

        Joy looks like:
        Reports the state of a joysticks axes and buttons.
        Header header           # timestamp in the header is the time the data is received from the joystick
        float32[] axes          # the axes measurements from a joystick
        int32[] buttons         # the buttons measurements from a joystick

        axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
              left stick         right stick
        """
        #-- Save the time
        self._last_time_cmd_rcv = time.time()
        axes = message.axes
        steer_msg = axes[0]
        throttle_msg = axes[4]

        # Conver into servo values
        self.actuators['throttle'].get_value_out(throttle_msg, 'throttle')
        self.actuators['steering'].get_value_out(steer_msg, 'steer')

        rospy.loginfo("Got a command v = %2.1f  s = %2.1f"%(throttle_msg, steer_msg))

        self.send_servo_msg()

    def set_actuators_idle(self):
        # Convert vel into servo values
        self.actuators['throttle'].get_value_out(0, 'throttle')
        self.actuators['steering'].get_value_out(0, 'steer')
        rospy.loginfo("Setting actutors to idle")
        self.send_servo_msg()

    def send_servo_msg(self):
        for actuator_name, servo_obj in iter(self.actuators.items()):
            self._servo_msg.servos[servo_obj.id-1].servo = servo_obj.id
            self._servo_msg.servos[servo_obj.id-1].value = servo_obj.value_out
            rospy.loginfo("Sending to %s command %d"%(actuator_name, servo_obj.value_out))

        self.ros_pub_servo_array.publish(self._servo_msg)

    @property
    def is_controller_connected(self):
        # print(time.time() - self._last_time_cmd_rcv)
        return(time.time() - self._last_time_cmd_rcv < self._timeout_s)

    def run(self):

        #--- Set the control rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # print(self._last_time_cmd_rcv, self.is_controller_connected)
            if not self.is_controller_connected:
                self.set_actuators_idle()

            rate.sleep()

if __name__ == "__main__":
    donkey = ROSDonkey()
    donkey.run()