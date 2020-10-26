import rospy
import numpy as np

from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase

from turtlesim.msg import Pose as PoseMsg
from geometry_msgs.msg import Twist as TwistMsg


#################################################################################
# DriverDetect
#################################################################################

class DriverDetect(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Names", str, ParamTypes.Optional)

class driver_detect(PrimitiveBase):
    def createDescription(self):
        self.setDescription(DriverDetect(), self.__class__.__name__)

    def execute(self):
        turtles = list(set([t[0].split('/')[2] for t in rospy.get_published_topics("/turtles")]))
        self.params["Names"].values = turtles
        return self.success("Detected turtles {}".format(turtles))


#################################################################################
# DriverRead
#################################################################################

class DriverRead(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Name", str, ParamTypes.Required)
        self.addParam("Output", dict, ParamTypes.Optional)

class driver_read(PrimitiveBase):
    def createDescription(self):
        self.setDescription(DriverRead(), self.__class__.__name__)

    def onStart(self):
        turtle = self.params["Name"].value
        topic = "/turtles/{}/pose".format(turtle)

        topics = [n for n, t in rospy.get_published_topics("/turtles/{}".format(turtle))]
        if topic not in topics:
            self.startError("Could not establish connection to turtle '{}'".format(turtle), -2)
            return False

        self._pose = None
        self._sub = rospy.Subscriber(topic, PoseMsg, self._read)
        return True

    def onEnd(self):
        self._sub.unregister()
        self._sub = None
        return True

    def _read(self, msg):
        self._pose = {
            "Name": self.params["Name"].value,
            "X": msg.x,
            "Y": msg.y,
            "R": msg.theta
        }

    def execute(self):
        turtle = self.params["Name"].value

        topics = [n for n, t in rospy.get_published_topics("/turtles/{}".format(turtle))]
        if self._sub.name not in topics:
            return self.fail("{}: Connection lost".format(turtle), -1)

        if self._pose is None:
            return self.step("{}: No pose received".format(turtle))

        self.params["State"].value = self._pose

        return self.step("{}: Connected".format(turtle))


#################################################################################
# DriverWrite
#################################################################################

class DriverWrite(SkillDescription):
    def createDescription(self):
        self.addParam("Name", str, ParamTypes.Required)
        self.addParam("Input", dict, ParamTypes.Optional)

class driver_write(PrimitiveBase):
    def createDescription(self):
        self.setDescription(DriverWrite(), self.__class__.__name__)

    def _wait_for_connection(self, timeout=0.5):
        d = rospy.Duration(0.05)
        while not self._pub.get_num_connections():
            rospy.sleep(d)
            timeout -= d.to_sec()
            if timeout <= 0.0:
                return False
        return True

    def onPreempt(self):
        self._write(0,0)
        return self.fail("Preempted", -1)

    def onStart(self):
        turtle = self.params["Name"].value
        self._pub = rospy.Publisher("/turtles/{}/cmd_vel".format(turtle), TwistMsg, queue_size=20)

        if not self._wait_for_connection():
            self.startError("Could not establish connection to turtle '{}'".format(turtle), -2)
            return False
        return True

    def onEnd(self):
        self._pub.unregister()
        self._pub = None
        return True

    def _write(self, state):
        msg = TwistMsg()
        msg.linear.x = state["X"]
        msg.linear.y = state["Y"]
        msg.angular.z = np.math.radians(state["R"])
        self._pub.publish(msg)

    def execute(self):
        turtle = self.params["Name"].value

        if not self._wait_for_connection():
            return self.fail("{}: Connection lost".format(turtle), -1)

        self._write(self.params["Input"].value)

        return self.step("{}: moving [{} {}]".format(turtle, self.params["Input"].value))
