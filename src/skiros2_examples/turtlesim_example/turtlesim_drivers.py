from skiros2_skill.core.skill import SkillDescription, ParamOptions
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
import rospy
import turtlesim.msg as ts
from std_srvs.srv import Empty as EmptySrv
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn as SpawnSrv, Kill as KillSrv, TeleportAbsolute as TeleportSrv
import threading, numpy

import numpy as np
import math


#################################################################################
# Spawn
#################################################################################

class Spawn(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Name", str, ParamTypes.Required)
        self.addParam("X", 0.0, ParamTypes.Required)
        self.addParam("Y", 0.0, ParamTypes.Required)
        self.addParam("Rotation", 0.0, ParamTypes.Required)

class spawn(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Spawn(), self.__class__.__name__)

    def execute(self):
        name = self.params["Name"].value
        try:
            spawner = rospy.ServiceProxy('spawn', SpawnSrv)
            resp = spawner(self.params["X"].value , self.params["Y"].value, math.radians(self.params["Rotation"].value), name)
        except rospy.ServiceException as e:
            return self.fail("Spawning turtle failed.", -1)

        return self.success("Spawned turtle {}".format(name))


#################################################################################
# Teleport
#################################################################################

class Teleport(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Name", str, ParamTypes.Required)
        self.addParam("X", 0.0, ParamTypes.Required)
        self.addParam("Y", 0.0, ParamTypes.Required)
        self.addParam("Rotation", 0.0, ParamTypes.Required)

class teleport(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Teleport(), self.__class__.__name__)

    def execute(self):
        name = self.params["Name"].value
        try:
            tele = rospy.ServiceProxy("/turtles/{}/teleport_absolute".format(name), TeleportSrv)
            resp = tele(self.params["X"].value, self.params["Y"].value, self.params["Rotation"].value)
        except rospy.ServiceException as e:
            return self.fail("Teleporting turtle failed.", -1)

        return self.success("Teleporting turtle {}".format(name))


#################################################################################
# Kill
#################################################################################

class Kill(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Name", str, ParamTypes.Required)

class kill(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Kill(), self.__class__.__name__)

    def execute(self):
        name = self.params["Name"].value
        try:
            killer = rospy.ServiceProxy('kill', KillSrv)
            resp = killer(name)
        except rospy.ServiceException as e:
            return self.fail("Killing turtle failed.", -1)

        return self.success("Killed turtle {}".format(name))


#################################################################################
# Reset
#################################################################################

class Reset(SkillDescription):
    def createDescription(self):
        pass

class reset(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Reset(), self.__class__.__name__)

    def execute(self):
        try:
            resetter = rospy.ServiceProxy('reset', EmptySrv)
            resp = resetter()
        except rospy.ServiceException as e:
            return self.fail("Reset simulation failed.", -1)

        return self.success("Reset simulation")



#################################################################################
# Detect
#################################################################################

class Detect(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Names", str, ParamTypes.Optional)

class detect(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Detect(), self.__class__.__name__)

    def execute(self):
        turtles = list(set([t[0].split('/')[2] for t in rospy.get_published_topics("/turtles")]))
        self.params["Names"].values = turtles
        return self.success("Detected turtles {}".format(turtles))


#################################################################################
# Monitor
#################################################################################

class Monitor(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Name", str, ParamTypes.Required)
        self.addParam("State", dict, ParamTypes.Optional)

class monitor(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Monitor(), self.__class__.__name__)


    def onStart(self):
        turtle = self.params["Name"].value
        topic = "/turtles/{}/pose".format(turtle)

        topics = [n for n, t in rospy.get_published_topics("/turtles/{}".format(turtle))]
        if topic not in topics:
            self.startError("Could not establish connection to turtle '{}'".format(turtle), -2)
            return False

        self._sub = rospy.Subscriber(topic, ts.Pose, self._receive)
        self._pose = None
        return True

    def onEnd(self):
        self._sub.unregister()
        self._sub = None
        return True

    def _receive(self, msg):
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
# Command
#################################################################################

class Command(SkillDescription):
    def createDescription(self):
        self.addParam("Name", str, ParamTypes.Required)
        self.addParam("Linear", float, ParamTypes.Required, "Linear velocity")
        self.addParam("Angular", float, ParamTypes.Required, "Angular velocity in degrees")

class command(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Command(), self.__class__.__name__)

    def _wait_for_connection(self, timeout=0.5):
        d = rospy.Duration(0.05)
        while not self._pub.get_num_connections():
            rospy.sleep(d)
            timeout -= d.to_sec()
            if timeout <= 0.0:
                return False
        return True

    def _send(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = math.radians(angular)
        self._pub.publish(msg)

    def onPreempt(self):
        self._send(0,0)
        return self.fail("Preempted", -1)

    def onStart(self):
        turtle = self.params["Name"].value
        self._pub = rospy.Publisher("/turtles/{}/cmd_vel".format(turtle), Twist, queue_size=20)

        if not self._wait_for_connection():
            self.startError("Could not establish connection to turtle '{}'".format(turtle), -2)
            return False

        return True

    def onEnd(self):
        self._pub.unregister()
        self._pub = None
        return True

    def execute(self):
        turtle = self.params["Name"].value

        if not self._wait_for_connection():
            return self.fail("{}: Connection lost".format(turtle), -1)

        self._send(self.params["Linear"].value, self.params["Angular"].value)

        return self.step("{}: moving [{} {}]".format(turtle, self.params["Linear"].value, self.params["Angular"].value))


