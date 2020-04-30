from skiros2_skill.core.skill import SkillDescription, ParamOptions
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
import rospy
import turtlesim.msg as ts
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn as SpawnSrv
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
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Optional)

class spawn(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Spawn(), self.__class__.__name__)

    def execute(self):
        turtle = self.params["Turtle"].value
        name = self.params["Name"].value
        if turtle.id == "":
            try:
                spawner = rospy.ServiceProxy('spawn', SpawnSrv)
                resp = spawner(self.params["X"].value , self.params["Y"].value, math.radians(self.params["Rotation"].value), name)
            except rospy.ServiceException as e:
                return self.fail("Spawning turtle failed.", -1)

            turtle.label = "turtlebot:" + name
            turtle.setProperty("turtlebot:TurtleName", "/{}".format(name))
            turtle.setData(":Position", [self.params["X"].value, self.params["Y"].value, 0.0])
            turtle.setData(":OrientationEuler", [0.0, 0.0, math.radians(self.params["Rotation"].value)])
            turtle.addRelation("skiros:Scene-0", "skiros:contain", "-1")
            turtle = self._wmi.add_element(turtle)
            self.params["Turtle"].value = turtle

            return self.success("Spawned turtle {}".format(name))
        return self.success("")


#################################################################################
# Command
#################################################################################

class Command(SkillDescription):
    def createDescription(self):
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Linear", float, ParamTypes.Required, "Linear velocity")
        self.addParam("Angular", float, ParamTypes.Required, "Angular velocity in degrees")

class command(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Command(), self.__class__.__name__)

    def _send_command(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = math.radians(angular)
        self.pose_pub.publish(msg)

    def onPreempt(self):
        return self.success("Preempted")

    def onEnd(self):
        self._send_command(0,0)
        return True

    def onStart(self):
        turtle = self.params["Turtle"].value.getProperty("turtlebot:TurtleName").value
        self.pose_pub = rospy.Publisher(turtle + "/cmd_vel", Twist, queue_size=20)
        return True

    def execute(self):
        turtle = self.params["Turtle"].value.getProperty("turtlebot:TurtleName").value
        self._send_command(self.params["Linear"].value, self.params["Angular"].value)
        return self.step("{}: moving at [{} {}]".format(turtle, self.params["Linear"].value, self.params["Angular"].value))


#################################################################################
# Monitor
#################################################################################

class Monitor(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)

class monitor(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Monitor(), self.__class__.__name__)

    def onPreempt(self):
        return self.success("Preempted.")

    def onStart(self):
        name = self.params["Turtle"].value.getProperty("turtlebot:TurtleName").value
        self._pose_sub = rospy.Subscriber(name + "/pose", ts.Pose, self._monitor)
        self._pose = None
        return True

    def onEnd(self):
        self._pose_sub.unregister()
        self._pose_sub = None
        return True

    def _monitor(self, msg):
        self._pose = [msg.x, msg.y, msg.theta]

    def execute(self):
        if self._pose is None:
            return self.step("No pose received")

        x,y,theta = self._pose

        turtle = self.params["Turtle"].value
        turtle.setData(":Position", [x, y, 0.0])
        turtle.setData(":OrientationEuler", [0.0, 0.0, theta])
        self.params["Turtle"].value = turtle

        return self.step("")



#################################################################################
# PoseController
#################################################################################

class PoseController(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Catch", False, ParamTypes.Required)
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Target", Element("sumo:Object"), ParamTypes.Required)
        self.addParam("Linear", float, ParamTypes.Optional)
        self.addParam("Angular", float, ParamTypes.Optional)
        self.addParam("MinVel", float, ParamTypes.Optional)

class pose_controller(PrimitiveBase):
    def createDescription(self):
        self.setDescription(PoseController(), self.__class__.__name__)

    def onPreempt(self):
        return self.success("Preempted.")

    def execute(self):
        turtle = self.params["Turtle"].value
        target = self.params["Target"].value

        turtle_pos = np.array(turtle.getData(":Position"))[:2]
        target_pos = np.array(target.getData(":Position"))[:2]
        vec = target_pos - turtle_pos
        distance = np.linalg.norm(vec)

        if self.params["MinVel"].value is not None:
            distance = max(self.params["MinVel"].value, distance)

        if self.params["Catch"].value and distance <= 0.001:
            return self.success("{} caught {}".format(turtle.label, target.label))

        turtle_rot = turtle.getData(":OrientationEuler")[2]

        a = vec / distance
        b = np.array([math.cos(turtle_rot), math.sin(turtle_rot)])
        angle = math.acos(a.dot(b))

        self.params["Linear"].value = distance / 4.0
        self.params["Angular"].value = math.degrees(angle) / 2.0

        return self.step("")

