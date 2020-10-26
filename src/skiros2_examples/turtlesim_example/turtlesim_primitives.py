import rospy
import numpy as np

from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase

from skiros2_common.core.world_element import Element


#################################################################################
# KeepAlive
#################################################################################
class KeepAlive(SkillDescription):
    def createDescription(self):
        pass

class keep_alive(PrimitiveBase):
    def createDescription(self):
        self.setDescription(KeepAlive(), self.__class__.__name__)

    def onPreempt(self):
        return self.step("Done")

    def execute(self):
        return self.step("")


#################################################################################
# Connect
#################################################################################

class Connect(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Name", str, ParamTypes.Optional)

class connect(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Connect(), self.__class__.__name__)

    def execute(self):
        name = self.params["Name"].value
        topics = [n for n, t in rospy.get_published_topics("/turtles/{}".format(name))]
        elements = [e for e in self.wmi.resolve_elements(Element("turtlebot:Turtle")) if e.getProperty("turtlebot:TurtleName").value == name]

        if not topics:
            for e in elements:
                self.wmi.remove_element(e)
            return self.fail("{}: Connection failed".format(name), -1)

        if not elements:
            turtle = self.wmi.get_template_element("turtlebot:turtle")
            turtle.setProperty("turtlebot:TurtleName", "{}".format(name))
            turtle.label = "turtlebot:" + name
            turtle = self.wmi.add_element(turtle)
            return self.success("{}: Established connection".format(name))

        return self.success("{}: Connected".format(name))



#################################################################################
# DetectChange
#################################################################################

class DetectChange(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Names", str, ParamTypes.Optional)

class detect_change(PrimitiveBase):
    def createDescription(self):
        self.setDescription(DetectChange(), self.__class__.__name__)

    def execute(self):
        server_turtles = set(self.params["Names"].values)

        wm_elements = self.wmi.resolve_elements(Element("turtlebot:Turtle"))
        wm_turtles = set([t.getProperty("turtlebot:TurtleName").value for t in wm_elements])

        add_turtles = server_turtles.difference(wm_turtles)
        remove_turtles = wm_turtles.difference(server_turtles)

        if not add_turtles and not remove_turtles:
            return self.success("No change")
        else:
            return self.fail("Change detected", -1)





#################################################################################
# Update
#################################################################################

class Update(SkillDescription):
    def createDescription(self):
        self.addParam("State", dict, ParamTypes.Optional)
        # self.addParam("Turtle", Element("turtlebot:Turtle"), ParamTypes.Optional)

class update(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Update(), self.__class__.__name__)

    def execute(self):
        state = self.params["State"].value
        if not state:
            return self.success("No valid state to synchronize")

        name = state['Name']

        wm_elements = self.wmi.resolve_elements(Element("turtlebot:Turtle"))
        wm_turtle = [t for t in wm_elements if t.getProperty("turtlebot:TurtleName").value == name]

        if not wm_turtle:
            return self.fail("{}: Turtle does not exist!".format(name), -1)

        turtle = wm_turtle[0]
        turtle.setData(":Position", [state['X'], state['Y'], 0.0])
        turtle.setData(":OrientationEuler", [0.0, 0.0, np.math.radians(state['R'])])
        self.wmi.update_element(turtle)

        return self.success("{}: Updated".format(name))










#################################################################################
# LinearController
#################################################################################
class LinearController(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Turtle", Element("turtlebot:Turtle"), ParamTypes.Required)
        self.addParam("Target", Element("sumo:Object"), ParamTypes.Required)
        self.addParam("Linear", float, ParamTypes.Optional)
        self.addParam("Gain", float, ParamTypes.Optional)

class linear_controller(PrimitiveBase):
    def createDescription(self):
        self.setDescription(LinearController(), self.__class__.__name__)

    def onPreempt(self):
        return self.success("Preempted.")

    def execute(self):
        turtle = self.params["Turtle"].value
        target = self.params["Target"].value

        turtle_pos = np.array(turtle.getData(":Position"))[:2]
        target_pos = np.array(target.getData(":Position"))[:2]
        vec = target_pos - turtle_pos
        distance = np.linalg.norm(vec)

        distance = max(distance, self.params["Gain"].value*3)

        self.params["Linear"].value = distance * self.params["Gain"].value

        return self.success("")



#################################################################################
# AngularController
#################################################################################
class AngularController(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Turtle", Element("turtlebot:Turtle"), ParamTypes.Required)
        self.addParam("Target", Element("sumo:Object"), ParamTypes.Required)
        self.addParam("Angular", float, ParamTypes.Optional)
        self.addParam("Gain", float, ParamTypes.Optional)


class angular_controller(PrimitiveBase):
    def createDescription(self):
        self.setDescription(AngularController(), self.__class__.__name__)

    def onPreempt(self):
        return self.success("Preempted.")

    def execute(self):
        turtle = self.params["Turtle"].value
        target = self.params["Target"].value

        # 2D case
        turtle_pos = np.array(turtle.getData(":Position"))[:2]
        target_pos = np.array(target.getData(":Position"))[:2]

        turtle_dir = target_pos - turtle_pos
        turtle_rot = turtle.getData(":OrientationEuler")[2]

        a = turtle_dir / np.linalg.norm(turtle_dir)
        b = np.array([np.math.cos(turtle_rot), np.math.sin(turtle_rot)])

        angle = np.arctan2(np.cross(b, a), np.dot(b,a))
        if np.isnan(angle): angle = 0.0

        self.params["Angular"].value = np.math.degrees(angle) * self.params["Gain"].value

        return self.success("")


#################################################################################
# PoseController
#################################################################################
class PoseController(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Catch", True, ParamTypes.Required)
        self.addParam("Turtle", Element("turtlebot:Turtle"), ParamTypes.Required)
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

        turtle_dir = target_pos - turtle_pos
        distance = np.linalg.norm(turtle_dir)

        if self.params["MinVel"].value is not None:
            distance = max(self.params["MinVel"].value, distance)

        if self.params["Catch"].value and distance <= 0.001:
            return self.success("{} caught {}".format(turtle.label, target.label))

        turtle_rot = turtle.getData(":OrientationEuler")[2]

        a = turtle_dir / distance
        b = np.array([np.math.cos(turtle_rot), np.math.sin(turtle_rot)])

        angle = np.arctan2(np.cross(b, a), np.dot(b,a))
        if np.isnan(angle): angle = 0.0

        self.params["Linear"].value = distance
        self.params["Angular"].value = np.math.degrees(angle) * 2

        return self.success("")

