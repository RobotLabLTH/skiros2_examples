from skiros2_skill.core.skill import SkillDescription, SkillBase, Serial, ParallelFf, ParallelFs, Selector, SerialStar, NoFail
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

import numpy as np
import rospy
import rosnode


class SkillFactory:

    def __init__(self, factory):
        self._factory = factory

    def While(self, cond, skill, then=None):
        s = self._factory.skill(ParallelFf())(cond, skill)
        if then is None:
            return s
        else:
            return self._factory.skill(Serial())(s, then)

    def Repeat(self, *skill):
        return self._factory.skill(ParallelFf())(
            self._factory.skill("KeepAlive", "keep_alive"),
            *skill
        )

    def RepeatUntilSuccess(self, *skill):
        return self._factory.skill(ParallelFs())(
            self._factory.skill("KeepAlive", "keep_alive"),
            *skill
        )

    def RepeatUntilTimeout(self, timeout, *skill):
        return self._factory.skill(ParallelFs())(
            self._factory.skill("Wait", "wait", specify={"Duration": timeout}),
            *skill
        )

    def RepeatForever(self, *skill):
        return self._factory.skill(ParallelFf())(
            self._factory.skill("KeepAlive", "keep_alive"),
            self._factory.skill(NoFail(ParallelFf()))(
                self._factory.skill("KeepAlive", "keep_alive"),
                *skill
            )
        )




#################################################################################
# ConnectTurtle
#################################################################################

class ConnectTurtle(SkillDescription):
    def createDescription(self):
        self.addParam("Name", str, ParamTypes.Required)

class connect(SkillBase):
    def createDescription(self):
        self.setDescription(Connect(), self.__class__.__name__)

    def expand(self, skill):
        turtle = self.params["Name"].value
        skill.setProcessor(ParallelFf())
        skill(
            self.skill(Serial())(
                self.skill("Connect", "connect"),
                self.skill("Monitor", "monitor"),
            ),
            self.skill("Update", "update"),
        )





#################################################################################
# ConnectAll
#################################################################################

class ConnectAll(SkillDescription):
    def createDescription(self):
        self.addParam("Names", str, ParamTypes.Required)

class connect_all(SkillBase):
    def createDescription(self):
        self.setDescription(ConnectAll(), self.__class__.__name__)
        self._expand_on_start = True

    def expand(self, skill):
        monitor = []
        for turtle in self.params["Names"].values:
            monitor.append(self.skill("Connect", "connect", specify={"Name": turtle}))

        skill.setProcessor(ParallelFf())
        skill(*monitor)


#################################################################################
# AutoConnectAll
#################################################################################

class AutoConnectAll(SkillDescription):
    def createDescription(self):
        pass

class auto_connect_all(SkillBase):
    def createDescription(self):
        self.setDescription(AutoConnectAll(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(Serial())
        skill(
            self.skill("Detect", "detect"),
            self.skill("ConnectAll", "connect_all")
        )



#################################################################################
# Synchronize
#################################################################################

class Synchronize(SkillDescription):
    def createDescription(self):
        self.addParam("Name", str, ParamTypes.Required)

class synchronize(SkillBase):
    def createDescription(self):
        self.setDescription(Synchronize(), self.__class__.__name__)

    def expand(self, skill):
        turtle = self.params["Name"].value
        skill.setProcessor(ParallelFf())
        skill(
            self.skill(Serial())(
                self.skill("Connect", "connect"),
                self.skill("Monitor", "monitor"),
            ),
            self.skill("Update", "update"),
        )


#################################################################################
# SynchronizeAll
#################################################################################

class SynchronizeAll(SkillDescription):
    def createDescription(self):
        self.addParam("Names", str, ParamTypes.Required)

class synchronize_all(SkillBase):
    def createDescription(self):
        self.setDescription(SynchronizeAll(), self.__class__.__name__)
        self._expand_on_start = True

    def expand(self, skill):
        sync = []
        for turtle in self.params["Names"].values:
            sync.append(self.skill("Synchronize", "synchronize", remap={"Name": turtle}, specify={turtle: turtle}))

        skill.setProcessor(ParallelFf())
        skill(*sync)


#################################################################################
# AutoSynchronizeAll
#################################################################################

class AutoSynchronizeAll(SkillDescription):
    def createDescription(self):
        pass

class auto_synchronize_all(SkillBase):
    def createDescription(self):
        self.setDescription(AutoSynchronizeAll(), self.__class__.__name__)

    def expand(self, skill):
        S = SkillFactory(self)

        skill.setProcessor(Serial())
        skill(
            self.skill("Detect", "detect"),
            S.RepeatForever(
                self.skill("SynchronizeAll", "synchronize_all"),
                self.skill("DetectChange", "detect_change"),
            )
        )



# #################################################################################
# # Synchronize
# #################################################################################

# class Synchronize(SkillDescription):
#     def createDescription(self):
#         pass

# class synchronize(SkillBase):
#     def createDescription(self):
#         self.setDescription(Synchronize(), self.__class__.__name__)

#     def expand(self, skill):
#         S = SkillFactory(self)

#         skill.setProcessor(Serial())
#         skill(
#             self.skill("DetectTurtles", "detect_turtles"),
#             self.skill("SyncWM", "sync_wm"),
#             # S.RepeatUntilTimeout(1.0,
#             self.skill("MonitorTurtles", "monitor_turtles")
#             # )
#         )

#################################################################################
# MonitorAll
#################################################################################

class MonitorTurtles(SkillDescription):
    def createDescription(self):
        self.addParam("Names", str, ParamTypes.Required)

class monitor_turtles(SkillBase):
    def createDescription(self):
        self.setDescription(MonitorTurtles(), self.__class__.__name__)
        self._expand_on_start = True

    def expand(self, skill):
        S = SkillFactory(self)

        monitor = []
        for turtle in self.params["Names"].values:
            monitor.append(self.skill("Monitor", "monitor", specify={"Name": turtle}))

        skill.setProcessor(ParallelFf())
        skill(
            self.skill("UpdateDetector", "update_detector"),
            *monitor
        )



#################################################################################
# Spawning
#################################################################################

class SpawnRandom(SkillDescription):
    def createDescription(self):
        self.addParam("Name", str, ParamTypes.Required)
        self.addParam("RangeX", [2.0, 8.0], ParamTypes.Optional)
        self.addParam("RangeY", [2.0, 8.0], ParamTypes.Optional)
        self.addParam("RangeR", [0.0, 360.0], ParamTypes.Optional)

class spawn_random(SkillBase):
    def createDescription(self):
        self.setDescription(SpawnRandom(), self.__class__.__name__)

    def expand(self, skill):
        range_x = self.params["RangeX"].values
        range_y = self.params["RangeY"].values
        range_r = self.params["RangeR"].values
        x = np.random.uniform(low=range_x[0], high=range_x[1])
        y = np.random.uniform(low=range_y[0], high=range_y[1])
        r = np.random.uniform(low=range_r[0], high=range_r[1])

        skill.setProcessor(Serial())
        skill(self.skill("Spawn", "spawn", specify={"X": x, "Y": y, "Rotation": r}))


#################################################################################
# Move
#################################################################################

class Move(SkillDescription):
    def createDescription(self):
        self.addParam("Turtle", Element("turtlebot:Turtle"), ParamTypes.Required)
        self.addParam("Distance", 0.0, ParamTypes.Required)
        self.addParam("Angle", 0.0, ParamTypes.Required)
        self.addParam("Duration", 1.0, ParamTypes.Optional)

class move(SkillBase):
    def createDescription(self):
        self.setDescription(Move(), self.__class__.__name__)

    def expand(self, skill):
        t = self.params["Duration"].value
        v = self.params["Distance"].value / t
        w = self.params["Angle"].value / t
        uid = id(self)

        S = SkillFactory(self)

        wait = self.skill("Wait", "wait", specify={"Duration": t})
        move = self.skill("Command", "command",
                   specify={"Linear{}".format(uid): v, "Angular{}".format(uid): w},
                   remap={"Linear": "Linear{}".format(uid), "Angular": "Angular{}".format(uid)})
        stop = self.skill("Command", "command",
                   specify={"Linear{}".format(uid): 0.0, "Angular{}".format(uid): 0.0},
                   remap={"Linear": "Linear{}".format(uid), "Angular": "Angular{}".format(uid)})

        skill.setProcessor(ParallelFs())
        skill(
            self.skill("Synchronize", "synchronize"),
            S.While(wait, move, then=stop)
        )


#################################################################################
# Patrol
#################################################################################

class Patrol(SkillDescription):
    def createDescription(self):
        self.addParam("Once", True, ParamTypes.Required)
        self.addParam("Turtle", Element("turtlebot:Turtle"), ParamTypes.Required)

class patrol(SkillBase):
    def createDescription(self):
        self.setDescription(Patrol(), self.__class__.__name__)

    def expand(self, skill):

        path = [
            self.skill("Move", "move", specify={"Distance": 3.0, "Angle": 0.0, "Duration": 2.0}),
            self.skill("Move", "move", specify={"Distance": 0.5, "Angle": 90.0}),
            self.skill("Move", "move", specify={"Distance": 3.0, "Angle": 0.0, "Duration": 2.0}),
            self.skill("Move", "move", specify={"Distance": 0.5, "Angle": 90.0}),
            self.skill("Move", "move", specify={"Distance": 3.0, "Angle": 0.0, "Duration": 2.0}),
            self.skill("Move", "move", specify={"Distance": 0.5, "Angle": 90.0}),
            self.skill("Move", "move", specify={"Distance": 3.0, "Angle": 0.0, "Duration": 2.0}),
            self.skill("Move", "move", specify={"Distance": 0.5, "Angle": 90.0}),
        ]

        S = SkillFactory(self)

        move = self.skill(SerialStar())(*path)

        skill.setProcessor(Serial())
        if self.params["Once"].value:
            skill(move)
        else:
            skill(S.Repeat(move))



#################################################################################
# Follow
#################################################################################

class Follow(SkillDescription):
    def createDescription(self):
        self.addParam("Turtle", Element("turtlebot:Turtle"), ParamTypes.Required)
        self.addParam("Target", Element("turtlebot:Turtle"), ParamTypes.Required)
        self.addParam("Gain", float, ParamTypes.Optional)

class follow(SkillBase):
    def createDescription(self):
        self.setDescription(Follow(), self.__class__.__name__)

    def expand(self, skill):
        uid = id(self)

        linear = self.skill("LinearController", "linear_controller",
                        specify={"Gain_Linear": 0.5 * self.params["Gain"].value},
                        remap={"Gain": "Gain_Linear", "Linear": "Linear{}".format(uid)})
        angular = self.skill("AngularController", "angular_controller",
                        specify={"Gain_Angular": self.params["Gain"].value},
                        remap={"Gain": "Gain_Angular", "Angular": "Angular{}".format(uid)})

        pose = self.skill(ParallelFf())(linear, angular)

        # pose = self.skill("PoseController", "pose_controller", specify={"MinVel": 2.0},
        #                 remap={"Linear": "Linear{}".format(uid), "Angular": "Angular{}".format(uid)})

        move = self.skill("Command", "command",
                        # specify={"Linear{}".format(uid): 0.0},
                        remap={"Linear": "Linear{}".format(uid), "Angular": "Angular{}".format(uid)})

        skill.setProcessor(ParallelFf())
        skill(
            self.skill("Synchronize", "synchronize"),
            pose,
            move,
        )



#################################################################################
# Orbit
#################################################################################

class Orbit(SkillDescription):
    def createDescription(self):
        self.addParam("Turtle1", Element("turtlebot:Turtle"), ParamTypes.Required)
        self.addParam("Turtle2", Element("turtlebot:Turtle"), ParamTypes.Required)

class orbit(SkillBase):
    def createDescription(self):
        self.setDescription(Orbit(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(ParallelFs())
        skill(
            self.skill("Follow", "follow", specify={"Gain": 1.0}, remap={"Turtle": "Turtle1", "Target": "Turtle2"}),
            self.skill("Follow", "follow", specify={"Gain": 2.0}, remap={"Turtle": "Turtle2", "Target": "Turtle1"}),
            self.skill("Wait", "wait", specify={"Duration": 10000.0})
        )
