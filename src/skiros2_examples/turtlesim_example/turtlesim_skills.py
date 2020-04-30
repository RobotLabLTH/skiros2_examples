from skiros2_skill.core.skill import SkillDescription, SkillBase, Serial, ParallelFf, ParallelFs, Selector, SerialStar
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

import numpy as np

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

        skill.setProcessor(SerialStar())
        skill(self.skill("Spawn", "spawn", specify={"X": x, "Y": y, "Rotation": r}))



#################################################################################
# Move
#################################################################################

class Move(SkillDescription):
    def createDescription(self):
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
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
        skill.setProcessor(ParallelFs())
        skill(
            self.skill("Monitor", "monitor"),
            self.skill("Command", "command",
                       specify={"Linear{}".format(uid): v, "Angular{}".format(uid): w},
                       remap={"Linear": "Linear{}".format(uid), "Angular": "Angular{}".format(uid)}),
            self.skill("Wait", "wait", specify={"Duration": t})
        )


#################################################################################
# Patrol
#################################################################################

class Patrol(SkillDescription):
    def createDescription(self):
        self.addParam("Once", True, ParamTypes.Required)
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)

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

        if self.params["Once"].value:
            skill.setProcessor(SerialStar())
            skill(*path)
        else:
            skill.setProcessor(ParallelFf())
            skill(
                self.skill(SerialStar())(*path),
                self.skill("Wait", "wait", specify={"Duration": 10000.0})
            )



#################################################################################
# Follow
#################################################################################

class Follow(SkillDescription):
    def createDescription(self):
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Target", Element("sumo:Object"), ParamTypes.Required)

class follow(SkillBase):
    def createDescription(self):
        self.setDescription(Follow(), self.__class__.__name__)

    def expand(self, skill):
        uid = id(self)
        skill.setProcessor(ParallelFs())
        skill(
            self.skill("Monitor", "monitor"),
            self.skill("PoseController", "pose_controller", specify={"MinVel": 2.0},
                       remap={"Linear": "Linear{}".format(uid), "Angular": "Angular{}".format(uid)}),
            self.skill("Command", "command",
                       remap={"Linear": "Linear{}".format(uid), "Angular": "Angular{}".format(uid)}),
            self.skill("Wait", "wait", specify={"Duration": 10000.0})
        )



#################################################################################
# Orbit
#################################################################################

class Orbit(SkillDescription):
    def createDescription(self):
        self.addParam("Turtle1", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Turtle2", Element("cora:Robot"), ParamTypes.Required)

class orbit(SkillBase):
    def createDescription(self):
        self.setDescription(Orbit(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(ParallelFs())
        skill(
            self.skill("Follow", "follow", remap={"Turtle": "Turtle1", "Target": "Turtle2"}),
            self.skill("Follow", "follow", remap={"Turtle": "Turtle2", "Target": "Turtle1"}),
            self.skill("Wait", "wait", specify={"Duration": 10000.0})
        )
