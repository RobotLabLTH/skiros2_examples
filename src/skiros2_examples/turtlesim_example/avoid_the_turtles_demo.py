from skiros2_skill.core.skill import SkillDescription, SkillBase, Serial, ParallelFf, ParallelFs, Selector, SerialStar, NoFail
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

enemy_turtles = ["Nina", "Pinta", "SantaMaria"]
my_turtle = "SuperT"
goal_turtle = "MissT"


#################################################################################
# TurtlesSetup
#################################################################################

class TurtlesSetup(SkillDescription):
    def createDescription(self):
        pass


class turtles_setup(SkillBase):
    def createDescription(self):
        self.setDescription(TurtlesSetup(), self.__class__.__name__)

    def expand(self, skill):
        starting_x = 4.5
        for turtle in enemy_turtles:
            skill.addChild(self.skill("Spawn", "spawn", remap={"Turtle": turtle}, specify={
                           "Name": turtle, "Y": 2.0, "X": starting_x, "Rotation": 90.}))
            starting_x = starting_x + 1

        skill.addChild(self.skill("Spawn", "spawn", remap={"Turtle": my_turtle}, specify={
            "Name": my_turtle, "Y": 5.0, "X": 1.5, "Rotation": 0.}))
        skill.addChild(self.skill("Spawn", "spawn", remap={"Turtle": goal_turtle}, specify={
            "Name": goal_turtle, "Y": 5.0, "X": 9.5, "Rotation": 0.}))

#################################################################################
# AvoidTheTurtlesRun
#################################################################################


class AvoidTheTurtlesRun(SkillDescription):
    def createDescription(self):
        pass


class avoid_the_turtles_run(SkillBase):
    def createDescription(self):
        self.setDescription(AvoidTheTurtlesRun(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(ParallelFf())
        duration = 1.0
        for turtle in enemy_turtles:
            turtle_element = self.wmi.resolve_element(Element("cora:Robot", "turtlebot:" + turtle))
            duration = duration + 2
            skill.addChild(self.skill("PatrolBaF", "back_and_forth",
                                      remap={"Turtle": turtle}, specify={turtle: turtle_element, "Duration": duration}))

#################################################################################
# Patrol
#################################################################################


class PatrolBaF(SkillDescription):
    def createDescription(self):
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Duration", float, ParamTypes.Required)


class back_and_forth(SkillBase):
    def createDescription(self):
        self.setDescription(PatrolBaF(), self.__class__.__name__)

    def expand(self, skill):
        d = self.params["Duration"].value
        path = [
            self.skill("Move", "move", specify={"Distance": 5.0, "Angle": 0.0, "Duration": d}),
            self.skill("Move", "move", specify={"Distance": -5.0, "Angle": 0.0, "Duration": d}),
        ]

        skill.setProcessor(ParallelFf())
        skill(
            self.skill(SerialStar())(*path),
            self.skill("Wait", "wait", specify={"Duration": 10000.0})
        )

#################################################################################
# SupertBehavior
#################################################################################


class SupertCoordinator(SkillDescription):
    def createDescription(self):
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("KitOrder", {"Nina": 1, "Pinta": 1, "SantaMaria": 1}, ParamTypes.Required)


class super_t_coordinator(SkillBase):
    def createDescription(self):
        self.setDescription(SupertCoordinator(), self.__class__.__name__)

    def expand(self, skill):
        super_t = self.wmi.resolve_element(Element("cora:Robot", "turtlebot:" + my_turtle))
        if self.params["Turtle"].value.id != super_t.id:
            raise Exception("Main turtle should be {}".format(my_turtle))
        final_target = self.wmi.resolve_element(Element("cora:Robot", "turtlebot:" + goal_turtle))
        skill.setProcessor(Serial())
        skill(
            self.skill("Follow", "follow", specify={"Target": final_target})
        )
