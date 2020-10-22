from skiros2_skill.core.skill import SkillDescription, SkillBase, Serial, ParallelFf, ParallelFs, Selector, SerialStar, NoFail
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element


#################################################################################
# AvoidTheTurtles
#################################################################################

class AvoidTheTurtles(SkillDescription):
    def createDescription(self):
        self.addParam("Names", str, ParamTypes.Required)


class avoid_the_turtles(SkillBase):
    def createDescription(self):
        self.setDescription(AvoidTheTurtles(), self.__class__.__name__)
        self._expand_on_start = True

    def expand(self, skill):
        monitor = []
        turtles = ["nina", "pinta", "santa_maria"]
        index = 0
        for turtle in turtles:
            index += 1
            monitor.append(self.skill("Spawn", "spawn", specify={"Name": turtle, "X": 2.0, "Y": 5.0 + index, "Rotation": 90}))

        # Spawn 3 turtles + player turtle
        # Add patrol behavior for 3 turtles
        # Add custom behavior for player turtle
        skill.setProcessor(ParallelFf())
        skill(*monitor)
