from skiros2_skill.core.skill import SkillDescription, SkillBase, ParallelFf
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element


#################################################################################
# Description
#################################################################################

class TrajectoryCoordinator(SkillDescription):
    def createDescription(self):
        #=======Params=========
        #self.addParam("Container", Element(":Location"), ParamTypes.World)
        #self.addParam("Object", Element(":Product"), ParamTypes.Optional)
        self.addParam("Pose", Element("skiros:TransformationPose"), ParamTypes.Optional)
        self.addParam("Pose2", Element("skiros:TransformationPose"), ParamTypes.Optional)

#################################################################################
# Implementation
#################################################################################

class trajectory_coordinator(SkillBase):
    def createDescription(self):
        self.setDescription(TrajectoryCoordinator(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(ParallelFf())
        skill(
            self.skill("TrajectoryGenerator", ""),
            self.skill("TrajectoryConsumer", "")
        )

