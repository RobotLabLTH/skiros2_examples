from skiros2_skill.core.skill import SkillDescription, SkillBase, ParallelFf, Serial
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element


#################################################################################
# Description
#################################################################################

class TrajectoryCoordinator(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Pose", Element("skiros:TransformationPose"), ParamTypes.Optional)
        self.addParam("Pose2", Element("skiros:TransformationPose"), ParamTypes.Optional)

#################################################################################
# Implementation
#################################################################################


class async_trajectory_coordinator(SkillBase):
    def createDescription(self):
        self.setDescription(TrajectoryCoordinator(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(Serial())
        skill(
            self.skill("TrajectoryGenerator", "async_trajectory_generator"),
            self.skill("TrajectoryConsumer", "trajectory_consumer")
        )

class sync_trajectory_coordinator(SkillBase):
    def createDescription(self):
        self.setDescription(TrajectoryCoordinator(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(ParallelFf())
        skill(
            self.skill("TrajectoryGenerator", "sync_trajectory_generator"),
            self.skill("TrajectoryConsumer", "trajectory_consumer")
        )
