from skiros2_skill.core.skill import SkillDescription, SkillBase, ParallelFs, SerialStar, Serial, ParallelFf
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element


#################################################################################
# Description
#################################################################################

class FollowPose(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Pose", Element("skiros:TransformationPose"), ParamTypes.Optional)
        self.addParam("Pose2", Element("skiros:TransformationPose"), ParamTypes.Optional)
        self.addParam("Pose3", Element("skiros:TransformationPose"), ParamTypes.Optional)

#################################################################################
# Implementation
#################################################################################


class follow_pose(SkillBase):
    def createDescription(self):
        self.setDescription(FollowPose(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(SerialStar())

        skill(
            self.skill(SerialStar())(
                self.skill("PoseGenerator", "",
                           specify={"x": 1., "y": 0., "z": 0.}),
                self.skill("PoseGenerator", "",
                           specify={"x": 1., "y": 1., "z": 1.}, remap={"Pose": "Pose2"}),
                self.skill("PoseGenerator", "",
                           specify={"x": 0., "y": 2., "z": 2.}, remap={"Pose": "Pose3"}),
            ),
            self.skill(ParallelFs())(
                self.skill("PoseMover", "pose_circle_mover", specify={"Direction": 2}),
                self.skill(SerialStar())(
                    self.skill("PoseFollowerTwoAxis", "pose_follower_two_axis",
                               specify={"Axis1": 0., "Axis2": 1.}),
                    self.skill("PoseFollowerThreeAxis", "pose_follower_three_axis"),
                    self.skill("PoseFollowerOneAxis", "pose_follower_one_axis",
                               specify={"Axis": 2.}, remap={"Pose": "Pose3"}),
                    self.skill("PoseFollowerThreeAxis", "pose_follower_three_axis",
                               remap={"Pose": "Pose3"})
                )

            )
        )
