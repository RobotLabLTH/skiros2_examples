from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase

import numpy as np

#################################################################################
# Descriptions
#################################################################################


class PoseGenerator(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Pose", Element("skiros:TransformationPose"), ParamTypes.Optional)
        self.addParam("x", float, ParamTypes.Required)
        self.addParam("y", float, ParamTypes.Required)
        self.addParam("z", float, ParamTypes.Required)


class PoseMover(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Pose", Element("skiros:TransformationPose"), ParamTypes.Required)
        self.addParam("Direction", 0, ParamTypes.Required, description="x: 0, y: 1, z: 2")


class PoseFollowerOneAxis(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Pose", Element("skiros:TransformationPose"), ParamTypes.Required)
        self.addParam("Pose2", Element("skiros:TransformationPose"), ParamTypes.Required)
        self.addParam("Axis", float, ParamTypes.Required)


class PoseFollowerTwoAxis(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Pose", Element("skiros:TransformationPose"), ParamTypes.Required)
        self.addParam("Pose2", Element("skiros:TransformationPose"), ParamTypes.Required)
        self.addParam("Axis1", float, ParamTypes.Required)
        self.addParam("Axis2", float, ParamTypes.Required)


class PoseFollowerThreeAxis(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Pose", Element("skiros:TransformationPose"), ParamTypes.Required)
        self.addParam("Pose2", Element("skiros:TransformationPose"), ParamTypes.Required)

#################################################################################
# Implementations
#################################################################################


class pose_generator(PrimitiveBase):
    """
    Creates a TransformationPose on the world model, if not already present
    """

    def createDescription(self):
        self.setDescription(PoseGenerator(), self.__class__.__name__)

    def execute(self):
        pose = self.params["Pose"].value
        if pose.id == "":
            pose.setData(":Position", [self.params["x"].value, self.params["y"].value, self.params["z"].value])
            pose.setData(":Orientation", [0.0, 0.0, 0.0, 1.0])
            pose.addRelation("skiros:Scene-0", "skiros:contain", "-1")
            self.wmi.add_element(pose)
        return self.success("Done")


class linear_mover(PrimitiveBase):
    """
    This primitive has 1 state when progress is < 10 and 1 state of success
    """

    def createDescription(self):
        self.setDescription(PoseMover(), self.__class__.__name__)

    def execute(self):
        pose = self.params["Pose"].value
        direction = self._params.getParamValue("Direction")
        position = pose.getData(":Position")
        position[direction] = position[direction] + 0.1
        pose.setData(":Position", position)
        self.params["Pose"].value = pose
        if self._progress_code < 10:
            return self.step("Changing position to: {}".format(position))
        else:
            return self.success("Done")


class angular_mover(PrimitiveBase):
    """
    This primitive has 1 state when progress is < 10 and 1 state of success
    """

    def createDescription(self):
        self.setDescription(PoseMover(), self.__class__.__name__)

    def onPreempt(self):
        return self.success("Done")

    def execute(self):
        pose = self.params["Pose"].value
        o = pose.getData(":OrientationEuler")
        d = self._params.getParamValue("Direction")
        o[d] = o[d] + 0.1
        pose.setData(":OrientationEuler", o)
        self.wmi.update_element_properties(pose, "AauSpatialReasoner")
        if self._progress_code < 10:
            return self.step("Changing orientation to: {}".format(o))
        else:
            return self.success("Done")


class rotation_mover(PrimitiveBase):
    """
    """

    def createDescription(self):
        self.setDescription(PoseMover(), self.__class__.__name__)

    def onPreempt(self):
        return self.success("Done")

    def execute(self):
        pose = self.params["Pose"].value
        o = pose.getData(":OrientationEuler")
        d = self._params.getParamValue("Direction")
        o[d] = o[d] + 0.1
        pose.setData(":OrientationEuler", o)
        self.wmi.update_element_properties(o, "AauSpatialReasoner")
        return self.step("Changing orientation to: {}".format(o))


class pose_follower_one_axis(PrimitiveBase):
    """
    This primitive makes a pose follow another one along one given axis
    It stops when is close to the target
    """

    dist = 0
    proximityThreshold = 0.05

    def createDescription(self):
        self.setDescription(PoseFollowerOneAxis(), self.__class__.__name__)

    def onPreempt(self):
        return self.success("Done")

    def execute(self):
        pose = self._params.getParamValue("Pose")
        pose2 = self._params.getParamValue("Pose2")
        position = pose.getData(":Position")
        position2 = pose2.getData(":Position")
        self.dist = 0
        axis = int(self._params.getParamValue("Axis"))
        diff = position2[axis] - position[axis]
        self.dist += diff**2
        if diff != 0:
            diff = diff / 4
            position2[axis] -= diff
        pose2.setData(":Position", position2)
        self.params["Pose2"].value = pose2

        if self.dist >= self.proximityThreshold:
            return self.step("Following pose: {}".format(position))
        else:
            return self.success("Successful following along axis: {}".format(axis))


class pose_follower_two_axis(PrimitiveBase):
    """
        This primitive makes a pose follow another one along two given axis
        It stops when is close to the target
    """

    dist = 0
    proximityThreshold = 0.05

    def createDescription(self):
        self.setDescription(PoseFollowerTwoAxis(), self.__class__.__name__)

    def onPreempt(self):
        return self.success("Done")

    def execute(self):
        pose = self._params.getParamValue("Pose")
        pose2 = self._params.getParamValue("Pose2")
        position = pose.getData(":Position")
        position2 = pose2.getData(":Position")
        self.dist = 0
        axis = [int(self._params.getParamValue("Axis1")), int(self._params.getParamValue("Axis2"))]
        for i in axis:
            diff = position2[i] - position[i]
            self.dist += diff**2
            if diff != 0:
                diff = diff / 4
            position2[i] -= diff
        pose2.setData(":Position", position2)
        self.params["Pose2"].value = pose2

        if self.dist >= self.proximityThreshold:
            return self.step("Following pose: {}".format(position))
        else:
            return self.success("Successful following along axis: {}".format(axis))


class pose_follower_three_axis(PrimitiveBase):
    """
    This primitive stops when is close to the target
    """
    dist = 0
    proximityThreshold = 0.05

    def createDescription(self):
        self.setDescription(PoseFollowerThreeAxis(), self.__class__.__name__)

    def onPreempt(self):
        return self.success("Done")

    def execute(self):
        pose = self._params.getParamValue("Pose")
        pose2 = self._params.getParamValue("Pose2")
        position = pose.getData(":Position")
        position2 = pose2.getData(":Position")
        self.dist = 0
        for i in range(0, 3):
            diff = position2[i] - position[i]
            self.dist += diff**2
            if diff != 0:
                diff = diff / 4
            position2[i] -= diff
        pose2.setData(":Position", position2)
        self.params["Pose2"].value = pose2

        if self.dist >= self.proximityThreshold:
            return self.step("Following pose: {}".format(position))
        else:
            return self.success("Successful following")


class pose_circle_mover(PrimitiveBase):
    """
    Makes the selected pose turn around the selected axis
    This primitive doesn't stop until it is preempted explicitely
    """
    angle = 0.05

    def createDescription(self):
        self.setDescription(PoseMover(), self.__class__.__name__)

    def onPreempt(self):
        return self.success("Done")

    def execute(self):
        pose = self.params["Pose"].value
        p = pose.getData(":Position")
        o = pose.getData(":OrientationEuler")
        d = self._params.getParamValue("Direction")
        rotationMatrix = np.array([[np.cos(self.angle), np.sin(self.angle)], [-np.sin(self.angle), np.cos(self.angle)]])
        if d == 2:
            # in that case we turn around the z axis
            xyRotated = np.dot(rotationMatrix, np.array([p[0], p[1]]))
            p[0] = xyRotated[0]
            p[1] = xyRotated[1]
        elif d == 1:
            # in that case we turn around the y axis
            xzRotated = np.dot(rotationMatrix, np.array([p[0], p[2]]))
            p[0] = xzRotated[0]
            p[2] = xzRotated[1]
        elif d == 0:
            # in that case we turn around the x axis
            yzRotated = np.dot(rotationMatrix, np.array([p[1], p[2]]))
            p[1] = yzRotated[0]
            p[2] = yzRotated[1]
        else:
            return self.fail("wrong direction", -1)

        o[d] = o[d] - self.angle
        pose.setData(":OrientationEuler", o)
        pose.setData(":Position", p)
        self.wmi.update_element_properties(pose, "AauSpatialReasoner")
        return self.step("turning")
