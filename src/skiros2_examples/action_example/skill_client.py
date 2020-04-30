from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
import skiros2_common.tools.logger as log
import actionlib
from actionlib.msg import TestAction, TestGoal

class ActionSkillDescription(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("InputParam", 0, ParamTypes.Required)
        self.addParam("OutputParam", 0, ParamTypes.Optional)
        #=======PreConditions=========

class DriveDescription(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Mode", 'DIFFERENTIAL', ParamTypes.Required)
        self.addParam("Target", '', ParamTypes.Required)
        #=======PreConditions=========

class test_action_skill(PrimitiveActionClient):
    """
    @brief A skill that connects to a test action server

    Goal and feeback is just an integer
    """
    def createDescription(self):
        self.setDescription(ActionSkillDescription(), self.__class__.__name__)

    def buildClient(self):
        return actionlib.SimpleActionClient('/test_action_server', TestAction)

    def buildGoal(self):
        return TestGoal(self.params["InputParam"].value)

    def onFeedback(self, msg):
        self.params["OutputParam"].value = msg.feedback
        return self.step("{}".format(self.params["OutputParam"].value))

