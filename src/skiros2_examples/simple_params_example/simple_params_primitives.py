from skiros2_skill.core.skill import SkillDescription, ParamOptions
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
import threading
try:
    import Queue as queue
except ImportError:
    import queue

#################################################################################
# Descriptions
#################################################################################

class TrajectoryGenerator(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Trajectory", dict, ParamTypes.Optional)
        self.addParam("Shutdown", False, ParamTypes.Optional)

class TrajectoryConsumer(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Trajectory", dict, ParamTypes.Required)
        self.addParam("Shutdown", bool, ParamTypes.Required)

#################################################################################
# Implementations
#################################################################################

import random
import time

class trajectory_generator(PrimitiveBase):
    """
    This primitive generates fake "trajectories", under form of dictionaries

    To support a planning process that could take a long time, the plan function is executed in a parallel thread
    and outputs a plan every second. The execute functions just updates the parameters.

    Since parameters can be updated ONLY in the EXECUTE function, a syncronized queue is necessary
    """
    q = queue.Queue(1)
    is_done = False
    iterations = 2

    def onStart(self):
        self.is_done = False
        self.worker = threading.Thread(target=self.plan)
        self.worker.start()
        return True

    def createDescription(self):
        self.setDescription(TrajectoryGenerator(), self.__class__.__name__)

    def plan(self):
        for i in range(0, self.iterations):
            self.q.put([random.random() for _ in xrange(5)])
            time.sleep(1)
        self.is_done = True

    def execute(self):
        if self.is_done:
            self.params["Shutdown"].setValue(True)
            return self.success("Done")
        if not self.q.empty():
            traj = {}
            traj["Trajectory"] = self.q.get()
            self.params["Trajectory"].addValue(traj)
            return self.step("Added trajectory: {}".format(self.params["Trajectory"].getValues()))
        return self.step("")

class trajectory_consumer(PrimitiveBase):
    """
    This primitive consumes one value in "trajectories" at each tick

    It continues to run until the variable Shutdown is set to True
    """
    def createDescription(self):
        self.setDescription(TrajectoryConsumer(), self.__class__.__name__)

    def execute(self):
        if self.params["Trajectory"].isSpecified():
            trajs = self.params["Trajectory"].getValues()
            traj = trajs.pop(0)
            return self.step("Consumed trajectory: {}".format(traj))
        if self.params["Shutdown"].value:
            return self.success("Done")
        return self.step("")
