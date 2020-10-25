import random
import time
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
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
        # =======Params=========
        self.addParam("Trajectory", dict, ParamTypes.Optional)


class TrajectoryConsumer(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Trajectory", dict, ParamTypes.Required)

#################################################################################
# Implementations
#################################################################################


class sync_trajectory_generator(PrimitiveBase):
    """
    This primitive generates several fake trajectories, under form of a dictionary
    """
    q = queue.Queue(1)
    iterations = 5

    def onStart(self):
        self.i = 0
        return True

    def createDescription(self):
        self.setDescription(TrajectoryGenerator(), self.__class__.__name__)

    def execute(self):
        traj = {}
        traj["Trajectory"] = [random.random() for _ in xrange(5)]
        self.params["Trajectory"].values.append(traj)
        self.i = self.i + 1
        if self.i >= self.iterations:
            return self.success("Planned trajectory: {}".format(self.params["Trajectory"].getValues()))
        else:
            return self.step("Planned trajectory: {}".format(self.params["Trajectory"].getValues()))


class async_trajectory_generator(PrimitiveBase):
    """
    This primitive generates a fake "trajectory", under form of a dictionary

    To support a planning process that could take a long time, the plan function
    is executed in a parallel thread and outputs a plan after 2 seconds. The
    execute functions just updates the parameters.

    Since parameters can be updated ONLY in the EXECUTE function, a syncronized
    queue is necessary
    """

    def onStart(self):
        self.q = queue.Queue(1)
        self.worker = threading.Thread(target=self.plan)
        self.worker.start()
        return True

    def createDescription(self):
        self.setDescription(TrajectoryGenerator(), self.__class__.__name__)

    def plan(self):
        time.sleep(2)
        self.q.put([random.random() for _ in xrange(5)])

    def execute(self):
        if not self.q.empty():
            traj = {}
            traj["Trajectory"] = self.q.get()
            self.params["Trajectory"].value = traj
            return self.success("Planned trajectory: {}".format(self.params["Trajectory"].getValues()))
        return self.step("Planning...")


class trajectory_consumer(PrimitiveBase):
    """
    This primitive consumes one value in "Trajectory"
    """

    def createDescription(self):
        self.setDescription(TrajectoryConsumer(), self.__class__.__name__)

    def execute(self):
        if self.params["Trajectory"].isSpecified():
            trajs = self.params["Trajectory"].values
            traj = trajs.pop()
            return self.success("Consumed trajectory: {}".format(traj))
        return self.success("Done")
