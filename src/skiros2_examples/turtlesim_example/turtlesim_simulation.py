import rospy
import numpy as np

from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase

from std_srvs.srv import Empty as EmptySrv
from turtlesim.srv import Spawn as SpawnSrv, Kill as KillSrv, TeleportAbsolute as TeleportSrv


#################################################################################
# SimulationSpawn
#################################################################################

class SimulationSpawn(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Name", str, ParamTypes.Required)
        self.addParam("X", 0.0, ParamTypes.Required)
        self.addParam("Y", 0.0, ParamTypes.Required)
        self.addParam("Rotation", 0.0, ParamTypes.Required)

class simulation_spawn(PrimitiveBase):
    def createDescription(self):
        self.setDescription(SimulationSpawn(), self.__class__.__name__)

    def execute(self):
        name = self.params["Name"].value
        try:
            spawner = rospy.ServiceProxy('/turtles/spawn', SpawnSrv)
            resp = spawner(self.params["X"].value , self.params["Y"].value, np.math.radians(self.params["Rotation"].value), name)
        except rospy.ServiceException as e:
            return self.fail("Spawning turtle failed.", -1)

        return self.success("Spawned turtle {}".format(name))


#################################################################################
# SimulationTeleport
#################################################################################

class SimulationTeleport(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Name", str, ParamTypes.Required)
        self.addParam("X", 0.0, ParamTypes.Required)
        self.addParam("Y", 0.0, ParamTypes.Required)
        self.addParam("Rotation", 0.0, ParamTypes.Required)

class simulation_teleport(PrimitiveBase):
    def createDescription(self):
        self.setDescription(SimulationTeleport(), self.__class__.__name__)

    def execute(self):
        name = self.params["Name"].value
        try:
            tele = rospy.ServiceProxy("/turtles/{}/teleport_absolute".format(name), TeleportSrv)
            resp = tele(self.params["X"].value, self.params["Y"].value, self.params["Rotation"].value)
        except rospy.ServiceException as e:
            return self.fail("Teleporting turtle failed.", -1)

        return self.success("Teleporting turtle {}".format(name))


#################################################################################
# SimulationKill
#################################################################################

class SimulationKill(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Name", str, ParamTypes.Required)

class simulation_kill(PrimitiveBase):
    def createDescription(self):
        self.setDescription(SimulationKill(), self.__class__.__name__)

    def execute(self):
        name = self.params["Name"].value
        try:
            killer = rospy.ServiceProxy('/turtles/kill', KillSrv)
            resp = killer(name)
        except rospy.ServiceException as e:
            return self.fail("Killing turtle failed.", -1)

        return self.success("Killed turtle {}".format(name))


#################################################################################
# SimulationReset
#################################################################################

class SimulationReset(SkillDescription):
    def createDescription(self):
        pass

class simulation_reset(PrimitiveBase):
    def createDescription(self):
        self.setDescription(SimulationReset(), self.__class__.__name__)

    def execute(self):
        try:
            resetter = rospy.ServiceProxy('/turtles/reset', EmptySrv)
            resp = resetter()
        except rospy.ServiceException as e:
            return self.fail("Reset simulation failed.", -1)

        return self.success("Reset simulation")

