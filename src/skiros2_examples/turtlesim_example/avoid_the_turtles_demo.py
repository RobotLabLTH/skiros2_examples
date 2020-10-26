from skiros2_skill.core.skill import SkillDescription, SkillBase, Serial, ParallelFf, ParallelFs, Selector, SerialStar, NoFail
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

enemy_turtles = ["Nina", "Pinta", "SantaMaria"]
my_turtle = "SuperT"
goal_turtle = "MissyT"


#################################################################################
# AvoidTheTurtlesSetup
#################################################################################

class AvoidTheTurtlesSetup(SkillDescription):
    def createDescription(self):
        pass


class avoid_the_turtles_setup(SkillBase):
    def createDescription(self):
        self.setDescription(AvoidTheTurtlesSetup(), self.__class__.__name__)

    def expand(self, skill):
        starting_x = 4.5
        for turtle in enemy_turtles:
            skill.addChild(self.skill("Spawn", "spawn", remap={"Turtle": turtle}, specify={
                           "Name": turtle, "Y": 2.0, "X": starting_x, "Rotation": 90.}))
            starting_x = starting_x + 1

        skill.addChild(self.skill("Spawn", "spawn", remap={"Turtle": my_turtle}, specify={
            "Name": my_turtle, "Y": 5.0, "X": 9.5, "Rotation": 0.}))
        skill.addChild(self.skill("Spawn", "spawn", remap={"Turtle": goal_turtle}, specify={
            "Name": goal_turtle, "Y": 5.0, "X": 1.5, "Rotation": 0.}))

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
        self.addParam("Duration", float, ParamTypes.Required)


class super_t_coordinator(SkillBase):
    def createDescription(self):
        self.setDescription(SupertCoordinator(), self.__class__.__name__)

    def expand(self, skill):
        turtle_element = self.wmi.resolve_element(Element("cora:Robot", "turtlebot:" + my_turtle))
        skill.setProcessor(ParallelFs())
        skill(
            self.skill("Monitor", "monitor"),
            self.skill("SupertController", "supert_controller", specify={"Turtle": turtle_element, "MinVel": 2.0}),
            self.skill("Command", "command"),
            self.skill("Wait", "wait", specify={"Duration": 10000.0})
        )


class SupertController(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Target", Element("sumo:Object"), ParamTypes.Required)


class supert_controller(PrimitiveBase):

    def createDescription(self):
        self.setDescription(PoseController(), self.__class__.__name__)

    def onPreempt(self):
        return self.success("Preempted.")

    def execute(self):
        turtle = self.params["Turtle"].value
        target = self.params["Target"].value

        turtle_pos = np.array(turtle.getData(":Position"))[:2]
        target_pos = np.array(target.getData(":Position"))[:2]
        vec = target_pos - turtle_pos
        distance = np.linalg.norm(vec)

        if self.params["MinVel"].value is not None:
            distance = max(self.params["MinVel"].value, distance)

        if self.params["Catch"].value and distance <= 0.001:
            return self.success("{} caught {}".format(turtle.label, target.label))

        turtle_rot = turtle.getData(":OrientationEuler")[2]

        a = vec / distance
        b = np.array([math.cos(turtle_rot), math.sin(turtle_rot)])
        angle = math.acos(a.dot(b))

        self.params["Linear"].value = distance / 4.0
        self.params["Angular"].value = math.degrees(angle) / 2.0

        return self.step("")
