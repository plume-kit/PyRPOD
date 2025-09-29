from pyrpod.mission.state_vectors import StateVectors
from pyrpod.mission.six_dof_dynamics import SixDOFDynamics
from pyrpod.mission.fuel_management import FuelManager
from pyrpod.mission.thruster_grouping import ThrusterGrouping
from pyrpod.mission.flight_eval import FlightEvaluator
from pyrpod.mission.post_processing import PostProcessor
from pyrpod.mission.orbital_transfer import OrbitalTransferEngine
from pyrpod.mission.MissionEnvironment import MissionEnvironment


class MissionPlanner:

    def __init__(self, environment: MissionEnvironment):
        self.environment = environment

        # Initialize submodules with context if needed
        self.state = StateVectors()
        self.flight_eval = FlightEvaluator(self.environment)
        self.orbital_transfer = OrbitalTransferEngine()
        self.post_processor = PostProcessor()
        self.dynamics = SixDOFDynamics()
        self.propellant = FuelManager(self.environment)
    
    def set_lm(self, LogisticsModule):
        """
            Simple setter method to set VV/LM used in analysis.

            NOTE: This begs the question: What's up with LM vs VV. Do we need both classes?
            If so, how do we handle inheritance between them? Previous efforts have broken
            the code. This is due to "hacky/minimal" effort. A follow up attempt would
            require research into how Python handles inheritance including "container classes".

            Parameters
            ----------
            LogisticsModule : LogisticsModule
                LogisticsModule Object containing inertial properties.

            Returns
            -------
            None
        """
        self.vv = LogisticsModule
    
    def set_jfh(self, JetFiringHistory):
        """
            Simple setter method to set JFH used in propellant usage calculations.

            Parameters
            ----------
            JetFiringHistory : JetFiringHistory
                JFH Object containing specifics of each firing.

            Returns
            -------
            None
        """
        self.jfh = JetFiringHistory

    def load_components(self):
        self.flight_eval.load_plan()

    def execute_mission(self):
        self.flight_eval.execute(self)

    def analyze_maneuver(self):
        dv, dw = self.state.get_deltas()
        self.dynamics.evaluate(dv, dw)

    def summarize_results(self):
        self.post_processor.plot_burn_profiles()
        self.post_processor.plot_mass_usage()
        self.orbital_transfer.summarize()
