from pyrpod.orbital import HohmannTransfer
from astropy import units as u
from pyrpod.logging_utils import get_logger

logger = get_logger("pyrpod.mission.orbital_transfer")

class OrbitalTransferEngine:
    def __init__(self):
        self.transfers = []

    def add_hohmann_transfer(self, h1, h2):
        # Add a Hohmann transfer case
        pass

    def summarize(self):
        # Print or return summary
        pass

    def init_hohmann_transfers(self):
        self.hohmann_transfers = []

    def add_hohmann_transfer(self, h1_km: float, h2_km: float, leg_id: str = None):
        """
        Computes and stores the delta-v and time of flight for a Hohmann transfer leg.

        Parameters:
            h1_km (float): Starting orbit altitude [km]
            h2_km (float): Ending orbit altitude [km]

            leg_id (str, optional): Descriptive name or identifier for the transfer leg
        """
        transfer = HohmannTransfer.HohmannTransfer(h1_km, h2_km, leg_id)
        result = transfer._compute_transfer()
        if leg_id:
            logger.info("Hohmann leg %s: %s", leg_id, result)
            result['leg_id'] = leg_id
        self.hohmann_transfers.append(result)

    def summarize_hohmann_transfers(self):
        """
        Prints a summary of all stored Hohmann transfers.
        """
        for i, t in enumerate(self.hohmann_transfers):
            dv1 = t['dv1'].to_value(u.km / u.s)
            dv2 = t['dv2'].to_value(u.km / u.s)
            dv_total = t['dv_total'].to_value(u.km / u.s)
            tof_hours = t['tof'].to_value(u.hour)

            logger.info(
                "Leg %s (%s): dv1=%.3f km/s, dv2=%.3f km/s, dv_total=%.3f km/s, ToF=%.2f hours",
                i + 1,
                t.get('leg_id', 'unnamed'),
                dv1, dv2, dv_total, tof_hours,
            )


