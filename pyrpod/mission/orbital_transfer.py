from pyrpod.orbital import HohmannTransfer

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
            print(leg_id)
            print(result)
            result['leg_id'] = leg_id
        self.hohmann_transfers.append(result)

    def summarize_hohmann_transfers(self):
        """
        Prints a summary of all stored Hohmann transfers.
        """
        for i, t in enumerate(self.hohmann_transfers):
            print(f"Leg {i+1} ({t.get('leg_id', 'unnamed')}):")
            print(f"  Δv1 = {t['dv1']:.3f} km/s, Δv2 = {t['dv2']:.3f} km/s")
            print(f"  Total Δv = {t['dv_total']:.3f} km/s")
            print(f"  Time of Flight = {t['tof'] / 60:.2f} hours\n")


