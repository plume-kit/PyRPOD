from astropy import units as u
from astropy.constants import G, R_earth, M_earth
import numpy as np

class HohmannTransfer:
    def __init__(self, r1_km: float, r2_km: float, leg_id: str = ""):
        self.leg_id = leg_id or f"{r1_km} km → {r2_km} km"
        self.r1_km = r1_km
        self.r2_km = r2_km

        # Constants
        self.R_earth = R_earth.to(u.km)
        self.G = G.to(u.m**3 / (u.kg * u.s**2))
        self.M_earth = M_earth.to(u.kg)
        self.mu_earth = (self.G * self.M_earth).to(u.m**3 / u.s**2)

        # Orbital radii
        self.r1 = (self.R_earth + r1_km * u.km).to(u.m)
        self.r2 = (self.R_earth + r2_km * u.km).to(u.m)

        self._compute_transfer()

    def _compute_transfer(self):
        # Circular speeds
        self.v1_circ = np.sqrt(self.mu_earth / self.r1)
        self.v2_circ = np.sqrt(self.mu_earth / self.r2)

        # Transfer orbit geometry
        self.a_trans = (self.r1 + self.r2) / 2

        # Speeds on transfer orbit
        self.v_trans1 = np.sqrt(self.mu_earth * (2 / self.r1 - 1 / self.a_trans))
        self.v_trans2 = np.sqrt(self.mu_earth * (2 / self.r2 - 1 / self.a_trans))

        # ΔV calculations
        self.dv1 = (self.v_trans1 - self.v1_circ).to(u.km / u.s)
        self.dv2 = (self.v2_circ - self.v_trans2).to(u.km / u.s)
        self.dv_total = self.dv1 + self.dv2

        # Time of flight
        self.tof = (np.pi * np.sqrt(self.a_trans**3 / self.mu_earth)).to(u.minute)

        return self.summary(verbose=False)

    def summary(self, verbose=True) -> dict:
        if verbose:
            print(f"\n=== Hohmann Transfer: {self.leg_id} ===")
            print(f"Initial altitude: {self.r1_km} km → Radius: {self.r1.to(u.km):.2f}")
            print(f"Final altitude:   {self.r2_km} km → Radius: {self.r2.to(u.km):.2f}")
            print(f"Gravitational parameter μ: {self.mu_earth:.3e}")
            print(f"Circular velocities: v1 = {self.v1_circ.to(u.km/u.s):.4f}, v2 = {self.v2_circ.to(u.km/u.s):.4f}")
            print(f"Transfer semi-major axis: {self.a_trans.to(u.km):.2f}")
            print(f"Transfer velocities: v_trans1 = {self.v_trans1.to(u.km/u.s):.4f}, v_trans2 = {self.v_trans2.to(u.km/u.s):.4f}")
            print(f"ΔV1 (inject):   {self.dv1:.4f}")
            print(f"ΔV2 (circular): {self.dv2:.4f}")
            print(f"Total ΔV:       {self.dv_total:.4f}")
            print(f"Time of flight: {self.tof:.2f}")

        return {
            "leg_id": self.leg_id,
            "r1_km": self.r1_km,
            "r2_km": self.r2_km,
            "dv1": self.dv1,
            "dv2": self.dv2,
            "dv_total": self.dv_total,
            "tof": self.tof
        }
