"""
Nomenclature
------------

A = normalization constant for a plume model
D = nozzle diameter, m
f = Maxwellian velocity distribution function at nozzle exit, s^3/m^3
Kn = Knudsen number
n = number density, m^-3
R_0 = nozzle radius, m
r = radial direction in a cylindrical coordinate system, m
S_0 = speed ratio
R = specific gas constant, kJ/(kg * K)
T = temperature, K
T_c = chamber temperature, K
P_c = chamber pressure kN/m^2
U, V, W = macroscopic velocity components, m/s
u, v, w = thermal velocity components, m/s
V_r = velocity component along the radial direction in a spherical coordinate system, m/s
U* = sonic velocity m/s
U_t = limiting velocity m/s
X, Y, Z = cartesian coordinates, m
beta = 1/2RT, sec^2/m^2 (beta = m / 2kT == 1 / 2RT; where k is the boltzmann constant)
gamma = specific heat ratio
epsilon = angular or azimuthal angle
theta = arctan(sqrt(Y^2 + Z^2)/X), general zenith angle with general point coordinates (X, Y, Z)
theta_max = limiting turning angle
kappa = plume model beaming exponent
rho_s = plume model nozzle throat density, kg/m3
psi = arctan(Z/X), specific zenith angle formed by points (0,0,0), (X, 0, Z), and x axis
Omega = integral domain
0 = properties at nozzle exit
1 = flowfield properties
' = simplified analytical results
"""

import warnings

import numpy as np
from scipy import integrate
from scipy.special import erf

#define constants
AVOGADROS_NUMBER = 6.0221e23
GAS_CONSTANT = 8.314


def get_K_factor(Q, S_0):
    '''
        Scaled special factor exp(-S_0^2) * K [Cai & Wang 2012, Eq. 10].

        The exp(-S_0^2) prefactor of the field solutions (Eqs. 5-8, 14)
        is combined analytically with Eq. 10's exp(Q * S_0^2) into
        exp(-S_0^2 * (1 - Q)). Since Q <= 1 both exponentials are <= 1,
        so this never overflows for large speed ratios, while the plain
        exp(Q * S_0^2) would. The [1 + erf(...)] factor is bounded by 2
        and needs no scaling.

        Parameters
        ----------
        Q : float
            special factor Q (full or simplified), 0 < Q <= 1
        S_0 : float
            molecular speed ratio at the nozzle exit

        Returns
        -------
        float
            exp(-S_0^2) * K(Q, S_0)
    '''
    erf_term = (1 + erf(S_0 * np.sqrt(Q))) * np.exp(-S_0 ** 2 * (1 - Q))
    term1 = Q * S_0 * np.exp(-S_0 ** 2)
    term2 = (0.5 + Q * S_0 ** 2) * np.sqrt(np.pi * Q)
    return Q * (term1 + term2 * erf_term)


def get_M_factor(Q, S_0):
    '''
        Scaled special factor exp(-S_0^2) * M [Cai & Wang 2012, Eq. 11].

        Overflow-safe exponential combination as in get_K_factor.

        Parameters
        ----------
        Q : float
            special factor Q (full or simplified), 0 < Q <= 1
        S_0 : float
            molecular speed ratio at the nozzle exit

        Returns
        -------
        float
            exp(-S_0^2) * M(Q, S_0)
    '''
    erf_term = (1 + erf(S_0 * np.sqrt(Q))) * np.exp(-S_0 ** 2 * (1 - Q))
    term1 = (1 + Q * S_0 ** 2) * np.exp(-S_0 ** 2)
    term2 = S_0 * (1.5 + Q * S_0 ** 2) * np.sqrt(np.pi * Q)
    return Q ** 2 * (term1 + term2 * erf_term)


def get_N_factor(Q, S_0):
    '''
        Scaled special factor exp(-S_0^2) * N [Cai & Wang 2012, Eq. 12].

        Overflow-safe exponential combination as in get_K_factor.

        Parameters
        ----------
        Q : float
            special factor Q (full or simplified), 0 < Q <= 1
        S_0 : float
            molecular speed ratio at the nozzle exit

        Returns
        -------
        float
            exp(-S_0^2) * N(Q, S_0)
    '''
    erf_term = (1 + erf(S_0 * np.sqrt(Q))) * np.exp(-S_0 ** 2 * (1 - Q))
    term1 = S_0 * Q ** 2 * (1.25 + Q * S_0 ** 2 / 2) * np.exp(-S_0 ** 2)
    term2 = 0.5 * np.sqrt(np.pi * Q ** 3)
    term3 = 0.75 + 3 * Q * S_0 ** 2 + Q ** 2 * S_0 ** 4
    return term1 + term2 * term3 * erf_term


def get_Q_full(r, epsilon, X, Z):
    '''
        Full special factor Q [Cai & Wang 2012, Eq. 9] in closed form.

        Eq. 9 defines Q = cos^2(psi) * [sum_n P_n(sin(psi) sin(epsilon))
        * (r/sqrt(X^2+Z^2))^n]^2 with P_n the Legendre polynomials. The
        series is the Legendre generating function sum_n P_n(x) t^n =
        1/sqrt(1 - 2*x*t + t^2) evaluated at x = sin(psi)sin(epsilon),
        t = r/sqrt(X^2+Z^2), which collapses (with cos^2(psi) =
        X^2/(X^2+Z^2)) to

            Q = X^2 / (X^2 + Z^2 - 2*Z*r*sin(epsilon) + r^2)

        so no series truncation or convergence loop is needed. The
        denominator equals X^2 + (Z - r*sin(epsilon))^2 +
        (r*cos(epsilon))^2 >= X^2 > 0, hence 0 < Q <= 1 always, which
        also guarantees the overflow-safe exponential combination in
        get_K_factor. On the centerline (Z = 0) this reduces to
        Q(r) = X^2/(X^2 + r^2).

        Parameters
        ----------
        r : float or ndarray
            radial integration variable over the exit disk, 0 <= r <= R_0 (m)
        epsilon : float or ndarray
            angular integration variable, -pi/2 <= epsilon <= pi/2 (rad)
        X : float
            axial coordinate of the field point, X > 0 (m)
        Z : float
            transverse coordinate of the field point (m)

        Returns
        -------
        float or ndarray
            special factor Q at (r, epsilon) for field point (X, 0, Z)
    '''
    return X ** 2 / (X ** 2 + Z ** 2 - 2 * Z * r * np.sin(epsilon) + r ** 2)


def get_far_field_velocity_normalized(S_0):
    '''
        Far-field centerline velocity asymptote, lim U_1 * sqrt(beta_0)
        as X -> infinity [Cai & Wang 2012, Eqs. 22 and 24]. A function of
        the exit speed ratio only: far enough from the exit the problem
        degenerates to a small-hole effusion flow, so no nozzle geometry
        factors remain.

        Evaluated by dividing numerator and denominator through by
        [1 + erf(S_0)] * exp(S_0^2), which would itself overflow for
        S_0 >~ 26; the residual factor exp(-S_0^2) / (1 + erf(S_0)) is
        bounded for S_0 > 0.

        Parameters
        ----------
        S_0 : float
            molecular speed ratio at the nozzle exit, S_0 > 0

        Returns
        -------
        float
            lim (X -> infinity) U_1(X, 0, 0) * sqrt(beta_0)
    '''
    inv_E = np.exp(-S_0 ** 2) / (1 + erf(S_0))
    sqpi = np.sqrt(np.pi)
    return S_0 + (inv_E + sqpi * S_0) / (S_0 * inv_E + (0.5 + S_0 ** 2) * sqpi)


def get_far_field_temp_ratio(S_0):
    '''
        Far-field centerline temperature asymptote, lim T_1/T_0 as
        X -> infinity [Cai & Wang 2012, Eqs. 23-24]. A function of the
        exit speed ratio only (see get_far_field_velocity_normalized).

        Implemented as -2/3 * G^2 + 4*N(Q=1)/(3*K(Q=1)) with G of
        Eq. 24, i.e. the Q -> 1 limit of Eq. 17; this reads Eq. 23's
        printed denominator "3 S_0 + (1/2 + S_0^2) sqrt(pi) [...]" with
        the 3 distributing over the whole denominator, 3*K(Q=1). The
        equivalence was verified independently to 40 digits. Overflow
        safety as in get_far_field_velocity_normalized.

        Parameters
        ----------
        S_0 : float
            molecular speed ratio at the nozzle exit, S_0 > 0

        Returns
        -------
        float
            lim (X -> infinity) T_1(X, 0, 0) / T_0
    '''
    inv_E = np.exp(-S_0 ** 2) / (1 + erf(S_0))
    sqpi = np.sqrt(np.pi)
    G = get_far_field_velocity_normalized(S_0)
    num = S_0 * (5 + 2 * S_0 ** 2) * inv_E + 2 * sqpi * (0.75 + 3 * S_0 ** 2 + S_0 ** 4)
    den = 3 * (S_0 * inv_E + (0.5 + S_0 ** 2) * sqpi)
    return -(2 / 3) * G ** 2 + num / den


def get_maxwellian_pressure(rho_inf, U, S, sigma, theta, T, T_w):
    '''
        Rarefied Gas Dynamics - Shen - eq. 4.19
        Gas-surface interaction model for pressure based on Maxwell's model.
        Assumes that the pressure experienced on the surface is a function of the wall temperature.
        Sigma is the fraction of reflections which are diffusive as opposed to specular.

        Parameters
        ----------
        rho_inf : float
                macroscopic density of the flowfield (kg / m^3)
        U : float
                gas velocity relative to the surface element (m / s)
        S : float
                speed ratio of the gas relative to the surface element
        sigma : float
                [0, 1], portion of molecules reflected diffusely
        theta : float
                plume centerline off-angle of current position (rad)
        T : float
                temperature of the oncoming gas flow (K)
        Tw : float
                temperature of the surface element.
                diffuse reflections correspond to temperature Tr,
                assumed to be the wall temperature, Tw (K)
        
        Returns
        -------
        float
            the pressure exerted on the surface element (N / m^2)
    '''

    p1 = ((2 - sigma) * (S * np.cos(theta)) / (np.sqrt(np.pi))) + ((sigma * np.sqrt(T_w)) / (2 * np.sqrt(T)))
    p1 *= np.exp(- (S * np.cos(theta)) ** 2)
    p2 = (2 - sigma) * ((S * np.cos(theta)) ** 2 + 0.5)
    p2 += (S * np.cos(theta) * (sigma / 2) * np.sqrt(np.pi * T_w / T))
    p2 *= 1 + erf(S * np.cos(theta))
    p = p1 + p2 
    p *= (rho_inf * U ** 2) / (2 * S ** 2)
    return p

def get_maxwellian_shear_pressure(rho_inf, U, S, sigma, theta):
    '''
        Rarefied Gas Dynamics - Shen - eq. 4.20
        Gas-surface interaction model for shear pressure based on Maxwell's model.
        Sigma is the fraction of reflections which are diffusive as opposed to specular.

        Parameters
        ----------
        rho_inf : float
                macroscopic density of the flowfield (kg / m^3)
        U : float
                gas velocity relative to the surface element (m / s)
        S : float
                speed ratio of the gas relative to the surface element
        sigma : float
                [0, 1], portion of molecules reflected diffusely
        theta : float
                plume centerline off-angle of current position (rad)
        
        Returns
        -------
        float
            the shear pressure exerted on the surface element (N / m^2)
    '''

    tau1 = np.exp(- (S * np.cos(theta)) ** 2)
    tau2 = np.sqrt(np.pi) * S * np.cos(theta)
    tau2 *= (1 + erf(S * np.cos(theta)))
    tau = tau1 + tau2
    tau *= -(sigma * rho_inf * np.sin(theta) * U ** 2) / (2 * np.sqrt(np.pi) * S)
    return tau

def get_maxwellian_heat_transfer(rho_inf, S, sigma, theta, T, T_r, R, gamma):
    '''
        Rarefied Gas Dynamics - Shen - eq. 4.45'
        Gas-surface interaction model for heat transfer based on Maxwell's model.
        Assumes the heat flux experienced on the surface is a function of the wall temperature.
        Sigma is the fraction of reflections which are diffusive as opposed to specular.

        Parameters
        ----------
        rho_inf : float
                macroscopic density of the flowfield (kg / m^3)
        S : float
                speed ratio of the gas relative to the surface element
        sigma : float
                [0, 1], portion of molecules reflected diffusely
        theta : float
                plume centerline off-angle of current position (rad)
        T : float
                temperature of the oncoming gas flow (K)
        Tr : float
                temperature of the surface element.
                diffuse reflections correspond to temperature Tr,
                assumed to be the wall temperature, Tw (K)
        R : float
                specific gas constant (J / kg * K)
        gamma : float
                ratio of specific heat capacities
        
        Returns
        -------
        float
            the pressure exerted on the surface element (N / m^2)
    '''
    q = (S ** 2) + (gamma / (gamma - 1)) - (((gamma + 1) * T_r) / (2 * (gamma - 1) * T))
    q *= np.exp(- (S * np.cos(theta)) ** 2) + (np.sqrt(np.pi) * (S * np.cos(theta)) * (1 + erf(S * np.cos(theta))))
    q -= 0.5 * np.exp(- (S * np.cos(theta)) ** 2)
    q *= sigma * rho_inf * R * T * np.sqrt(R * T / (2 * np.pi))
    return q

class Simons:
    '''
    Class responsible for solving gas kinetics with cosine law.
    Assumes flow in the boundary is inviscid.

    TODO fix broken plots and addplotting for pressures + broken when gamma = 2?

    Attributes
    ----------

    gamma : float
        Ratio of specific heats of the gas.
    R: float
        Specific gas constant (J / kg * K)
    T_c : float
        Chamber temperature of the thruster (K).
    P_c : float
        Chamber pressure of the thruster (N / m^2).
    r : float
        Distance from the evaluated point to the nozzle exit center (m)
    R_0 : float
        Nozzle exit radius (m).
    A : float
        normalization constant

    Methods
    -------
    get_nozzle_throat_density()
        Returns density at the throat, from chamber pressure/temp, and specific gas constant.
        Assumes isentropic flow from chamber to throat.

    get_limiting_turn_angle()
        Solve for limiting turn angle from specific heat ratio [Lumpkin 1999].

    get_plume_angular_density_decay_function(theta)
       Solve for the density decay function at a given off-centerline angle [Cai 2012].
       Expression for kappa is from Boyton 1967/68. 

    get_normalization_constant()
        Solve for normalization constant for the plume [Lumpkin 1999].

    get_sonic_velocity()
        Returns the sonic velocity of a flow, assuming isentropic flow from the chamber to the throat.

    get_limiting_velocity()
        Returns the limiting velocity of the flow.

    get_num_density_ratio(theta)
        Number density from continuity equation with constant mass flux across different spherical surfaces.
        Return the ratio of number density at an analyzed point outside of the exit vs at the exit.
    '''
    def __init__(self, gamma, R, T_c, P_c, R_0, r, kappa=None):
        '''
            Simple constructor, saves parameters to self.

            Parameters
            ----------
            gamma : float
                    ratio of specific heat capacities
            R : float
                    specific gas constant (J / kg * K)
            T_c : float
                    chamber temperature of the thruster (K)
            P_c : float
                    chamber pressure of the thruster (N / m^2)
            R_0 : float
                    nozzle exit radius (m)
            r : float
                    distance from the evaluated point
                    to the nozzle exit center (m)
            kappa : float, optional
                    plume beaming exponent for the cosine-law decay
                    function [Cai & Wang 2012, Eq. 26]. Defaults to
                    Boyton's kappa = 2/(gamma - 1) [Cai & Wang 2012,
                    Sec. II.C, refs. 22-23]. Paper-cited alternatives:
                    kappa = 2 (Ashkenas & Sherman, ref. 20) and
                    kappa = 1/(gamma - 1) (Albini, ref. 21).

            Returns
            -------
            None.
        '''
        self.gamma = gamma
        self.R = R
        #???should i also include throat temp and throat pressure for consistency???
        self.T_c = T_c
        self.P_c = P_c
        self.r = r
        self.R_0 = R_0
        if kappa is None:
            kappa = 2 / (gamma - 1) #Boyton 1967/68 from Cai2012 [22][23]
        self.kappa = kappa

        self.set_normalization_constant()

    def get_nozzle_throat_density(self):
        '''
            Returns density at the throat, from chamber pressure/temp, and specific gas constant.
            Assumes isentropic flow from chamber to throat.

            Parameters
            ----------

            Returns
            -------
            float
                density of gas at the nozzle throat (kg / m^3)
        '''
        #ideal gas law P_throat = rho_throat * R * T_throat
        #therefore: rho_throat = P_throat / (R * T_throat)
        #isentropic ratios at M = 1: T*/T_c = 2/(gamma+1),
        #P*/P_c = (2/(gamma+1))^(gamma/(gamma-1))
        T_throat = self.T_c * (2 / (self.gamma + 1))
        P_throat = self.P_c * (2 / (self.gamma + 1)) ** (self.gamma / (self.gamma - 1))
        rho_throat = P_throat / (self.R * T_throat)
        return rho_throat

    def get_limiting_turn_angle(self):
        '''
            From Lumpkin 1999. Solve for limiting turn angle from specific heat ratio.

            Parameters
            ---------

            Returns
            -------
            float
                limiting turning angle (rad)
        '''
        theta_max = (np.pi / 2) * (np.sqrt((self.gamma + 1) / (self.gamma - 1)) - 1)
        return theta_max

    def get_plume_angular_density_decay_function(self, theta):
        '''
            Solve for the density decay function at a given off-centerline
            angle [Cai & Wang 2012, Eq. 26], f(theta) =
            cos^kappa(pi*theta/(2*theta_max)) with the beaming exponent
            kappa chosen at construction (default Boyton 2/(gamma-1)).

            For theta >= theta_max the plume model region is empty (the
            gas cannot turn past the limiting angle), so the decay
            function is 0. Without this guard the cosine goes negative
            and fractional kappa would produce complex numbers.

            Parameters
            ----------
            theta : float
                    plume centerline off-angle of current position (rad)

            Returns
            -------
            float
                evaluation of plume angular density decay function at theta
        '''
        theta_max = self.get_limiting_turn_angle()
        if theta >= theta_max:
            return 0.0
        f = (np.cos((np.pi / 2) * (theta / theta_max))) ** self.kappa
        return f

    def set_normalization_constant(self):
        '''
            From Lumpkin 1999. Setter for normalization constant.
            Numerically integrates sin(theta) * cos^kappa(pi*theta/(2*theta_max))
            from 0 to the limiting turn angle. The integrand is real and
            non-negative on [0, theta_max] for any kappa, so no complex
            arithmetic can arise.

            Parameters
            ----------
            None.

            Returns
            -------
            None.
        '''
        theta_max = self.get_limiting_turn_angle()
        kappa = self.kappa

        def integrand(theta):
            return np.sin(theta) * np.cos((np.pi / 2) * (theta / theta_max)) ** kappa

        integral, _ = integrate.quad(integrand, 0, theta_max)
        self.A = 0.5 * np.sqrt((self.gamma - 1) / (self.gamma + 1)) / integral
        return
    
    def get_sonic_velocity(self):
        '''
            Method that returns the sonic velocity of a flow given specific heat ratio,
            specific gas constant, and chamber temperature.
            This assumes isentropic flow from the chamber to the throat.

            Parameters
            ----------

            Returns
            -------
            float
                sonic velocity (m/s)
        '''
        T_throat = self.T_c * (2 / (self.gamma + 1)) #isentropic ratio at M = 1
        sonic_velocity = np.sqrt(self.gamma * self.R * T_throat)
        return sonic_velocity

    def get_limiting_velocity(self):
        '''
            Calculates the limiting velocity of the plume. This is based on the 
            specific heat ratio and the sonic velocity.

            Paramters
            ---------
            None.

            Returns
            -------
            float
                limiting velocity of the flow
        '''
        sonic_velocity = self.get_sonic_velocity()
        U_t = np.sqrt((self.gamma + 1) / (self.gamma - 1)) * sonic_velocity
        return U_t

    '''
    def get_static_pressure(self, rho_ratio):
            #TODO - determine where this formula came from?
            #Returns static pressure based on density and limiting velocity.

        rho = rho_ratio * self.get_nozzle_throat_density()
        P_static = 1/3 * rho * self.get_limiting_velocity()
        return P_static
    '''

    def get_num_density_ratio(self, theta):
        '''
            Number density from continuity equation with constant mass flux
            across different spherical surfaces [Cai & Wang 2012, Eq. 25].

            NOTE: this ratio is THROAT-referenced, n/n_s: the returned
            value normalizes by the number density at the nozzle throat
            (rho_s in the paper), not at the nozzle exit. For a ratio
            comparable to the gas-kinetic classes (which normalize by the
            exit density n_0), use get_num_density_ratio_exit.

            Returns 0.0 for theta >= theta_max (empty plume region).

            Parameters
            ----------
            theta : float
                    Angle off centerline of the current point being analyzed (rad).

            Returns
            -------
            float
                number density at the analyzed point normalized by the
                nozzle THROAT number density, n/n_s
        '''
        theta_max = self.get_limiting_turn_angle()
        if theta >= theta_max:
            return 0.0
        f = self.get_plume_angular_density_decay_function(theta)
        rho_ratio = self.A * ((self.R_0/self.r) ** 2) * f #rho_ratio = density / nozzle throat denisty aka rho / rho_s
        n_ratio = rho_ratio
        return n_ratio

    def get_num_density_ratio_exit(self, theta, exit_mach):
        '''
            Exit-referenced number density ratio n/n_0.

            Rescales the throat-referenced Eq. 25 result by the isentropic
            density ratio between the throat (M = 1) and the nozzle exit
            (M = exit_mach):

                n_s / n_0 = [(1 + (gamma-1)/2 * M_e^2) / ((gamma+1)/2)]^(1/(gamma-1))

            This makes the Simons model directly comparable to the
            gas-kinetic classes (SimplifiedGasKinetics,
            CollisionlessGasKinetics), which normalize by the exit
            number density n_0.

            Parameters
            ----------
            theta : float
                    Angle off centerline of the current point being analyzed (rad).
            exit_mach : float
                    Mach number at the nozzle exit plane.

            Returns
            -------
            float
                number density at the analyzed point normalized by the
                nozzle EXIT number density, n/n_0
        '''
        throat_to_exit = ((1 + (self.gamma - 1) / 2 * exit_mach ** 2)
                          / ((self.gamma + 1) / 2)) ** (1 / (self.gamma - 1))
        return self.get_num_density_ratio(theta) * throat_to_exit

# TODO save plume constants into self
class SimplifiedGasKinetics:
    '''
        Class responsible for solving gas kinetics with a simplified, collisionless,
        free-jet, expansion from a round exit into a vacuum.
        From Cai 2012.
        Assumes special factor Q simplifies to Q' = X^2 / (X^2 + Z^2).

        Attributes
        ----------

        distance : float
            distance from the nozzle exit center to the point being analyzed (m)
        
        theta : float
            plume centerline off-angle of current position (rad)
        
        X : float
            the x-coorinate of the point being analyzed (m)

        Z : float
            the y-coordinate of the point being analyzed (m)

        R_0 : float
            the thruster nozzle exit radius (m)

        U_0 : float
            the macroscopic exit velocity (m/s)
        
        R : float
            specific gas constant (J / kg * K)

        gamma : float
            ratio of specific heats of the gas

        T_0 : float
            the macroscopic exit temperature (K)

        n_0 : float
            the number density at the nozzle exit (# particles / m^3)

        molar_mass : float
            molar mass of the fluid (kg/mol)

        beta_0 : float
            sqrt(1 / 2RT_0) | (m/s)

        S_0 : float
            molecular speed ratio at the nozzle exit

        Q_simple : float
            special factor Q, simplified [Cai 2012, II.B]

        K_simple : float
            special factor K, simplified [Cai 2012, II.B]

        M_simple : float
            special factor M, simplified [Cai 2012, II.B]

        N_simple : float
            special factor N, simplified [Cai 2012, II.B]
        
        Methods
        --------
        set_thruster_characteristics(TODO)

        get_beta(T)

        get_speed_ratio(U, beta)

        set_Q_simple()

        set_K_simple()

        set_M_simple()

        set_N_simple()

        get_num_density_ratio()

        get_U_normalized()

        get_W_normalized()

        get_temp_ratio()

        get_num_density_centerline()

        get_velocity_centerline()

        get_temp_centerline()

        get_pressure()

        get_heat_flux()
    '''
    #maybe group the special factors into one method and return an array of them?
    def __init__(self, distance, theta, thruster_characteristics, T_w, sigma):
        '''
            Simple constructor. Can be reworked to save constants for a plume.
        '''

        # save information about the coordinate being analyzed
        self.distance = distance
        self.theta = theta
        self.X = distance * np.cos(theta)
        self.Z = distance * np.sin(theta)

        # save thruster-specific characteristics
        self.set_thruster_characteristics(thruster_characteristics)

        # save gas kinetic special factors
        self.set_Q_simple()
        self.set_K_simple()
        self.set_M_simple()
        self.set_N_simple()

        # save gas-surface interaction parameters
        self.T_w = T_w
        self.sigma = sigma

        return

    def set_thruster_characteristics(self, thruster_characteristics):
        '''
            Setter for thruster-specific characteristics.

            Parameters
            ----------
            thruster_characteristics : TODO
                stores thruster-specific characteristics:
                R0, U0, R, gamma, T0, n0.

            Returns
            -------
            None.
        '''
        
        self.R_0 = float(thruster_characteristics['d'] / 2)
        self.U_0 = float(thruster_characteristics['ve'])
        self.R = float(thruster_characteristics['R'])
        self.gamma = float(thruster_characteristics['gamma'])
        self.T_0 = float(thruster_characteristics['Te'])
        self.n_0 = float(thruster_characteristics['n'])
        self.molar_mass = GAS_CONSTANT / self.R # kg/mol

        self.beta_0 = self.get_beta(self.T_0)
        self.S_0 = self.get_speed_ratio(self.U_0, self.beta_0)

        return

    def get_beta(self, T):
        '''
            Solves for beta at a given temperature.

            Parameters
            ----------
            T : float
                temperature (K)

            Returns
            -------
            float
                beta at the given temperature
        '''
        beta = 1 / np.sqrt(2 * self.R * T)
        return beta

    def get_speed_ratio(self, U, beta):
        '''
            Method to solve for the speed ratio of the flow at a specified flow.

            Parameters
            ----------
            U : float
                macroscopic velocity magnitude of the flow (m/s)
            
            beta : float
                beta (m^-1 / s^-1)
            
            Returns
            -------
            float
                speed ratio of the flow
        '''
        S = U * beta
        return S

    def set_Q_simple(self):
        '''
            Setter for Q in its simplified form. Returns Q'.

            Parameters
            ----------
            None.

            Returns
            -------
            None.
        '''
        Q_simple = self.X ** 2 / (self.X ** 2 + self.Z ** 2)
        self.Q_simple = Q_simple

        return
    
    def set_K_simple(self):
        '''
            Setter for simplified special factor K [Cai & Wang 2012, Eq. 10]
            with Q substituted by Q'. Stored scaled by exp(-S_0^2) for
            overflow safety (see get_K_factor); ratio methods (U, W, T)
            are unaffected since the scaling cancels, and the density
            method uses the scaled factor directly.

            Parameters
            ----------
            None.

            Returns
            -------
            None.
        '''
        self.K_simple = get_K_factor(self.Q_simple, self.S_0)

        return

    def set_M_simple(self):
        '''
            Setter for simplified special factor M [Cai & Wang 2012, Eq. 11]
            with Q substituted by Q'. Stored scaled by exp(-S_0^2) for
            overflow safety (see get_K_factor).

            Paramters
            ---------
            None.

            Returns
            -------
            None.
        '''
        self.M_simple = get_M_factor(self.Q_simple, self.S_0)

        return

    def set_N_simple(self):
        '''
            Setter for simplified special factor N [Cai & Wang 2012, Eq. 12]
            with Q substituted by Q'. Stored scaled by exp(-S_0^2) for
            overflow safety (see get_K_factor).

            Parameters
            ----------
            None.

            Returns
            -------
            None.
        '''
        self.N_simple = get_N_factor(self.Q_simple, self.S_0)

        return
    
    def get_num_density_ratio(self):
        '''
            Method to calculate the number denisty at a point (X, 0, Z) outside of the nozzle.
            This density is normalized over the number density at the nozzle exit.

            Parameters
            ----------
            None.

            Returns
            -------
            float
                number density at a point (X, 0, Z) vs number density at the nozzle exit
        '''
        # num_density_ratio = n_1s(X, 0, Z) / n_0 [Cai & Wang 2012, Eq. 14]
        # K_simple already carries the exp(-S_0^2) prefactor (see set_K_simple)
        num_density_ratio = (self.K_simple / (2 * np.sqrt(np.pi)) * (self.R_0 / self.X) ** 2)
        return num_density_ratio
    
    def get_U_normalized(self):
        '''
            Method to calculate the macroscopic x-component of velocity at a point (X, 0, Z) outside of the nozzle.
            This velocity component is normalized with the parameter beta at the exit. 
            Beta relates velocity to speed ratio.

            Parameters
            ----------
            None.

            Returns
            -------
            float
                returns U normalized
        '''
        # U_normalized = U_1s (X, 0, Z) * sqrt(beta) [Cai & Wang 2012, Eq. 15]
        U_normalized = self.M_simple / self.K_simple
        return U_normalized
    
    def get_W_normalized(self):
        '''
            Method to calculate the macroscopic z-component of velocity at a point (X, 0, Z) outside of the nozzle.
            This velocity component is normalized with the parameter beta at the exit. 
            Beta relates velocity to speed ratio.

            Parameters
            ----------
            None.

            Returns
            -------
            float
                returns W normalized
        '''
        # W_normalized = W_1s (X, 0, Z) * sqrt(beta) [Cai & Wang 2012, Eq. 16]
        W_normalized = (self.M_simple / self.K_simple) * (self.Z / self.X)
        return W_normalized

    def get_temp_ratio(self):
        '''
            Method to calculate the temperature at a point (X, 0, Z) outside of the nozzle.
            This temperature is normalized over the temperature at the nozzle exit.

            Parameters
            ----------
            None.

            Returns
            -------
            float
                ratio of temperature at a point (X, 0, Z) to the temperature at the nozzle exit
        '''
        # T_ratio = T_1s / T_0 [Cai & Wang 2012, Eq. 17]
        T_ratio = ((-2 * self.M_simple ** 2) / (3 * self.Q_simple * self.K_simple ** 2))
        T_ratio += (4 * self.N_simple / (3 * self.K_simple))
        return T_ratio
    
    def get_num_density_centerline(self):
        '''
            Method to calculate the number denisty at a point (X, 0, 0) outside of the nozzle.
            This density is normalized over the number density at the nozzle exit.

            Parameters
            ----------
            None.

            Returns
            -------
            float
                number density at a point on the centerline
                vs the number density at the nozzle exit
        '''
        # n_1(X, 0, 0) / n_0 [Cai & Wang 2012, Eq. 18]
        p1 = self.X / np.sqrt(self.X ** 2 + self.R_0 ** 2)
        p2 = self.R_0 / np.sqrt(self.X ** 2 + self.R_0 ** 2)
        n_ratio = 0.5 + 0.5 * erf(self.S_0) - (p1 * np.exp(-self.S_0 ** 2 * p2 ** 2) / 2) * (1 + erf(p1 * self.S_0))

        return n_ratio
    
    def get_velocity_centerline(self):
        '''
            Method to calculate the macroscopic velocity at a point (X, 0, 0) outside of the nozzle.
            This velocity is normalized with the parameter beta at the exit. 
            Beta relates velocity to speed ratio.

            TODO split U_ratio to multiple lines

            Parameters
            ----------
            None.

            Returns
            -------
            float
                velocity at a point on the centerline (X, 0, 0) normalized
                with the parameter beta at the exit
        '''
        # U_1(X, 0, 0) * sqrt(beta_0) [Cai & Wang 2012, Eq. 19]
        p1 = self.X / np.sqrt(self.X ** 2 + self.R_0 ** 2)
        p2 = self.R_0 / np.sqrt(self.X ** 2 + self.R_0 ** 2)
        n_ratio = self.get_num_density_centerline()
        U_ratio = 1 / (2 * n_ratio) * ((p2 ** 2 * np.exp(- self.S_0 ** 2) / np.sqrt(np.pi)) + (self.S_0 * (1 + erf(self.S_0))) - (np.exp(- p2 ** 2 * self.S_0 ** 2) * p1 ** 3 * self.S_0 * (1 + erf(p1 * self.S_0))))
        return U_ratio
    
    def get_temp_centerline(self):
        '''
            Method to calculate the temperature at a point (X, 0, 0) outside of the nozzle.
            This temperature is normalized over the temperature at the nozzle exit.

            Parameters
            ----------
            None.

            Returns
            -------
            float
                temperature on the point on the centerline (X, 0, 0) 
                vs temperature at the nozzle exit
        '''
        # T_1(X, 0, 0) / T_0 [Cai & Wang 2012, Eq. 21]
        # N_simple already carries the exp(-S_0^2) prefactor (see set_N_simple)
        n_ratio = self.get_num_density_centerline()
        U1 = self.get_velocity_centerline()
        integral = 0.5 * self.N_simple * self.R_0 ** 2
        temp_ratio = 4 / (3 * n_ratio * np.sqrt(np.pi) * self.X ** 2) * integral - (U1 ** 2 / (3/2))
        return temp_ratio
    
    def get_pressure(self):
        '''
            Method to call gas-surface interaction model. Passes thruster characteristics and
            normalized plume parameters to the Maxwell model to solve for pressre.

            Parameters
            ----------
            None.

            Returns
            -------
            float
                pressure on the surface outside of the nozzle exit (N / m^2)
        '''
        if self.theta != 0: # not on centerline
            n_inf = self.n_0 * self.get_num_density_ratio()
            rho_inf = n_inf * self.molar_mass / AVOGADROS_NUMBER
            T = self.T_0 * self.get_temp_ratio()

            u = self.get_U_normalized() / self.beta_0
            w = self.get_W_normalized() / self.beta_0
            U = np.sqrt(u ** 2 + w ** 2)
            beta = self.get_beta(T)
            S = U * beta

            pressure = get_maxwellian_pressure(rho_inf, U, S, self.sigma, self.theta, T, self.T_w)
            return pressure
        else: # on centerline
            n_inf = self.n_0 * self.get_num_density_centerline()
            rho_inf = n_inf * self.molar_mass / AVOGADROS_NUMBER

            T = self.T_0 * self.get_temp_centerline()

            U = self.get_velocity_centerline() / self.beta_0
            beta = self.get_beta(T)
            S = U * beta

            pressure = get_maxwellian_pressure(rho_inf, U, S, self.sigma, self.theta, T, self.T_w)
        return pressure
    
    def get_shear_pressure(self):
        '''
            Method to call gas-surface interaction model. Passes thruster characteristics and
            normalized plume parameters to the Maxwell model to solve for shear pressre.

            Parameters
            ----------
            None.

            Returns
            -------
            float
                shear pressure on the surface outside of the nozzle exit (N / m^2)
        '''
        if self.theta != 0: # not on centerline
            n_inf = self.n_0 * self.get_num_density_ratio()
            rho_inf = n_inf * self.molar_mass / AVOGADROS_NUMBER
            T = self.T_0 * self.get_temp_ratio()

            u = self.get_U_normalized() / self.beta_0
            w = self.get_W_normalized() / self.beta_0
            U = np.sqrt(u ** 2 + w ** 2)
            beta = self.get_beta(T)
            S = U * beta

            shear_pressure = get_maxwellian_shear_pressure(rho_inf, U, S, self.sigma, self.theta)
            return shear_pressure
        else: # on centerline
            n_inf = self.n_0 * self.get_num_density_centerline()
            rho_inf = n_inf * self.molar_mass / AVOGADROS_NUMBER

            T = self.T_0 * self.get_temp_centerline()

            U = self.get_velocity_centerline() / self.beta_0
            beta = self.get_beta(T)
            S = U * beta

            shear_pressure = get_maxwellian_shear_pressure(rho_inf, U, S, self.sigma, self.theta)
        return shear_pressure
    
    def get_heat_flux(self):
        '''
            Method to call gas-surface interaction model. Passes thruster characteristics and
            normalized plume parameters to the Maxwell model to solve for heat flux.

            Parameters
            ----------
            None.

            Returns
            -------
            float
                heat flux on the surface outside of the nozzle exit (W / m^2)
        '''
        if self.theta != 0: # not on centerline
            n_inf = self.n_0 * self.get_num_density_ratio()
            rho_inf = n_inf * self.molar_mass / AVOGADROS_NUMBER

            T = self.T_0 * self.get_temp_ratio()

            u = self.get_U_normalized() / self.beta_0
            w = self.get_W_normalized() / self.beta_0
            U = np.sqrt(u ** 2 + w ** 2)
            beta = self.get_beta(T)
            S = U * beta

            heat_flux = get_maxwellian_heat_transfer(rho_inf, S, self.sigma, self.theta, T, self.T_w, self.R, self.gamma)
            return heat_flux
        else: # on centerline
            n_inf = self.n_0 * self.get_num_density_centerline()
            rho_inf = n_inf * self.molar_mass / AVOGADROS_NUMBER

            T = self.T_0 * self.get_temp_centerline()

            U = self.get_velocity_centerline() / self.beta_0
            beta = self.get_beta(T)
            S = U * beta

            heat_flux = get_maxwellian_heat_transfer(rho_inf, S, self.sigma, self.theta, T, self.T_w, self.R, self.gamma)
        return heat_flux


class CollisionlessGasKinetics(SimplifiedGasKinetics):
    '''
        Full collisionless analytical plume model [Cai & Wang 2012,
        Sec. II.A, Eqs. 5-12]: a free jet expanding from a round exit
        into vacuum, evaluated at a point (X, 0, Z) in front of the
        nozzle (X > 0). Unlike the parent SimplifiedGasKinetics (which
        substitutes the far-field Q' of Eq. 13), this class integrates
        the exact special factor Q of Eq. 9 -- via its closed form, see
        get_Q_full -- over the finite exit disk, so it remains valid in
        the near field.

        The field solutions (Eqs. 5-8) are double integrals over
        r in [0, R_0] and epsilon in [-pi/2, pi/2]. They are evaluated
        with tensor-product Gauss-Legendre quadrature: the integrands
        are analytic on the compact rectangle (the denominator of Q is
        bounded below by X^2 > 0), so Gauss-Legendre converges
        geometrically. The order is doubled (40 -> 80 -> 160) until two
        successive orders agree to QUAD_RTOL; the finer result is kept.
        In practice order 40 already reaches machine precision except
        very near the nozzle lip (X -> 0, Z ~ R_0), where the density
        field has a singularity [Cai & Wang 2012, Sec. III].

        Centerline closed forms (Eqs. 18-21) and the Maxwell gas-surface
        interface are inherited from SimplifiedGasKinetics; the exact
        analytical solutions reduce to Eqs. 18-21 on the centerline, so
        the inherited methods are exact there. The inherited surface
        methods (get_pressure, get_shear_pressure, get_heat_flux)
        dispatch to this class's overridden field getters, so they are
        fed by the full-model n, U, W, T.

        Attributes
        ----------
        (all of SimplifiedGasKinetics, plus)

        I_K : float
            integral of r * exp(-S_0^2) * K over the exit disk [Eq. 5]

        I_M : float
            integral of r * exp(-S_0^2) * M over the exit disk [Eq. 6]

        I_W : float
            integral of (Z - r sin(epsilon)) * r * exp(-S_0^2) * M over
            the exit disk [Eq. 7]

        I_N : float
            integral of r * exp(-S_0^2) * N over the exit disk [Eq. 8]

        Methods
        -------
        get_num_density_ratio()

        get_U_normalized()

        get_W_normalized()

        get_Vr_normalized()

        get_temp_ratio()

        get_pressure_ratio()

        (get_pressure / get_shear_pressure / get_heat_flux and the
        centerline closed forms are inherited from SimplifiedGasKinetics)
    '''

    QUAD_ORDERS = (40, 80, 160)
    QUAD_RTOL = 1e-9

    def __init__(self, distance, theta, thruster_characteristics, T_w, sigma):
        '''
            Mirrors SimplifiedGasKinetics(distance, theta,
            thruster_characteristics, T_w, sigma) exactly; X = d*cos(theta)
            and Z = d*sin(theta) are derived internally. Additionally
            precomputes the four field integrals of Eqs. 5-8.
        '''
        super().__init__(distance, theta, thruster_characteristics, T_w, sigma)
        self.set_field_integrals()

        return

    def _compute_field_integrals(self, order):
        '''
            Evaluate the four exit-disk integrals of Eqs. 5-8 with a
            tensor-product Gauss-Legendre rule of the given order per
            axis, over r in [0, R_0] and epsilon in [-pi/2, pi/2].

            The K, M, N factors carry the exp(-S_0^2) prefactor of the
            field solutions (see get_K_factor), keeping every term
            bounded for large speed ratios.

            Parameters
            ----------
            order : int
                number of Gauss-Legendre nodes per axis

            Returns
            -------
            tuple of float
                (I_K, I_M, I_W, I_N)
        '''
        nodes, weights = np.polynomial.legendre.leggauss(order)
        # map [-1, 1] to [0, R_0] (radial) and [-pi/2, pi/2] (angular)
        r = 0.5 * self.R_0 * (nodes + 1)
        w_r = 0.5 * self.R_0 * weights
        eps = 0.5 * np.pi * nodes
        w_eps = 0.5 * np.pi * weights

        R, E = np.meshgrid(r, eps, indexing='ij')
        W2D = np.outer(w_r, w_eps)

        Q = get_Q_full(R, E, self.X, self.Z)
        K = get_K_factor(Q, self.S_0)
        M = get_M_factor(Q, self.S_0)
        N = get_N_factor(Q, self.S_0)

        I_K = np.sum(W2D * R * K)
        I_M = np.sum(W2D * R * M)
        I_W = np.sum(W2D * (self.Z - R * np.sin(E)) * R * M)
        I_N = np.sum(W2D * R * N)
        return I_K, I_M, I_W, I_N

    def set_field_integrals(self):
        '''
            Setter for the field integrals I_K, I_M, I_W, I_N of
            Eqs. 5-8, with quadrature-order doubling until two
            successive orders agree to QUAD_RTOL (see class docstring
            for the accuracy rationale). Warns if the finest order is
            reached without convergence (only possible extremely close
            to the nozzle-lip singularity).

            Parameters
            ----------
            None.

            Returns
            -------
            None.
        '''
        previous = None
        for order in self.QUAD_ORDERS:
            current = self._compute_field_integrals(order)
            if previous is not None and self._integrals_converged(previous, current):
                break
            previous = current
        else:
            warnings.warn(
                'CollisionlessGasKinetics quadrature did not converge to '
                'rtol={} at order {} for point (X={}, Z={}); using finest '
                'result.'.format(self.QUAD_RTOL, self.QUAD_ORDERS[-1],
                                 self.X, self.Z),
                RuntimeWarning)

        self.I_K, self.I_M, self.I_W, self.I_N = current

        return

    def _integrals_converged(self, previous, current):
        '''
            Convergence test between two quadrature orders. I_K, I_M and
            I_N are strictly positive, so a relative test applies; I_W
            can legitimately vanish (centerline, Eq. 20), so its
            difference is measured against the natural magnitude of its
            integrand, (|Z| + R_0) * I_M.

            Parameters
            ----------
            previous : tuple of float
                integrals from the coarser rule
            current : tuple of float
                integrals from the finer rule

            Returns
            -------
            bool
                True when every integral has converged to QUAD_RTOL
        '''
        rtol = self.QUAD_RTOL
        I_K0, I_M0, I_W0, I_N0 = previous
        I_K1, I_M1, I_W1, I_N1 = current
        scale_W = (abs(self.Z) + self.R_0) * abs(I_M1)
        return (abs(I_K1 - I_K0) <= rtol * abs(I_K1)
                and abs(I_M1 - I_M0) <= rtol * abs(I_M1)
                and abs(I_N1 - I_N0) <= rtol * abs(I_N1)
                and abs(I_W1 - I_W0) <= rtol * scale_W)

    def get_num_density_ratio(self):
        '''
            Number density at (X, 0, Z) normalized by the exit number
            density, n_1/n_0 [Cai & Wang 2012, Eq. 5]. The exp(-S_0^2)
            prefactor is carried inside I_K (see get_K_factor).

            Parameters
            ----------
            None.

            Returns
            -------
            float
                n_1(X, 0, Z) / n_0
        '''
        return self.I_K / (np.pi ** 1.5 * self.X ** 2)

    def get_U_normalized(self):
        '''
            Macroscopic x-velocity at (X, 0, Z) normalized by
            sqrt(beta_0), i.e. U_1 * sqrt(beta_0) [Cai & Wang 2012,
            Eq. 6]. The shared prefactor and n_0/n_1 reduce the ratio to
            I_M / I_K.

            Parameters
            ----------
            None.

            Returns
            -------
            float
                U_1(X, 0, Z) * sqrt(beta_0)
        '''
        return self.I_M / self.I_K

    def get_W_normalized(self):
        '''
            Macroscopic z-velocity at (X, 0, Z) normalized by
            sqrt(beta_0), i.e. W_1 * sqrt(beta_0) [Cai & Wang 2012,
            Eq. 7]. Eq. 7's integrand factor is read as
            (Z - r sin(epsilon)); the printed "(Z - r sin(theta))" is a
            typo -- sin(epsilon) is the convention consistent with Q
            (Eq. 9) and it makes W vanish on the centerline as Eq. 20
            requires. Eq. 7's extra 1/X (prefactor 1/X^3 vs 1/X^2)
            reduces the ratio to I_W / (X * I_K).

            Parameters
            ----------
            None.

            Returns
            -------
            float
                W_1(X, 0, Z) * sqrt(beta_0)
        '''
        return self.I_W / (self.X * self.I_K)

    def get_Vr_normalized(self):
        '''
            Radial (spherical, from the nozzle exit center) velocity
            component in the Y = 0 plane, normalized by sqrt(beta_0):
            V_r = (X*U + Z*W)/sqrt(X^2 + Z^2) [Cai & Wang 2012,
            Figs. 16-18].

            Parameters
            ----------
            None.

            Returns
            -------
            float
                V_r(X, 0, Z) * sqrt(beta_0)
        '''
        U = self.get_U_normalized()
        W = self.get_W_normalized()
        return (self.X * U + self.Z * W) / np.sqrt(self.X ** 2 + self.Z ** 2)

    def get_temp_ratio(self):
        '''
            Temperature at (X, 0, Z) normalized by the exit temperature,
            T_1/T_0 [Cai & Wang 2012, Eq. 8]. With beta_0 = 1/(2*R*T_0),
            the kinetic term -(U_1^2 + W_1^2)/(3*R*T_0) equals
            -(2/3) * [(U_1 sqrt(beta_0))^2 + (W_1 sqrt(beta_0))^2].

            Parameters
            ----------
            None.

            Returns
            -------
            float
                T_1(X, 0, Z) / T_0
        '''
        U = self.get_U_normalized()
        W = self.get_W_normalized()
        return -(2 / 3) * (U ** 2 + W ** 2) + (4 / 3) * self.I_N / self.I_K

    def get_pressure_ratio(self):
        '''
            Flowfield static pressure at (X, 0, Z) normalized by the exit
            static pressure: p_1/p_0 = (n_1/n_0) * (T_1/T_0) from the
            ideal gas law with the LOCAL temperature. Cai & Wang 2012
            (p. 64, Fig. 13 discussion) explicitly warn that computing
            the local pressure as n(X, 0, Z) * k * T_0 is theoretically
            invalid, because the flowfield temperature is always lower
            than the exit temperature T_0.

            Parameters
            ----------
            None.

            Returns
            -------
            float
                p_1(X, 0, Z) / p_0
        '''
        return self.get_num_density_ratio() * self.get_temp_ratio()
