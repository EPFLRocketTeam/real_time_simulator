# Author : Michaël Tasev
# Last update : 16 October 2020
# EPFL Rocket Team, 1015 Lausanne, Switzerland

import numpy as np
import numpy.linalg as lin
import math
import time

from scipy.integrate import ode, solve_ivp
from dataclasses import dataclass

from Rocket.Stage import Stage
from Rocket.Rocket import Rocket
from Rocket.Body import Body
from Functions.Models.stdAtmosUS import stdAtmosUS
from Functions.Models.drag import drag
from Functions.Models.Nose_drag import Nose_drag
from Functions.Models.drag_shuriken import drag_shuriken
from Functions.Models.wind_model import wind_model
from Functions.Models.normal_lift import normal_lift
from Functions.Math.normalize_vector import normalize_vector
from Functions.Math.rotmat import rotmat
from Functions.Math.quat2rotmat import quat2rotmat
from Functions.Math.rot2anglemat import rot2anglemat
from Functions.Math.quat_evolve import quat_evolve
from Functions.Math.rot2quat import rot2quat
from Functions.Models.pitch_damping_moment import pitch_damping_moment
from Functions.Models.Mass_Non_Lin import Mass_Non_Lin
from Functions.Models.Thrust import Thrust
from Functions.Models.Mass_Properties import Mass_Properties


class Simulator3D:
    """

    """

    @dataclass
    class SimAuxResults:
        Margin: np.array(0)
        Alpha: np.array
        Cn_alpha: np.array
        Xcp: np.array
        Cd: np.array
        Mass: np.array
        CM: np.array
        Il: np.array
        Ir: np.array
        Delta: np.array
        Nose_Alpha: np.array
        Nose_delta: np.array

    global tmp_Margin, tmp_Alpha, tmp_Cn_alpha, tmp_Xcp, tmp_Cd, tmp_Mass, tmp_CM, tmp_Il, tmp_Ir, tmp_Delta
    global tmp_Nose_Alpha, tmp_Nose_Delta

    global simAuxResults
    simAuxResults = SimAuxResults(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    def __init__(self, rocket: Rocket, atmosphere: stdAtmosUS):
        self.x_0 = np.array([0, 0])
        self.t0 = 0
        self.state = [self.x_0]
        self.time = [self.t0]

        self.rocket = rocket
        self.Environment = atmosphere

    def Dynamics_Rail_1DOF(self, t, s):

        x = s[0]
        v = s[1]


        # Rocket inertia
        Mass, dMdt = Mass_Non_Lin(t, self.rocket)


        # Environment
        g = 9.81
        a = self.Environment.get_speed_of_sound(s[0] + self.Environment.ground_altitude)
        rho = self.Environment.get_density(s[0] + self.Environment.ground_altitude)
        nu = self.Environment.get_viscosity(s[0] + self.Environment.ground_altitude)

        # Gravity
        G = -g*np.cos(self.Environment.Rail_Angle)*Mass

        T = Thrust(t, self.rocket)

        # TODO: Add drag influences (done?)
        CD = drag(self.rocket, 0, v, nu, a)
        D = -0.5*rho*self.rocket.get_max_cross_section_surface*CD*v**2

        F_tot = G + T*self.rocket.get_motor_fac() + D

        x_dot = v
        v_dot = 1/Mass * (F_tot - v*dMdt)

        CD_AB = 0  # TODO: Insert reference to drag_shuriken or other


        return x_dot, v_dot

    def Compute_aero(self, s, thrust_force):

        x = s[0:3]
        v = s[3:6]
        q = s[6:10]
        w = s[10:13]
        propellant_mass = s[13]


        # Normalise quaternion
        q = normalize_vector(q)
        

        # Rotation matrix from rocket coordinates to Earth coordinates
        c = quat2rotmat(q)
        angle = rot2anglemat(c)

        # Rocket principle frame vectors expressed in Earth coordinates
        ya = c.dot(np.array([1, 0, 0]).transpose())  # Yaw axis
        pa = c.dot(np.array([0, 1, 0]).transpose())  # Pitch axis
        ra = c.dot(np.array([0, 0, 1]).transpose())  # Roll axis

        # Earth coordinates vectors expressed in Earth's frame
        xe = np.array([1, 0, 0]).transpose()
        ye = np.array([0, 1, 0]).transpose()
        ze = np.array([0, 0, 1]).transpose()

		# Mass properties				
        m = self.rocket.get_empty_mass() + propellant_mass
        dMdt = np.linalg.norm(thrust_force)/(self.rocket.get_motor_Isp()*9.81)
        cg = (self.rocket.get_dry_cg()*self.rocket.get_empty_mass() + self.rocket.get_propellant_cg()*propellant_mass)/m #from tip of nosecone
        Sm = self.rocket.get_max_cross_section_surface
        #I = c.transpose().dot(self.rocket.get_rocket_inertia()).dot(c)
        I = c.dot(self.rocket.get_rocket_inertia()).dot(c.transpose())

        # Environment
        g = 9.81  # Gravity [m/s^2]
        rho = self.Environment.get_density(x[2] + self.Environment.ground_altitude)

        nu = self.Environment.get_viscosity(x[2] + self.Environment.ground_altitude) # !!! take 200 us

        a = self.Environment.get_speed_of_sound(x[2] + self.Environment.ground_altitude)


        # Aerodynamic corrective forces --------------------
        # Compute center of mass angle of attack
        v_cm = v - self.Environment.get_V_inf()*self.Environment.V_dir


        v_cm_mag = np.linalg.norm(v_cm)
        alpha_cm = math.atan2(np.linalg.norm(np.cross(ra, v_cm)), np.dot(ra, v_cm)) # !!! take 200 us

        # Mach number
        Mach = v_cm_mag / a

        # Normal lift coefficient and center of pressure
        CNa, Xcp, CNa_bar, CP_bar = normal_lift(self.rocket, alpha_cm, 1.1, Mach, angle[2], 1)

        # Stability margin
        margin = Xcp - cg

        # Compute rocket angle of attack
        if np.linalg.norm(w) != 0:
            w_norm = w / np.linalg.norm(w)
        else:
            w_norm = np.zeros((3, 1))
  
        wind_dir = np.dot(ra, w_norm)
        if wind_dir >  1: wind_dir =  1
        if wind_dir < -1: wind_dir = -1
                
        v_rel = v_cm + margin * math.sin(math.acos(wind_dir)) * np.cross(ra, w) # center of mass speed
        v_mag = np.linalg.norm(v_rel)
        v_norm = normalize_vector(v_rel)

        # Angle of attack
        v_cross = np.cross(ra, v_norm)
        v_cross_norm = normalize_vector(v_cross)
        alpha = math.atan2(np.linalg.norm(np.cross(ra, v_norm)), np.dot(ra, v_norm))
        delta = math.atan2(np.linalg.norm(np.cross(ra, ze)), np.dot(ra, ze))

        # Normal force
        na = np.cross(ra, v_cross)
        if np.linalg.norm(na) == 0:
            n = np.array([0, 0, 0]).transpose()
        else:
            n = 0.5 * rho * Sm * CNa * alpha * v_mag ** 2 * na/ (np.linalg.norm(na)+0.05) # --> constant added to avoid division by small number

        # Drag
        # Drag coefficient
        cd = drag(self.rocket, alpha, v_mag, nu, a)*self.rocket.CD_fac  # !!! take 3000 us !!! -> actually half of the computation time
        
        # Drag force
        d = -0.5 * rho * Sm * cd * v_mag ** 2 * v_norm

        # Moment estimation ------------------------

        # Aerodynamic corrective moment
        mn = np.linalg.norm(n) * margin * v_cross_norm

        # Aerodynamic damping moment
        w_pitch = w - np.dot(w, ra) * ra
        cdm = pitch_damping_moment(self.rocket, rho, CNa_bar, CP_bar, dMdt, cg, np.linalg.norm(w_pitch), v_mag)
        md = -0.5 * rho * cdm * Sm * v_mag ** 2 * normalize_vector(w_pitch)


        self.rocket.set_aero(n+d, mn+md)




    def Dynamics_6DOF(self, t, s, thrust_force, thrust_torque):
        start_time = time.time() # -----------------------------------------------------------------
        
        x = s[0:3]
        v = s[3:6]
        q = s[6:10]
        w = s[10:13]
        propellant_mass = s[13]


        # Normalise quaternion
        q = normalize_vector(q)
        

        # Rotation matrix from rocket coordinates to Earth coordinates
        c = quat2rotmat(q)
        angle = rot2anglemat(c)

        # Rocket principle frame vectors expressed in Earth coordinates
        ya = c.dot(np.array([1, 0, 0]).transpose())  # Yaw axis
        pa = c.dot(np.array([0, 1, 0]).transpose())  # Pitch axis
        ra = c.dot(np.array([0, 0, 1]).transpose())  # Roll axis

        # Earth coordinates vectors expressed in Earth's frame
        xe = np.array([1, 0, 0]).transpose()
        ye = np.array([0, 1, 0]).transpose()
        ze = np.array([0, 0, 1]).transpose()

				# Mass properties				
        m = self.rocket.get_empty_mass() + propellant_mass
        dMdt = np.linalg.norm(thrust_force)/(self.rocket.get_motor_Isp()*9.81)
        cg = (self.rocket.get_dry_cg()*self.rocket.get_empty_mass() + self.rocket.get_propellant_cg()*propellant_mass)/m
        Sm = self.rocket.get_max_cross_section_surface
        #I = c.transpose().dot(self.rocket.get_rocket_inertia()).dot(c)
        I = c.dot(self.rocket.get_rocket_inertia()).dot(c.transpose())

        # Environment
        g = 9.81  # Gravity [m/s^2]
        rho = self.Environment.get_density(x[2] + self.Environment.ground_altitude)

        nu = self.Environment.get_viscosity(x[2] + self.Environment.ground_altitude) # !!! take 200 us

        a = self.Environment.get_speed_of_sound(x[2] + self.Environment.ground_altitude)


				# Force computation: Thrust, gravity, drag and lift --------------------------

        # Thrust
        # X, Y, Z force in rocket frame, reoriented to world frame
        T = c.dot(thrust_force.transpose())

        # Gravity
        G = -g * m * ze

        # Aerodynamic corrective forces
        # Compute center of mass angle of attack
        v_cm = v - wind_model(t, self.Environment.get_turb(x[2] + self.Environment.ground_altitude),
                              self.Environment.get_V_inf()*self.Environment.V_dir, 'None' , x[2]) # TODO : V_dir


        v_cm_mag = np.linalg.norm(v_cm)
        alpha_cm = math.atan2(np.linalg.norm(np.cross(ra, v_cm)), np.dot(ra, v_cm)) # !!! take 200 us

        # Mach number
        Mach = v_cm_mag / a

        # Normal lift coefficient and center of pressure
        CNa, Xcp, CNa_bar, CP_bar = normal_lift(self.rocket, alpha_cm, 1.1, Mach, angle[2], 1)

        # Stability margin
        margin = Xcp - cg

        # Compute rocket angle of attack
        if np.linalg.norm(w) != 0:
            w_norm = w / np.linalg.norm(w)
        else:
            w_norm = np.zeros((3, 1))
  
        wind_dir = np.dot(ra, w_norm)
        if wind_dir >  1: wind_dir =  1
        if wind_dir < -1: wind_dir = -1
                
        v_rel = v_cm + margin * math.sin(math.acos(wind_dir)) * np.cross(ra, w) # center of mass speed
        v_mag = np.linalg.norm(v_rel)
        v_norm = normalize_vector(v_rel)

        # Angle of attack
        v_cross = np.cross(ra, v_norm)
        v_cross_norm = normalize_vector(v_cross)
        alpha = math.atan2(np.linalg.norm(np.cross(ra, v_norm)), np.dot(ra, v_norm))
        delta = math.atan2(np.linalg.norm(np.cross(ra, ze)), np.dot(ra, ze))

        # Normal force
        na = np.cross(ra, v_cross)
        if np.linalg.norm(na) == 0:
            n = np.array([0, 0, 0]).transpose()
        else:
            n = 0.5 * rho * Sm * CNa * alpha * v_mag ** 2 * na/ (np.linalg.norm(na)+0.05) # --> constant added to avoid division by small number


        # Drag
        # Drag coefficient
        cd = drag(self.rocket, alpha, v_mag, nu, a)*self.rocket.CD_fac  # !!! take 3000 us !!! -> actually half of the computation time
        
        # Drag force
        d = -0.5 * rho * Sm * cd * v_mag ** 2 * v_norm
        
        

        # Total forces
        f_tot = T + G #+ n + d

        # Moment estimation

        # Aerodynamic corrective moment
        mn = np.linalg.norm(n) * margin * v_cross_norm

        # Aerodynamic damping moment
        w_pitch = w - np.dot(w, ra) * ra
        cdm = pitch_damping_moment(self.rocket, rho, CNa_bar, CP_bar, dMdt, cg, np.linalg.norm(w_pitch), v_mag)
        md = -0.5 * rho * cdm * Sm * v_mag ** 2 * normalize_vector(w_pitch)
        

        m_tot = c.dot(thrust_torque.transpose())# + mn + md 
        

        # Translational dynamics
        X_dot = v
        V_dot = 1/m*(f_tot - v*dMdt)

        # State derivatives
        q_dot = quat_evolve(q, w)
        w_dot = np.linalg.lstsq(I, m_tot, rcond=None)[0]

        S_dot = np.concatenate((X_dot, V_dot, q_dot, w_dot, np.array([-dMdt])))

        self.rocket.set_sensor_data(V_dot, w, x[2], c)

        #print(1000*(time.time()-start_time))

        return S_dot

    def Dynamics_Parachute_3DOF(self, t, s, rocket, main):
        x = s[0:3]
        v = s[3:6]


        rho = self.Environment.get_density(x[2] + self.Environment.ground_altitude)

        # Aerodynamic force
        v_rel = -v + wind_model(t, self.Environment.get_turb(x[2] + self.Environment.ground_altitude),
                                self.Environment.get_V_inf()*self.Environment.V_dir, self.Environment.get_turb_model(), x[2])

        M = self.rocket.get_empty_mass() - self.rocket.pl_mass

        if main:
            SCD = self.rocket.get_para_main_SCD()
        else:
            SCD = self.rocket.get_para_drogue_SCD()

        D = 0.5 * rho * SCD * np.linalg.norm(v_rel) * v_rel

        # Gravity force
        g = np.array([0, 0, -9.81])
        G = g * M

        dXdt = v
        dVdt = (D+G)/M

        dsdt = np.concatenate((dXdt, dVdt))
        return dsdt

    def Dynamics_3DOF(self, t, s):

        X = s[0:3]
        V = s[3:6]

        XE = np.array([1, 0, 0])
        YE = np.array([0, 1, 0])
        ZE = np.array([0, 0, 1])

        a = self.Environment.get_speed_of_sound(X[2] + self.Environment.ground_altitude)
        rho = self.Environment.get_density(X[2] + self.Environment.ground_altitude)
        nu = self.Environment.get_viscosity(X[2] + self.Environment.ground_altitude)


        M = self.rocket.get_empty_mass()

        V_rel = V - wind_model(t, self.Environment.get_turb(X[2] + self.Environment.ground_altitude),
                               self.Environment.get_V_inf()*self.Environment.V_dir,
                               self.Environment.get_turb_model(), X[2])

        G = -9.81 * M * ZE

        CD = drag(self.rocket, 0, np.linalg.norm(V_rel), nu, a)

        D = -0.5 * rho * self.rocket.get_max_cross_section_surface * CD * V_rel * np.linalg.norm(V_rel)


        X_dot = V
        V_dot = 1 / M * (D + G)


        S_dot = np.concatenate((X_dot, V_dot))
        return S_dot

    def Nose_Dynamics_3DOF(self, t, s, Environment):

        X = s[0:3]
        V = s[3:6]

        XE = np.array([1, 0, 0]).transpose()
        YE = np.array([0, 1, 0]).transpose()
        ZE = np.array([0, 0, 1]).transpose()

        # atmosphere
        a = self.Environment.get_speed_of_sound(X[2] + self.Environment.ground_altitude)
        rho = self.Environment.get_density(X[2] + self.Environment.ground_altitude)
        nu = self.Environment.get_viscosity(X[2] + self.Environment.ground_altitude)

        M = self.rocket.get_mass(t)

        V_rel = V - wind_model(t, self.Environment.get_turb(X[0] + self.Environment.ground_altitude),
                               self.Environment.get_V_inf(),
                               self.Environment.get_turb_model(), X[2])

        G = -9.81 * M * ZE
        CD = Nose_drag(self.rocket, 0, np.linalg.norm(V_rel), nu, a)
        D = -0.5 * rho * self.rocket.get_max_cross_section_surface * CD * V_rel * np.linalg.norm(V_rel)

        X_dot = V
        V_dot = 1 / M * (D + G)

        return X_dot, V_dot

    def Nose_Dynamics_6DOF(self, t, s):

        X = s[0:3]
        V = s[3:6]
        Q = s[6:10]
        W = s[10:13]

        # Check quaternion norm
        Q = normalize_vector(Q)

        # Rotation matrix from rocket coordinates to Earth coordinates
        C = quat2rotmat(Q)
        angle = rot2anglemat(C)

        # Rocket principle frame vectors expressed in earth coordinates
        YA = C * np.array([1, 0, 0]).transpose()
        PA = C * np.array([0, 1, 0]).transpose()
        RA = C * np.array([0, 0, 1]).transpose()

        # Earth coordinates vectors expressed in earth's frame
        XE = np.array([1, 0, 0]).transpose()
        YE = np.array([0, 1, 0]).transpose()
        ZE = np.array([0, 0, 1]).transpose()

        # Rocket inertia
        M = self.rocket.get_mass(t)
        dMdt = self.rocket.get_dmass_dt(t)
        CM = self.rocket.get_cg(t)
        Sm = self.rocket.get_max_cross_section_surface
        I_L = self.rocket.get_long_inertia(t)
        I_R = self.rocket.get_rot_inertia(t)
        I = C.transpose() * ([[I_L, 0, 0],
                              [0, I_L, 0],
                              [0, 0, I_R]]) * C

        g = 9.81

        # atmosphere
        a = self.Environment.get_speed_of_sound(X[2] + self.Environment.ground_altitude)
        rho = self.Environment.get_density(X[2] + self.Environment.ground_altitude)
        nu = self.Environment.get_viscosity(X[2] + self.Environment.ground_altitude)

        # Thrust
        # Oriented along roll axis of rocket frame, expressed, in earth coordinates
        T = self.rocket.get_thrust(t) * RA

        G = -g * M * ZE

        # Compute center of mass angle of attack
        Vcm = V - wind_model(t, self.Environment.get_turb(X[0] + self.Environment.ground_altitude),
                             self.Environment.get_v_inf(),
                             self.Environment.get_turb_model(), X[2])

        Vcm_mag = np.linalg.norm(Vcm)
        alpha_cm = math.atan2(np.linalg.norm(np.cross(RA, Vcm)), np.dot(RA, Vcm))

        # Mach number
        Mach = np.linalg.norm(Vcm_mag) / a

        # Normal lift coefficient and center of pressure
        CNa, Xcp, CNa_bar, CP_bar = normal_lift(self.rocket, alpha_cm, 1.1, Mach, angle[2], 1)

        # Stability margin
        margin = Xcp - CM

        # Compute rocket angle of attack
        if np.linalg.norm(W) != 0:
            w_norm = W / np.linalg.norm(W)
        else:
            w_norm = np.zeros(3, 1)

        Vrel = Vcm + margin * math.sin(math.acos(np.dot(RA, w_norm))) * np.cross(RA, W)
        Vmag = np.linalg.norm(Vrel)
        Vnorm = normalize_vector(Vrel)

        # Angle of attack
        Vcross = np.cross(RA, Vnorm)
        Vcross_norm = normalize_vector(Vcross)
        alpha = math.atan2(np.linalg.norm(np.cross(RA, Vnorm)), np.dot(RA, Vnorm))
        delta = math.atan2(np.linalg.norm(np.cross(RA, ZE)), np.dot(RA, ZE))

        # Normal force
        NA = np.cross(RA, Vcross)
        if np.linalg.norm(NA) == 0:
            N = np.array([0, 0, 0]).transpose
        else:
            N = 0.5 * rho * Sm * CNa * alpha * Vmag ** 2 * NA / np.linalg.norm(NA)

        # Drag
        # Drag coefficient
        CD = drag(self.rocket, alpha, Vmag, nu, a)  # TODO : * cd_fac (always 1 ?)
        ab_phi = self.rocket.ab_phi  # TODO : find a way to deal with airbrakes, /!\ magic number
        if t > self.rocket.get_burn_time:
            CD = CD + drag_shuriken(self.rocket, ab_phi, alpha, Vmag, nu)

        # Drag force
        D = -0.5 * rho * Sm * CD * Vmag ** 2 * Vnorm

        # Total forces
        motor_fac = self.rocket.motor_fac  # TODO : always 1 ?
        F_tot = T * motor_fac + G + N + D

        # Moment estimation

        # Aerodynamic corrective moment
        MN = np.linalg.norm(N) * margin * Vcross_norm

        # Aerodynamic damping moment
        w_pitch = W - np.dot(W, RA) * RA
        cdm = pitch_damping_moment(self.rocket, rho, CNa_bar, CP_bar, dMdt, CM, np.linalg.norm(w_pitch), Vmag)
        MD = -0.5 * rho * cdm * Sm * Vmag ** 2 * normalize_vector(w_pitch)

        m_tot = MN + MD

        tmp_Nose_Alpha = alpha
        tmp_Nose_Delta = delta

        return V, 1 / M * (F_tot + V * dMdt), quat_evolve(Q, W), lin.lstsq(I, m_tot)

    def Payload_Dynamics_3DOF(self, t, s, Environment):

        X = s[0:3]
        V = s[3:6]

        XE = np.array([1, 0, 0]).transpose()
        YE = np.array([0, 1, 0]).transpose()
        ZE = np.array([0, 0, 1]).transpose()

        # atmosphere
        a = self.Environment.get_speed_of_sound(X[2] + self.Environment.ground_altitude)
        rho = self.Environment.get_density(X[2] + self.Environment.ground_altitude)
        nu = self.Environment.get_viscosity(X[2] + self.Environment.ground_altitude)

        M = self.rocket.get_mass(t)

        V_rel = V - wind_model(t, self.Environment.get_turb(X[0] + self.Environment.ground_altitude),
                               self.Environment.get_v_inf(),
                               self.Environment.get_turb_model(), X[2])

        G = -9.81 * M * ZE

        SCD = 2.56 * 10 ** (-2)
        D = -0.5 * rho * SCD * V_rel * np.linalg.norm(V_rel)

        X_dot = V
        V_dot = 1 / M * (D + G)

        return X_dot, V_dot

    def RailSim(self):

        def off_rail(t, y): return y[0] - self.Environment.Rail_Length

        off_rail.terminal = True
        off_rail.direction = 1

        # Initial Conditions
        X0 = np.array([0, 0])

        # Time span
        tspan = np.array([0, 5])

        # Options

        print(tspan, X0)
        # intergration
        self.integration_ivp = solve_ivp(self.Dynamics_Rail_1DOF, tspan, X0, events=off_rail)

        T1 = self.integration_ivp.t
        S1 = self.integration_ivp.y
        return T1, S1

    def FlightSim(self, tspan, arg2, arg3=None, arg4=None, arg5=None):

        if arg3 is None and arg4 is None and arg5 is None:
            # Compute initial conditions based on rail output values
            V = arg2

            # Rail vector
            C_rail = rotmat(self.Environment.Rail_Azimuth, 3) * rotmat(self.Environment.Rail_Angle, 2) * rotmat(
                self.Environment.Rail_Azimuth, 3).transpose()
            RV = C_rail.dot(np.array([0, 0, 1]).transpose())

            # Initial Conditions
            X0 = RV * self.Environment.Rail_Length
            V0 = RV * V
            Q0 = rot2quat(C_rail.transpose())
            W0 = np.array([0, 0, 0]).transpose()
            S0 = np.concatenate((X0,V0,Q0,W0), axis=0)

        elif arg3 is not None and arg4 is not None and arg5 is not None:

            # Set initial conditions based on the exact value of the state vector
            X0 = arg2
            V0 = arg3
            Q0 = arg4
            W0 = arg5
            S0 = np.concatenate((X0,V0,Q0,W0), axis=0)

        else:
            print("ERROR: In flight simulator, function accepts either 3 or 6 arguments")

        def apogee(t, y):
            return y[5]

        apogee.terminal = True
        apogee.direction = -1


        self.integration_ivp = solve_ivp(self.Dynamics_6DOF, tspan, S0, events=apogee)

        T2 = self.integration_ivp.t
        S2 = self.integration_ivp.y
        T2E = self.integration_ivp.t_events
        S2E = self.integration_ivp.y_events
        I2E = np.where(T2 == T2E)

        return T2, S2, T2E, S2E, I2E

    def DrogueParaSim(self, T0, X0, V0):

        # Initial conditions
        S0 = np.concatenate((X0, V0), axis=0)

        # time span
        tspan = np.array([T0, 500])

        def MainEvent(t, y, rocket, main):
            return (y[2] > rocket.get_para_main_event()) - 0.5

        MainEvent.terminal = True
        MainEvent.direction = -1

        print(self.rocket.get_para_main_event())
        # integration
        self.integration_ivp = solve_ivp(self.Dynamics_Parachute_3DOF, tspan, S0, args=[self.rocket, 0], events=MainEvent)

        T3 = self.integration_ivp.t
        S3 = self.integration_ivp.y
        T3E = self.integration_ivp.t_events
        S3E = self.integration_ivp.y_events
        I3E = np.where(T3 == T3E)

        return T3, S3, T3E, S3E, I3E

    def MainParaSim(self, T0, X0, V0):
        # Initial conditions
        S0 = np.concatenate((X0, V0), axis=0)

        # time span
        tspan = np.array([T0, 500])

        def CrashEvent(t, y, rocket, main):
            return (y[2] > 0) - 0.5

        CrashEvent.terminal = True
        CrashEvent.direction = -1

        # integration
        self.integration_ivp = solve_ivp(self.Dynamics_Parachute_3DOF, tspan, S0, args=[self.rocket, 1], events=CrashEvent)

        T4 = self.integration_ivp.t
        S4 = self.integration_ivp.y
        T4E = self.integration_ivp.t_events
        S4E = self.integration_ivp.y_events
        I4E = np.where(T4 == T4E)

        return T4, S4, T4E, S4E, I4E

    def CrashSim(self, T0, X0, V0):

        # Initial conditions
        S0 = np.concatenate((X0, V0), axis=0)
        print(S0, T0)

        # time span
        tspan = np.array([T0, 100])

        def CrashEvent(t, y):
            return (y[2] > 0) - 0.5

        CrashEvent.terminal = True
        CrashEvent.direction = -1

        # integration
        self.integration_ivp = solve_ivp(self.Dynamics_3DOF, tspan, S0, events=CrashEvent)

        T5 = self.integration_ivp.t
        S5 = self.integration_ivp.y
        T5E = self.integration_ivp.t_events
        S5E = self.integration_ivp.y_events
        I5E = np.where(T5 == T5E)

        return T5, S5, T5E, S5E, I5E

    def Nose_CrashSim_3DOF(self, T0, X0, V0):
        # Initial conditions
        S0 = np.array([X0, V0]).transpose()

        # time span
        tspan = np.array([T0, 100])

        def CrashEvent(t, y):
            return (y[0] > 0) - 0.5

        CrashEvent.terminal = True
        CrashEvent.direction = -1

        # integration
        self.integration_ivp = solve_ivp(self.Nose_Dynamics_3DOF, tspan, S0, event=CrashEvent)

        T6 = self.integration_ivp.t
        S6 = self.integration_ivp.y
        T6E = self.integration_ivp.t_events
        S6E = self.integration_ivp.y_events
        I6E = np.where(T6 == T6E)

        return T6, S6, T6E, S6E, I6E

    def Nose_CrashSim_6DOF(self, tspan, arg2, arg3=None, arg4=None, arg5=None):

        if arg3 is not None and arg4 is not None and arg5 is not None:

            # Set initial conditions based on the exact value of the state vector
            X0 = arg2
            V0 = arg3
            Q0 = arg4
            W0 = arg5
            S0 = np.array([X0, V0, Q0, W0]).transpose()

        else:
            print("ERROR: In flight simulator, function accepts either 3 or 6 arguments")

        def CrashEvent(t, y):
            return (y[0] > 0) - 0.5

        CrashEvent.terminal = True
        CrashEvent.direction = -1

        self.integration_ivp = solve_ivp(self.Nose_Dynamics_6DOF, tspan, S0, event=CrashEvent)

        T6 = self.integration_ivp.t
        S6 = self.integration_ivp.y
        T6E = self.integration_ivp.t_events
        S6E = self.integration_ivp.y_events
        I6E = np.where(T6 == T6E)

        return T6, S6, T6E, S6E, I6E

    def PayloadCrashSim(self, T0, X0, V0):

        # Initial conditions
        S0 = np.array([X0, V0]).transpose()

        # time span
        tspan = np.array([T0, 100])

        def CrashEvent(t, y):
            return (y[0] > 0) - 0.5

        CrashEvent.terminal = True
        CrashEvent.direction = -1

        # integration
        self.integration_ivp = solve_ivp(self.Payload_Dynamics_3DOF, tspan, S0, event=CrashEvent)

        T7 = self.integration_ivp.t
        S7 = self.integration_ivp.y
        T7E = self.integration_ivp.t_events
        S7E = self.integration_ivp.y_events
        I7E = np.where(T7 == T7E)

        return T7, S7, T7E, S7E, I7E

    def FlightOutputFunc(self, T, S, flag):
        status = 0

        if simAuxResults.Margin:
            np.append(simAuxResults.Margin, tmp_Margin)
        if simAuxResults.Alpha:
            np.append(simAuxResults.Alpha, tmp_Alpha)
        if simAuxResults.Cn_alpha:
            np.append(simAuxResults.Cn_alpha, tmp_Cn_alpha)
        if simAuxResults.Xcp:
            np.append(simAuxResults.Xcp, tmp_Xcp)
        if simAuxResults.Cd:
            np.append(simAuxResults.Cd, tmp_Cd)
        if simAuxResults.Mass:
            np.append(simAuxResults.Mass, tmp_Mass)
        if simAuxResults.CM:
            np.append(simAuxResults.CM, tmp_CM)
        if simAuxResults.Il:
            np.append(simAuxResults.Il, tmp_Il)
        if simAuxResults.Ir:
            np.append(simAuxResults.Ir, tmp_Ir)
        if simAuxResults.Delta:
            np.append(simAuxResults.Delta, tmp_Delta)
        if simAuxResults.Nose_Alpha:
            np.append(simAuxResults.Nose_Alpha, tmp_Nose_Alpha)
        if simAuxResults.Nose_delta:
            np.append(simAuxResults.Nose_delta, tmp_Nose_Delta)
        return status

    def CrashOutputFunc(self, T, S, flag):
        status = 0

        if simAuxResults.Nose_Alpha:
            np.append(simAuxResults.Nose_Alpha, tmp_Nose_Alpha)
        if simAuxResults.Nose_delta:
            np.append(simAuxResults.Nose_delta, tmp_Nose_Delta)
        return status


