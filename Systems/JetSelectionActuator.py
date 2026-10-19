# DON'T CHANGE ANY OF THE BELOW; NECESSARY FOR JOINING SIMULATION
import os, sys, time, datetime, traceback
import spaceteams as st
def custom_exception_handler(exctype, value, tb):
    error_message = "".join(traceback.format_exception(exctype, value, tb))
    st.logger_fatal(error_message)
sys.excepthook = custom_exception_handler
st.connect_to_sim(sys.argv)
import numpy as np
# DON'T CHANGE ANY OF THE ABOVE; NECESSARY FOR JOINING SIMULATION
################################################################
# Now, when below the boilerplate section, you can import more things

from scipy import optimize
import csv
import copy


dbl = st.VarType.double
dv3 = st.VarType.doubleV3
dv4 = st.VarType.doubleV4


def IfHas_GetParam(entity: st.Entity, varType: st.VarType, key: str | list[str], default):
    if entity.HasParam(key):
        return entity.GetParam(varType, key)
    else:
        return default


def ImportFile(filepath, thruster_indices_to_include: list[int] = [], 
    thruster_indices_to_ignore: list[int] = []):
    x = []
    y = []
    z = []
    nx = []
    ny = []
    nz = []
    t = []

    # Choose mode; whitelist overrides blacklist.
    mode = "AllThrusters"
    if len(thruster_indices_to_include) != 0:
        mode = "Whitelist"
    elif len(thruster_indices_to_ignore) != 0:
        mode = "Blacklist"

    with open(filepath) as jetconfig:
        reader = csv.reader(jetconfig)
        idx = 0
        for row in reader:
            inclusion_condition = True
            if mode == "Whitelist":
                inclusion_condition = idx in thruster_indices_to_include
            elif mode == "Blacklist":
                inclusion_condition = idx not in thruster_indices_to_ignore
            if idx != 0 and inclusion_condition:
                x.append(float(row[0]))
                y.append(float(row[1]))
                z.append(float(row[2]))
                nx.append(float(row[3]))
                ny.append(float(row[4]))
                nz.append(float(row[5]))
                t.append(float(row[6]))
            idx += 1
    return x, y, z, nx, ny, nz, t


def ComputeActivityVectors(jet_x, jet_y, jet_z, jet_nx, jet_ny, jet_nz, jet_thrust, num_jets, mass, inertia_inv):
    activityVectors = []
    for ii in range(num_jets):  # For each jet
        # Get the data for the ii-th jet
        r = np.array([jet_x[ii], jet_y[ii], jet_z[ii]])
        n = np.array([jet_nx[ii], jet_ny[ii], jet_nz[ii]])

        n = n / np.linalg.norm(n)
        F = jet_thrust[ii]

        # calculate the output force and torque for each jet
        force = -F * n  # t * -n
        torque = np.cross(r, force)  # r X f

        # calculate the output accelerations for each jet
        a_trans = force / mass
        a_rot = (inertia_inv @ np.array([torque]).T).T[0]
        activityVectors.append([a_trans[0], a_trans[1], a_trans[2], a_rot[0], a_rot[1], a_rot[2]])
    return activityVectors


def GetThrustersToFire(output, dt_min, dt_tick):
    thrusters = []
    for ii in range(len(output)):
        if abs(output[ii]) * dt_tick > dt_min:
            thrusters.append(ii)
    return thrusters


def GetOnJets(thrusters_to_fire, num_jets) -> list[float]:
    on_jets = []
    for i in range(num_jets):
        if i in thrusters_to_fire:
            on_jets.append(1.0)
        else:
            on_jets.append(0.0)
    return on_jets


def GetEffectiveForceAndTorque(A, thrusters_to_fire):
    control = np.zeros(6)
    for i in range(A.shape[0]):
        if i in thrusters_to_fire:
            control += A[i]
    return control


def GetEffectiveForceAndTorque_VariableOnTime(A, onTimes, dt_tick: float):
    g0: float = 9.8
    Isp: float = 80.0
    control = np.zeros(6)
    fuelUsage = 0.0
    for i in range(A.shape[0]):
        timeToFire = abs(onTimes[i])# * dt_tick
        thisControl = timeToFire * A[i]
        control += thisControl
        fuelUsage += np.linalg.norm(thisControl[0:3]) / (Isp * g0)
    return control, fuelUsage

"""
https://ahrs.readthedocs.io/en/latest/filters/angular.html
"""
def QuaternionRate(q, omega):
    wx = omega[0]
    wy = omega[1]
    wz = omega[2]
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]
    q_dot = np.array([-wx * qx - wy * qy - wz * qz, 
                      wx * qw + wz * qy - wy * qz, 
                      wy * qw - wz * qx + wx * qz, 
                      wz * qw + wy * qx - wx * qy])
    return q_dot


def AxisAngle_to_Quat(e, phi):
    # "e" is a normalized 3-vector
    # "phi" is an angle in degrees
    e1 = e[0]
    e2 = e[1]
    e3 = e[2]
    phi_r = np.radians(phi)
    q1 = e1 * np.sin(phi_r / 2)
    q2 = e2 * np.sin(phi_r / 2)
    q3 = e3 * np.sin(phi_r / 2)
    q4 = np.cos(phi_r / 2)
    q = np.array([q1, q2, q3, q4])
    return q


def Quat_to_AxisAngle(q):
    # "q" is a normalized 4-vector (quaternion)
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    phi_r = np.arccos(2 * q4 * q4 - 1)
    if abs(phi_r) < 1e-8:
        return np.array([1.0, 0.0, 0.0], 0.0)
    e1 = q1 / np.sin(phi_r / 2)
    e2 = q2 / np.sin(phi_r / 2)
    e3 = q3 / np.sin(phi_r / 2)
    e = np.array([e1, e2, e3]) / np.linalg.norm(np.array([e1, e2, e3]))
    return e, phi_r


def Quat_to_DCM(q):
    # "q" is a normalized 4-vector (quaternion), scalar-last
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    I = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    part1 = I * (q4 * q4 - q1 * q1 - q2 * q2 - q3 * q3)
    part2 = 2 * np.array([[q1 * q1, q1 * q2, q1 * q3], [q1 * q2, q2 * q2, q2 * q3], [q1 * q3, q2 * q3, q3 * q3]])
    part3 = 2 * q4 * np.array([[0, -q3, q2], [q3, 0, -q1], [-q2, q1, 0]])
    C = part1 + part2 - part3
    return C


def remap(val, in_lower, in_upper, out_lower, out_upper):
    factor = (val - in_lower) / (in_upper - in_lower)
    out_val = out_lower + factor * (out_upper - out_lower)
    if out_val > out_upper:
        return out_upper
    elif out_val < out_lower:
        return out_lower
    return out_val


def AverageQuaternions(quaternions):
    """
    Calculate average quaternion

    :params quaternions: is a Nx4 numpy matrix and contains the quaternions
        to average in the rows.
        The quaternions are arranged as (w,x,y,z), with w being the scalar

    :returns: the average quaternion of the input. Note that the signs
        of the output quaternion can be reversed, since q and -q
        describe the same orientation
    """

    # Number of quaternions to average
    samples = quaternions.shape[0]
    M = np.zeros(shape=(4, 4), dtype=np.float64)

    # Calculate M
    for i in range(0, samples):
        quat = quaternions[i, :]
        M = np.outer(quat, quat) + M
    M = (1.0 / samples) * M

    # Compute eigenvalues and eigenvectors
    eigen_values, eigen_vectors = np.linalg.eig(M)
    
    # Sort by largest eigenvalue
    eigen_vectors = eigen_vectors[:, eigen_values.argsort()[::-1]]
    return np.real(np.ravel(eigen_vectors[:, 0]))


this = st.GetThisSystem()

exit_flag = False

# Import Entity jet configuration:
configPath = this.GetParam(st.VarType.string, "JetConfigName")
configPathReal = st.path_utils.AssetPathToReal(st.AssetType.Generic, configPath)
X, Y, Z, NX, NY, NZ, T = ImportFile(configPathReal)

# Entities and frames
en: st.Entity = this.GetParam(st.VarType.entityRef, "ControlledEntity")
bodyFrame: st.frames.Frame = en.GetBodyFixedFrame()
resFrame: st.frames.Frame = en.getResidentFrame()

num_main_jets = len(X)
en.AddOrSetParam(st.VarType.int32, ["RCSConfig", "NumMain"], num_main_jets)
for i in range(num_main_jets):
    en.AddParamArray(dbl, ["RCSConfig", "Main" + str(i)], [X[i], Y[i], Z[i], NX[i], NY[i], NZ[i], T[i]])
en.AddOrSetParam(st.VarType.bool, ["RCSConfig", "Ready"], True)

# System update loop:
t0: st.timestamp = st.SimGlobals.SimClock.GetTimeNow()
sim_elapsed_time: float = 0.0
while not exit_flag:
    ### BEGIN THIS TIMESTEP ###
    tf: st.timestamp = st.SimGlobals.SimClock.GetTimeNow()
    dt: datetime.timedelta = tf.as_datetime() - t0.as_datetime()

    # CCT params:
    mass = en.getMass()
    I = en.getInertia(bodyFrame)
    I_inv = np.linalg.inv(I)
    dt_min = IfHas_GetParam(en, dbl, ["JetSelection", "RCSMinOnTime"], 0.1)
    accCommand = IfHas_GetParam(en, dv3, ["JetSelection", "CommandedAccel"], np.zeros(3))
    angAccCommand = IfHas_GetParam(en, dv3, ["JetSelection", "CommandedAngAccel"], np.zeros(3))

    maxAccel: float = en.GetParam(st.VarType.double, ["Thrusters", "MaxAccel_m_s2"])
    maxAngAccel: float = en.GetParam(st.VarType.double, ["Thrusters", "MaxAngAccel_rad_s2"])
    if np.linalg.norm(accCommand) > maxAccel:
        accCommand = maxAccel * accCommand / np.linalg.norm(accCommand)
    if np.linalg.norm(angAccCommand) > maxAngAccel:
        angAccCommand = maxAngAccel * angAccCommand / np.linalg.norm(angAccCommand)

    # Command angular accel
    u_acc_jetsel = np.zeros(3)
    u_angacc_jetsel = np.zeros(3)

    if not np.isnan(np.linalg.norm(angAccCommand)):
        # Activity vectors and simplex method setup:
        A = ComputeActivityVectors(X, Y, Z, NX, NY, NZ, T, num_main_jets, mass, I_inv)
        A = np.asarray(A)
        C = np.full(num_main_jets, 1.0)
        b = np.hstack((accCommand, angAccCommand))

        thrusters_to_fire = []

        # Running simplex method if the command is finite and non-zero:
        norm_b = np.linalg.norm(b)
        onTimes = np.zeros(num_main_jets)
        if norm_b > 1e-9 and norm_b != np.inf and not np.isnan(norm_b):
            result = optimize.linprog(C, A_eq=A.T, b_eq=b, method='revised simplex')
            onTimes = result.x
            thrusters_to_fire = GetThrustersToFire(result.x, dt_min, dt.total_seconds())

        # Actual controls and jets to turn on:
        control, fuelUsage = GetEffectiveForceAndTorque_VariableOnTime(A, onTimes, dt.total_seconds())
        u_acc_jetsel += control[0:3]
        u_angacc_jetsel += control[3:6]
        on_jets = GetOnJets(thrusters_to_fire, num_main_jets)
        if en.HasParamArray(["RCSConfig", "OnThrustersMain"]):
            en.SetParamArray(st.VarType.double, ["RCSConfig", "OnThrustersMain"], thrusters_to_fire)
        else:
            en.AddParamArray(st.VarType.double, ["RCSConfig", "OnThrustersMain"], thrusters_to_fire)

        prev_fuelUsage: float = en.GetParam(st.VarType.double, ["Stats", "FuelUsed"])
        en.SetParam(st.VarType.double, ["Stats", "FuelUsed"], prev_fuelUsage + fuelUsage)
    
    en.AddOrSetParam(dv3, ["JetSelection", "ResultantAccel"], u_acc_jetsel)
    en.AddOrSetParam(dv3, ["JetSelection", "ResultantAngAccel"], u_angacc_jetsel)

    # st.OnScreenAlert(f"Jet selection commanded force:  {mass * u_acc_jetsel}", "JetSelectionForce", st.Severity.Info)
    # st.OnScreenAlert(f"Jet selection commanded torque: {I @ u_angacc_jetsel}", "JetSelectionTorque", st.Severity.Info)

    directCommand: bool = this.GetParam(st.VarType.bool, "DirectCommand")

    forceEffect = st.PhysEffect(st.PhysEffectType.Force, bodyFrame)
    if directCommand:
        forceEffect.setVector(mass * accCommand)
    else:
        forceEffect.setVector(mass * u_acc_jetsel)
    en.setPhysEffect("ThrusterForce", forceEffect)

    torqueEffect = st.PhysEffect(st.PhysEffectType.Torque, bodyFrame)
    if directCommand:
        torqueEffect.setVector(I @ angAccCommand)
    else:
        torqueEffect.setVector(I @ u_angacc_jetsel)
    en.setPhysEffect("ThrusterTorque", torqueEffect)

    ### END THIS TIMESTEP ###

    # st.OnScreenAlert("q = " + str(np.round(en.getRotation().Quat_WRT(resFrame), 3)), "Attitude", st.Severity.Info)
    # st.OnScreenAlert("w = " + str(np.round(en.getAngVelocity().WRT_ExprIn(resFrame) * 180.0 / np.pi, 3)), "AngVel", st.Severity.Info)
    
    t0 = tf
    sim_elapsed_time += dt.total_seconds()
    time.sleep(1.0 / this.GetParam(dbl, "LoopFreqHz"))

st.leave_sim()
