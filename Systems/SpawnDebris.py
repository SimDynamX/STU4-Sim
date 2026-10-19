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

import pathlib
import random
import numpy.typing as npt


def clampFloat(val: float, outMin: float, outMax: float) -> float:
    return min(max(val, outMin), outMax)


def clampVector(val: npt.NDArray, outMin: npt.NDArray, outMax: npt.NDArray) -> npt.NDArray:
    return np.minimum(np.maximum(val, outMin), outMax)


def remapFloat(val: float, inMin: float, inMax: float, outMin: float, outMax: float) -> float:
    return clampFloat(outMin + (outMax - outMin) * (val - inMin) / (inMax - inMin), outMin, outMax)


def remapVector(val: npt.NDArray, inMin: npt.NDArray, inMax: npt.NDArray, outMin: npt.NDArray, outMax: npt.NDArray) -> npt.NDArray:
    return clampVector(outMin + (outMax - outMin) * (val - inMin) / (inMax - inMin), outMin, outMax)


# def AddParamOps(paramMap: st.ParamMap, loc: npt.NDArray, vel: npt.NDArray):
#     paramMap.AddOrSetParam(st.VarType.double, ["Graphics", "Scale"], 200)
#     paramMap.AddOrSetParam(st.VarType.doubleV3, "Location", loc)
#     paramMap.AddOrSetParam(st.VarType.doubleV3, "Velocity", vel)


this = st.GetThisSystem()
doSpawning: bool = st.SimGlobals.GetSimEntity().GetParam(st.VarType.bool, "DoSpawning")

phobos: st.Entity = this.GetParam(st.VarType.entityRef, "Phobos")
phobosFrame = phobos.GetBodyFixedFrame()
mu: float = phobos.GetParam(st.VarType.double, ["Dynamics", "GravitationalParameter_km3_s2"]) * 1e9
radius: float = phobos.GetParam(st.VarType.double, ["#Planet", "General", "Radius_m"])

spread_xyz_m: npt.NDArray = this.GetParam(st.VarType.doubleV3, "Spread_m")
verticalOffset: float = this.GetParam(st.VarType.double, "VerticalOffset_m")

shuttle: st.Entity = this.GetParam(st.VarType.entityRef, "Shuttle")
shuttleFrame = shuttle.GetBodyFixedFrame()
shuttleLoc = shuttle.getLocation().WRT_ExprIn(phobosFrame)
shuttleNWU = st.PlanetUtils.NorthWestUpFromLocation(shuttleLoc, radius)
shuttleLoc += shuttleNWU.up() * verticalOffset

locMin: npt.NDArray = shuttleLoc - spread_xyz_m
locMax: npt.NDArray = shuttleLoc + spread_xyz_m

debrisEntityConfigPaths = this.GetParamArray(st.VarType.string, "DebrisEntityConfigPaths")
debrisWeights = this.GetParamArray(st.VarType.double, "DebrisWeights")
weightSum = sum(debrisWeights)
debrisWeights = [debrisWeights[i] / weightSum for i in range(len(debrisWeights))]

debrisEntityPaths: list[pathlib.Path] = []
for i in range(len(debrisEntityConfigPaths)):
    debrisEntityPaths.append(st.path_utils.AssetPathToReal(st.AssetType.EntityConfig, debrisEntityConfigPaths[i]))

debrisEntities: list[st.Entity] = []

seed = 100
rng = np.random.default_rng(seed)
numDebris: int = this.GetParam(st.VarType.int32, "NumDebris")
if doSpawning:
    random.seed(seed)
    np.random.seed(seed)
    for i in range(numDebris):
        loc_random: npt.NDArray = np.random.random(3)  # location
        vel_random: float = np.random.random()  # velocity magnitude
        azimuth_random: float = np.random.random()  # velocity direction
        angle_random: float = np.random.random()  # angle of rotation (for attitude)
        axis_random: npt.NDArray = np.random.random(3)  # axis of rotation (for attitude)
        angvel_random: npt.NDArray = np.random.random(3)  # angular velocity
        size_random: float = np.random.random()  # size

        # Make azimuth
        azimuth: float = remapFloat(azimuth_random, 0.0, 1.0, 0.0, 2.0 * np.pi)

        # Make location
        loc = remapVector(loc_random, np.zeros(3), np.ones(3), locMin, locMax)
        NWU = st.PlanetUtils.NorthWestUpFromLocation(loc, radius)
        
        # Make tangential velocity
        vel_mag_max: float = np.sqrt(mu / radius)
        vel_mag: float = remapFloat(vel_random, 0.0, 1.0, 0.05 * vel_mag_max, 0.1 * vel_mag_max)
        vel: npt.NDArray = vel_mag * (np.cos(azimuth) * NWU.north() - np.sin(azimuth) * NWU.west())

        # Make w-last quaternion
        angle = remapFloat(angle_random, 0.0, 1.0, 0.0, 2.0 * np.pi)
        axis = remapVector(axis_random, np.zeros(3), np.ones(3), -np.ones(3), np.ones(3))
        axis = axis / np.linalg.norm(axis)
        cosAngle = np.cos(0.5 * angle)
        sinAngle = np.sin(0.5 * angle)
        quat = np.array([sinAngle * axis[0], sinAngle * axis[1], sinAngle * axis[2], cosAngle])
        quat = quat / np.linalg.norm(quat)

        # Make angular velocity
        angvel_mag_max = 0.2  # rad/s
        angvel = remapVector(angvel_random, np.zeros(3), np.ones(3), -np.ones(3), np.ones(3))
        angvel = angvel_mag_max * angvel / np.linalg.norm(angvel)
        
        # Make debris
        randomChosenDebris = random.choices(debrisEntityPaths, weights=debrisWeights, k=1)[0]
        debrisEntityConfig = st.file_io.ReadEntityConfig(randomChosenDebris)

        # Name
        debrisName: str = f"Debris{i}"
        debrisEntityConfig.SetParam(st.VarType.string, ["#Required", "Name"], debrisName)

        # Scale, diameter, etc.
        scaleMin: float = debrisEntityConfig.GetParam(st.VarType.double, "ScaleMin")
        scaleMax: float = debrisEntityConfig.GetParam(st.VarType.double, "ScaleMax")
        size: float = remapFloat(size_random, 0.0, 1.0, scaleMin, scaleMax)
        debrisEntityConfig.SetParam(st.VarType.double, ["Graphics", "Scale"], size)

        debrisRadarDiameter_base: float = debrisEntityConfig.GetParam(st.VarType.double, "RadarDiameter_m")
        debrisEntityConfig.SetParam(st.VarType.double, "RadarDiameter_m", debrisRadarDiameter_base * size)
        debrisEntityConfig.SetParam(st.VarType.double, ["Graphics", "VisualRadius_m"], 0.5 * debrisRadarDiameter_base * size)

        # Loc/vel/rot/angvel
        debrisEntityConfig.AddOrSetParam(st.VarType.doubleV3, "Location", loc)
        debrisEntityConfig.AddOrSetParam(st.VarType.doubleV3, "Velocity", vel)
        debrisEntityConfig.AddOrSetParam(st.VarType.doubleV4, "Rotation", quat)
        debrisEntityConfig.AddOrSetParam(st.VarType.doubleV3, "AngVelocity", angvel)

        # Mass and resident frame
        defaultMass: float = debrisEntityConfig.GetParam(st.VarType.double, ["Dynamics", "Mass"])
        debrisEntityConfig.SetParam(st.VarType.double, ["Dynamics", "Mass"], defaultMass * size ** 3)
        debrisEntityConfig.AddOrSetParam(st.VarType.entityRef, ["Dynamics", "ResidentFrame"], phobos)

        newDebrisEntity = st.SimGlobals.AddEntity(debrisEntityConfig)
        st.SimGlobals.GetSimEntity().AddOrSetParam(st.VarType.entityRef, ["Debris", debrisName], newDebrisEntity)

        debrisEntities.append(newDebrisEntity)
    # Get everything ready for running the sim once it gets saved
    st.SimGlobals.GetSimEntity().SetParam(st.VarType.bool, "DoSpawning", False)
    # st.SimGlobals.SimClock.SetTimescale(1.0)
    st.SimGlobals.GetSimEntity().AddOrSetParam(st.VarType.double, ["Time", "Timescale"], 1.0)

    saveSimPayload = st.ParamMap()
    saveSimPayload.AddParam(st.VarType.string, "FilePath", 
                            str(st.path_utils.AssetPathToReal(st.AssetType.SimConfig, 
                            f"Local/Repos/aero-astro-STU4-Backend/Sims/STU4_{numDebris}Rocks.simconf")))
    st.SimGlobals.Publish("SaveSim", saveSimPayload)

else:
    for i in range(numDebris):
        debrisName: str = f"Debris{i}"
        debrisEntity: st.Entity = st.SimGlobals.GetSimEntity().GetParam(st.VarType.entityRef, ["Debris", debrisName])
        debrisEntities.append(debrisEntity)

        additionalRandomImpulse = st.PhysEffect(st.PhysEffectType.Impulse, phobosFrame)
        deltaV_1sigma_m_s = 0.02
        additionalRandomImpulse.setVector(rng.normal(0.0, debrisEntity.getMass() * deltaV_1sigma_m_s, size=3))
        debrisEntity.setPhysEffect("AdditionalRandomImpulse", additionalRandomImpulse)
        
        additionalRandomAngImpulse = st.PhysEffect(st.PhysEffectType.AngImpulse, phobosFrame)
        deltaV_1sigma_rad_s = 0.2
        additionalRandomAngImpulse.setVector(rng.normal(0.0, deltaV_1sigma_rad_s, size=3))
        debrisEntity.setPhysEffect("AdditionalRandomAngImpulse", additionalRandomAngImpulse)

baseFrame = st.SimGlobals.GetBaseFrame()
inertialFrame: st.frames.Frame = baseFrame

trajectoryAdjustmentLimit_m: float = 100.0
v_rock_typical_m_s: float = 2.0
adjustment_deltaV_factor: float = 0.01

exit_flag = False
doTrajectoryAdjustment = False
iteration = 0
while not exit_flag:
    time.sleep(1.0 / this.GetParam(st.VarType.double, "LoopFreqHz"))

    idx: int = iteration % numDebris
    framed_state = debrisEntities[idx].getLocVelAcc()
    grav_fe = st.SimGlobals.SampleVectorField("Gravity", framed_state).ExprIn(phobosFrame)

    # debrisLoc = framed_state.loc_WRT_ExprIn(phobosFrame)
    # grav_fe = -mu * debrisLoc / (np.linalg.norm(debrisLoc) ** 3)

    gravPhysEffect = st.PhysEffect(st.PhysEffectType.Acceleration, phobosFrame)
    gravPhysEffect.setVector(grav_fe)
    debrisEntities[idx].setPhysEffect("Gravity", gravPhysEffect)

    debrisWRTShuttle = framed_state.loc_WRT(shuttleFrame).ExprIn(phobosFrame)
    d_rock_shuttle_m: float = np.linalg.norm(debrisWRTShuttle)
    if d_rock_shuttle_m > trajectoryAdjustmentLimit_m and doTrajectoryAdjustment:
        huntingImpulse = st.PhysEffect(st.PhysEffectType.Impulse, phobosFrame)
        t_intercept: float = d_rock_shuttle_m / v_rock_typical_m_s
        v_debris = framed_state.vel_WRT_ExprIn(phobosFrame)
        v_target = v_debris - debrisWRTShuttle / t_intercept
        deltaV_hunting_m_s = adjustment_deltaV_factor * (v_target - v_debris)
        impulse_hunting_kgm_s = debrisEntities[idx].getMass() * deltaV_hunting_m_s
        huntingImpulse.setVector(impulse_hunting_kgm_s)
        debrisEntities[idx].setPhysEffect("HuntingImpulse", huntingImpulse)

        # st.OnScreenLogMessage(f"Hunting delta V = {deltaV_hunting_m_s}", "HuntingDeltaV", st.Severity.Info)
        # st.OnScreenLogMessage(f"Hunting impulse = {impulse_hunting_kgm_s}", "HuntingImpulse", st.Severity.Info)

    iteration += 1

st.leave_sim()
