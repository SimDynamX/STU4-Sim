def before_init():
    pass


def before_runtime():
    pass


# DON'T CHANGE ANY OF THE BELOW; NECESSARY FOR JOINING SIMULATION
import os, sys, time, datetime, traceback
import spaceteams as st
def custom_exception_handler(exctype, value, tb):
    error_message = "".join(traceback.format_exception(exctype, value, tb))
    st.logger_fatal(error_message)
sys.excepthook = custom_exception_handler
st.BeforeInit(before_init)
st.BeforeRuntime(before_runtime)
st.connect_to_sim(sys.argv)
import numpy as np
# DON'T CHANGE ANY OF THE ABOVE; NECESSARY FOR JOINING SIMULATION
################################################################
# Now, when below the boilerplate section, you can import more things

import numpy.typing as npt


def clampFloat(val: float, outMin: float, outMax: float) -> float:
    return min(max(val, outMin), outMax)


def clampVector(val: npt.NDArray, outMin: npt.NDArray, outMax: npt.NDArray) -> npt.NDArray:
    return np.minimum(np.maximum(val, outMin), outMax)


def remapFloat(val: float, inMin: float, inMax: float, outMin: float, outMax: float) -> float:
    return clampFloat(outMin + (outMax - outMin) * (val - inMin) / (inMax - inMin), outMin, outMax)


def remapVector(val: npt.NDArray, inMin: npt.NDArray, inMax: npt.NDArray, outMin: npt.NDArray, outMax: npt.NDArray) -> npt.NDArray:
    return clampVector(outMin + (outMax - outMin) * (val - inMin) / (inMax - inMin), outMin, outMax)


this = st.GetThisSystem()

doSpawning = st.SimGlobals.GetSimEntity().GetParam(st.VarType.bool, "DoSpawning")
if doSpawning:
    while True:
        time.sleep(1.0 / this.GetParam(st.VarType.double, "LoopFreqHz"))

phobos: st.Entity = this.GetParam(st.VarType.entityRef, "Phobos")
phobosFrame = phobos.GetBodyFixedFrame()
shuttle: st.Entity = this.GetParam(st.VarType.entityRef, "Shuttle")
shuttleFrame = shuttle.GetBodyFixedFrame()

mu: float = phobos.GetParam(st.VarType.double, ["Dynamics", "GravitationalParameter_km3_s2"]) * 1e9
radius: float = phobos.GetParam(st.VarType.double, ["#Planet", "General", "Radius_m"])

# Errors
rng = np.random.default_rng(100)
r_1sigma_m = this.GetParam(st.VarType.double, "Position_1sigma_m")
v_1sigma_m_s = this.GetParam(st.VarType.double, "Velocity_1sigma_m_s")
diameter_1sigma_fraction = this.GetParam(st.VarType.double, "Diameter_1sigma_fraction")

debris: list[st.Entity] = []
debrisData: list[npt.NDArray] = []

# debrisLocations: list[npt.NDArray] = []
# debrisRangeRates: list[float] = []
# debrisDiameters: list[float] = []
numDebris: int = this.GetParam(st.VarType.int32, "NumDebris")
for i in range(numDebris):
    debrisName: str = f"Debris{i}"
    pieceOfDebris: st.Entity = st.SimGlobals.GetSimEntity().GetParam(st.VarType.entityRef, ["Debris", debrisName])
    debris.append(pieceOfDebris)

    debrisFramedState = debris[i].getLocVelAcc()
    debrisLoc = debrisFramedState.loc_WRT(shuttleFrame).ExprIn(phobosFrame)
    debrisVel = debrisFramedState.vel_WRT(shuttleFrame).ExprIn(phobosFrame)
    debrisRangeRate: float = debrisVel.dot(debrisLoc / np.linalg.norm(debrisLoc))
    debrisRadarDiameter: float = debris[i].GetParam(st.VarType.double, "RadarDiameter_m")
    debrisClosestPoint = np.linalg.norm(debrisLoc) - 0.5 * debrisRadarDiameter

    debrisLoc_error = np.linalg.cholesky(np.diag([r_1sigma_m ** 2, r_1sigma_m ** 2, r_1sigma_m ** 2])) \
        @ rng.normal(0.0, 1.0, size=3)
    debrisClosestPoint_error = rng.normal(0.0, r_1sigma_m, size=1).item()
    debrisRangeRate_error = rng.normal(0.0, v_1sigma_m_s, size=1).item()
    debrisRadarDiameter_error = rng.normal(0.0, debrisRadarDiameter * diameter_1sigma_fraction, size=1).item()

    debrisLoc += debrisLoc_error
    debrisClosestPoint += debrisClosestPoint_error
    debrisRangeRate += debrisRangeRate_error
    debrisRadarDiameter += debrisRadarDiameter_error

    debrisData.append(np.array([[debrisLoc[0], debrisLoc[1], debrisLoc[2]],
                                [debrisClosestPoint, debrisRangeRate, debrisRadarDiameter],
                                [0.0, 0.0, 0.0]]))

    # debrisLocations.append(debrisLoc)
    # debrisRangeRates.append(debrisRangeRate)
    # debrisDiameters.append(debrisRadarDiameter)

# sortedIndices = sorted(range(len(debrisLocations)), key=lambda i: np.linalg.norm(debrisLocations[i]), reverse=True)
# debrisLocations = [debrisLocations[i] for i in sortedIndices]
# debrisRangeRates = [debrisRangeRates[i] for i in sortedIndices]
# debrisDiameters = [debrisDiameters[i] for i in sortedIndices]

sortedDebrisData = sorted(debrisData, key=lambda thisDebris: np.linalg.norm(thisDebris[0, :]))
shuttle.SetParamArray(st.VarType.doubleM3x3, ["Radar", "SortedDebrisData"], sortedDebrisData)

baseFrame = st.SimGlobals.GetBaseFrame()
inertialFrame: st.frames.Frame = baseFrame

sortFrequency = int(numDebris / 10)
numToDisplay: int = shuttle.GetParam(st.VarType.int32, ["Radar", "NumToDisplay"])
maxDisplayDistance_m: float = shuttle.GetParam(st.VarType.double, ["Radar", "MaxDisplayDistance_m"])

t0: datetime.datetime = st.SimGlobals.SimClock.GetTimeNow().as_datetime()
exit_flag = False
iteration = 0
while not exit_flag:
    time.sleep(1.0 / this.GetParam(st.VarType.double, "LoopFreqHz"))

    T_shuttlePhobos = shuttle.getRotation().DCM_WRT(phobosFrame)

    idx: int = iteration % numDebris
    debrisFramedState = debris[idx].getLocVelAcc()
    debrisLoc = debrisFramedState.loc_WRT(shuttleFrame).ExprIn(phobosFrame)
    debrisVel = debrisFramedState.vel_WRT(shuttleFrame).ExprIn(phobosFrame)
    debrisRangeRate = debrisVel.dot(debrisLoc / np.linalg.norm(debrisLoc))
    debrisRadarDiameter = debris[idx].GetParam(st.VarType.double, "RadarDiameter_m")
    debrisClosestPoint = np.linalg.norm(debrisLoc) - 0.5 * debrisRadarDiameter

    debrisLoc_error = np.linalg.cholesky(np.diag([r_1sigma_m ** 2, r_1sigma_m ** 2, r_1sigma_m ** 2])) \
        @ rng.normal(0.0, 1.0, size=3)
    debrisClosestPoint_error = rng.normal(0.0, r_1sigma_m, size=1).item()
    debrisRangeRate_error = rng.normal(0.0, v_1sigma_m_s, size=1).item()
    debrisRadarDiameter_error = rng.normal(0.0, debrisRadarDiameter * diameter_1sigma_fraction, size=1).item()

    debrisLoc += debrisLoc_error
    debrisClosestPoint += debrisClosestPoint_error
    debrisRangeRate += debrisRangeRate_error
    debrisRadarDiameter += debrisRadarDiameter_error

    t_now: datetime.datetime = st.SimGlobals.SimClock.GetTimeNow().as_datetime()
    t_from_start: float = (t_now - t0).total_seconds()
    debrisData[idx] = np.array([[debrisLoc[0], debrisLoc[1], debrisLoc[2]],
                                [debrisClosestPoint, debrisRangeRate, debrisRadarDiameter],
                                [t_from_start, 0.0, 0.0]])

    # debrisLocations[idx] = debrisLoc
    # debrisRangeRates[idx] = debrisRangeRate
    # debrisDiameters[idx] = debrisRadarDiameter

    if idx % sortFrequency == 0:
        sortedDebrisData = sorted(debrisData, key=lambda thisDebris: np.linalg.norm(thisDebris[0, :]))
        shuttle.SetParamArray(st.VarType.doubleM3x3, ["Radar", "SortedDebrisData"], sortedDebrisData)

        lenToDisplay = min(len(sortedDebrisData), numToDisplay)
        # st.OnScreenLogMessage(f"lenToDisplay = {lenToDisplay}", "RadarDisplay", st.Severity.Info)

        if lenToDisplay > 0:
            firstRange = np.linalg.norm(sortedDebrisData[0][0, :])
            firstClosestPoint = sortedDebrisData[0][1, 0]
            firstRangeRate = sortedDebrisData[0][1, 1]
            firstDiameter = sortedDebrisData[0][1, 2]
            dataDisplay: npt.NDArray = np.array([firstRange, firstRangeRate, firstDiameter])
            shuttle.SetParam(st.VarType.doubleV3, ["Radar", "DataDisplay"], dataDisplay)

        cylindricalDisplay: list[npt.NDArray] = []
        for j in range(lenToDisplay):
            # thisDataLoc = T_shuttlePhobos @ sortedDebrisData[j][0, :]
            thisDataLoc = st.frames.FramedVector(sortedDebrisData[j][0, :], phobosFrame).ExprIn(shuttleFrame)
            thisData_rho2 = thisDataLoc[0] ** 2 + thisDataLoc[1] ** 2
            
            if thisData_rho2 <= maxDisplayDistance_m * maxDisplayDistance_m:
                thisData_angle: float = np.arctan2(thisDataLoc[1], thisDataLoc[0])
                thisData_z: float = thisDataLoc[2]
                thisData_diameter = sortedDebrisData[j][1, 2]
                cylindricalDisplay.append(np.array([np.sqrt(thisData_rho2) / maxDisplayDistance_m, 
                                        thisData_angle, 
                                        thisData_z / maxDisplayDistance_m, 
                                        thisData_diameter / maxDisplayDistance_m]))

        shuttle.SetParamArray(st.VarType.doubleV4, ["Radar", "CylindricalDisplay"], cylindricalDisplay)

        # st.OnScreenLogMessage(f"{[float(c[0]) * maxDisplayDistance_m for c in cylindricalDisplay]}", "RadarDisplay", st.Severity.Info)
        # st.OnScreenLogMessage(f"{[float(c[1]) for c in cylindricalDisplay]}", "RadarDisplay", st.Severity.Info)
        # st.OnScreenLogMessage(f"{[float(c[2]) for c in cylindricalDisplay]}", "RadarDisplay", st.Severity.Info)
        # st.OnScreenLogMessage(f"{[float(c[3]) for c in cylindricalDisplay]}", "RadarDisplay", st.Severity.Info)

    iteration += 1

st.leave_sim()
