# DON'T CHANGE ANY OF THE BELOW; NECESSARY FOR JOINING SIMULATION
import os, sys, time, datetime, traceback
import spaceteams as st
def custom_exception_handler(exctype, value, tb):
    error_message = "".join(traceback.format_exception(exctype, value, tb))
    st.logger_fatal(error_message)
sys.excepthook = custom_exception_handler
st.connect_to_sim(sys.argv)
# DON'T CHANGE ANY OF THE ABOVE; NECESSARY FOR JOINING SIMULATION
################################################################
# Now, when below the boilerplate section, you can import more things

import numpy as np
import numpy.typing as npt


this = st.GetThisSystem()
phobos: st.Entity = this.GetParam(st.VarType.entityRef, "Phobos")
phobosFrame = phobos.GetBodyFixedFrame()
shuttle: st.Entity = this.GetParam(st.VarType.entityRef, "Shuttle")
shuttleFrame = shuttle.GetBodyFixedFrame()
target: st.Entity = this.GetParam(st.VarType.entityRef, "Target")
targetFrame = target.GetBodyFixedFrame()

missionSuccess: bool = False
missionFailure: bool = False


def clampFloat(val: float, outMin: float, outMax: float) -> float:
    return min(max(val, outMin), outMax)


def remapFloat(val: float, inMin: float, inMax: float, outMin: float, outMax: float) -> float:
    return clampFloat(outMin + (outMax - outMin) * (val - inMin) / (inMax - inMin), outMin, outMax)


def remapFloat_unclamped(val: float, inLow: float, inHigh: float, outLow: float, outHigh: float) -> float:
    return outLow + (outHigh - outLow) * (val - inLow) / (inHigh - inLow)


def DistancePenalty(distanceToShip: float, initial_distanceToShip: float) -> float:
    return distanceToShip / initial_distanceToShip


def TimeScore(time_s: float, nominalTime_s: float) -> float:
    if time_s != 0.0:
        return nominalTime_s / time_s
    else:
        return 0.0


def DamagePenalty(damageTaken_J: float, maxAllowedDamage_J: float) -> float:
    return 2.0 * damageTaken_J / maxAllowedDamage_J


def FuelPenalty(fuelUsed_kg: float, nominalFuel_kg: float) -> float:
    return max(fuelUsed_kg / nominalFuel_kg, 0.0)


def FullScoreWin(time_s: float, nominalTime_s: float, damageTaken_J: float, maxAllowedDamage_J: float, fuelUsed_kg: float, nominalFuel_kg: float) -> float:
    rawScore = TimeScore(time_s, nominalTime_s) - DamagePenalty(damageTaken_J, maxAllowedDamage_J) - FuelPenalty(fuelUsed_kg, nominalFuel_kg)
    return remapFloat_unclamped(rawScore, -0.5, 0.5, 0.0, 100.0)


def FullScoreLoss(distanceToShip: float, initial_distanceToShip: float, damageTaken_J: float, maxAllowedDamage_J: float, fuelUsed_kg: float, nominalFuel_kg: float) -> float:
    rawScore = -DistancePenalty(distanceToShip, initial_distanceToShip) - DamagePenalty(damageTaken_J, maxAllowedDamage_J) - FuelPenalty(fuelUsed_kg, nominalFuel_kg)
    return remapFloat_unclamped(rawScore, -0.5, 0.5, 0.0, 100.0)


def YouWon(time_s: float, nominalTime_s: float, damageTaken_J: float, maxDamage_J: float, fuelUsed_kg: float, nominalFuel_kg: float) -> None:
    score = FullScoreWin(time_s, nominalTime_s, damageTaken_J, maxDamage_J, fuelUsed_kg, nominalFuel_kg)
    st.OnScreenAlert(f"Successfully escaped Phobos with a score of {score:.1f}; congratulations!", "Winning", st.Severity.Warning)


def YouLost(distanceToShip: float, initial_distanceToShip: float, 
            damageTaken_J: float, maxDamage_J: float, fuelUsed_kg: float, nominalFuel_kg: float) -> None:
    score = FullScoreLoss(distanceToShip, initial_distanceToShip, damageTaken_J, maxDamage_J, fuelUsed_kg, nominalFuel_kg)
    st.OnScreenAlert(f"Crashed into some debris and finished with a score of {score:.1f}; Please try again :)", "Losing", st.Severity.Warning)


def TooMuchDamage(damageTaken_J: float, maxAllowedDamage_J: float) -> None:
    damage_TNT: float = damageTaken_J / 4.184e6
    maxDamage_TNT: float = maxAllowedDamage_J / 4.184e6
    st.OnScreenAlert(f"Accumulated {damage_TNT:.3f} kg of TNT worth of damage, exceeding the shuttle's damage threshold of {maxDamage_TNT:.3f} kg of TNT.", "TooMuchDamage", st.Severity.Error)


t0: st.timestamp = st.SimGlobals.SimClock.GetTimeNow()
initial_distanceToShip: float = np.linalg.norm(shuttle.getLocation().WRT_ExprIn(targetFrame))

st.OnScreenAlert("Backend Sim Version: 2026/10/30", "SimVersion", st.Severity.Info)

sim_elapsed_time: float = 0.0
exit_flag: bool = False
while not exit_flag:
    if not missionSuccess:
        
        ################
        # Damage check #
        ################

        damage_J: float = shuttle.GetParam(st.VarType.double, ["Stats", "Damage"])
        maxDamage_J: float = this.GetParam(st.VarType.double, "DamageThreshold_J")
        fuelUsed_kg: float = shuttle.GetParam(st.VarType.double, ["Stats", "FuelUsed"])
        nominalFuel_kg: float = this.GetParam(st.VarType.double, "NominalFuel_kg")

        nominalTime_s: float = this.GetParam(st.VarType.double, "NominalTime_s")
        distanceToShip: float = np.linalg.norm(shuttle.getLocation().WRT_ExprIn(targetFrame))
        
        if damage_J > maxDamage_J and not missionFailure:
            missionFailure = True
            TooMuchDamage(damage_J, maxDamage_J)
            YouLost(distanceToShip, initial_distanceToShip, damage_J, maxDamage_J, fuelUsed_kg, nominalFuel_kg)
            st.OnScreenAlert(f"Sim elapsed time   = {round(sim_elapsed_time, 2)} s", "SET", st.Severity.Warning)
            st.OnScreenAlert(f"Distance to ship   = {round(distanceToShip, 2)} m", "Distance", st.Severity.Warning)
            st.OnScreenAlert(f"Total damage taken = {round(damage_J, 2)} J", "DamageTaken", st.Severity.Warning)
            st.OnScreenAlert(f"Total fuel used    = {round(fuelUsed_kg, 2)} kg", "FuelUsed", st.Severity.Warning)

        #################
        # Winning check #
        #################

        if not missionFailure:
            keepOutRadius = this.GetParam(st.VarType.double, "KeepOutRadius_m")
            if np.linalg.norm(shuttle.getLocation().WRT(targetFrame).ExprIn(phobosFrame)) <= keepOutRadius:
                YouWon(sim_elapsed_time, nominalTime_s, damage_J, maxDamage_J, fuelUsed_kg, nominalFuel_kg)
                missionSuccess = True
                st.OnScreenAlert(f"Sim elapsed time   = {round(sim_elapsed_time, 2)} s", "SET", st.Severity.Warning)
                st.OnScreenAlert(f"Distance to ship   = {round(distanceToShip, 2)} m", "Distance", st.Severity.Warning)
                st.OnScreenAlert(f"Total damage taken = {round(damage_J, 2)} J", "DamageTaken", st.Severity.Warning)
                st.OnScreenAlert(f"Total fuel used    = {round(fuelUsed_kg, 2)} kg", "FuelUsed", st.Severity.Warning)

    #####################
    # Step time forward #
    #####################

    tf: st.timestamp = st.SimGlobals.SimClock.GetTimeNow()
    dt: datetime.timedelta = tf.as_datetime() - t0.as_datetime()
    
    t0 = tf
    sim_elapsed_time += dt.total_seconds()
    time.sleep(1.0 / this.GetParam(st.VarType.double, "LoopFreqHz"))

st.leave_sim()