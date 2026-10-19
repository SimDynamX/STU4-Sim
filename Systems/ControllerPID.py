def before_init():
    pass # Replace this to run code during the Sync state that needs to finish before Init starts anywhere in the sim.
    # Sync state is where Sim Editing is done, so code in here shouldn't change parameters that would
    # make saving and reloading the sim problematic.

def before_runtime():
    pass # Replace this to run code during the Init state that needs to finish before Runtime starts anywhere in the sim.

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


this = st.GetThisSystem()

thc_pulsing_x = False
thc_pulsing_y = False
thc_pulsing_z = False

firing = 0

integrated_AngVel_error_x = 0
integrated_AngVel_error_y = 0
integrated_AngVel_error_z = 0

prev_desired_AngVel_x = 0
prev_desired_AngVel_y = 0
prev_desired_AngVel_z = 0

controlled_entity: st.Entity = st.get_entities()[0]
bodyFrame = controlled_entity.GetBodyFixedFrame()
inertial_frame_en: st.Entity = this.GetParam(st.VarType.entityRef, "InertialFrame")
inertial_frame = inertial_frame_en.GetBodyFixedFrame()

# System update loop:
exit_flag = False
while not exit_flag:
    
    # RHC control modes
    rhc_control_mode = controlled_entity.GetParam(st.VarType.string, ["ControlCmd", "RHC", "Mode"])
    if rhc_control_mode == "Direct":
        command_rot_x = controlled_entity.GetParam(st.VarType.double, ["ControlCmd", "RHC", "Roll_Right"])
        command_rot_y = controlled_entity.GetParam(st.VarType.double, ["ControlCmd", "RHC", "Pitch_Down"])
        command_rot_z = controlled_entity.GetParam(st.VarType.double, ["ControlCmd", "RHC", "Yaw_Left"])

        strength_torque = controlled_entity.GetParam(st.VarType.double, ["Thrusters", "TorqueStrength_Nm"])
        torque_x = strength_torque * command_rot_x
        torque_y = strength_torque * command_rot_y
        torque_z = strength_torque * command_rot_z
        torqueVec = np.array([torque_x, torque_y, torque_z])

        # effect = st.PhysEffect(st.PhysEffectType.Torque, bodyFrame)
        # effect.setVector(torqueVec)
        # controlled_entity.setPhysEffect("ThrusterTorque", effect)

        angAccelVec = np.linalg.inv(controlled_entity.getInertiaTensor(bodyFrame)) @ torqueVec
        controlled_entity.AddOrSetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAngAccel"], angAccelVec)

        # attitude = controlled_entity.getRotation()
        # attitude321 = (180.0/ np.pi) * st.math.DCM_to_Euler321(attitude.DCM_WRT(inertial_frame))
        # attitudeQuat = attitude.Quat_WRT(inertial_frame)

        # angvel = controlled_entity.getAngVelocity().WRT(inertial_frame).ExprIn(bodyFrame)

        # These can cause a ton of overhead
        # st.OnScreenAlert(f"Attitude 321  = {np.round(attitude321, 4)}", "AttitudeEuler", st.Severity.Info)
        # st.OnScreenAlert(f"Attitude Quat = {np.round(attitudeQuat, 4)}", "AttitudeQuat", st.Severity.Info)
        # st.OnScreenAlert(f"Ang Velocity  = {np.round(np.rad2deg(angvel), 4)}", "AngVel", st.Severity.Info)

        
    elif rhc_control_mode == "Impulse":
        pass #TODO

    # PID Control Mode
    elif rhc_control_mode == "PID":
        # get RHC control inputs (scaled from -1 to 1)
        command_rot_x: float = controlled_entity.GetParam(st.VarType.double, ["ControlCmd", "RHC", "Roll_Right"])
        command_rot_y: float = controlled_entity.GetParam(st.VarType.double, ["ControlCmd", "RHC", "Pitch_Down"])
        command_rot_z: float = controlled_entity.GetParam(st.VarType.double, ["ControlCmd", "RHC", "Yaw_Left"])

        # add deadband to deal with controller drift
        RHC_deadband = 0.1
        if (command_rot_x > -RHC_deadband) and (command_rot_x < RHC_deadband):
            command_rot_x = 0.0
        if (command_rot_y > -RHC_deadband) and (command_rot_y < RHC_deadband):
            command_rot_y = 0.0
        if (command_rot_z > -RHC_deadband) and (command_rot_z< RHC_deadband):
            command_rot_z = 0.0

        # set the max angular velocity value to scale the controller inputs
        max_AngVel = 0.5
        
        # calculate desired angular velocity based on control input
        desired_AngVel_x = command_rot_x * max_AngVel
        desired_AngVel_y = command_rot_y * max_AngVel
        desired_AngVel_z = command_rot_z * max_AngVel

        # get current angular velocity values
        # current_AngVel = controlled_entity.GetParam(st.VarType.doubleV3, ["AngVelocity"])
        current_AngVel = controlled_entity.getAngVelocity().WRT(inertial_frame).ExprIn(bodyFrame)
        current_AngVel_x: float = current_AngVel[0]
        current_AngVel_y: float = current_AngVel[1]
        current_AngVel_z: float = current_AngVel[2]

        # calculate angular velocity error
        AngVel_error_x = desired_AngVel_x - current_AngVel_x
        AngVel_error_y = desired_AngVel_y - current_AngVel_y
        AngVel_error_z = desired_AngVel_z - current_AngVel_z

        # These can cause a ton of overhead
        # st.OnScreenAlert(f"Desired AngVel = {np.round(
        #     np.array([desired_AngVel_x, desired_AngVel_y, desired_AngVel_z]), 4)}", "DesiredAngVel", st.Severity.Info)
        # st.OnScreenAlert(f"Current AngVel = {np.round(current_AngVel, 4)}", "CurrentAngVel", st.Severity.Info)
        # st.OnScreenAlert(f"AngVel Error   = {np.round(
        #     np.array([AngVel_error_x, AngVel_error_y, AngVel_error_z]), 4)}", "AngVelError", st.Severity.Info)

        # # calculate accumulated error
        # duration: float = 1.0 / this.GetParam(st.VarType.double, "LoopFreqHz")
        # integrated_AngVel_error_x: float = integrated_AngVel_error_x + AngVel_error_x * duration
        # integrated_AngVel_error_y: float = integrated_AngVel_error_y + AngVel_error_y * duration
        # integrated_AngVel_error_z: float = integrated_AngVel_error_z + AngVel_error_z * duration


        # # Calculate desired angular acceleration
        # desired_AngAcc_x = (desired_AngVel_x - prev_desired_AngVel_x) / duration
        # desired_AngAcc_y = (desired_AngVel_y - prev_desired_AngVel_y) / duration
        # desired_AngAcc_z = (desired_AngVel_z - prev_desired_AngVel_z) / duration

        # # update the prev_desired_AngVel values for the next iteration
        # prev_desired_AngVel_x = desired_AngVel_x
        # prev_desired_AngVel_y = desired_AngVel_y
        # prev_desired_AngVel_z = desired_AngVel_z

        # # get current angular acceleration values
        # # current_AngAcc = controlled_entity.GetParam(st.VarType.doubleV3, ["AngAcceleration"])
        # current_AngAcc = controlled_entity.getAngAcceleration().WRT(inertial_frame).ExprIn(controlled_entity.GetBodyFixedFrame())
        # current_AngAcc_x = current_AngAcc[0]
        # current_AngAcc_y = current_AngAcc[1]
        # current_AngAcc_z = current_AngAcc[2]

        # # calculate angular acceleration error
        # AngAcc_error_x = desired_AngAcc_x - current_AngAcc_x
        # AngAcc_error_y = desired_AngAcc_y - current_AngAcc_y
        # AngAcc_error_z = desired_AngAcc_z - current_AngAcc_z


        # PID Controller
        kp_pitch = 10.0
        kp_yaw = 10.0
        kp_roll = 10.0
        ki = 0.0
        kd = 0.0

        strength_torque = controlled_entity.GetParam(st.VarType.double, ["Thrusters", "TorqueStrength_Nm"])

        torque_x = kp_roll * AngVel_error_x * strength_torque# + ki*integrated_AngVel_error_x * strength_torque + kd*AngAcc_error_x * strength_torque
        torque_y = kp_pitch * AngVel_error_y * strength_torque# + ki*integrated_AngVel_error_y * strength_torque + kd*AngAcc_error_y * strength_torque
        torque_z = kp_yaw * AngVel_error_z * strength_torque# + ki*integrated_AngVel_error_z * strength_torque + kd*AngAcc_error_z * strength_torque

        if (torque_x > strength_torque):
            torque_x = strength_torque
        if (torque_x < -strength_torque):
            torque_x = -strength_torque

        if (torque_y > strength_torque):
            torque_y = strength_torque
        if (torque_y < -strength_torque):
            torque_y = -strength_torque

        if (torque_z > strength_torque):
            torque_z = strength_torque
        if (torque_z < -strength_torque):
            torque_z = -strength_torque

        #record all of the control system data for easy plotting
        controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Desired", "AngVel_X"], desired_AngVel_x)
        controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Desired", "AngVel_Y"], desired_AngVel_y)
        controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Desired", "AngVel_Z"], desired_AngVel_z)
        # controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Desired", "AngAcc_X"], desired_AngAcc_x)
        # controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Desired", "AngAcc_Y"], desired_AngAcc_y)
        # controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Desired", "AngAcc_Z"], desired_AngAcc_z)
        
        
        controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Current", "AngVel_X"], current_AngVel_x)
        controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Current", "AngVel_Y"], current_AngVel_y)
        controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Current", "AngVel_Z"], current_AngVel_z)
        # controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Current", "AngAcc_X"], current_AngAcc_x)
        # controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Current", "AngAcc_Y"], current_AngAcc_y)
        # controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Current", "AngAcc_Z"], current_AngAcc_z)
        

        controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Error", "AngVel_X"], AngVel_error_x)
        controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Error", "AngVel_Y"], AngVel_error_y)
        controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Error", "AngVel_Z"], AngVel_error_z)
        # controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Error", "AngAcc_X"], AngAcc_error_x)
        # controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Error", "AngAcc_Y"], AngAcc_error_y)
        # controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Error", "AngAcc_Z"], AngAcc_error_z)

        
        controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Torque", "X"], torque_x)
        controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Torque", "Y"], torque_y)
        controlled_entity.SetParam(st.VarType.double, ["AttitudeControlSystem", "Torque", "Z"], torque_z)
        torqueVec = np.array([torque_x, torque_y, torque_z])
       
        #apply the torques
        # effect = st.PhysEffect(st.PhysEffectType.Torque, inertial_frame)
        # effect = st.PhysEffect(st.PhysEffectType.Torque, bodyFrame)
        # effect.setVector(torqueVec)

        # controlled_entity.setPhysEffect("ThrusterTorque", effect)

        angAccelVec = np.linalg.inv(controlled_entity.getInertiaTensor(bodyFrame)) @ torqueVec
        controlled_entity.AddOrSetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAngAccel"], angAccelVec)

    # THC control modes
    thc_control_mode = controlled_entity.GetParam(st.VarType.string, ["ControlCmd", "THC", "Mode"])
    if thc_control_mode == "Direct":
        command_trans_x = controlled_entity.GetParam(st.VarType.double, ["ControlCmd", "THC", "X"])
        command_trans_y = controlled_entity.GetParam(st.VarType.double, ["ControlCmd", "THC", "Y"])
        command_trans_z = controlled_entity.GetParam(st.VarType.double, ["ControlCmd", "THC", "Z"])

        strength_force = controlled_entity.GetParam(st.VarType.double, ["Thrusters", "ForceStrength_N"])
        force_x = strength_force * command_trans_x
        force_y = strength_force * command_trans_y
        force_z = strength_force * command_trans_z
        forceVec = np.array([force_x, force_y, force_z])
        
        # effect = st.PhysEffect(st.PhysEffectType.Force, bodyFrame)
        # effect.setVector(forceVec)

        # controlled_entity.setPhysEffect("ThrusterForce", effect)
        # Continuous force until it's updated or removed

        accelVec = forceVec / controlled_entity.getMass()
        controlled_entity.AddOrSetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAccel"], accelVec)

    elif thc_control_mode == "Impulse":
        # mass = controlled_entity.GetParam(st.VarType.double, ["Dynamics", "Mass"])
        # I = controlled_entity.GetParam(st.VarType.doubleM3x3, ["Dynamics", "Inertia_BodyFrame"])

        ctrl_input_x: float = controlled_entity.GetParam(st.VarType.double, ["ControlCmd", "THC", "X"])
        ctrl_input_y: float = controlled_entity.GetParam(st.VarType.double, ["ControlCmd", "THC", "Y"])
        ctrl_input_z: float = controlled_entity.GetParam(st.VarType.double, ["ControlCmd", "THC", "Z"])

        command_trans_x: float = 0.0
        command_trans_y: float = 0.0
        command_trans_z: float = 0.0
        
        IMPULSE_THRESHOLD = 0.5
        THRESHOLD_HYSTERESIS = 0.1

        if thc_pulsing_x:
            if abs(ctrl_input_x) < (IMPULSE_THRESHOLD - THRESHOLD_HYSTERESIS):
                thc_pulsing_x = False
                st.OnScreenLogMessage("Stopped pulsing THC X", "Thrusters", st.Severity.Info)
        else:
            if abs(ctrl_input_x) > (IMPULSE_THRESHOLD + THRESHOLD_HYSTERESIS):
                thc_pulsing_x = True
                command_trans_x = ctrl_input_x
                st.OnScreenLogMessage("Pulsing THC X", "Thrusters", st.Severity.Info)

        if thc_pulsing_y:
            if abs(ctrl_input_y) < (IMPULSE_THRESHOLD - THRESHOLD_HYSTERESIS):
                thc_pulsing_y = False
                st.OnScreenLogMessage("Stopped pulsing THC Y", "Thrusters", st.Severity.Info)
        else:
            if abs(ctrl_input_y) > (IMPULSE_THRESHOLD + THRESHOLD_HYSTERESIS):
                thc_pulsing_y = True
                command_trans_y = ctrl_input_y
                st.OnScreenLogMessage("Pulsing THC Y", "Thrusters", st.Severity.Info)
        
        if thc_pulsing_z:
            if abs(ctrl_input_z) < (IMPULSE_THRESHOLD - THRESHOLD_HYSTERESIS):
                thc_pulsing_z = False
                st.OnScreenLogMessage("Stopped pulsing THC Z", "Thrusters", st.Severity.Info)
        else:
            if abs(ctrl_input_z) > (IMPULSE_THRESHOLD + THRESHOLD_HYSTERESIS):
                thc_pulsing_z = True
                command_trans_z = ctrl_input_z
                st.OnScreenLogMessage("Pulsing THC Z", "Thrusters", st.Severity.Info)

        # If there is a command, add impulse for it
        if (command_trans_x != 0.0) or (command_trans_y != 0.0) or (command_trans_z != 0.0):
            strength_force = controlled_entity.GetParam(st.VarType.double, ["Thrusters", "ForceStrength_N"])
            strength_impulse = strength_force / 60.0  # multiply by UE sim dt (lowkey mega scuffed)
            # strength_impulse /= controlled_entity.GetParam(st.VarType.double, ["Dynamics", "Mass"])
            #TODO sense trigger and recentering per axis
            force_x: float = strength_impulse * command_trans_x
            force_y: float = strength_impulse * command_trans_y
            force_z: float = strength_impulse * command_trans_z
            forceVec = np.array([force_x, force_y, force_z])
            firing += 1
            
            effect = st.PhysEffect(st.PhysEffectType.Impulse, controlled_entity.GetBodyFixedFrame())
            impulse_vec = np.array([force_x, force_y, force_z])
            effect.setVector(impulse_vec)

            st.OnScreenLogMessage(f"Applying impulse of magntidue {np.linalg.norm(impulse_vec)} N*s", 
                                  "Impulse", st.Severity.Info)
            
            accelVec = forceVec / controlled_entity.getMass()
            controlled_entity.AddOrSetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAccel"], accelVec)

            # controlled_entity.setPhysEffect(f"ThrusterImpulse{firing}", effect)

    time.sleep(1.0 / this.GetParam(st.VarType.double, "LoopFreqHz"))

st.leave_sim()

