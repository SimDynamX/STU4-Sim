def before_init():
    pass # Replace this to run code during the Sync state that needs to finish before Init starts anywhere in the sim.
    # Sync state is where Sim Editing is done, so code in here shouldn't change parameters that would
    # make saving and reloading the sim problematic.

def before_runtime():
    pass # Replace this to run code during the Init state that needs to finish before Runtime starts anywhere in the sim.

#################################################################
# DON'T CHANGE ANY OF THE BELOW; NECESSARY FOR JOINING SIMULATION
import os, sys, time, datetime, traceback
import spaceteams as st
def custom_exception_handler(exctype, value, tb):
    error_message = "".join(traceback.format_exception(exctype, value, tb))
    st.logger_fatal(error_message)
    exit(1)
sys.excepthook = custom_exception_handler
st.BeforeInit(before_init)
st.BeforeRuntime(before_runtime)
st.connect_to_sim(sys.argv)
import numpy as np
# DON'T CHANGE ANY OF THE ABOVE; NECESSARY FOR JOINING SIMULATION
#################################################################

from pynput import keyboard


def on_press(key : keyboard.Key | keyboard.KeyCode):
    entity: st.Entity = st.get_entities()[0]
    try:
        # RHC Pitch:
        if key == keyboard.Key.up:
            entity.SetParam(st.VarType.double, ["ControlCmd", "RHC", "Pitch_Down"], 1.0)
        elif key == keyboard.Key.down:
            entity.SetParam(st.VarType.double, ["ControlCmd", "RHC", "Pitch_Down"], -1.0)
        # RHC Yaw:
        elif key == keyboard.Key.left:
            entity.SetParam(st.VarType.double, ["ControlCmd", "RHC", "Yaw_Left"], 1.0)
        elif key == keyboard.Key.right:
            entity.SetParam(st.VarType.double, ["ControlCmd", "RHC", "Yaw_Left"], -1.0)
        elif hasattr(key, 'char'):
            # RHC Roll:
            if key.char == '9':
                entity.SetParam(st.VarType.double, ["ControlCmd", "RHC", "Roll_Right"], -1.0)
            elif key.char == '0':
                entity.SetParam(st.VarType.double, ["ControlCmd", "RHC", "Roll_Right"], 1.0)
            # X-Axis:
            elif key.char == 'i':
                entity.SetParam(st.VarType.double, ["ControlCmd", "THC", "Z"], 1.0)
            elif key.char == 'k':
                entity.SetParam(st.VarType.double, ["ControlCmd", "THC", "Z"], -1.0)
            # Y-Axis:
            elif key.char == 'j':
                entity.SetParam(st.VarType.double, ["ControlCmd", "THC", "Y"], 1.0)
            elif key.char == 'l':
                entity.SetParam(st.VarType.double, ["ControlCmd", "THC", "Y"], -1.0)
            # Z-Axis:
            elif key.char == 'h':
                entity.SetParam(st.VarType.double, ["ControlCmd", "THC", "X"], 1.0)
            elif key.char == 'n':
                entity.SetParam(st.VarType.double, ["ControlCmd", "THC", "X"], -1.0)
    except AttributeError:
        st.logger_warn('special key {0} pressed'.format(key))


def on_release(key : keyboard.Key | keyboard.KeyCode):
    entity: st.Entity = st.get_entities()[0]
    try:
        # RHC Pitch:
        if key == keyboard.Key.up:
            entity.SetParam(st.VarType.double, ["ControlCmd", "RHC", "Pitch_Down"], 0.0)
        elif key == keyboard.Key.down:
            entity.SetParam(st.VarType.double, ["ControlCmd", "RHC", "Pitch_Down"], 0.0)
        # RHC Yaw:
        elif key == keyboard.Key.left:
            entity.SetParam(st.VarType.double, ["ControlCmd", "RHC", "Yaw_Left"], 0.0)
        elif key == keyboard.Key.right:
            entity.SetParam(st.VarType.double, ["ControlCmd", "RHC", "Yaw_Left"], 0.0)
        elif hasattr(key, 'char'):
            # RHC Roll:
            if key.char == '9':
                entity.SetParam(st.VarType.double, ["ControlCmd", "RHC", "Roll_Right"], 0.0)
            elif key.char == '0':
                entity.SetParam(st.VarType.double, ["ControlCmd", "RHC", "Roll_Right"], 0.0)
            # X-Axis:
            elif key.char == 'i':
                entity.SetParam(st.VarType.double, ["ControlCmd", "THC", "Z"], 0.0)
            elif key.char == 'k':
                entity.SetParam(st.VarType.double, ["ControlCmd", "THC", "Z"], 0.0)
            # Y-Axis:
            elif key.char == 'j':
                entity.SetParam(st.VarType.double, ["ControlCmd", "THC", "Y"], 0.0)
            elif key.char == 'l':
                entity.SetParam(st.VarType.double, ["ControlCmd", "THC", "Y"], 0.0)
            # Z-Axis:
            elif key.char == 'h':
                entity.SetParam(st.VarType.double, ["ControlCmd", "THC", "X"], 0.0)
            elif key.char == 'n':
                entity.SetParam(st.VarType.double, ["ControlCmd", "THC", "X"], 0.0)
    except AttributeError:
        st.logger_warn('special key {0} released'.format(key))


this = st.GetThisSystem()

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

thc : st.Entity = this.GetParam(st.VarType.entityRef, "thc")
rhc : st.Entity = this.GetParam(st.VarType.entityRef, "rhc")

exit_flag = False
while not exit_flag:
    entity: st.Entity = st.get_entities()[0]
    thc_vector = np.array([thc.GetParam(st.VarType.double, ["THC", "X"]),
                           thc.GetParam(st.VarType.double, ["THC", "Y"]),
                           thc.GetParam(st.VarType.double, ["THC", "Z"])])
    if np.any(thc_vector):
        entity.SetParam(st.VarType.double, ["ControlCmd", "THC", "X"], thc_vector[0])
        entity.SetParam(st.VarType.double, ["ControlCmd", "THC", "Y"], thc_vector[1])
        entity.SetParam(st.VarType.double, ["ControlCmd", "THC", "Z"], thc_vector[2])
    else:
        entity.SetParam(st.VarType.double, ["ControlCmd", "THC", "X"], 0)
        entity.SetParam(st.VarType.double, ["ControlCmd", "THC", "Y"], 0)
        entity.SetParam(st.VarType.double, ["ControlCmd", "THC", "Z"], 0)

    # Ordered as x,y,z components of angular velocity, not euler angles
    rhc_vector = np.array([rhc.GetParam(st.VarType.double, ["RHC", "Roll_Right"]), 
                           rhc.GetParam(st.VarType.double, ["RHC", "Pitch_Down"]),
                           rhc.GetParam(st.VarType.double, ["RHC", "Yaw_Left"])])
    if np.any(rhc_vector):
        entity.SetParam(st.VarType.double, ["ControlCmd", "RHC", "Roll_Right"], rhc_vector[0])
        entity.SetParam(st.VarType.double, ["ControlCmd", "RHC", "Pitch_Down"], rhc_vector[1])
        entity.SetParam(st.VarType.double, ["ControlCmd", "RHC", "Yaw_Left"], rhc_vector[2])
    else:
        entity.SetParam(st.VarType.double, ["ControlCmd", "RHC", "Roll_Right"], 0)
        entity.SetParam(st.VarType.double, ["ControlCmd", "RHC", "Pitch_Down"], 0)
        entity.SetParam(st.VarType.double, ["ControlCmd", "RHC", "Yaw_Left"], 0)

    time.sleep(1.0 / this.GetParam(st.VarType.double, "LoopFreqHz"))

st.leave_sim()
