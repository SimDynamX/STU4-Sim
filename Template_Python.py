# THIS COMMENT LINE SHOULD BE THE FIRST LINE OF THE FILE
# DON'T CHANGE ANY OF THE BELOW; NECESSARY FOR JOINING SIMULATION
import os, sys, time, datetime, traceback
import spaceteams as sc
def custom_exception_handler(exctype, value, tb):
    error_message = "".join(traceback.format_exception(exctype, value, tb))
    sc.logger_fatal(error_message)
    exit(1)
sys.excepthook = custom_exception_handler
sc.connect_to_sim(sys.argv)
import numpy as np
# DON'T CHANGE ANY OF THE ABOVE; NECESSARY FOR JOINING SIMULATION
#################################################################

# Now, when below the boilerplate section, you can import more things
# import matplotlib.pyplot as plt


# Do whatever, now.

sc.logger_info("Hello world!")

# Get access to entities through something like this:

#entity_foo = sc.GetSimEntity().GetParam(sc.VarType.entityRef, "FooEntity")

# this = sc.GetThisSystem()

# Here's an example of how to do a loop.
# In Python Systems, looping functionality is not done for you like in C++.

# exit_flag = False
# while not exit_flag:
#     # NOTE: Doesn't yet do timekeeping to factor in time to compute below code.
#     # NOTE: you'll need to add this system instance param if you want to use it.
#     # time.sleep(1.0 / this.GetParam(sc.VarType.double, "LoopFreqHz"))

#     # Do stuff inside of loop
    
#     entities = sc.get_entities()
#     for en in entities:
#         # Maybe do per-entity stuff
    
#     # maybe at some point end the loop
#     exit_flag = True


sc.leave_sim()
