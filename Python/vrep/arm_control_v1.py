import vrep
import numpy as np

try:
    # close any open connections
    vrep.simxFinish(-1)
    # Connect to the V-REP continuous server
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5)

    if clientID != -1: # if we connected successfully
        print ('Connected to remote API server')

        vrep.simxSynchronous(clientID,True)

        joint_names = [
            "kinematicsTest_joint1",
            "kinematicsTest_joint2",
            "kinematicsTest_joint3",
            "kinematicsTest_joint4",
            "kinematicsTest_joint5",
            "kinematicsTest_joint6",
            "kinematicsTest_joint7"
        ]

        #Get joint angle handles
        joint_handles = [vrep.simxGetObjectHandle(clientID,name, vrep.simx_opmode_blocking)[1] for name in joint_names]

        # Get the handle of the target
        _, target_xyz = vrep.simxGetObjectHandle(
            clientID,
            "kinematicsTest_IK_Target",
            vrep.simx_opmode_blocking
        )

        # Set up streaming
        dt = .001
        vrep.simxSetFloatingParameter(
            clientID,
            vrep.sim_floatparam_simulation_time_step,
            dt, # specify a simulation time step
            vrep.simx_opmode_oneshot)

        # Start the simulation
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking)

    else:
        raise Exception('Failed connecting to remote API server')

finally:
    pass
