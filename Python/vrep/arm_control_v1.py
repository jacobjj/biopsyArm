import vrep
import numpy as np
import rospy
from msg import haptic_pos


from std_msgs.msg import Float32MultiArray

haptic_transform = np.asarray([
                        [1,0,0],
                        [0,0,-1],
                        [0,1,0]
                        ])

def scale_x(x):
    scale_mat = np.diag([1.0/160,1.0/70,1.0/200])
    #scale_mat = np.eye(3)
    return np.matmul(scale_mat,x)


try:
    # Initialize node for ros
    rospy.init_node("BiopsyArmTest",anonymous=True)

    #Initialize the Haptic device
    phantom = haptic_pos()
    rospy.Subscriber('pose_msg',Float32MultiArray,phantom.callback,queue_size= 1)

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
        joint_handles = [
            vrep.simxGetObjectHandle(
                clientID,
                name,
                vrep.simx_opmode_blocking)[1]
            for name in joint_names
        ]

        #Get the end-effector position
        _, arm_x_handle = vrep.simxGetObjectHandle(
            clientID,
            "N",
            vrep.simx_opmode_blocking)

        # Get the handle of the target
        _, target_xyz = vrep.simxGetObjectHandle(
            clientID,
            "kinematicsTest_IK_Target",
            vrep.simx_opmode_blocking
        )

        # Calculating the offset
        arm_x_0 = vrep.simxGetObjectPosition(
            clientID,
            arm_x_handle,
            -1,
            vrep.simx_opmode_blocking)

        hd_x_0 = scale_x(np.matmul(haptic_transform,phantom.hd_transform[0:3,3]))

        x_off = arm_x_0[1]-hd_x_0


        # Set up streaming
        dt = .001
        vrep.simxSetFloatingParameter(
            clientID,
            vrep.sim_floatparam_simulation_time_step,
            dt, # specify a simulation time step
            vrep.simx_opmode_oneshot)

        # Start the simulation
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking)

        # On Shutdown
        def shutDown_vrep():
            vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)
            vrep.simxFinish(clientID)

        rospy.on_shutdown(shutDown_vrep)

        while not rospy.is_shutdown():

            # Get the IK Jacobian
            res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(
                clientID,
                'base',
                vrep.sim_scripttype_childscript,
                'GetIkJacobian',
                {},    # inputIntsn
                {},    # inputFloats
                {},    # inputStrings
                '',    # inputBuffer
                vrep.simx_opmode_blocking
            )

            print(retInts)
            # Get the Haptic device postion
            hd_x = scale_x(np.matmul(haptic_transform, phantom.hd_transform[0:3,3]))

            # Clutch
            if not phantom.hd_button1:
                des_x = hd_x+ x_off
            else:
                # Wait till the button is released
                while phantom.hd_button1:
                    pass
                hd_x = scale_x(np.matmul(haptic_transform,phantom.hd_transform[0:3,3]))
                arm_x = vrep.simxGetObjectPosition(
                    clientID,
                    arm_x_handle,
                    -1,
                    vrep.simx_opmode_blocking)
                # Recalculating the off-set
                x_off = arm_x[1] - hd_x
            """
            # Set position of the target -> This is from the haptic device
            vrep.simxSetObjectPosition(
                clientID,
                target_xyz,
                -1,# Setting the absolute position
                position=des_x,
                operationMode=vrep.simx_opmode_blocking
            )
            """

            # Calculate desired endeffector pose
            des_pose = np.concatenate((des_x,np.zeros((3,))))

            # set the joint angles position
            J = np.reshape(retFloats,[6,7])
            J_pinv = np.linalg.pinv(J)
            for joint_handle,des_joint_angle in zip(joint_handles,des_pose):
                des_joint_angles = np.matmul(J_pinv,des_pose)
                vrep.simxSetJointPosition(
                    clientID,
                    joint_handle,
                    des_joint_angle,
                    vrep.simx_opmode_blocking)

            # move simulation ahead one time step
            vrep.simxSynchronousTrigger(clientID)

    else:
        raise Exception('Failed connecting to remote API server')

finally:
    # stop the simulation
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)

    # Before closing the connection to V-REP,
    # make sure that the last command sent out had time to arrive.
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
    print('connection closed...')
