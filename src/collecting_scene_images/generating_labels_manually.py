# Initialization
import os
import pybullet as p
import pybullet_data
import math
import numpy as np
import time
#from save_data import save_to_excel
import numpy as np
#import open3d as o3d
import pandas as pd 
#from collect_data import calculate_sizes, calculate_fall
import random

# Initialize PyBullet
#p.connect(p.DIRECT)
p.connect(p.GUI)

orientation = p.getQuaternionFromEuler([0, 0, 0])  # roll, pitch, yaw

#trayUid = p.loadURDF(os.path.join(urdfRootPath, "tray/traybox.urdf"), basePosition=[0.65, 0, 0.04],useFixedBase=True)

object_urdf_list = ["teddy_vhacd.urdf","duck_vhacd.urdf","objects/mug.urdf","cube_small.urdf"]
object_urdf_list2 = ["random_urdfs/000/000.urdf","random_urdfs/888/888.urdf","random_urdfs/023/023.urdf","random_urdfs/521/521.urdf" ,"random_urdfs/666/666.urdf" ,"random_urdfs/202/202.urdf" ,"random_urdfs/765/765.urdf" ]
object_urdf_list3=["009_gelatin_box.urdf","013_apple.urdf","015_peach.urdf","018_plum.urdf","037_scissors.urdf","048_hammer.urdf","055_baseball.urdf","063-b_marbles.urdf","065-a_cups.urdf","073-d_lego_duplo.urdf","073-h_lego_duplo.urdf","073-m_lego_duplo.urdf","072-f_toy_airplane.urdf","072-a_toy_airplane.urdf"]
p.setAdditionalSearchPath(pybullet_data.getDataPath())
urdfRootPath=pybullet_data.getDataPath()
urdfRootPath2="C:/Users/oumay/graspingcollisions/object2urdf/examples/ycb"

#Env initialisation
p.setGravity(0, 0, -9.81)
p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55, -0.35, 0.2])

def generate_random_pos():
    x = random.uniform(0.4, 0.9)
    y = random.uniform(-0.4, 0.4)
    #z = random.uniform(0.06, 0.1)
    return [x,y,0.12]

def calculate_sizes(objectUid):
    aabbMin, aabbMax = p.getAABB(objectUid)
    width = aabbMax[0] - aabbMin[0]
    height = aabbMax[1] - aabbMin[1]
    depth = aabbMax[2] - aabbMin[2]

def control_gripper(robot, width):
    """
    Control the gripper fingers to open to the specified width.
    
    Args:
        robot (int): The ID of the robot.
        width (float): The desired width of the gripper opening.
    """
    # Gripper joint indices for the Franka Panda (example values, adjust as needed)
    left_finger_joint = 9
    right_finger_joint = 10
    
    # Calculate target positions for the fingers
    # Assuming the gripper opens symmetrically
    target_position = width / 2
    
    # Set the target positions for the finger joints
    p.setJointMotorControl2(
        robot,
        left_finger_joint,
        p.POSITION_CONTROL,
        targetPosition=target_position,
    )
    p.setJointMotorControl2(
        robot,
        right_finger_joint,
        p.POSITION_CONTROL,
        targetPosition=target_position,
    )

def compute_ik(robot, end_effector_link, target_position, target_orientation):
    joint_angles = p.calculateInverseKinematics(
        robot,
        end_effector_link,
        target_position,
        targetOrientation=target_orientation,
    )
    # Return only the first 7 joints (arm joints)
    return joint_angles[:7]

def generate_trajectory(start_angles, target_angles, num_steps=50):
    trajectory = []
    for t in np.linspace(0, 1, num_steps):
        interpolated_angles = (1 - t) * np.array(start_angles) + t * np.array(target_angles)
        trajectory.append(interpolated_angles)
    return trajectory

def execute_trajectory(robot, joint_indices, trajectory):
    for angles in trajectory:
        for i, joint_index in enumerate(joint_indices):
            p.setJointMotorControl2(
                robot,
                joint_index,
                p.POSITION_CONTROL,
                targetPosition=angles[i],
                maxVelocity=7,
            )
    p.setJointMotorControl2(
        robot,
        5,
        p.POSITION_CONTROL,
        targetPosition=np.pi/2,
        maxVelocity=7,
    )

ListObjct=pd.read_excel("C:/Users/oumay/graspingcollisions/src/collecting_data_manualy/data/scenes_final.xlsx")
MainObjct=pd.read_excel("C:/Users/oumay/graspingcollisions/src/collecting_data_manualy/data/grasp_object_combination.xlsx")
unlabeled_scenes=pd.read_excel("C:/Users/oumay/graspingCollision 2/src/creating_graphs/unlabeled_graphs_analysis.xlsx", sheet_name="Unlabeled_Graphs")

# Reset index to ensure consecutive numbering starting from 0
unlabeled_scenes = unlabeled_scenes.reset_index(drop=True)

print(f"Total unlabeled scenes: {len(unlabeled_scenes)}")
print(f"Unlabeled scenes index range: {unlabeled_scenes.index.min()} to {unlabeled_scenes.index.max()}")

m=0
B=0
start_index = 60 # Change this value to your desired start index

for i in range(start_index, len(unlabeled_scenes)):
    if i >= len(unlabeled_scenes):
        print(f"Reached end of unlabeled scenes at index {i}")
        break
        
    print(f"Processing unlabeled scene {i+1}/{len(unlabeled_scenes)}")
    m=unlabeled_scenes["graspCombo"][i]
    scene_id=unlabeled_scenes["scene_id"][i]
    print("graspCombo",m)
    print("scene_id",scene_id)
    
    # Find all objects for this scene and grasp combination
    scene_objects = []
    for x in range(len(ListObjct)):
        if ListObjct["sceneId"][x] == scene_id and ListObjct["grasp_obj_combination"][x] == m:
            scene_objects.append(x)
    
    if len(scene_objects) == 0:
        print(f"No objects found for scene_id {scene_id} and graspCombo {m}, skipping...")
        continue
    
    if len(scene_objects) < 6:
        print(f"Warning: Only {len(scene_objects)} objects found for scene_id {scene_id} and graspCombo {m}")
    
    print(f"Found {len(scene_objects)} objects for this scene")
    j = scene_objects[0]  # Main object index
    print("j",j)
    print("ListObjct",ListObjct["object_id"][j])
    p.setGravity(0, 0, -9.81)
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55, 0.2, 0.2])
    
    position = [float(num) for num in MainObjct["position"][m].split(",")]

    pandaUid = p.loadURDF(os.path.join(urdfRootPath, "franka_panda/panda.urdf"), useFixedBase=True)
    tableUid = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[position[0], position[1], -0.50],useFixedBase=True)
    quaternion  = [float(num) for num in MainObjct["quaternion"][m].split(",")]
    grasp_box_details   = [float(num) for num in MainObjct["grasp_box_details"][m].split(",")]
    globalScaling   =   MainObjct["globalScaling"][m]
    target_position = [float(num) for num in MainObjct["target_position"][m].split(",")]
    target_orientation = [float(num) for num in MainObjct["target_orientation"][m].split(",")]
    
    orientation = p.getQuaternionFromEuler([0.5, 0, 0]) 
    f=0
    joint_indices = [0, 1, 2, 3, 4, 5, 6]  # Joint indices for the Panda arm
    current_angles = [p.getJointState(pandaUid, i)[0] for i in joint_indices]
    end_effector_link = 11  # End-effector link index
    target_angles = compute_ik(pandaUid, end_effector_link, target_position, target_orientation)
    trajectory = generate_trajectory(current_angles, target_angles)


    objectList=[]
    pos = [float(num) for num in ListObjct["Object_cordXYZ"][j].split(",")]
    print("pos1",pos)
    pos=[a_i + b_i for a_i, b_i in zip([0.00,0.0,0.03], pos)]
    if m== 59:
        pos=[a_i - b_i for a_i, b_i in zip(pos,[0.01,0.01,0.0] )]
    if m== 65:
        pos=[a_i - b_i for a_i, b_i in zip(pos,[0.00,0.02,0.0] )]
    objectUid = p.loadURDF(os.path.join(urdfRootPath, ListObjct["object_id"][j]), basePosition=pos,baseOrientation=quaternion , useMaximalCoordinates=True)

    if ListObjct["object_id"][j] in object_urdf_list3:
        pos = [float(num) for num in ListObjct["Object_cordXYZ"][j].split(",")]
        pos=[a_i + b_i for a_i, b_i in zip([0.00,0.0,0.07], pos)] 
        objectUid = p.loadURDF(os.path.join(urdfRootPath2, ListObjct["object_id"][j]), basePosition=pos,globalScaling=0.12 )
        
    s=calculate_sizes(objectUid)
    print("s1",s)
    mass1=p.getDynamicsInfo(objectUid, -1)[0]
    objectList.append([ListObjct["sceneId"][j],ListObjct["object_id"][j],objectUid,pos,s,mass1])
    print("main object")

    # Load other objects in the scene
    for o in range(1, min(6, len(scene_objects))):  # Load up to 5 additional objects
        obj_index = scene_objects[o]
        if ListObjct["object_id"][obj_index] in object_urdf_list or ListObjct["object_id"][obj_index] in object_urdf_list2:
                pos = [float(num) for num in ListObjct["Object_cordXYZ"][obj_index].split(",")]
                pos=[a_i + b_i for a_i, b_i in zip([0.00,0.0,0.05], pos)]
                globalScaling=1
                objectUid = p.loadURDF(os.path.join(urdfRootPath, ListObjct["object_id"][obj_index]), basePosition=pos,baseOrientation=[0,0,0,1], globalScaling=globalScaling)
                if objectUid < 0:
                   print(f"Failed to load URDF: ",ListObjct["object_id"][obj_index])
        if ListObjct["object_id"][obj_index] in object_urdf_list3:
            pos = [float(num) for num in ListObjct["Object_cordXYZ"][obj_index].split(",")]
            pos=[a_i + b_i for a_i, b_i in zip([0.00,0.0,0.07], pos)]  
            if ListObjct["object_id"][obj_index] in ["072-f_toy_airplane.urdf","072-a_toy_airplane.urdf"]:
                pos=[a_i + b_i for a_i, b_i in zip([0.00,0.0,0.04], pos)] 
            if ListObjct["object_id"][obj_index] in  ["009_gelatin_box.urdf","013_apple.urdf","018_plum.urdf","063-b_marbles.urdf","015_peach.urdf","055_baseball.urdf"]: 
                globalScaling=0.1
            else:
                globalScaling=0.14
            
            objectUid = p.loadURDF(os.path.join(urdfRootPath2, ListObjct["object_id"][obj_index]), basePosition=pos,baseOrientation=[0,0,0,1],globalScaling=globalScaling)
            if objectUid < 0:
                    print(f"Failed to load URDF: ",ListObjct["object_id"][obj_index])

        s=calculate_sizes(objectUid)
        mass1=p.getDynamicsInfo(objectUid, -1)[0]
        objectList.append([ListObjct["sceneId"][obj_index],ListObjct["object_id"][obj_index],objectUid,pos,s,mass1])

    object_uids = []
    for obj in objectList:
        object_uids.append(obj[2])
    
    while len(object_uids) < 6:
        object_uids.append(None)
    
    objectUid = object_uids[0] if len(object_uids) > 0 else None
    objectUid1 = object_uids[1] if len(object_uids) > 1 else None
    objectUid2 = object_uids[2] if len(object_uids) > 2 else None
    objectUid3 = object_uids[3] if len(object_uids) > 3 else None
    objectUid4 = object_uids[4] if len(object_uids) > 4 else None
    objectUid5 = object_uids[5] if len(object_uids) > 5 else None
    
    max_force1_obm, max_lateral_friction1_obm, max_overlap1_obm = [], [], []
    max_force2_obm, max_lateral_friction2_obm, max_overlap2_obm = [], [], []
    max_force1_ob1, max_lateral_friction1_ob1, max_overlap1_ob1 = [], [], []
    max_force2_ob1, max_lateral_friction2_ob1, max_overlap2_ob1 = [], [], []
    max_force1_ob2, max_lateral_friction1_ob2, max_overlap1_ob2 = [], [], []
    max_force2_ob2, max_lateral_friction2_ob2, max_overlap2_ob2 = [], [], []
    max_force1_ob3, max_lateral_friction1_ob3, max_overlap1_ob3 = [], [], []
    max_force2_ob3, max_lateral_friction2_ob3, max_overlap2_ob3 = [], [], []
    max_force1_ob4, max_lateral_friction1_ob4, max_overlap1_ob4 = [], [], []
    max_force2_ob4, max_lateral_friction2_ob4, max_overlap2_ob4 = [], [], []
    max_force1_ob5, max_lateral_friction1_ob5, max_overlap1_ob5 = [], [], []
    max_force2_ob5, max_lateral_friction2_ob5, max_overlap2_ob5 = [], [], []
    max_force1_ob6, max_lateral_friction1_ob6, max_overlap1_ob6 = [], [], []
    max_force2_ob6, max_lateral_friction2_ob6, max_overlap2_ob6 = [], [], []
    
    state_t = 0.
    current_state = 0
    grasped = False
    gripper_joints = [9, 10]  # Gripper joint indices for the Panda robot
    arm_joints = [0, 1, 2, 3, 4, 5, 6]  # Arm joint indices for the Panda robot
    state_t = 0
    state_durations = [2.0, 1.0, 1.0, 0.0, 0.0]  # Durations for each state
    grasped = False
    constraint_id = None  # To store the constraint ID
    current_state = 0

    while current_state <= 4:
        state_t += 1. / 60.
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)

        if current_state == 0:
            left_finger_joint = 9
            right_finger_joint = 10
            target_position = 0.08
            p.setJointMotorControl2(
            pandaUid,
            left_finger_joint,
            p.POSITION_CONTROL,
            targetPosition=target_position,
            )
            p.setJointMotorControl2(
            pandaUid,
            right_finger_joint,
            p.POSITION_CONTROL,
            targetPosition=target_position,
            )
            execute_trajectory(pandaUid, joint_indices, trajectory)

        if current_state == 1:
            if not grasped:
                p.setJointMotorControl2(pandaUid, 9, p.POSITION_CONTROL, 0.0, force=200)
                p.setJointMotorControl2(pandaUid, 10, p.POSITION_CONTROL, 0.0, force=200)

        if current_state == 2:
            target_positions = [0.0, -0.5, 0.0, -1.5, 0.0, 1.5, 1.5]  # Example target positions for lifting
            for arm_i, joint_index in enumerate(arm_joints):
                p.setJointMotorControl2(pandaUid, joint_index, p.POSITION_CONTROL, target_positions[arm_i],maxVelocity=7)

        # if current_state == 3:
        #     target_positions = [0.5, -0.5, 0.0, -1.5, 0.0, 1.5, 1.5]  # Example target positions for moving
        #     for arm_i, joint_index in enumerate(arm_joints):
        #         p.setJointMotorControl2(pandaUid, joint_index, p.POSITION_CONTROL, target_positions[arm_i])

        # if current_state == 4:
        #     p.setJointMotorControl2(pandaUid, gripper_joints[0], p.POSITION_CONTROL, 0.08, force=200)
        #     p.setJointMotorControl2(pandaUid, gripper_joints[1], p.POSITION_CONTROL, 0.08, force=200)
        
        if state_t > state_durations[current_state]:
            current_state += 1
            if current_state >= len(state_durations):
                current_state = 5
            state_t = 0
        time.sleep(0.1)
        p.stepSimulation()
        
    for obj_uid in object_uids:
        if obj_uid is not None:
            p.removeBody(obj_uid)
    
    p.removeBody(pandaUid)
    p.removeBody(tableUid)
    print(f"Completed scene {i+1}, objectUid: {objectUid}")
    
    p.resetSimulation()
    print("i",i)