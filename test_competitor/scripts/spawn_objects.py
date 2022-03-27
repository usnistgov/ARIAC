#!/usr/bin/env python

import sys
import math
import rospy
import random
import rospkg
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest
from geometry_msgs.msg import Pose, Vector3, Quaternion

class ModelSpawner():
    model_names = ['pump', 'sensor', 'regulator', 'battery', 'random']
    
    def __init__(self, type, spacing):
        rospy.init_node("model_spawner")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
    
        self.model_type = type
        self.spacing = spacing
        self.colors = ['red', 'blue', 'green']
    
        self.rp = rospkg.RosPack()
        self.max_d = 0.2
    
    def spawn_models(self):
        poses = []
        
        positions = []
        for i in range(4):
            pose = Pose()
            
            while True:
                x_pos, y_pos = random.uniform(-self.max_d, self.max_d), random.uniform(-self.max_d, self.max_d)
                if not positions:
                    positions.append((x_pos, y_pos))
                    break 
                
                distances = []
                for x, y in positions:
                    distances.append(math.sqrt((x-x_pos)**2 + (y-y_pos)**2))
                
                if min(distances) < 0.2:
                    continue
                else:
                    positions.append((x_pos, y_pos))
                    break              

            pose.position = Vector3(x_pos, y_pos, 0.05)
            pose.orientation = Quaternion(0, 0, 0, 1)
            poses.append(pose)
        
        spawner = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        for i, pose in enumerate(poses):
            req = SpawnModelRequest()
            
            model_color = random.choice(self.colors)
            # model_color = 'blue'
            if self.model_type == 'random':
                model_name = self.model_names[random.randrange(4)]
            else:
                model_name = self.model_type  
            
            fname = (self.rp.get_path("nist_gear") + "/models/assembly_" + model_name + "_" + 
                    model_color + "_ariac/model.sdf")
        
            xml = open(fname, 'r').read()
            
            req.model_name = model_name + "_" + str(i)
            req.model_xml = xml
            req.initial_pose = pose
            req.robot_namespace = ""
            req.reference_frame = 'torso_tray'
            
            spawner(req)

if __name__ == "__main__":
    if sys.argv[1] in ModelSpawner.model_names:
        type = sys.argv[1]
    else:
        print("Not a valid model type")
        exit()
    
    if len(sys.argv) > 2:
        if 0.1 <= float(sys.argv[2]) <= 0.25:
            spacing = float(sys.argv[2])
        else:
            print("Not a valid spacing")
            exit()
    else:
        spacing = 0.2
        
    spawner = ModelSpawner(type, spacing)
    spawner.spawn_models()
