from vision_subscriber import DetectionSubscriber
from feasibility_subscriber import FeasibilitySubscriber
# from feasibility_graph import FeasibilitySubscriber
from action_execution import AnytreeInterface
from transformers_inference import Inference
from transformers import AutoModelForCausalLM, AutoTokenizer
import rospy
import torch
import os
seed=0
torch.manual_seed(seed)

robot_origin=[-1,0.6,0.57,0,0,1.57]
# from huggingface_hub import login
# hf_token = os.getenv("HF_TOKEN")
# if hf_token:
#     login(hf_token)
# else:
#     print("Hugging Face token is not set. Please set HF_TOKEN environment variable.")
from huggingface_hub import login
hf_token = "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"
login(hf_token)

def robot_function(inp):
    action_list = inp.split(',')
    # print('action_list:', action_list)
    # for action in action_list:
    #     if 'back' in action:
    #         global robot_origin
    #         robot_origin = robot.omni_base.pose
            # print('robot_origin is set to ',str(robot_origin))
    for action in action_list:
        temp = [item for item in action.split(' ') if item] 
        if temp[0]=='failed':
            return
        print('--------------------------------------')
        print('Executing action: '+action)

        if temp[0]=='pick':
            if vision.is_seen(temp[1]):
                re = robot.pick_up_object_client(vision.get_pose(temp[1]))
            else:
                print('Pick up action is called but no {} is found'.format(temp[1]))
                return False, 'vision'
        elif temp[0]=='turn':
            re = robot.turn(temp[1])
        elif temp[0]=='search':
            # re = robot.search(temp[-1],vision_func=vision.is_seen)
            pass
        elif temp[0]=='go' or temp[0]=='move':
            if temp[1]=='back':
                re = robot.move(robot_origin)
                print('result of go back: ',re)
            elif temp[1]=='closer':
                re = robot.move([0.5,0,0])
        elif temp[0]=='place':
            re = robot.put_object_on_surface_client([-1, 1.2, 0.75]) 
            pass
        if re == False:
            return False
        
    return True


rospy.init_node('inteliplan_robot')

# initialize robot poses
robot =  AnytreeInterface()
robot.base_controller.go_abs([-0.2, 0, 0.57,0,0,0])

# Intialize the model and tokenizer
# model_path = "/inteliplan_ws/src/inteliplan_robot/inteliplan_interface/models/fetchme"
model_path = "./../models/fetchme"
model = AutoModelForCausalLM.from_pretrained(model_path,load_in_4bit=True, device_map="auto" )
tokenizer = AutoTokenizer.from_pretrained(model_path)


test = Inference(model, tokenizer)
vision = DetectionSubscriber()
feasibility = FeasibilitySubscriber()
# scene = {'scene':{'file':'../worlds/room.g', 'mesh_type': False}}
# map_bounds = [[-3, 3],[-2, 2],[0.5, 1.3]] # limits on map dimensions
# feasibility = FeasibilitySubscriber(scene, map_bounds, DEBUG=False)
test.interactive_session(vision_func=vision.is_seen, vision_loc=vision.get_pose, feasibility_func=feasibility.get_score, robot_func=robot_function)
rospy.spin()