#!/usr/bin/env python

# Software License Agreement (Apache License)
#
# Copyright 2016 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import print_function

import argparse
import math
import os
import random
import subprocess
import sys
import pprint
import em
import rospkg
import yaml


rospack = rospkg.RosPack()
world_dir = os.path.join(rospack.get_path('nist_gear'), 'worlds')
launch_dir = os.path.join(rospack.get_path('nist_gear'), 'launch')
gantry_dir = os.path.join(rospack.get_path('nist_gear'), 'robots/gantry/gantry_description/urdf')
kitting_dir = os.path.join(rospack.get_path('nist_gear'), 'robots/kitting/urdf/')
template_files = [
    os.path.join(world_dir, 'ariac.world.template'),
    os.path.join(launch_dir, 'gear.launch.template'),
    os.path.join(launch_dir, 'gear.urdf.xacro.template'),
    os.path.join(gantry_dir, 'gantry.urdf.xacro.template'),
]

arm_template_file = os.path.join(kitting_dir, 'kitting.urdf.xacro.template')

arm_configs = {
    'kitting': {
        'arm_type': 'ur10',
        'pose': {
            'xyz': [0.3, 0.92, 0.9],
            'rpy': [0.0, 0.0, 0.0]
        },
        'default_initial_joint_states': {
            'elbow_joint': 1.74,
            'linear_arm_actuator_joint': 0,
            'shoulder_lift_joint': -1.25,
            'shoulder_pan_joint': 0,
            'wrist_1_joint': -2.66,
            'wrist_2_joint': -1.51,
            'wrist_3_joint': 0,
        }
    },
}


possible_products = [
    'assembly_battery_red',
    'assembly_battery_green',
    'assembly_battery_blue',
    'assembly_pump_red',
    'assembly_pump_green',
    'assembly_pump_blue',
    'assembly_regulator_red',
    'assembly_regulator_green',
    'assembly_regulator_blue',
    'assembly_sensor_red',
    'assembly_sensor_green',
    'assembly_sensor_blue',
]
sensor_configs = {
    'break_beam': None,
    'camera': None,
    'proximity_sensor': None,
    'logical_camera': None,
    'laser_profiler': None,
    'depth_camera': None,
    'rgbd_camera': None,
    'quality_control': None,
}

default_agv_origins = {
    'agv1': [-2.265685, 4.675404, 0],
    'agv2': [-2.265685, 1.367643, 0],
    'agv3': [-2.265685, -1.333917, 0],
    'agv4': [-2.265685, -4.696062, 0],
}


agv1_y = 4.675404
agv2_y = 1.367643
agv3_y = -1.333917
agv4_y = -4.696062

agv1_agv2_AS1_x = -5.60
agv1_agv2_AS2_x = -10.590274

agv3_agv4_AS3_x = -5.60
agv3_agv4_AS4_x = -10.590274

# Pose of all the different stations for each agv
# KS is the kitting station
stations = {
    'agv1': {
        'ks1': {
            'pose': {
                'xyz': [-2.265685, agv1_y, 0]
            }
        },
        'as1': {
            'pose': {
                'xyz': [agv1_agv2_AS1_x, agv1_y, 0]
            }
        },
        'as2': {
            'pose': {
                'xyz': [agv1_agv2_AS2_x, agv1_y, 0]
            }
        },
    },
    'agv2': {
        'ks2': {
            'pose': {
                'xyz': [-2.265685, agv2_y, 0]
            }
        },
        'as1': {
            'pose': {
                'xyz': [agv1_agv2_AS1_x, agv2_y, 0]
            }
        },
        'as2': {
            'pose': {
                'xyz': [agv1_agv2_AS2_x, agv2_y, 0]
            }
        },
    },
    'agv3': {
        'ks3': {
            'pose': {
                'xyz': [-2.265685, agv3_y, 0]
            }
        },
        'as3': {
            'pose': {
                'xyz': [agv3_agv4_AS3_x, agv3_y, 0]
            }
        },
        'as4': {
            'pose': {
                'xyz': [agv3_agv4_AS4_x, agv3_y, 0]
            }
        },
    },
    'agv4': {
        'ks4': {
            'pose': {
                'xyz': [-2.265685, agv4_y, 0]
            }
        },
        'as3': {
            'pose': {
                'xyz': [agv3_agv4_AS3_x, agv4_y, 0]
            }
        },
        'as4': {
            'pose': {
                'xyz': [agv3_agv4_AS4_x, agv4_y, 0]
            }
        },
    }

}

# The pose should be changed based on the new environment configuration
default_sensors = {
    'quality_control_sensor_1': {
        'type': 'quality_control',
        'pose': {
            'xyz': [-2.393395, 4.702724, 1.506952],
            'rpy': [-3.141593, 1.570796, 0]
        }
    },
    'quality_control_sensor_2': {
        'type': 'quality_control',
        'pose': {
            'xyz': [-2.393394, 1.361367, 1.506952],
            'rpy': [-3.141593, 1.570796, 0]
        }
    },
    'quality_control_sensor_3': {
        'type': 'quality_control',
        'pose': {
            'xyz': [-2.393393, -1.325227, 1.506952],
            'rpy': [-3.141593, 1.570796, 0]
        }
    },
    'quality_control_sensor_4': {
        'type': 'quality_control',
        'pose': {
            'xyz': [-2.393395, -4.698129, 1.506952],
            'rpy': [-3.141593, 1.570796, 0]
        }
    }
}

default_belt_models = {
}


bin_width = 0.6
bin_depth = 0.6
bin_height = 0.72
bin_angle = 0.0

station_width = 0.6
station_depth = 0.6
station_height = 1.20
station_angle = 0.0


# 0.842058 + 2.378535
default_bin_origins = {
    'bin1': [-1.898993, 3.379920, 0],
    'bin2': [-1.898993, 2.565006, 0],
    'bin3': [-2.651690, 2.565006, 0],
    'bin4': [-2.651690, 3.379920, 0],
    'bin5': [-1.898993, -3.379920, 0],
    'bin6': [-1.898993, -2.565006, 0],
    'bin7': [-2.651690, -2.565006, 0],
    'bin8': [-2.651690, -3.379920, 0],
}


# Dictionary
default_station_origins = {
    'as1': [-7.3, 3, 0],
    'as2': [-12.3, 3, 0],
    'as3': [-7.3, -3, 0],
    'as4': [-12.3, -3, 0],
}

# brief_case_offset_x = 0.555643
brief_case_offset_x = 0.08
brief_case_offset_y = 0.095839
brief_case_offset_z = station_height

default_briefcase_origins = {
    'briefcase1': [default_station_origins['as1'][0] + brief_case_offset_x, default_station_origins['as1'][1] + brief_case_offset_y, brief_case_offset_z],
    'briefcase2': [default_station_origins['as2'][0] + brief_case_offset_x, default_station_origins['as2'][1] + brief_case_offset_y, brief_case_offset_z],
    'briefcase3': [default_station_origins['as3'][0] + brief_case_offset_x, default_station_origins['as3'][1] + brief_case_offset_y, brief_case_offset_z],
    'briefcase4': [default_station_origins['as4'][0] + brief_case_offset_x, default_station_origins['as4'][1] + brief_case_offset_y, brief_case_offset_z],
}

configurable_options = {
    'insert_models_over_bins': False,
    'insert_models_over_stations': False,
    'enable_robot_camera': False,
    'disable_shadows': False,
    'belt_population_cycles': 0,
    'gazebo_state_logging': False,
    'spawn_extra_models': False,
    'unthrottled_physics_update': False,
    'model_type_aliases': {
        'belt_model_type1': 'part1',
        'belt_model_type2': 'part2',
    },
    'visualize_sensor_views': False,
    'visualize_drop_regions': False,
}
default_time_limit = 500  # seconds
max_count_per_model = 30  # limit on the number of instances of each model type


def update_dict(tree, key, value):
    """Return true if update, else false"""

    if key in tree:
        tree[key].update(value)
        return True
    for branch in tree.values():
        if update_dict(branch, key, value):
            return True
    return False


def initialize_model_id_mappings(random_seed=None):
    global global_model_count, model_id_mappings
    global_model_count = {}  # global count of how many times a model type has been created

    randomize = False
    if random_seed is not None:
        randomize = True
        random.seed(random_seed)

    # Initialize the mapping between model index and ID that will exist for each model type
    model_id_mappings = {}

    # Initialize the list of random IDs that will be used in the mappings
    # The IDs will be unique across different model types
    max_model_id = max_count_per_model * len(possible_products)  # can be larger for more spread
    random_ids = random.sample(range(0, max_model_id), max_model_id)
    for model_type in possible_products:
        if not randomize:
            # Just use ordinary mapping
            model_id_mappings[model_type] = list(range(1, max_count_per_model + 1))
        else:
            # Use random IDs for the mapping
            model_id_mappings[model_type] = random_ids[:max_count_per_model]
            del random_ids[:max_count_per_model]


def str2bool(v):
    """Helper for converting strings to Booleans.

    The code was copied from https://stackoverflow.com/a/43357954

    Parameters
    ----------
    v : str
        The string to convert.

    Returns
    -------
    bool
        True if the input string is 'yes', 'true', 't', 'y', or '1'
        False if the input string is 'no', 'false', 'f', 'n', or '0'
    """
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


def prepare_arguments(parser):
    add = parser.add_argument
    add('-n', '--dry-run', action='store_true', default=False,
        help='print generated files to stdout, but do not write them to disk')
    add('-v', '--verbose', action='store_true', default=False,
        help='output additional logging to console')
    add('-o', '--output', default='/tmp/ariac/',
        help='directory in which to output the generated files')
    add('--development-mode', '-d', action='store_true', default=False,
        help='if true the competition mode environment variable will not be set (default false)')
    add('--no-gui', action='store_true', default=False,
        help="don't run the gazebo client gui")
    add('--load-moveit', action='store_true', default=False,
        help='automatically launch move_group node for both robots')
    # add('--load-kitting-moveit', action='store_true', default=False,
    #     help='automatically launch move_group node for kitting robot')
    # add('--load-gantry-moveit', action='store_true', default=False,
    #     help='automatically launch move_group node for gantry robot')
    add('-l', '--state-logging', action='store', type=str2bool, nargs='?',
        help='generate gazebo state logs (will override config file option)')
    add('--log-to-file', action='store_true', default=False,
        help='direct the output of the gazebo ros node to log file instead of the console')
    add('--visualize-sensor-views', action='store_true', default=False,
        help='visualize the views of sensors in gazebo')
    mex_group = parser.add_mutually_exclusive_group(required=False)
    add = mex_group.add_argument
    add('config', nargs='?', metavar='CONFIG',
        help='yaml string that is the configuration')
    add('-f', '--file', nargs='+', help='list of paths to yaml files that contain the '
        'configuration (contents will be concatenated)')


eval_local_vars = {n: getattr(math, n) for n in dir(math) if not n.startswith('__')}


def expand_to_float(val):
    if isinstance(val, str):
        return float(eval(val, {}, eval_local_vars))
    return float(val)


def expand_yaml_substitutions(yaml_dict):
    for k, v in yaml_dict.items():
        if isinstance(v, dict):
            yaml_dict[k] = expand_yaml_substitutions(v)
        if k in ['xyz', 'rpy']:
            yaml_dict[k] = [expand_to_float(x) for x in v]
        if k in ['initial_joint_states']:
            yaml_dict[k] = {kp: expand_to_float(vp) for kp, vp in v.items()}
    return yaml_dict


class ArmInfo:
    def __init__(self, name, arm_type, initial_joint_states, pose):
        self.name = name
        self.type = arm_type
        self.initial_joint_states = initial_joint_states
        self.pose = pose


class ModelInfo:
    def __init__(self, model_type, pose, reference_frame):
        self.type = model_type
        self.pose = pose
        self.reference_frame = reference_frame


class SensorInfo:
    def __init__(self, name, sensor_type, pose):
        self.name = name
        self.type = sensor_type
        self.pose = pose


class AGVInfo:
    def __init__(self, id, pose):
        self.id = id
        self.pose = pose


class PoseInfo:
    def __init__(self, xyz, rpy):
        self.xyz = [str(f) for f in xyz]
        self.rpy = [str(f) for f in rpy]


class DropRegionInfo:
    def __init__(self, name, drop_region_min, drop_region_max, destination, frame, model_type):
        self.name = name
        self.min = [str(f) for f in drop_region_min]
        self.max = [str(f) for f in drop_region_max]
        self.destination = destination
        self.frame = frame
        self.type = model_type


def get_field_with_default(data_dict, entry, default_value):
    if entry in data_dict:
        return data_dict[entry]
    else:
        return default_value


def get_required_field(entry_name, data_dict, required_entry):
    if required_entry not in data_dict:
        print("Error: '{0}' entry does not contain a required '{1}' entry"
              .format(entry_name, required_entry),
              file=sys.stderr)
        sys.exit(1)
    return data_dict[required_entry]


def replace_type_aliases(model_type):
    if model_type in configurable_options['model_type_aliases']:
        model_type = configurable_options['model_type_aliases'][model_type]
    return model_type


def model_count_post_increment(model_type):
    global global_model_count
    try:
        count = global_model_count[model_type]
    except KeyError:
        count = 0
    global_model_count[model_type] = count + 1
    return count


def get_next_model_id(model_type):
    return model_id_mappings[model_type][model_count_post_increment(model_type)]


def create_pose_info(pose_dict, offset=None):
    xyz = get_field_with_default(pose_dict, 'xyz', [0, 0, 0])
    rpy = get_field_with_default(pose_dict, 'rpy', [0, 0, 0])
    for key in pose_dict:
        if key not in ['xyz', 'rpy']:
            print("Warning: ignoring unknown entry in 'pose': " + key, file=sys.stderr)
    if offset is not None:
        xyz = [sum(i) for i in zip(xyz, offset)]
    return PoseInfo(xyz, rpy)


def create_arm_info(name, arm_dict):
    arm_type = arm_dict['arm_type']
    initial_joint_states = arm_dict['default_initial_joint_states']
    pose = create_pose_info(arm_dict['pose'])
    return ArmInfo(name, arm_type, initial_joint_states, pose)


def create_sensor_info(name, sensor_data, allow_protected_sensors=False, offset=None):
    sensor_type = get_required_field(name, sensor_data, 'type')
    pose_dict = get_required_field(name, sensor_data, 'pose')
    for key in sensor_data:
        if key not in ['type', 'pose']:
            print("Warning: ignoring unknown entry in '{0}': {1}"
                  .format(name, key), file=sys.stderr)
    if sensor_type not in sensor_configs:
        if not allow_protected_sensors:
            print("Error: given sensor type '{0}' is not one of the known sensor types: {1}"
                  .format(sensor_type, sensor_configs.keys()), file=sys.stderr)
            sys.exit(1)
    pose_info = create_pose_info(pose_dict, offset=offset)
    return SensorInfo(name, sensor_type, pose_info)


def create_sensor_infos(sensors_dict, allow_protected_sensors=False, offset=None):
    sensor_infos = {}
    for name, sensor_data in sensors_dict.items():
        sensor_infos[name] = create_sensor_info(
            name, sensor_data,
            allow_protected_sensors=allow_protected_sensors, offset=offset)
    return sensor_infos


def create_model_info(model_name, model_data):
    model_type = get_required_field(model_name, model_data, 'type')
    model_type = replace_type_aliases(model_type)
    pose_dict = get_required_field(model_name, model_data, 'pose')
    reference_frame = get_field_with_default(model_data, 'reference_frame', '')
    for key in model_data:
        if key not in ['type', 'pose', 'reference_frame']:
            print("Warning: ignoring unknown entry in '{0}': {1}"
                  .format(model_name, key), file=sys.stderr)
    pose_info = create_pose_info(pose_dict)
    return ModelInfo(model_type, pose_info, reference_frame)


def create_models_to_spawn_infos(models_to_spawn_dict):
    models_to_spawn_infos = {}
    for reference_frame, reference_frame_data in models_to_spawn_dict.items():
        models = get_required_field(reference_frame, reference_frame_data, 'models')
        for model_name, model_to_spawn_data in models.items():
            model_to_spawn_data['reference_frame'] = reference_frame
            model_info = create_model_info(model_name, model_to_spawn_data)
            # assign each model a unique name because gazebo can't do this
            # if the models all spawn at the same time
            scoped_model_name = reference_frame.replace('::', '|') + '|' + \
                model_info.type + '_' + str(get_next_model_id(model_info.type))
            models_to_spawn_infos[scoped_model_name] = model_info
    return models_to_spawn_infos


def create_agv_info(agv_yaml_dict):
    agv_info = {}
    for agv_name_yaml, agv_info_dict_yaml in agv_yaml_dict.items():
        if agv_name_yaml in default_agv_origins:
            agv_id = agv_name_yaml[3:]
            agv_info[agv_id] = {}
            location = agv_info_dict_yaml['location']
            all_stations = stations[agv_name_yaml]
            if location in all_stations:
                station = all_stations[location]
                station_xyz = station['pose']['xyz']
                station_rpy = [0, 0, -1.570796]
                agv_info[agv_id]['pose'] = PoseInfo(station_xyz, station_rpy)
                agv_info[agv_id]['location'] = location
            else:
                print('=' * 80)
                print("Error: " + agv_name_yaml + " can not be assigned the station: " + location, file=sys.stderr)
                print('=' * 80)
    return agv_info


def create_models_over_bins_infos(models_over_bins_dict):
    models_to_spawn_infos = {}
    for bin_name, bin_dict in models_over_bins_dict.items():
        if bin_name in default_bin_origins:
            offset_xyz = [
                default_bin_origins[bin_name][0] - bin_depth / 2,
                default_bin_origins[bin_name][1] - bin_width / 2,
                bin_height + 0.08]
            # Allow the origin of the bin to be over-written
            if 'xyz' in bin_dict:
                offset_xyz = bin_dict['xyz']
        else:
            offset_xyz = get_required_field(bin_name, bin_dict, 'xyz')

        models = get_required_field(bin_name, bin_dict, 'models') or {}
        for model_type, model_to_spawn_dict in models.items():
            model_to_spawn_data = {}
            model_to_spawn_data['type'] = model_type
            model_to_spawn_data['reference_frame'] = 'world'
            xyz_start = get_required_field(
                model_type, model_to_spawn_dict, 'xyz_start')
            xyz_end = get_required_field(
                model_type, model_to_spawn_dict, 'xyz_end')
            rpy = get_required_field(model_type, model_to_spawn_dict, 'rpy')
            rpy[1] = -bin_angle
            num_models_x = get_required_field(
                model_type, model_to_spawn_dict, 'num_models_x')
            num_models_y = get_required_field(
                model_type, model_to_spawn_dict, 'num_models_y')
            step_size = [
                (xyz_end[0] - xyz_start[0]) / max(1, num_models_x - 1),
                (xyz_end[1] - xyz_start[1]) / max(1, num_models_y - 1)]

            # Create a grid of models
            for idx_x in range(num_models_x):
                for idx_y in range(num_models_y):
                    model_x_offset = xyz_start[0] + idx_x * step_size[0]
                    xyz = [
                        offset_xyz[0] + model_x_offset,
                        offset_xyz[1] + xyz_start[1] + idx_y * step_size[1],
                        offset_xyz[2] + xyz_start[2] + model_x_offset * math.tan(bin_angle)]
                    model_to_spawn_data['pose'] = {'xyz': xyz, 'rpy': rpy}
                    model_info = create_model_info(model_type, model_to_spawn_data)
                    # assign each model a unique name because gazebo can't do this
                    # if the models all spawn at the same time
                    scoped_model_name = bin_name + '|' + \
                        model_info.type + '_' + str(get_next_model_id(model_type))
                    model_info.bin = bin_name
                    models_to_spawn_infos[scoped_model_name] = model_info
    return models_to_spawn_infos


def create_briefcase_over_stations_infos(models_over_stations_dict):
    models_to_spawn_infos = {}
    for station_name in default_station_origins:
        model_to_spawn_data = {}
        model_to_spawn_data['type'] = 'assembly_briefcase'
        model_to_spawn_data['reference_frame'] = 'world'
        xyz = [brief_case_offset_x, brief_case_offset_y, brief_case_offset_z]
        rpy = [0, 0, 0]

        # make sure the station name is expected
        if station_name in default_station_origins:
            offset_xyz = [
                 default_station_origins[station_name][0] + xyz[0],
                 default_station_origins[station_name][1] + xyz[1],
                 xyz[2]]

            model_to_spawn_data['pose'] = {'xyz': offset_xyz, 'rpy': rpy}
            model_info = create_model_info('assembly_briefcase', model_to_spawn_data)
            # assign each model a unique name because gazebo can't do this
            # if the models all spawn at the same time
            scoped_model_name = station_name + '|assembly_briefcase' + station_name.replace('station', '')
            model_info.briefcase = station_name
            models_to_spawn_infos[scoped_model_name] = model_info
    return models_to_spawn_infos


def create_models_over_stations_infos(models_over_stations_dict):
    models_to_spawn_infos = {}
    for station_name, station_dict in models_over_stations_dict.items():
        # print("********", station_name)
        models = get_required_field(station_name, station_dict, 'models') or {}
        station_id = station_name.replace('as', '')
        for model_type, model_to_spawn_dict in models.items():
            # print("********", model_type)
            model_to_spawn_data = {}
            model_to_spawn_data['type'] = model_type
            model_to_spawn_data['reference_frame'] = 'world'
            xyz = get_required_field(
                model_type, model_to_spawn_dict, 'xyz')
            rpy = get_required_field(model_type, model_to_spawn_dict, 'rpy')

            # print("********", xyz)
            # print("********", rpy)

            if station_name in default_station_origins:
                offset_xyz = [
                    default_briefcase_origins['briefcase'+station_id][0] + xyz[0],
                    default_briefcase_origins['briefcase'+station_id][1] + xyz[1],
                    default_briefcase_origins['briefcase'+station_id][2] + xyz[2]]
                # print("********", offset_xyz)
                # spawn_briefcase_over_stations_infos(station_name, default_station_origins[station_name])

            model_to_spawn_data['pose'] = {'xyz': offset_xyz, 'rpy': rpy}
            model_info = create_model_info(model_type, model_to_spawn_data)
            # assign each model a unique name because gazebo can't do this
            # if the models all spawn at the same time
            scoped_model_name = station_name + '|assembly_briefcase' + str(station_id) + '|' + \
                    model_info.type + '_' + str(get_next_model_id(model_type))
            model_info.station = station_name
            models_to_spawn_infos[scoped_model_name] = model_info
    return models_to_spawn_infos


def create_models_over_agvs_infos(agv_yaml_dict):
    models_to_spawn_infos = {}
    agv_info = {}
    for agv_name_yaml, agv_info_dict_yaml in agv_yaml_dict.items():
        if agv_name_yaml in default_agv_origins:
            agv_id = agv_name_yaml[3:]
            agv_info[agv_id] = {}
            location = agv_info_dict_yaml['location']
            all_stations = stations[agv_name_yaml]
            if location in all_stations:
                station = all_stations[location]
                station_xyz = station['pose']['xyz']
                station_rpy = [0, 0, -1.570796]
                agv_info[agv_id] = PoseInfo(station_xyz, station_rpy)
            else:
                print('=' * 80)
                print("Error: " + agv_name_yaml + " can not be assigned the station: " + location, file=sys.stderr)
                print('=' * 80)
            # spawn parts on the agv
            if 'products' in agv_info_dict_yaml:
                product_dict = agv_info_dict_yaml['products']
                # create_models_over_agvs_infos(agv_name_yaml,station_xyz,station_rpy, product_dict)
                for part_info in product_dict.items():
                    model_to_spawn_data = {}
                    # part_name = part_info[0]
                    part_pose_type = part_info[1]
                    # for p_pose in part_pose_type.items():
                    # print(part_pose_type['pose']['xyz'])
                    # print(part_pose_type['type'])
                    model_type = part_pose_type['type']
                    # tray_x = 0.0
                    # tray_y = 0.15
                    # tray_z = 0.75

                    tray_x_in_world = station_xyz[0]+0.15
                    tray_y_in_world = station_xyz[1]
                    tray_z_in_world = 0.75
                    # xyz = [station_xyz[0] + tray_y + part_pose_type['pose']['xyz'][1],
                    # station_xyz[1] + tray_x + part_pose_type['pose']['xyz'][0],
                    # station_xyz[2] + tray_z + 0.3]

                    xyz = [tray_x_in_world + part_pose_type['pose']['xyz'][1],
                    tray_y_in_world - part_pose_type['pose']['xyz'][0],
                    tray_z_in_world + 0.1]

                    rpy = [part_pose_type['pose']['rpy'][0], part_pose_type['pose']['rpy'][1], (part_pose_type['pose']['rpy'][2])-1.571]

                    model_to_spawn_data['type'] = model_type
                    model_to_spawn_data['reference_frame'] = 'world'
                    model_to_spawn_data['pose'] = {'xyz': xyz, 'rpy': rpy}
                    model_info = create_model_info(model_type, model_to_spawn_data)
                    # assign each model a unique name because gazebo can't do this
                    # # if the models all spawn at the same time
                    scoped_model_name = agv_name_yaml + '|tray_' + agv_id + "|" + \
                    model_info.type + '_' + str(get_next_model_id(model_type))
                    model_info.agv = agv_name_yaml
                    models_to_spawn_infos[scoped_model_name] = model_info
    return models_to_spawn_infos


def create_belt_model_infos(belt_models_dict, models_over_bins):
    belt_model_infos = {}
    for obj_type, spawn_times in belt_models_dict.items():
        count = 0
        for spawn_time, belt_model_dict in spawn_times.items():
            obj_type = replace_type_aliases(obj_type)

            # let's check if this type is also in the bins
            if models_over_bins:
                for key in models_over_bins:
                    if obj_type in key:
                        # print("OBJECT TYPE", obj_type)
                        count += 1
                # print("KEY", key)
            if obj_type not in belt_model_infos:
                belt_model_infos[obj_type] = {}
            belt_model_dict['type'] = obj_type
            belt_model_infos[obj_type][spawn_time] = create_model_info('belt_model', belt_model_dict)
            belt_model_infos[obj_type]['start_index'] = count+1
    # print(belt_model_infos)
    return belt_model_infos


def create_drops_info(drops_dict):
    drops_info = {}
    drop_region_infos = []
    drop_regions_dict = get_required_field('drops', drops_dict, 'drop_regions')
    for drop_name, drop_region_dict in drop_regions_dict.items():
        frame = get_field_with_default(drop_region_dict, 'frame', 'world')
        drop_region_min = get_required_field('drop_region', drop_region_dict, 'min')
        drop_region_min_xyz = get_required_field('min', drop_region_min, 'xyz')
        drop_region_max = get_required_field('drop_region', drop_region_dict, 'max')
        drop_region_max_xyz = get_required_field('max', drop_region_max, 'xyz')
        destination_info = get_required_field('drop_region', drop_region_dict, 'destination')
        # print(drop_name)
        # print(destination_info)
        destination = create_pose_info(destination_info)
        product_type = get_required_field('drop_region', drop_region_dict, 'product_type_to_drop')
        product_type = replace_type_aliases(product_type)
        drop_region_infos.append(
            DropRegionInfo(
                drop_name, drop_region_min_xyz, drop_region_max_xyz,
                destination, frame, product_type))
    drops_info['drop_regions'] = drop_region_infos
    return drops_info


def create_order_info(name, order_dict):
    '''
    Below is an example of an order to work with this function
    name: order_0, order_1, etc

orders:
  order_0:
    kitting_robot: on
    assembly_robot: on
    announcement_condition: time
    announcement_condition_value: 0.0
    kitting:

      shipment_count: 1
      agvs: [agv1]
      destinations: [station4]
      products:
        part_0:
          type: assembly_battery_blue
          pose:
            xyz: [0.1, -0.1, 0]
            rpy: [0, 0, 0]
    assembly:
      announcement_condition: time
      announcement_condition_value: 0.0
      shipment_count: 1
      stations: [station4]
      products:
        part_0:
          type: assembly_battery_blue
          pose:
            xyz: [-0.032465, 0.174845, 1.35]
            rpy: [0, 0, 0]
    '''

    # get the list of robots to disable
    disable_robot = order_dict.get('disable_robot', [])
    disable_kitting_robot = []
    disable_assembly_robot = []

    # for item in disable_robot:
    #     print(item)

    if disable_robot:
        # print(len(disable_robot), type(disable_robot))
        robot_name, location, number_of_parts = disable_robot
        if robot_name != "kitting_robot" and robot_name != "assembly_robot":
            error_message = """[FATAL] In yaml file: the field disable_robot contains a wrong robot type.
            Options are kitting_robot or assembly_robot
            Given robot name is: {}""".format(robot_name)
            print(error_message, file=sys.stderr)

    order_priority = order_dict.get('priority', 1)
    kitting_robot_health = order_dict.get('kitting_robot_health', 1)
    assembly_robot_health = order_dict.get('assembly_robot_health', 1)
    announcement_condition = get_required_field(name, order_dict, 'announcement_condition')
    announcement_condition_value = get_required_field(
        name, order_dict, 'announcement_condition_value')

    returned_dict = {}
    kitting_flag = False
    assembly_flag = False

    if ('kitting' not in order_dict) and ('assembly' not in order_dict):
        error_message = """ [FATAL] In yaml file: There is no kitting or assembly shipment"""
        print(error_message, file=sys.stderr)

    if 'kitting' in order_dict:
        kitting_dict = order_dict['kitting']
        kitting_flag = True
        shipment_count = get_field_with_default(kitting_dict, 'shipment_count', 1)
        agvs = get_field_with_default(kitting_dict, 'agvs', ["any"] * shipment_count)
        stations = get_required_field(name, kitting_dict, 'destinations')
        products_dict = get_required_field(name, kitting_dict, 'products')
        products = []
        for product_name, product_dict in products_dict.items():
            products.append(create_model_info(product_name, product_dict))

        returned_dict['kitting_shipment_count'] = shipment_count
        returned_dict['kitting_agvs'] = agvs
        returned_dict['kitting_agv_stations'] = stations
        returned_dict['kitting_products'] = products

    if 'assembly' in order_dict:
        assembly_flag = True
        assembly_dict = order_dict['assembly']
        shipment_count = get_field_with_default(assembly_dict, 'shipment_count', 1)
        stations = get_required_field(name, assembly_dict, 'stations')
        products_dict = get_required_field(name, assembly_dict, 'products')
        products = []
        for product_name, product_dict in products_dict.items():
            products.append(create_model_info(product_name, product_dict))

        returned_dict['assembly_shipment_count'] = shipment_count
        returned_dict['assembly_stations'] = stations
        returned_dict['assembly_products'] = products

    if disable_robot:
        returned_dict['disable_robot'] = disable_robot
    else:
        returned_dict['disable_robot'] = []
    returned_dict['announcement_condition'] = announcement_condition
    returned_dict['announcement_condition_value'] = announcement_condition_value
    returned_dict['kitting_flag'] = kitting_flag
    returned_dict['assembly_flag'] = assembly_flag
    returned_dict['priority'] = order_priority
    returned_dict['kitting_robot_health'] = kitting_robot_health
    returned_dict['assembly_robot_health'] = assembly_robot_health

    return returned_dict


def create_order_infos(orders_dict):
    order_infos = {}
    for order_name, order_dict in orders_dict.items():
        order_infos[order_name] = create_order_info(order_name, order_dict)
    return order_infos


def create_faulty_products_info(faulty_products_dict):
    faulty_product_infos = {}
    for product_name in faulty_products_dict:
        faulty_product_infos[product_name] = product_name  # no other info for now
    return faulty_product_infos


def create_bin_infos():
    bin_infos = {}
    for bin_name, xyz in default_bin_origins.items():
        bin_infos[bin_name] = PoseInfo(xyz, [0, bin_angle, 3.14159])
        # print(bin_infos[bin_name])
    return bin_infos


def create_station_infos():
    station_infos = {}
    for station_name, xyz in default_station_origins.items():
        station_infos[station_name] = PoseInfo(xyz, [0, 0, 0])
    return station_infos


def create_material_location_info(belt_models, models_over_bins, models_over_agvs):
    material_locations = {}

    # Specify in which agv the different products can be found
    for agv_product_name, agv_product in models_over_agvs.items():
        if agv_product.type in material_locations:
            material_locations[agv_product.type].update([agv_product.agv])
        else:
            material_locations[agv_product.type] = {agv_product.agv}

    # Specify that belt products can be found on the conveyor belt
    # print("material", belt_models)
    for keys, values in belt_models.items():
        # print('keys:', keys)
        # print('values:', values)
        for key, value in values.items():
            if isinstance(key, float):
                # print('KEY:', key)
                # print('VALUE:', type(value))
                if value.type in material_locations:
                    material_locations[value.type].update(['belt'])
                else:
                    material_locations[value.type] = {'belt'}

    # for _, spawn_times in belt_models.items():
    #     for spawn_time, product in spawn_times.items():
    #         if product.type in material_locations:
    #             material_locations[product.type].update(['belt'])
    #         else:
    #             material_locations[product.type] = {'belt'}

    # Specify in which bin the different bin products can be found
    for product_name, product in models_over_bins.items():
        # print("bin product name: ", product_name)
        # print("bin product type: ", product.type)
        if product.type in material_locations:
            material_locations[product.type].update([product.bin])
        else:
            material_locations[product.type] = {product.bin}

    return material_locations


def create_options_info(options_dict):
    options = configurable_options
    for option, val in options_dict.items():
        options[option] = val
    return options


def prepare_template_data(config_dict, args):
    template_data = {
        'arms': [create_arm_info(name, conf) for name, conf in arm_configs.items()],
        'robot_camera': {},
        'sensors': create_sensor_infos(default_sensors, allow_protected_sensors=True),
        'agv_infos': {},
        'models_to_insert': {},
        'models_to_spawn': {},
        # 'belt_models': create_belt_model_infos(default_belt_models),
        'belt_models': {},
        'faulty_products': {},
        'drops': {},
        'orders': {},
        'options': {'insert_agvs': True},
        'time_limit': default_time_limit,
        'bin_height': bin_height,
        'station_height': station_height,
        'world_dir': world_dir,
        'joint_limited_ur10': config_dict.pop('joint_limited_ur10', False),
        'sensor_blackout': {},
    }
    # Process the options first as they may affect the processing of the rest
    options_dict = get_field_with_default(config_dict, 'options', {})
    template_data['options'].update(create_options_info(options_dict))
    if args.state_logging is not None:
        template_data['options']['gazebo_state_logging'] = args.state_logging
    if args.visualize_sensor_views:
        template_data['options']['visualize_sensor_views'] = True

    models_over_bins = {}
    models_over_belt = {}

    # always process models over bins before belt models
    for key, value in config_dict.items():
        if key == 'models_over_bins':
            models_over_bins = create_models_over_bins_infos(value)
            template_data['models_to_insert'].update(models_over_bins)
    # print(models_over_bins)

    # take care of belt models using models_over_bins
    for key, value in config_dict.items():
        if key == 'belt_models':
            models_over_belt = create_belt_model_infos(value, models_over_bins)
            template_data['belt_models'].update(models_over_belt)

    # print(models_over_belt)

    for key, value in config_dict.items():
        if key == 'sensors':
            template_data['sensors'].update(
                create_sensor_infos(value))
        elif key == 'agv_infos':
            template_data['agv_infos'].update(create_agv_info(value))
            # print(template_data['agv_infos'])
            models_over_agvs = create_models_over_agvs_infos(value)
            template_data['models_to_insert'].update(models_over_agvs)
        elif key == 'models_over_stations':
            models_over_stations = create_models_over_stations_infos(value)
            template_data['models_to_insert'].update(models_over_stations)
            # briefcase_over_stations = create_briefcase_over_stations_infos(value)
            # template_data['models_to_insert'].update(briefcase_over_stations)
        elif key == 'briefcase_over_stations':
            pass
        elif key == 'belt_models':
            pass
        elif key == 'models_over_bins':
            pass
            # models_over_belt = create_belt_model_infos(value, models_over_bins)
            # template_data['belt_models'].update(models_over_belt)
        #     template_data['belt_models'].update(create_belt_model_infos(value))
        elif key == 'drops':
            template_data['drops'].update(create_drops_info(value))
            # print(template_data['drops'])
        elif key == 'faulty_products':
            template_data['faulty_products'].update(create_faulty_products_info(value))
        elif key == 'orders':
            template_data['orders'].update(create_order_infos(value))
            # print(template_data['orders'])
        elif key == 'sensor_blackout':
            template_data['sensor_blackout'].update(value)
        elif key == 'options':
            pass
        elif key == 'models_to_spawn':
            template_data['models_to_spawn'].update(
                create_models_to_spawn_infos(value))
        elif key == 'time_limit':
            template_data['time_limit'] = value
        elif key == 'robot_camera':
            if value['enable']:
                template_data['options']['enable_robot_camera'] = True
            else:
                template_data['options']['enable_robot_camera'] = False
            # print(template_data['options']['enable_robot_camera'])
        # elif key == 'use_robot_camera':
        #     template_data['use_robot_camera'] = value
        else:
            print("Error: unknown top level entry '{0}'".format(key), file=sys.stderr)
            sys.exit(1)
    template_data['bins'] = create_bin_infos()
    template_data['stations'] = create_station_infos()
    template_data['material_locations'] = create_material_location_info(
        template_data['belt_models'] or {},
        models_over_bins,
        models_over_agvs,
    )
    template_data['possible_products'] = possible_products
    return template_data


def generate_files(template_data):
    files = {}
    for template_file in template_files:
        with open(template_file, 'r') as f:
            data = f.read()
        files[template_file] = em.expand(data, template_data)
    # Generate files for each arm
    for arm_info in template_data['arms']:
        template_data['arm'] = arm_info
        with open(arm_template_file, 'r') as f:
            data = f.read()
        files[arm_info.name + '.urdf.xacro'] = em.expand(data, template_data)
    return files
    return files


def main(sysargv=None):
    parser = argparse.ArgumentParser(
        description='Prepares and then executes a gazebo simulation based on configurations.')
    prepare_arguments(parser)
    args = parser.parse_args(sysargv)
    config_data = args.config or ''
    if args.file is not None:
        for file in args.file:
            with open(file, 'r') as f:
                comp_config_data = f.read()
                config_data += comp_config_data
    dict_config = yaml.load(config_data) or {}
    expanded_dict_config = expand_yaml_substitutions(dict_config)
    if args.verbose:
        print(yaml.dump({'Using configuration': expanded_dict_config}))

    random_seed = expanded_dict_config.pop('random_seed', None)
    initialize_model_id_mappings(random_seed)

    template_data = prepare_template_data(expanded_dict_config, args)
    files = generate_files(template_data)
    if not args.dry_run and not os.path.isdir(args.output):
        if os.path.exists(args.output) and not os.path.isdir(args.output):
            print('Error, given output directory exists but is not a directory.', file=sys.stderr)
            sys.exit(1)
        print('creating directory: ' + args.output)
        os.makedirs(args.output)
    for name, content in files.items():
        if name.endswith('.template'):
            name = name[:-len('.template')]
        name = os.path.basename(name)
        if args.dry_run:
            print('# file: ' + name)
            print(content)
        else:
            file_path = os.path.join(args.output, name)
            print('writing file ' + file_path)
            with open(file_path, 'w+') as f:
                f.write(content)
    cmd = [
        'roslaunch',
        os.path.join(args.output, 'gear.launch'),
        'world_path:=' + os.path.join(args.output, 'ariac.world'),
        'gear_urdf_xacro:=' + os.path.join(args.output, 'gear.urdf.xacro'),
        'gantry_urdf_xacro:=' + os.path.join(args.output, 'gantry.urdf.xacro'),
        'kitting_urdf_xacro:=' + os.path.join(args.output, 'kitting.urdf.xacro'),
    ]
    if args.log_to_file:
        cmd.append('gazebo_ros_output:=log')
    if args.verbose:
        cmd += ['verbose:=true']
    if args.no_gui:
        cmd += ['gui:=false']
    if args.load_moveit:
        cmd += ['load_moveit:=true']

    if not args.development_mode:
        os.environ['ARIAC_COMPETITION'] = '1'

    print('Running command: ' + ' '.join(cmd))
    if not args.dry_run:
        try:
            p = subprocess.Popen(cmd)
            p.wait()
        except KeyboardInterrupt:
            pass
        finally:
            p.wait()
        return p.returncode


if __name__ == '__main__':
    # Filter out any special ROS remapping arguments.
    # This is necessary if the script is being run from a ROS launch file.
    import rospy
    filtered_argv = rospy.myargv(sys.argv)
    sys.exit(main(filtered_argv[1:]))
