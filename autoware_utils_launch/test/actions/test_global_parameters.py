# Copyright 2026 The Autoware Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import io
import pathlib

from launch import LaunchContext
from launch.actions import PopLaunchConfigurations
from launch.actions import PushLaunchConfigurations
from launch.frontend import Parser
from launch.substitutions import TextSubstitution

from autoware_utils_launch.actions import GlobalParameters

PARAM_FILE_1 = str(pathlib.Path(__file__).parent / "example1.param.yaml")
PARAM_FILE_2 = str(pathlib.Path(__file__).parent / "example2.param.yaml")
PARAM_DATA_1 = [("param_1_name", 10), ("param_2_name", "foo")]
PARAM_DATA_2 = [("param_3_name", 20.0), ("param_4_name", True)]


def test_single_path():
    context = LaunchContext()
    GlobalParameters(PARAM_FILE_1).execute(context)
    assert context.launch_configurations == {"global_params": PARAM_DATA_1}


def test_path_list():
    context = LaunchContext()
    GlobalParameters(f"[{PARAM_FILE_1}, {PARAM_FILE_2}]").execute(context)
    assert context.launch_configurations == {"global_params": PARAM_DATA_1 + PARAM_DATA_2}


def test_multiple_actions_are_merged():
    context = LaunchContext()
    GlobalParameters(PARAM_FILE_1).execute(context)
    GlobalParameters(PARAM_FILE_2).execute(context)
    assert context.launch_configurations == {"global_params": PARAM_DATA_1 + PARAM_DATA_2}


def test_missing_path_is_ignored():
    context = LaunchContext()
    GlobalParameters(f"[{PARAM_FILE_1}, /no/such/file.param.yaml]").execute(context)
    assert context.launch_configurations == {"global_params": PARAM_DATA_1}


def test_only_missing_path():
    context = LaunchContext()
    GlobalParameters("/no/such/file.param.yaml").execute(context)
    assert context.launch_configurations == {"global_params": []}


def test_path_is_substituted():
    context = LaunchContext()
    paths = [TextSubstitution(text=PARAM_FILE_1)]
    GlobalParameters(paths).execute(context)
    assert context.launch_configurations == {"global_params": PARAM_DATA_1}


def test_params_are_scoped():
    context = LaunchContext()
    push_conf = PushLaunchConfigurations()
    pop_conf = PopLaunchConfigurations()

    push_conf.execute(context)
    GlobalParameters(PARAM_FILE_1).execute(context)
    assert context.launch_configurations == {"global_params": PARAM_DATA_1}
    pop_conf.execute(context)
    assert context.launch_configurations == {}


def test_frontend_xml():
    xml = f"""
    <launch>
        <autoware_global_parameters paths="{PARAM_FILE_1}"/>
        <autoware_global_parameters paths="[{PARAM_FILE_2}]"/>
    </launch>
    """
    entity, parser = Parser.load(io.StringIO(xml))
    description = parser.parse_description(entity)

    context = LaunchContext()
    for action in description.entities:
        assert isinstance(action, GlobalParameters)
        action.execute(context)
    assert context.launch_configurations == {"global_params": PARAM_DATA_1 + PARAM_DATA_2}
