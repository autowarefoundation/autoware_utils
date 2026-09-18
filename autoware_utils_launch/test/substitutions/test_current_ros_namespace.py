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

from launch import LaunchContext
from launch.actions import LogInfo
from launch.actions import PopLaunchConfigurations
from launch.actions import PushLaunchConfigurations
from launch.frontend import Parser
from launch.frontend.parse_substitution import parse_substitution
from launch.utilities import perform_substitutions
from launch_ros.actions import PushRosNamespace
import pytest

from autoware_utils_launch.substitutions import CurrentRosNamespace


def test_no_namespace():
    context = LaunchContext()
    assert CurrentRosNamespace().perform(context) == ""


def test_single_namespace():
    context = LaunchContext()
    PushRosNamespace("foo").execute(context)
    assert CurrentRosNamespace().perform(context) == "/foo"


def test_nested_namespace():
    context = LaunchContext()
    PushRosNamespace("foo").execute(context)
    PushRosNamespace("bar").execute(context)
    assert CurrentRosNamespace().perform(context) == "/foo/bar"


def test_absolute_namespace():
    context = LaunchContext()
    PushRosNamespace("foo").execute(context)
    PushRosNamespace("/bar").execute(context)
    assert CurrentRosNamespace().perform(context) == "/bar"


def test_namespace_is_scoped():
    context = LaunchContext()
    push_conf = PushLaunchConfigurations()
    pop_conf = PopLaunchConfigurations()

    push_conf.execute(context)
    PushRosNamespace("foo").execute(context)
    assert CurrentRosNamespace().perform(context) == "/foo"
    pop_conf.execute(context)
    assert CurrentRosNamespace().perform(context) == ""


def test_parse_without_arguments():
    substitutions = parse_substitution("$(current-ros-namespace)")
    assert len(substitutions) == 1
    assert isinstance(substitutions[0], CurrentRosNamespace)


def test_parse_with_arguments():
    # The parser wraps the TypeError raised by the substitution, so match the message instead.
    with pytest.raises(Exception, match="current-ros-namespace substitution expects 0 arguments"):
        parse_substitution("$(current-ros-namespace arg)")


def test_frontend_xml():
    xml = """
    <launch>
        <push-ros-namespace namespace="foo"/>
        <log message="$(current-ros-namespace)/test"/>
        <push-ros-namespace namespace="bar"/>
        <log message="$(current-ros-namespace)/test"/>
    </launch>
    """
    entity, parser = Parser.load(io.StringIO(xml))
    description = parser.parse_description(entity)

    context = LaunchContext()
    results = []
    for action in description.entities:
        if isinstance(action, PushRosNamespace):
            action.execute(context)
        if isinstance(action, LogInfo):
            results.append(perform_substitutions(context, action.msg))
    assert results == ["/foo/test", "/foo/bar/test"]
