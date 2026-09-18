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


import pathlib

from launch import Action
from launch.frontend import Entity
from launch.frontend import Parser
from launch.frontend import expose_action
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
import yaml


@expose_action("autoware_global_parameters")
class GlobalParameters(Action):
    def __init__(self, paths: SomeSubstitutionsType, **kwargs):
        super().__init__(**kwargs)
        self._paths = normalize_to_list_of_substitutions(paths)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        _, kwargs = super().parse(entity, parser)
        kwargs["paths"] = parser.parse_substitution(entity.get_attr("paths"))
        return cls, kwargs

    def execute(self, context: LaunchContext):
        paths = perform_substitutions(context, self._paths)
        paths = yaml.safe_load(paths)
        paths = paths if type(paths) is list else [paths]
        paths = [pathlib.Path(path) for path in paths]

        # If the file does not exist, leave it to exception handling.
        global_params = context.launch_configurations.get("global_params", [])
        for path in paths:
            with path.open("r") as fp:
                params = yaml.safe_load(fp)["/**"]["ros__parameters"]
            global_params.extend(params.items())
        context.launch_configurations["global_params"] = global_params
