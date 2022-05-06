# Copyright 2022 Christoph Hellmann Santos
#           2019 Open Source Robotics Foundation, Inc
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

"""Module for the Composable Node action."""
from asyncio import events
import threading
from tkinter.filedialog import LoadFileDialog
from typing import Optional, cast
from typing import Iterable
from typing import List
from typing import Text
from pathlib import Path
import launch
from launch import LaunchContext, SomeSubstitutionsType
from launch_ros.parameters_type import SomeParameters
from launch_ros.remap_rule_type import SomeRemapRules
from launch.action import Action
from launch.condition import Condition
from launch.substitutions import LocalSubstitution

from launch.utilities import ensure_argument_type
from launch.utilities import perform_substitutions
from launch.utilities import normalize_to_list_of_substitutions

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.utilities import add_node_name
from launch_ros.utilities import normalize_remap_rules, normalize_parameters
from launch_ros.utilities import make_namespace_absolute
from launch_ros.utilities import prefix_namespace
from launch_ros.utilities import get_node_name_count
from launch_ros.utilities import to_parameters_list
from launch_ros.utilities import evaluate_parameters
from launch_ros.utilities.normalize_parameters import normalize_parameter_dict
from launch_ros.ros_adapters import get_ros_node

from rclpy.validate_namespace import validate_namespace
from rclpy.validate_node_name import validate_node_name

from ..events import LoadNodeEvent

import composition_interfaces.srv

from .composable_node_container import ComposableNodeContainer
class ComposableNode(Action):
    """Action that adds a node to a container for execution."""
    UNSPECIFIED_NODE_NAME = '<node_name_unspecified>'
    UNSPECIFIED_NODE_NAMESPACE = '<node_namespace_unspecified>'

    def __init__(
        self, *, 
        component: SomeSubstitutionsType,
        package: SomeSubstitutionsType,
        target_container: Optional[SomeSubstitutionsType] = None,
        condition: Optional[Condition] = None,
        name: Optional[SomeSubstitutionsType] = None,
        namespace: Optional[SomeSubstitutionsType] = None,
        parameters: Optional[SomeParameters] = None,
        remappings: Optional[SomeRemapRules] = None,
        extra_arguments: Optional[Iterable[SomeSubstitutionsType]] = None
        ) -> None:
        """Constructs a Composable Node action

        :param component: Name of the component to be loaded
        :param package: Package that holds the component
        :param condition: Condition for the execution of the action
          , defaults to None
        :param name: Name of the node that will be loaded, defaults to None
        :param namespace: Namespace for the node, defaults to None
        :param parameters: Parameters to pass to the node, defaults to None
        :param remappings: Remappings to pass to the node, defaults to None
        :param extra_arguments: Additional arguments, defaults to None
        """
        super().__init__(condition=condition)
        if parameters is not None:
            ensure_argument_type(parameters, (list), 'parameters', 'Node')
            normalized_params = normalize_parameters(parameters)
        self.__component = component
        self.__package = package
        self.__node_name = name
        self.__target_container = target_container
        self.__node_namespace = namespace
        self.__parameters = [] if parameters is None else normalized_params
        self.__remappings = [] if remappings is None else list(normalize_remap_rules(remappings))
        self.__extra_args = [] if extra_arguments is None else normalize_parameters(extra_arguments)
        self.__process_started_handler = None
        self.__expanded_package = None
        self.__expanded_component = None
        self.__expanded_node_name = None
        self.__expanded_node_namespace = None
        self.__expanded_parameters = None
        self.__expanded_target_container = None
        self.__final_node_name = None
        self.__expanded_remappings = None
        self.__expanded_extra_args = None

        self.__substitutions_performed = False
        self.__loaded = False
        self.__logger = launch.logging.get_logger(__name__)

    @property
    def node_name(self):
        """Getter for node_name."""
        if self.__final_node_name is None:
            raise RuntimeError("cannot access 'node_name' before executing action")
        return self.__final_node_name

    @property
    def node_package(self):
        """Getter for node_package."""
        if self.__expanded_package is None:
            raise RuntimeError("cannot access 'package' before executing action")
        return self.__expanded_package

    @property
    def node_component(self):
        """Getter for node_component."""
        if self.__expanded_component is None:
            raise RuntimeError("cannot access 'component' before executing action")
        return self.__expanded_component

    @property
    def has_target_container(self) -> bool:
        """True when target container is specified."""
        if self.__target_container is None:
            return False
        return True

    @property
    def target_container(self):
        """Getter for target_container."""
        if self.__expanded_target_container is None:
            raise RuntimeError("cannot access 'component' before executing action")
        return self.__expanded_target_container
    
    @target_container.setter
    def set_target_container(
        self, 
        target_container_name: Optional[SomeSubstitutionsType] = None):
        """Setter for target_container"""
        self.__target_container = target_container_name

    @property
    def loaded(self) -> bool:
        return self.__loaded

    @loaded.setter
    def loaded(self, value: bool):
        self.__loaded = value

    def _perform_substitutions(self, context: LaunchContext) -> None:
        """Performs substitutions on the inputs

        :param context: context used for the substitutions
        """
        if self.__substitutions_performed:
            return
        self.__substitutions_performed = True

        # Handle node_name
        if self.__node_name is not None:
            self.__expanded_node_name = perform_substitutions(
                context, normalize_to_list_of_substitutions(self.__node_name))
            validate_node_name(self.__expanded_node_name)
        self.__expanded_node_name.lstrip('/')

        # Handle namespace
        expanded_node_namespace: Optional[Text] = None
        if self.__node_namespace is not None:
            expanded_node_namespace = perform_substitutions(
                context, normalize_to_list_of_substitutions(self.__node_namespace))
        base_ns = context.launch_configurations.get('ros_namespace', None)
        expanded_node_namespace = make_namespace_absolute(
            prefix_namespace(base_ns, expanded_node_namespace))
        if expanded_node_namespace is not None:
            self.__expanded_node_namespace = expanded_node_namespace
            validate_namespace(self.__expanded_node_namespace)
        
        # Set final node_name
        self.__final_node_name = prefix_namespace(
            self.__expanded_node_namespace, self.__expanded_node_name)
        
        # Expand component
        self.__expanded_component = perform_substitutions(
            context=context,
            subs=normalize_to_list_of_substitutions(subs=self.__component))

        # Expand package
        self.__expanded_package = perform_substitutions(
            context=context,
            subs=normalize_to_list_of_substitutions(subs=self.__package))

        # Expand parameters
        params_container = context.launch_configurations.get('global_params', None)
        parameters = []
        if params_container is not None:
            for param in params_container:
                if isinstance(param, tuple):
                    subs = normalize_parameter_dict({param[0]: param[1]})
                    parameters.append(subs)
                else:
                    param_file_path = Path(param).resolve()
                    assert param_file_path.is_file()
                    subs = ParameterFile(param_file_path)
                    parameters.append(subs)
        if self.__parameters is not None:
            parameters.extend(list(self.__parameters))
        if parameters:
            self.__expanded_parameters = [
                param.to_parameter_msg() for param in to_parameters_list(
                    context,
                    self.__expanded_node_name,
                    self.__expanded_node_namespace,
                    evaluate_parameters(
                        context, parameters
                    )
                )
            ]
        
        # expand remappings
        remappings = []
        global_remaps = context.launch_configurations.get('ros_remaps', None)
        if global_remaps:
            remappings.extend([f'{src}:={dst}' for src, dst in global_remaps])
        if self.__remappings:
            remappings.extend([
                f'{perform_substitutions(context, src)}:={perform_substitutions(context, dst)}'
                for src, dst in self.__remappings
            ])
        if remappings:
            self.__expanded_remappings = remappings

        # expand extra arguments
        if self.__extra_args is not None:
            self.__expanded_extra_args = [
                param.to_parameter_msg() for param in to_parameters_list(
                    context, 
                    self.__expanded_node_name,
                    self.__expanded_node_namespace,
                    evaluate_parameters(
                        context, self.__extra_args
                    )
                )
            ]
        
        # expand target container
        if self.__target_container is not None:
            self.__expanded_target_container = perform_substitutions(
                context=context,
                subs=normalize_to_list_of_substitutions(
                    subs=self.__target_container))
    
    def _on_container_available(self, context: LaunchContext):
        typed_event = cast(launch.events.process.ProcessStarted, context.locals.event)
        action = cast(Node, typed_event.action)
        matcher = match_container_by_name(self.__expanded_target_container)
        if matcher(action):
            # Create a client to load nodes in the target container.
            request = composition_interfaces.srv.LoadNode.Request()
            request.node_name = self.__expanded_node_name
            request.plugin_name = self.__expanded_component
            request.package_name = self.__expanded_package
            if self.__expanded_parameters is not None:
                request.parameters = self.__expanded_parameters
            else:
                request.parameters = []
            if self.__expanded_remappings is not None:
                request.remap_rules = self.__expanded_remappings
            else:
                request.remap_rules = []
            if self.__expanded_extra_args is not None:
                request.extra_arguments = self.__expanded_extra_args
            else:
                request.extra_arguments = []
            
            event = LoadNodeEvent(
                target_container_matcher=match_container_by_name(self.__expanded_target_container),
                load_request=request,
                action=self
            )
            if self.__process_started_handler is not None:
                context.unregister_event_handler(self.__process_started_handler)
            context.asyncio_loop.call_soon_threadsafe(lambda: context.emit_event_sync(event))
            


    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """Execute the Action

        :param context: The context to execute in
        :return: None
        """
        self._perform_substitutions(context=context)
        if self.__expanded_target_container is None:
            raise RuntimeError("Target container not specified")

        count = get_node_name_count(context=context, node_name=self.__expanded_node_name)
        if count < 1:
            self.__process_started_handler = launch.EventHandler(
                matcher=lambda event: isinstance(event, launch.events.process.ProcessStarted),
                entities=[launch.actions.OpaqueFunction(function=self._on_container_available)],
            )
            context.register_event_handler(self.__process_started_handler)
        else:
            self._on_container_available(context=context)

        return None


def match_container_by_name(container_name: Text):
    if not container_name.startswith('/'):
        container_name = f'/{container_name}'
    return lambda action: action.node_name == container_name





        


