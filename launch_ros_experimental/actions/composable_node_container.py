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

import launch
import threading
from launch_ros.actions import Node
from typing import Optional, List, cast
from launch import SomeSubstitutionsType
from launch import LaunchContext
from launch.action import Action
from launch_ros.utilities import add_node_name
from launch_ros.utilities import get_node_name_count

from launch_ros.ros_adapters import get_ros_node
import composition_interfaces
# from launch_ros_experimental.actions.composable_node import ComposableNode
from launch_ros_experimental.events.load_node_event import LoadNodeEvent 
from launch.frontend import expose_action
from launch.frontend import Entity, Parser
@expose_action("container_exp")
class ComposableNodeContainer(Node):
    
    def __init__(
            self, 
            *, 
            name: Optional[SomeSubstitutionsType] = None, 
            namespace: Optional[SomeSubstitutionsType] = None, 
            **kwargs) -> None:
        super().__init__(name=name, namespace=namespace, **kwargs)
        self.__rclpy_load_node_client = None
        self.__logger = launch.logging.get_logger(__name__)

    def _on_load_node(self, context):
        self.__logger.info("Event caught.")
        if isinstance(context.locals.event, LoadNodeEvent): 
            typed_event = cast(LoadNodeEvent, context.locals.event)
            if not typed_event.target_container_matcher(self):
                self.__logger.info("Event does not match.")
                return
            context.add_completion_future(
                context.asyncio_loop.run_in_executor(None, self._load_node, typed_event.load_request, typed_event.action, context))

    def _load_node(
        self,
        request: composition_interfaces.srv.LoadNode.Request,
        action,
        context: LaunchContext
    ) -> None:
        """
        Load node synchronously.

        :param request: service request to load a node
        :param context: current launch context
        """
        while not self.__rclpy_load_node_client.wait_for_service(timeout_sec=1.0):
            if context.is_shutdown:
                self.__logger.warning(
                    "Abandoning wait for the '{}' service, due to shutdown.".format(
                        self.__rclpy_load_node_client.srv_name
                    )
                )
                return

        # Asynchronously wait on service call so that we can periodically check for shutdown
        event = threading.Event()

        def unblock(future):
            nonlocal event
            event.set()

        self.__logger.debug(
            "Calling the '{}' service with request:" 
            "\nnode_name: {} plugin_name:{} package_name: {}".format(
                self.__rclpy_load_node_client.srv_name, 
                request.node_name,
                request.plugin_name,
                request.package_name,
            )
        )

        response_future = self.__rclpy_load_node_client.call_async(request)
        response_future.add_done_callback(unblock)

        while not event.wait(1.0):
            if context.is_shutdown:
                self.__logger.warning(
                    "Abandoning wait for the '{}' service response, due to shutdown.".format(
                        self.__rclpy_load_node_client.srv_name),
                )
                response_future.cancel()
                return

        # Get response
        if response_future.exception() is not None:
            raise response_future.exception()
        response = response_future.result()

        self.__logger.debug("Received response '{}'".format(response))

        node_name = response.full_node_name if response.full_node_name else request.node_name
        if response.success:
            if node_name is not None:
                add_node_name(context, node_name)
                node_name_count = get_node_name_count(context, node_name)
                if node_name_count > 1:
                    container_logger = launch.logging.get_logger(
                        self.node_name
                    )
                    container_logger.warning(
                        'there are now at least {} nodes with the name {} created within this '
                        'launch context'.format(node_name_count, node_name)
                    )
            self.__logger.info("Loaded node '{}' in container '{}'".format(
                response.full_node_name, self.node_name
            ))
            action.loaded = True
        else:
            self.__logger.error(
                "Failed to load node '{}' of type '{}' in container '{}': {}".format(
                    node_name, request.plugin_name, self.node_name,
                    response.error_message
                )
            )

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Parse node_container."""
        _, kwargs = super().parse(entity, parser)
        return cls, kwargs

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """
        Execute the action.

        Most work is delegated to :meth:`launch_ros.actions.Node.execute`, except for the
        composable nodes load action if it applies.
        """
        super().execute(context)

        self.__rclpy_load_node_client = get_ros_node(context).create_client(
            composition_interfaces.srv.LoadNode, 
            '{}/_container/load_node'.format(
                self.node_name
            )
        )
        context.register_event_handler(launch.EventHandler(
            matcher=lambda event: isinstance(event, LoadNodeEvent),
            entities=[launch.actions.OpaqueFunction(function=self._on_load_node)],
        ))
        return None