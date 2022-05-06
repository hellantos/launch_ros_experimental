from typing import List, Optional, cast
from .composable_node import ComposableNode
from launch.frontend import expose_action
from launch.frontend import Entity
from launch.frontend import Parser

import functools
import threading
import launch
from launch import Action, SomeSubstitutionsType
from launch import LaunchContext
from launch_ros.ros_adapters import get_ros_node
from launch_ros.events.lifecycle import ChangeState, StateTransition

import lifecycle_msgs



@expose_action("composable_lc_node")
class ComposableLifecycleNode(ComposableNode):
    def __init__(
            self, 
            *,
            name: SomeSubstitutionsType,
            namespace: SomeSubstitutionsType,
            **kwargs 
        ):
        super().__init__(name=name, namespace=namespace, **kwargs)

    def _on_transition_event(self, context, msg):
        try:
            event = StateTransition(action=self, msg=msg)
            self.__current_state = ChangeState.valid_states[msg.goal_state.id]
            context.asyncio_loop.call_soon_threadsafe(lambda: context.emit_event_sync(event))
        except Exception as exc:
            self.__logger.error(
                "Exception in handling of 'lifecycle.msg.TransitionEvent': {}".format(exc))

    def _call_change_state(self, request, context: launch.LaunchContext):
        while not self.__rclpy_change_state_client.wait_for_service(timeout_sec=1.0):
            if context.is_shutdown:
                self.__logger.warning(
                    "Abandoning wait for the '{}' service, due to shutdown.".format(
                        self.__rclpy_change_state_client.srv_name),
                )
                return

        # Asynchronously wait so that we can periodically check for shutdown.
        event = threading.Event()

        def unblock(future):
            nonlocal event
            event.set()

        response_future = self.__rclpy_change_state_client.call_async(request)
        response_future.add_done_callback(unblock)

        while not event.wait(1.0):
            if context.is_shutdown:
                self.__logger.warning(
                    "Abandoning wait for the '{}' service response, due to shutdown.".format(
                        self.__rclpy_change_state_client.srv_name),
                )
                response_future.cancel()
                return

        if response_future.exception() is not None:
            raise response_future.exception()
        response = response_future.result()

        if not response.success:
            self.__logger.error(
                "Failed to make transition '{}' for LifecycleNode '{}'".format(
                    ChangeState.valid_transitions[request.transition.id],
                    self.node_name,
                )
            )

    def _on_change_state_event(self, context: launch.LaunchContext) -> None:
        typed_event = cast(ChangeState, context.locals.event)
        if not typed_event.lifecycle_node_matcher(self):
            return None
        request = lifecycle_msgs.srv.ChangeState.Request()
        request.transition.id = typed_event.transition_id
        context.add_completion_future(
            context.asyncio_loop.run_in_executor(None, self._call_change_state, request, context))


    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Parse composable_lc_node."""
        _, kwargs = super().parse(entity, parser)        

        return cls, kwargs



    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        self._perform_substitutions(context)  # ensure self.node_name is expanded
        if '<node_name_unspecified>' in self.node_name:
            raise RuntimeError('node_name unexpectedly incomplete for lifecycle node')
        node = get_ros_node(context)
        # Create a subscription to monitor the state changes of the subprocess.
        self.__rclpy_subscription = node.create_subscription(
            lifecycle_msgs.msg.TransitionEvent,
            '{}/transition_event'.format(self.node_name),
            functools.partial(self._on_transition_event, context),
            10)
        # Create a service client to change state on demand.
        self.__rclpy_change_state_client = node.create_client(
            lifecycle_msgs.srv.ChangeState,
            '{}/change_state'.format(self.node_name))
        # Register an event handler to change states on a ChangeState lifecycle event.
        context.register_event_handler(launch.EventHandler(
            matcher=lambda event: isinstance(event, ChangeState),
            entities=[launch.actions.OpaqueFunction(function=self._on_change_state_event)],
        ))
        return super().execute(context)
    