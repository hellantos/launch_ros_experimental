# Copyright 2022 Christoph Hellmann Santos
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
from typing import Callable

from launch.event import Event
from composition_interfaces.srv import LoadNode

class LoadNodeEvent(Event):

    def __init__(
        self,
        *,
        target_container_matcher: Callable[['ComposableNodeContainer'], bool],
        load_request: LoadNode.Request,
        action: 'ComposableNode'
        ) -> None:
        super().__init__()
        self.__target_container_matcher = target_container_matcher
        self.__load_request = load_request
        self.__action = action

    @property
    def target_container_matcher(self) -> Callable[['ComposableNodeContainer'], bool]:
        return self.__target_container_matcher

    @property
    def load_request(self) -> LoadNode.Request:
        return self.__load_request

    @property
    def action(self) -> 'ComposableNode':
        return self.__action