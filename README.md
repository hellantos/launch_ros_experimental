# Launch ROS experimental

This repository contains some experimental features for
use with launch_ros. They break the existing API in launch_ros therefore they are hosted here.

Composition with Lifecycle Nodes:
* **ComposableNodeContainer:**
  Listens to LoadNodeEvent, then loads the node via request specified in event.
* **ComposableNode now an Action:**
  It waits for the specified container to become available.
  Then it triggers loading its component by emitting an event to the ComposableNodeContainer that will then load the Node.
* **ComposableLifecyleNode inherits from ComposableNode:**
  It waits for the specified container to become available.
  Then it triggers loading its component by emitting an event to the ComposableNodeContainer that will then load the Node.
  It has the same Event interface as LifecycleNode.



You can use it like this:
```python

import launch
from launch_ros_experimental.actions import ComposableNode, ComposableNodeContainer, ComposableLifecycleNode
from launch_ros.actions import LifecycleTransition
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
    )

    node = ComposableNode(
        package='composition',
        component='composition::Talker',
        name='talker',
        target_container='my_container'
    )
    lifecycle_node = ComposableLifecycleNode(
        package='composition',
        component='composition::LifecycleTalker',
        name='lctalker',
        namespace='',
        target_container='my_container'  
    )

    lifecycle_node_1 = ComposableLifecycleNode(
        package='composition',
        component='composition::LifecycleTalker',
        name='lctalker_1',
        namespace='',
        target_container='my_container'  
    )

    lifecycle_transition = LifecycleTransition(
        lifecycle_node_names=['lctalker', 'lctalker_1'],
        transitions_ids=[
            Transition.TRANSITION_CONFIGURE,
            Transition.TRANSITION_ACTIVATE]
    )


    return launch.LaunchDescription([container, node, lifecycle_node, lifecycle_node_1, lifecycle_transition])

```

Usage in XML frontend:

```xml
<launch>
    <container_exp name="container" exec="component_container" pkg="rclcpp_components" namespace="" output="screen"/>
    <composable_node_exp name="talker" component="composition::Talker" pkg="composition" target_container="container"/>
    <composable_lc_node name="lc_talker" component="composition::LifecycleTalker" pkg="composition" target_container="container"/>
    <transition>
        <lifecycle_node name="lc_talker"/>
        <transition label="configure"/>
        <transition label="activate"/>
    </transition>
</launch>
```
For testing you can use https://github.com/ipa-cmh/demos/tree/lifecycle_components which has the composition LifecycleTalker component.
