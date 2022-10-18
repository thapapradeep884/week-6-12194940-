# Managing Dependencies with rosdep

## How do I use the rosdep tool?

Now that we have a fundamental grasp of rosdep, package.xml, and rosdistro, we are ready to use the program. The initialization of rosdep must be done first before using it for the first time using:
```
sudo rosdep init
rosdep update
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p1.PNG)


Finally, we may install dependencies using rosdep install. To install all dependencies, this is frequently run just once in a workspace with many packages. A call for that might appear as follows if the source-code-containing directory src was present in the workspace's root.

```
rosdep install --from-paths src -y --ignore-src
```
![image](https://github.com/thapapradeep884/IMAGE/blob/main/p2.PNG)

# Creating an action

## Prerequisites

Installing Colcon and ROS 2 is necessary.

Create an area and a collection of files called "action tutorials interfaces":

(Remember to source first from your ROS 2 installation.)

```
mkdir -p ros2_ws/src #you can reuse existing workspace with this naming convention
cd ros2_ws/src
ros2 pkg create action_tutorials_interfaces
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p3.PNG)

## Tasks
## 1. Defining an action

The following format is used in.action files to describe actions:

```
# Request
---
# Result
---
# Feedback
```
In our ROS 2 package, create a directory called "action" and name it "action tutorials interfaces":

```
cd action_tutorials_interfaces
mkdir action
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p4.PNG)


In the action directory, create a file with the name Fibonacci. action that contains the information below:

```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

## 2. Building an action

Before we can utilize the new Fibonacci action type in our code, the definition must be delivered to the pipeline that creates Rosidl code.

To accomplish this, the lines listed below must be added to our CMakeLists.txt file before the ament package() line in the action tutorials interfaces:

```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```

Additionally, we must add the required dependencies to our package.xml file:

```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

We must rely on action messages since action definitions include additional metadata (e.g. goal IDs).

It should now be possible to build the package containing the definition of the Fibonacci action:

```
# Change to the root of the workspace
cd ~/ros2_ws
# Build
colcon build
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p5.PNG)

We are done!

Action types frequently begin with the word "action" and the name of the package. Therefore, we will refer to our new action by its full name: action tutorials interfaces/action/Fibonacci.

We can check to see if our action was built successfully using the command line tool:

```
# Source our workspace
# On Windows: call install/setup.bat
. install/setup.bash
# Check that our action definition exists
ros2 interface show action_tutorials_interfaces/action/Fibonacci
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p6.PNG)


The panel ought to display a definition of the Fibonacci operation.

# Writing an action server and client

## Prerequisites

Both the action tutorials interfaces package and the Fibonacci.action interface from the earlier tutorial, "Creating an action," are needed.

## Tasks

## 1. Writing an action server

You should make a new file called fibonacci action server.py in your home directory and add the following code to it:

```
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Fibonacci.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
```

Let's attempt to run our action server:

```
python3 fibonacci_action_server.py
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p7.PNG)

By using the command line interface, we can direct a goal to another terminal:

```
ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p8.PNG)

In the terminal that is running the action server, you should see the logged message "Executing objective..." followed by a warning that the goal state was not established. If the goal handle state is not set in the execute callback, the aborted state is assumed by default.

To demonstrate that the objective was accomplished, use the successfully() function on the goal handle:

```
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        goal_handle.succeed()

        result = Fibonacci.Result()
        return result
```

If you restart the action server at this point and transmit another goal, you ought to see the goal accomplished with the status SUCCEED.

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p9.PNG)

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p10.PNG)

Let's check that the provided Fibonacci sequence is computed and returned by our target execution:

```
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')


        sequence = [0, 1]



        for i in range(1, goal_handle.request.order):

            sequence.append(sequence[i] + sequence[i-1])


        goal_handle.succeed()

        result = Fibonacci.Result()

        result.sequence = sequence

        return result
```

The sequence is calculated, put in the result message field, and we move on to the return.

Restart the action server and send a new goal. The goal should be achieved with the anticipated outcomes coming in sequential sequence.

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p11.PNG)

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p12.PNG)

## 1.2 Publishing feedback

The sequence will now be saved in a feedback message instead of the previous sequence variable. After each revision of the feedback message in the for-loop for impact, we post it and then go to sleep:

```
import time


import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')


        feedback_msg = Fibonacci.Feedback()

        feedback_msg.partial_sequence = [0, 1]


        for i in range(1, goal_handle.request.order):

            feedback_msg.partial_sequence.append(

                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])

            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))

            goal_handle.publish_feedback(feedback_msg)

            time.sleep(1)


        goal_handle.succeed()

        result = Fibonacci.Result()

        result.sequence = feedback_msg.partial_sequence

        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
```

After restarting the action server, we can confirm that feedback has been published by using the command line tool with the —feedback option:

```
ros2 action send_goal --feedback fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p13.PNG)


![image](https://github.com/thapapradeep884/IMAGE/blob/main/p14.PNG)

## 2. Writing an action client
The action client will also be limited to a single file. Next, create a new file with the name fibonacci action client.py. To the new file, add the following boilerplate code:

```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    future = action_client.send_goal(10)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
```

Launching the action server we previously developed will allow us to test our action client:

```
python3 fibonacci_action_server.py
```

Run the action client in an other terminal.

```
python3 fibonacci_action_client.py
```

The following messages must be printed as the action server completes the task:

```
[INFO] [fibonacci_action_server]: Executing goal...
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5])
# etc.
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p15.PNG)

The client's activity should start and finish as soon as possible. Although we now have a functioning action client, we get no feedback or outcomes.

## 2.1 Getting a result

For the goal we submitted, we must first get a goal handle. The goal handle can then be used to request the outcome.

Here is the complete code for this example:

```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

Go ahead and attempt to run our Fibonacci action client while an action server is running in a separate terminal!

```
python3 fibonacci_action_client.py
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p16.PNG)

The goal's acceptance and the result should be shown in the logs.

## 2.2 Getting feedback

We can send our action client goals. Nice! However, it would be great to have some feedback on the objectives we provide to the action server.

Here is the complete code for this example:

```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

For us, everything is prepared. If we run our action client, feedback should appear on your screen.

```
python3 fibonacci_action_client.py
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p17.PNG)


### 4.Composing multiple nodes in a single process

**4.1. To discover avilable components**

We issue the following commands to check the workspace's accessible components.

```
ros2 component types
```

The terminal returns all of the components that are available:

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p18.PNG)

**4.2. Run-time composition using ROS services with a publisher and subscriber**

In the first terminal, we launch the component container:

```
ros2 run rclcpp_components component_container
```

We perform the following command in the second terminal, and it will output the name of the component, allowing us to confirm that the container is running using the 'ros2' command line tools.

```
ros2 component list
```
Following this, we load the talker component on the second terminal:

```
ros2 component load /ComponentManager composition composition::Talker
```

This command will return the node name and the distinctive ID of the loaded component:

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p19.PNG)

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p20.PNG)
![image](https://github.com/thapapradeep884/IMAGE/blob/main/p21.PNG)

The listener component is then loaded by running the code below on the second terminal:

```
ros2 component load /ComponentManager composition composition::Listener
```
The results are displayed in the pictures up top.

Finally, we may launch the 'ros2' command-line tool to check the container's state:

```
ros2 component list
```

We can see the result as follows:

```
/ComponentManager
   1  /talker
   2  /listener
```


**4.3. Run-time composition using ROS services with a server and client**

It is very similar steps to what we did using talker and listener. 

In the first terminal, we run:

```
ros2 run rclcpp_components component_container
```

and then we run the following commands in the second terminal to view the **server** and **client** source code:

```
ros2 component load /ComponentManager composition composition::Server
ros2 component load /ComponentManager composition composition::Client
```
The output of the aforementioned commands may be seen in the images below:

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p22.PNG)

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p23.PNG)

**4.4. Compile-time composition using ROS services**

This example demonstrates how the same shared libraries may be used to create a single executable that runs a number of different components.

All four of the aforementioned elements—the "talker," "listener," "server," and "client"—are included in the executable.

In one terminal, 

```
ros2 run composition manual_composition
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p24.PNG)


**4.5. Run-time composition using dlopen**

By constructing a generic container process and explicitly passing the libraries to load without using ROS interfaces, this demonstration provides an alternative to run-time composition. Each library will be opened by the procedure, and one instance of each "rclcpp::Node" class will be created in the library's source code.


```
ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p25.PNG)


**4.6. Composition using launch actions**

Although the command line tools are useful for troubleshooting and diagnosing component configurations, it is frequently more practical to start a number of components at once. The 'ros2' launch's functionality can be used to automate this procedure.

```
ros2 launch composition composition_demo.launch.py
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p26.PNG)


### 5.Creating a launch file

The 'rqt graph' and 'turtlesim' packages that we previously loaded are used to construct a launch file.

**5.1. Setup**

To store the launch files, we must make a new directory:

```
mkdir launch
```

**5.2. Writing the launch file**


In the newly created directory, we created a new file named `turtlesim_mimic_launch.py` and paste the following codes into the file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```

**5.3. ros2 launch**

We enter the previously generated directory and issue the following commands to launch the launch file:

```
cd launch
ros2 launch turtlesim_mimic_launch.py
```

The output is shown in the image below:

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p27.PNG)

The following messages are displayed in two open turtlesim windows:

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p28.PNG)

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p29.PNG)

_In order to see the sytem in action, we open a new terminal and run the `ros2 topic pub` command on `/turtlesim1/turtle1/cmd_vel` topic to get the first turtle moving._
```ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}" ```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p30.PNG)

**Both the turtle move in the same path.**


**5.4. Introspect the system with rqt_graph**

Without shutting down the machine, open a new terminal and run "rqt graph."

```
rqt_graph
```

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p31.PNG)


### 6.Integrating launch files into ROS2 packages

**6.1. Creating a package**

In the beginning, we make a workspace for the package:

```
mkdir -p launch_ws/src
cd launch_ws/src
```

and create a python package:

```
ros2 pkg create py_launch_example --build-type ament_python
```


**6.2. Creating the structure to hold launch files **

When searching for python packages after creating them, do the following.:

![image](https://github.com/thapapradeep884/IMAGE/blob/main/p32.PNG)

And we need to tell Python's setup tools about our launch files using the 'data files' argument of'setup' in order to colcon to launch files.

We enter the following codes within the "setup.py" file:

```python
import os
from glob import glob
from setuptools import setup

package_name = 'py_launch_example'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
```

**6.3. Writing the launch file**

We create a brand-new launch file called "my script launch.py" inside the "launch" directory.
The 'generate launch description()' function, which provides a 'launch.LaunchDescription()' to be utilized by the 'ros2 launch' verb, should be defined in the launch file at this point.

```python
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker'),
  ])
```

**6.4. Building and running the launch file**

We navigate to the workspace's top level and create the file using:

```
colcon build
```
If the build goes well, we should be able to launch the program as follows:

```
ros2 launch py_launch_example my_script_launch.py
```

### 7.Using Substitutions

**7.1. Creating and Setting up the package**

We create a new package of build_type `ament_python` named `launch_tutorial` :

```
ros2 pkg create launch_tutorial --build-type ament_python
```
Now we build a directory called "launch" inside of that package.

```
mkdir launch_tutorial/launch
```

The launch file is then successfully installed by editing the "setup.py" file and adding the necessary adjustments.

```python
import os
from glob import glob
from setuptools import setup

package_name = 'launch_tutorial'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
```

**7.2. Parent Launch File**
Following the aforementioned procedures, we produced a launch file with the following codes in it called "example main.launch.py" in the launch folder of the "launch tutorial" directory:

```python
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_tutorial'),
                    'example_substitutions.launch.py'
                ])
            ]),
            launch_arguments={
                'turtlesim_ns': 'turtlesim2',
                'use_provided_red': 'True',
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
    ])
```

**7.3. Substitutions example launch file**

The same folder receives a new file called "example substitutions.launch.py"

```python
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        spawn_turtle,
        change_background_r,
        TimerAction(
            period=2.0,
            actions=[change_background_r_conditioned],
        )
    ])
```

**7.4. Building the package**

We run the build command in the root of the workspace. 

```
colcon build
```

**7.5. Launching Example**

Now we can use the ros2 launch command to run the example main.launch.py file.

```
ros2 launch launch_tutorial example_main.launch.py
```

![image](https://user-images.githubusercontent.com/113494159/196023691-55605922-f633-4f1a-b057-47543e52db44.png)
A turtlesim node is launched against a blue backdrop. The backdrops then shift between purple and pink as a second turtle hatches.

### 8.Using Event Handlers

**8.1. Event handler example launch file**
We made a new file called "example event handlers.launch.py" in the same directory, which is inside the "launch" folder of the "launch tutorial" package.

```python
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=turtlesim_node,
                on_start=[
                    LogInfo(msg='Turtlesim started, spawning turtle'),
                    spawn_turtle
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessIO(
                target_action=spawn_turtle,
                on_stdout=lambda event: LogInfo(
                    msg='Spawn request says "{}"'.format(
                        event.text.decode().strip())
                )
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=spawn_turtle,
                on_completion=[
                    LogInfo(msg='Spawn finished'),
                    change_background_r,
                    TimerAction(
                        period=2.0,
                        actions=[change_background_r_conditioned],
                    )
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=turtlesim_node,
                on_exit=[
                    LogInfo(msg=(EnvironmentVariable(name='USER'),
                            ' closed the turtlesim window')),
                    EmitEvent(event=Shutdown(
                        reason='Window closed'))
                ]
            )
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')]
                )]
            )
        ),
    ])
```

**8.2. Building and Running the CommandWe return to the workspace's root after adding the file and execute the build command there.**

We return to the workspace's root after adding the file and execute the build command there.
```
colcon build
```
After building, it is important to source the package and run the following codes for the output:


ros2 launch launch_tutorial example_event_handlers.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200


![image](https://user-images.githubusercontent.com/113494159/196024114-0e8025ee-186e-4b6c-bc8a-5f518d23b78d.png)

It spawns the second turtle and starts a turtlesim node with a blue backdrop. The background color then changes to pink and then purple.
When the turtlesim window closes, the launch file also shuts down. 










