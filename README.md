# Managing Dependencies with rosdep

## How do I use the rosdep tool?

Now that we have a fundamental grasp of rosdep, package.xml, and rosdistro, we are ready to use the program. The initialization of rosdep must be done first before using it for the first time using:
```
sudo rosdep init
rosdep update
```

![image](https://user-images.githubusercontent.com/92040822/194984951-6c15ca19-ef70-459a-bc1d-c3dcbed8acb9.png)


Finally, we may install dependencies using rosdep install. To install all dependencies, this is frequently run just once in a workspace with many packages. A call for that might appear as follows if the source-code-containing directory src was present in the workspace's root.

```
rosdep install --from-paths src -y --ignore-src
```
![image](https://user-images.githubusercontent.com/92040822/194985049-f97687a2-a070-432b-8d50-15b0e4f318d9.png)

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

![image](https://user-images.githubusercontent.com/92040822/194985781-d285bf6d-dc69-4898-b207-e77806cc20ab.png)

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

![image](https://user-images.githubusercontent.com/92040822/194997859-cf7b8875-7ee6-4fb1-ad5d-61ea9c9b3ec8.png)


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

![image](https://user-images.githubusercontent.com/92040822/194997429-6b430f6e-8bbd-4bb3-9f57-33c69ba07dec.png)

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

![image](https://user-images.githubusercontent.com/92040822/194998140-8b4ab337-de02-4b82-8eed-09df6c031cbf.png)


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

![image](https://user-images.githubusercontent.com/92040822/195005805-079536ff-680e-41f3-8b1f-52c91067fd85.png)

By using the command line interface, we can direct a goal to another terminal:

```
ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```

![image](https://user-images.githubusercontent.com/92040822/195005945-5bed8f77-710a-4cc2-b45e-e13fcd590e2c.png)

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

![image](https://user-images.githubusercontent.com/92040822/195006346-83e3bae1-0ce7-4693-97e1-ea8ce72be268.png)

![image](https://user-images.githubusercontent.com/92040822/195006367-2b341169-9964-4b03-b992-d3ee7165785a.png)

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

![image](https://user-images.githubusercontent.com/92040822/195006747-a1500020-d012-434b-9ad3-87ba9987323e.png)

![image](https://user-images.githubusercontent.com/92040822/195006803-99357e2b-2932-4885-8205-e26e372d6f1a.png)

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

![image](https://user-images.githubusercontent.com/92040822/195008024-7aed6b72-3e4a-4baa-ae36-de3bee6fdd54.png)


![image](https://user-images.githubusercontent.com/92040822/195007345-05b776e6-bec4-47ab-bd78-f714953600e0.png)

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

![image](https://user-images.githubusercontent.com/92040822/195008493-54dd71ba-08eb-4e15-a8fd-607641d65326.png)

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

![image](https://user-images.githubusercontent.com/92040822/195009217-a65fdda3-72f8-46c5-b863-a95166ea9dce.png)

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

![image](https://user-images.githubusercontent.com/92040822/195010081-8356976d-ace7-4f85-acc7-2d20318cd105.png)










