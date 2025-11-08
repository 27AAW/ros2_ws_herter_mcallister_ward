1. Describe a robotics task where an Action is superior to a Service. Explain why.

    Actions can be better than services when they may take a significant amount of time to complete. This is because they can provide continuous feedback during their execution as well as the option of being preempted/cancelled if they run too long or another task has a higher priority. 

    For example, a robot trying to navigate to a particular waypoint may have the navigation formatted as an action. As the robot progresses toward the waypoint, the rest of the program can monitor it's position, and in the case of an emergency, sensor failure, or user override, the action can be stopped. 

2. What are the three main components of an Action (goal, result, feedback)? Describe the purpose of each.

    The three fundamendal pieces of an Action are the goal, feedback, and result. The goal is sent by the action client node to the action server node and contains the information about the task to complete. The action server provides feedback on the status of the task while it is executing. The result is sent back to the client node and contains the final data from the task as well as success/failure and any error messages. 

3. Why is the ability to preempt (cancel) a goal crucial in robotics?

    The ability to preempt or cancel a task is crucial for two main reasons. First, as a safety item, the ability to cancel whatever the robot is doing when receiving either an automated or manual abort command can stop the robot from damaging itself, its surroundings, or people nearby. Additionally, the robot needs to be responsive to task priority. When a new higher-priority becomes available, the robot needs to be able to stop its current action and transition to the new one. 