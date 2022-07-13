# hello_world_example
A simple attempt to get all the individual nodes in the system working together. The camera will be position the bots and they will follow a simple behaviour based on their position. Eg change colour. (Potentially a useful simple example for future interns to understand the system?)

The ROS aspect:

Every time the camera detects a new tag it hasnt seen before, it will create a new topic. For example, if it detects tag with ID 10, it will create a new topic called '/robot_10_position'. These are stored in a dictionary with their ID (in integer form) as their keyword.

The node that is using these positions can subscribe to the '/robot_IDs' topic which will post the IDs that are currently active. The reciever program should check the IDs that are being used and then attempt to make subscribers to '/robot_X_position' and then create them if they do not already exist. Example shown in 'hello_world_central_program.py'. The code currently only publishes a /robot_IDs message when it detects a new tag, which may cause problems depending on when and in which order the scripts are run. This will be solved by getting the topic to publish all IDs on a regular basis.
