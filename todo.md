
To learn:

- Touch Designer for visual in middle screen?
- What format do we need to be creating video in for feeding to the projectors? (e.g. resolution, frame rate)

To make:

- Connection to an LLM (a local one ideally) for a 'voice' for the robot. Can use OpenAI's API for testing. 
- Map out system flow (state machine type of thing).
- Layout of recent images from RPis for passing through to projectors -> ensure resolution looks okay.
- Clock node or something for syncing robot motions with the visuals on the middle screen.

To test:

- Test gRPC method for sending images from Rpi to control computer... use ROS instead ??
- Test iRC simulation software and if we can connect to it via the code written so far.

To decide:

- Image segmentation on main computer or on Raspberry Pis? Code at the moment assumes that images are sent from Raspberry Pis to the main computer and then processed there. (Does Mediapipe require an internet connection?)
- Method for start up of the whole system - bash script? One that re-starts itself on failure.

Things to check once we have the physical robot:

- Connection reliability: Is it okay to connect to the robot once at the beginning and then run for the whole night, or should the program disconnect/reconnect periodically? (Code is currently formatted to disconnect and reconnect everytime a new command string is sent to the robot)
- Do we need a comp running Windows to operate it? Is Linux ok?
