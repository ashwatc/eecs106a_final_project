# EECS 106A Final Project - Search and Rescue

Node: move arm in predetermined path for scan
Service: closest AR tag to cone
Service: turtlebot AR tag

Node: AR tags
Publish: list of AR tags in view with coordinates

Node: plan_next_target
Looks at all AR tags in view, calculates minimum manhattan distance to goal pose in World frame
Calculates transform between target AR tag and turtlebot in world frame
subscribe: AR tags, last stopped position of turtlebot, turtlebot boolean status
publish: intermediate target pose in turtlebot frame


Node: move baxter arm in orientation restricted path from start to goal
stop when sees new AR tag
subscribe: turtlebot boolean status, list of AR tags in view
Client to: move arm in predetermined path for scan

Node: turtlebot_move
subcribe: intermediate target pose in turtlebot frame
publish: boolean status (moving or stopped)

Node: turtlebot_seek
