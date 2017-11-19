## Source code of Team T2 robot control program of ARC 2017

Publish the source code of the Team T2 robot who participated in ARC (Amazon Robotics Challenge) 2017 to the public. This source code is BSD Style License (open source format).

Folder structure and processing outline of the source code are as follows.

### Folder structure

+ #### t2_robot_vision

 1.  Perform capture and image correction using the camera (Intel RealSense Camera SR 300) and execute recognition processing.
The recognition processing is realized by the following processing.

 2. Segment of image 

 3. Various recognition algorithms,
   - LineMod, YOLO, distance learning, plane detection and the like.

 4. Integration / compensation of results (ICP)

 5. Coordinate conversion (output)
   - It uses open source libraries such as openCV and PCL.

* #### t2_ui

 Output messages from t2_task_planner and enter commands.

* #### t2_task_planner

 Execution management and overall control of the work plan of Pick task and Stow task are performed, and calculate of gripping points and release points by gripping / box packing plan.
 
 In the grasping / box packing plan, coordinate conversion by the Eigen library (open source)

 We are planning box packing using Octomap (open source) data.

 The Jsoncpp library (open source) is used for outputting item information.

* #### t2_motion_planner

 It has trajectory generation and trajectory execution management by the motion plan of the robot arm, and it has execution  management of robot hand, measurement management function of weight scale.

 The planning interface of MoveIt! (open source) is used for motion planning.

 VisualizationMarker of RViz (open source) is used for visualization of generation trajectory.

* #### t2_planning_scene_updater

 It has the function to update the VR space used for the operation plan.

 Manage reflection of item recognition results, attach / detach of items and interference check subjects.

 PlanningScene interface of MoveIt! (open source) is used for updating the VR space.
