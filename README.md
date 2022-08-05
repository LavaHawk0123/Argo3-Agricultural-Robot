# Argo3-Agricultural-Robot
Argo3 is a conceptual robot which is shaped like a crate with 4 wheels and a robotic manipulator attached capable of autonomously traversing a Greenhouse and picking tomatoes off plants

### To launch the simulation : 
```
git pull https://github.com/LavaHawk0123/Argo3-Agricultural-Robot.git
catkin_make
source devel/setup.bash
roslaunch ebot_gazebo ebot_trolley_ur5.launch
```

### For Traversal
```
source devel/setup.bash
rosrun ebot_nav att.py
```

### Traversal : 

![Argo3_Trav](https://user-images.githubusercontent.com/75236655/183002222-af88384d-d076-4d75-b95e-0ffe551459e8.gif)
