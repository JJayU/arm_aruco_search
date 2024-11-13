# Aruco searching project

### Launching:

```
ros2 launch arm05_sim world.launch.py
```

```
ros2 run search_aruco_action search_aruco_action_server.py
```

```
ros2 action send_goal --feedback /SearchAruco search_aruco_action/action/SearchArUco "{timeout: 100.0}"
```
