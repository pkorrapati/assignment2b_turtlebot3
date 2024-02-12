# Results

## Circle, Slow [w=0.1]
![Image, Circular Trajectory, Slow Speed,   w=0.1](/images/Circle_Speed_Slow.png)
![Video, Circular Trajectory, Slow Speed,   w=0.1](/videos/Circle_Speed_Slow.mp4)

## Circle, Normal [w=0.3]
![Image, Circular Trajectory, Normal Speed, w=0.3](/images/Circle_Speed_Normal.png)
![Video, Circular Trajectory, Normal Speed, w=0.3](/videos/Circle_Speed_Normal.mp4)

## Circle, Fast [w=0.5]
![Image, Circular Trajectory, Fast Speed,   w=0.5](/images/Circle_Speed_Fast.png)
![Video, Circular Trajectory, Fast Speed,   w=0.5](/videos/Circle_Speed_Fast.mp4)

## Square, Slow [w=0.1]
![Image, Square Trajectory, Slow Speed,   w=0.1](/images/Square_Speed_Slow.png)
![Video, Square Trajectory, Slow Speed,   w=0.1](/videos/Square_Speed_Slow.mp4)

## Square, Normal [w=0.1]
![Image, Square Trajectory, Normal Speed, w=0.3](/images/Square_Speed_Normal.png)
![Video, Square Trajectory, Normal Speed, w=0.3](/videos/Square_Speed_Normal.mp4)

## Square, Fast [w=0.1]
![Image, Square Trajectory, Fast Speed,   w=0.5](/images/Square_Speed_Fast.png)
![Video, Square Trajectory, Fast Speed,   w=0.5](/videos/Square_Speed_Fast.mp4)

# Prerequisites
Depends on matplotlib python library

```console
sudo apt install python3-pip
python3 -m pip install matplotlib
```
# Launching files
## Circular Trajectory
Launch circular trajectory

```console
roslaunch assignment2b_turtlebot3 move.launch code:='circle'
```

Launch circular trajectory (Slow)

```console
roslaunch assignment2b_turtlebot3 move.launch code:='circle' w:=0.1
```

Launch circular trajectory (Fast)

```console
roslaunch assignment2b_turtlebot3 move.launch code:='circle' w:=0.5
```
## Square Trajectory
Launch square trajectory

```console
roslaunch assignment2b_turtlebot3 move.launch code:='square'
```

Launch square trajectory (Slow)

```console
roslaunch assignment2b_turtlebot3 move.launch code:='square' w:=0.1 v:=0.1
```

Launch square trajectory (Fast)

```console
roslaunch assignment2b_turtlebot3 move.launch code:='square' w:=0.5 v:=0.5
```