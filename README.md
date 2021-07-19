## Very lazy mapping package

The package gets the poses (`myicub_ros/Markers`) from the topic `/markers`, and addes the obstacles to moveIt. Obstacles dimentions sets from `config/setup.yaml`.

### Installation

Install dependancies and than do

```
git clone https://github.com/be2rlab/lazy_mapping_moveit.git
```

```
catkin build
```


### How to use

Launch the moveIt and add MotionPlanning instance. For exampla, we use *ur5*

```
roslaunch ur5e_moveit_config demo.launch
```

Run

```
roslaunch lazy_mapping_moveit obstacles_to_moveit.launch
```

Publish the message

```
rostopic pub -r 1 /markers myicub_ros/Markers "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
markers:
- header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  id: 0.0
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 1.0
      y: 0.0
      z: 0.0
      w: 1.0" 
```

where

- `id` is identifacator of a obstacle (for examle, an AR-code ID)

- `pose` is the obstacle pose

### About `config/setup.yaml`

It includes the list `obstacles`, where each element is list too. The list has the following view `[id, x_dim, y_dim, z_dim]`.