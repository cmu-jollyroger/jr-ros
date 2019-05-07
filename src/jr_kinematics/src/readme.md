
# Service calls and default tests

Note the x,y,z are in meters and all angles are in degrees
## For arm points downwards: 

```
rosservice call /jr_arm_execute_cmd "target_pose:
position: {x: 0.2, y: 0.0, z: 0.5}
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
z_offset: 0.0
device_type: 0
device_orient: 0
intial_rot: 0" 
```


## For arm point straight to device 

```
rosservice call /jr_arm_execute_cmd "target_pose:
position: {x: 0.4, y: 0.0, z: 0.5}
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
z_offset: 0.0
device_type: 0
device_orient: 1
intial_rot: 0" 
```

## For xyz correction 

#### Move end eff up 10 cm i.e. z offset 
```
rosservice call /jr_arm_correct_cmd "delta_pose:
  position:                         
    x: 0.0                                     
    y: 0.0
    z: 0.10
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0" 

```


#### Move end eff forward 10 cm i.e. x offset 
```
rosservice call /jr_arm_correct_cmd "delta_pose:
  position:                         
    x: 0.10                                     
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0" 

```

#### Move robot left by 5 degrees from curent pos
```
rosservice call /jr_arm_correct_cmd "delta_pose:
  position:                         
    x: 0.0                                     
    y: -5.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0" 

```

Note it is delta pose, so x and z are the relative change in distance. 

Also note that y is the the degrees of rotation around current joint angle of the base to move the robot left or right. 

## Rotate Jammer by angle 
#### In this example rotation angle is 90 degrees
```
rosservice call /jr_hand_execute_cmd "rotation: 90
delta_z: 0.0" 
```
