# Speed and Separation Monitor based on fixed areas 

The module needs the position of the human in the cell. The robot slows down if the human enters in predefined zones.
The module subscribes:  

- /poses_ a message of the type geometry_msgs::PoseArray containing the poses of human body parts (i.e. a human) in the scene reference frame. 

The module publishes: 

- /safe_ovr_1: a message of the type std_msgs::Int64 containing the safety override 0-100 (the topics are two, with the same content, to have a redundant communication channel) 


```yaml
ssm:
  override_increase_speed: 10 # [%/s]   # override changing rate when the human is outside the areas
  override_decrease_speed: 100 # [%/s]  # override changing rate when the human enters in the areas
  areas:  # list of areas
  - name: "area0"  # name of the area
    override: 50   # maximum override
    corners:       # area corners (X-Y) in world frame
    - [0,0]
    - [0,  1.9]
    - [1.8,1.9]
    - [1.8,0]
  - name: "area1"
    override: 30
    corners:
    - [0,0]
    - [0,1.5]
    - [1.8,1.5]
    - [1.8,0]
  - name: "stop_area"
    override: 0
    corners:
    - [0,0]
    - [0,1.0]
    - [1.8,1.0]
    - [1.8,0]
```