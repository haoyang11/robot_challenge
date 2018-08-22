# robot_challenge
this is a robot game code and we got the first place.
this game include perception,location and path planning

## perception 
 ### darknet
    1. we use keras version of yolo v3 but failed  maybe the loss function is wrong(https://github.com/qqwweee/keras-yolo3)
    2. we use the offical version of yolo and it worked(https://pjreddie.com/darknet/yolo/)
    3. some detail of training:
    (1) balance data may help train the model
    (2) batchsize influence the result
    (3) use kmeans and mask shows the anchor  model used(yolo tool: https://github.com/Jumabek/darknet_scripts)
    (4) some scenery may need data to train
    (5) use big picture
 ### tradtion method
    1. use HSV to detect ball
    2. use binary value to judge robot
    3. use hough transform to detect line

## location
    1. use projection to get the 2-dim  coordinate
    2. use width and height  and ratio and distance  and area to calculate  position


## control  and path planning 
    1. delay, use control program in pi
    2. stable is most import part in practice
    3. use service rather than topic to communication
    4. use master and slaver to transfer picture

