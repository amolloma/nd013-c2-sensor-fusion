## Midterm Writeup

### Following are the examples of vehicles as viewed in the plotted 3d point cloud

#### Example 1
![](/img/lidar_1.png) ![](/img/Intensity_img_1.png)

The above image shows three cars with varying level of details. The associated intensity image corroborates the lidar.
Vehicle features identified include:
    - Front and Rear tires
    - Front left mirror 
    - Rear windshield bumper

#### Example 2:
![](/img/lidar_2.png)

Identifiable vehicle feature in the above lidar :
    - Front windshield
    - Front and Rear right tires
    - Passenger door/windows

#### Example 3:
![](/img/lidar_3.png) ![](/img/Intensity_img_3.png)

Identifiable vehicle feature in the above lidar and associated intensity image :
    - Front and Rear right tires
    - Passenger door/windows
    - Side mirrors

#### Example 4:
![](/img/lidar_4.png)

Identifiable vehicle feature in the above lidar :
    - Front windshields of multiple cars
    - Front and Rear right tires
    - Truck rear, trailer, tailgate

#### Example 5:
![](/img/Lidar_5.png)

Identifiable vehicle feature in the above lidar :
    - Front windshields, 
    - Right and left mirrors
    - Tires, rear windshield

#### Example 6:
![](/img/lidar_6.png)

Identifiable vehicle feature in the above lidar :
    - Truck Front Bumper, Truck Roof
    - Front and Rear right tires
    - Truck rear, trailer

#### Example 7:
![](/img/lidar_7.png)

Identifiable vehicle feature in the above lidar :
    - Tail light and number plate of left most car
    - Front and Rear right tires
    - Car Roof , Rear windows, Side windows

#### Example 8:
![](/img/lidar_8.png)

Identifiable vehicle feature in the above lidar :
    - Truck Front Bumper, Truck Roof
    - Ladder on the rear of the Truck, A small trailer
    - Front and Rear right tires
    - Front and Side Windows

#### Example 9:
![](/img/lidar_9.png)

Identifiable vehicle feature in the above lidar :
    - Jeep Side view, with side doors & windows
    - Front and Rear right tires
    - Rear bumper and taillights

#### Example 10:
![](/img/lidar_10.png)

Identifiable vehicle feature in the above lidar :
    - Truck Front Bumper, Truck Roof
    - Truck Front lights
    - Car roof, front lights, windshield

### Following are the Precision and Recall Plots

#### Using Sequence 1, Frames 50 to 150
![](/img/precision_recall.png)

#### Ideal Precision/Recall using Groundtruths as labels
![](/img/Ideal_precision_recall.png)

### Model Based Object Detection in BEV
Set the following parameters in the loop_over_dataset before running
    - data_filename = # Sequence 1
    - line 61 : model = 'fpn_resnet'
    - line 62 : sequence = 1
    - configs_det = det.load_configs(model_name='fpn_resnet')
    - exec_detection = ['bev_from_pcl', 'detect_objects']
    - exec_visualization = ['show_objects_in_bev_labels_in_camera']

Following results are for frame 50, 51, and 150:

#### Frame 50 
![](/img/camera_detect_1.png)
![](/img/bev_detect_1.png)

#### Frame 51
![](/img/camera_detect_2.png)
![](/img/bev_detect_2.png)

#### Frame 150 
![](/img/camera_detect_3.png)
![](/img/bev_detect_3.png)

Following screen shots of successful terminal outputs
![](/img/frame_150_run.png)

![](/img/frame_50_run.png)