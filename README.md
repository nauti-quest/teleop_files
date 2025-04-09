Pinger Files

This folder contains the pinger files for the Blue Robotics 1D pinger. 
The main python file can be found under /pinger_files/src
The python file is executed using the following command in the rasp pi

```bash
python3 pinger_data_read.py 
```
This should print the depth estimate and the confidence level in the terminal.

There is also a possibility to then take the data and publish it directly into a rostopic as required.

Usb_cam_files

This folder contains the launch files to publish images using a usb webcam/laptop camera

```bash
/usb_cam_files/config
```
has the following camera.yaml file which has the following configuration params

| Parameter                | Description                                | Default       |
|--------------------------|--------------------------------------------|---------------|
| `video_device`           | Device path                                | `/dev/video2` |
| `image_width` / `height`| Output resolution                          | `640x480`     |
| `pixel_format`           | YUV/RAW format supported by camera         | `yuyv`        |
| `framerate`              | Polling frequency in Hz                    | `30`          |
| `camera_name`            | ROS internal name                          | `head_camera` |
| `camera_frame_id`        | TF frame ID used in ROS                    | `head_camera` |
| `color_format`           | Format for encoding stream                 | `yuv422p`     |

Launch File

To launch the usb cam node

```bash
cd ~/usb_cam_files
roslaunch usb_cam_files usb_cam.launch
```
