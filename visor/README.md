# ViSor

Visual Stimuli processor (ViSor) is an image processing module focused on saliency-based object detection. This pkg optionally works with Kinect or a regular webcam, aside from the Baxter's built-in camera.

## Using an external camera as the main visual input device

Several arguments need to be adjusted when Baxter's camera/webcam/Kinect is used, in order to use the built-in camera. To do so, check out `camera_setting.xml` in `visor` pkg's launch folder, and change the arg `connected_to_baxter`/`use_webcam`/`use_kinect` to `true.` Make sure that only one of them are selected, when you decided to use one of them. Then, check the arg `kinect_img_in`, `usbcam_img_in` or `baxter_L/R/H_img_in` for `L/R/H` refers to the left hand/right hand/head camera.

## Using an imageloader as a camera alternative

Another option is to use an `imageloader` pkg in the `visor` pkg. 
This is a convenient option to try out ERIS without having to be connected to a real hardware. To do so, make sure that the three `args` above are set to `false`, provide yourself a dataset of scene, and place them in the format according to the arg `scene_label.scene_ext` inside the `scene_dir` directory.

The dataset can be downloaded [HERE](https://www.dropbox.com/s/1mpaxysue6ji8m9/ERIS_Dataset.tar.gz?dl=0 "ERIS_dataset")

## Instruction

1. Launch the ViSor core module.

    ```
    roslaunch visor core.launch
    ```

2. Run `scenectrl` to process the image w/o encoding the memories. This is useful to test the `visor` perception module without overwriting the existing memories.

    ```
    rosrun visor scenectrl
    ```

    In the `scenectrl` terminal, press `spacebar` to trigger manually the image processing captured by Baxter, or `c` to cycle the input scene images.

In this case, no memories will be generated and consolidated. To run `ViSor` with `ERIS`, please check out the README.md in `ERIS` pkg.

#### Future plans:

- object tracking
- multiple modules of object detection algorithm

## Dependencies:

- fftw3 for gist calculation.

    ```
    sudo apt-get install libfftw3 libfftw3-dev libfftw3-doc
    ```

- usb_cam stack for webcam.
- Cheese uses the gstreamer library, which utlilizes the video4linux2 API to capture video and stills from a webcam. It can also apply some special effects.

    ```
    sudo apt-get install cheese
    ```

- GUVCView is a graphical front-end for UVC drivers built using GTK+. It is way better than cheesefor controlling webcam and recording audio/video.

    ```
    sudo apt-get install guvcview
    ```
