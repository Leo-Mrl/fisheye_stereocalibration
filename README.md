# Stereo-calibration and epipolar geometry of fisheye cameras

![alt text](https://i.imgur.com/1dyE7hL.png)


## 1. Some background

In order to make the best use of this repo, basic knowledge of computer vision is expected, as well as being familiar with [camera calibration](https://www.youtube.com/watch?v=-9He7Nu3u8s) and [epipolar geometry](https://www.youtube.com/watch?v=DgGV3l82NTk).

This work was initially realized as part of my master's thesis, while interning at the now defunct computer vision startup [Orah](https://www.crunchbase.com/organization/videostitch). If you are looking for a thorough explanation about epipolar geometry, and why the need for a fisheye lens epipolar model, you can download my manuscript [here](https://drive.google.com/file/d/1-5c6zPi8DRgZte_1ch5Ky9jknYPYzOsw/view?usp=sharing).

In a nutshell : standard camera calibration uses a rectilinear camera model (i.e. projecting the 3D image onto a rectangle). This models yields great results in most use cases, but performs poorly when used on wide-angle fisheye cameras (especially when FOV > 180), because of the circular shape of the lens being too extreme for the rectilinear model to accurately describe it (even when making use of advanced mathematical models of image distortion in addition to the camera model). Therefore, we chose to perform the camera calibration using a spherical lens model, and built an interactive demonstration of its epipolar geometry (then yielding epipolar curves instead of epipolar lines).


## 2. Mac users : you need XQuartz first

If you are a Mac user, you can still run this code, but will need to install [XQuartz](https://www.xquartz.org/) beforehand. After that, launch XQuartz from the application panel, go to `Preferences` -> `Security` -> `Allow connections from network clients`. This is required to allow the interactive display through Docker.


## 3. Quickly running the demo code

The easiest way to run the demo code is to use [Docker](https://www.docker.com/products/docker-desktop). From there, the script located at `setup.sh` does everything on its own :
```
git clone https://github.com/Leo-Mrl/fisheye_stereocalibration.git && cd fisheye_stereocalibration/
chmod +x setup.sh && . ./setup.sh
```
Be aware that it will download a 7.5GB Docker image.

Also, the images I use for demonstration purpose are conveniently located in the `/data_fisheyestereo/` directory of the Docker image. Those pictures are courtesy of [Sourishg](https://github.com/sourishg/fisheye-stereo-calibration).

The demo script will first calibrate both cameras independently, then perform their stereo-calibration (i.e. estimate the extrinsic parameters between them), and finally display an interactive window allowing you to click a point on the left image that will result in the drawing of an epipolar curve on the right image.


## 4. Customizing the code and running on your own data

If you would like to run the demo on a different pair of images, or on your own data, you are welcome to edit the line `37` in `setup.sh`:

`./run_demo {data_path} {img_number}`

`data_path` : path to the directory containing the image pairs. The program expects the same number of images for both cameras, and will look for filenames containing the `left` and `right` keywords. The `/data_fisheyestereo/` directory contains image pairs that works well for demo purpose, but feel free to try it using your own data as well.

`img_number` : to select the image pair you want to draw the epipolar curves on.
