# Image Feed Test Script

## Purpose

This script offers the flexibility to use a host camera (USB/built-in) versus a Pi camera.

## Prequisites

### Pi Camera Module Dependencies
A good read is [PyImageSearch Pi Camera Tutorial]

Quick Dependency Install Commands for Pi Camera

```sh
# Using raspi-config

sudo raspi-config # Enable the camera through the menu using arrow keys and enter.

# bash hack equivalent of raspi-config for enabling Pi camera

sudo ./enable_pi_cam

# Feel free to run this command in a virtualenv if you prefer

pip install "picamera[array]"

# bash hack equivalent of raspi-config for enabling Pi camera

sudo ./disable_pi_cam

# script encluded to disable/enable cam as an argument

sudo ./set_pi_cam 1 # enables Pi camera
sudo ./set_pi_cam 0 # disables Pi camera
```

### OpenCV Install Instructions

* [OpenCV For Python On Linux](https://docs.opencv.org/3.4.1/d2/de6/tutorial_py_setup_in_ubuntu.html)
* [OpenCV For Python On Windows](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_setup/py_setup_in_windows/py_setup_in_windows.html)
* [OpenCV For Python On Mac](https://www.pyimagesearch.com/2016/12/19/install-opencv-3-on-macos-with-homebrew-the-easy-way/) Good Luck :)

## How-to use the script

```sh
./pi_video_stream -h # Prints help and usage menu
```

## Bugs

For any bugs related to this, feel free to add a Git Issue and assign it to `@binary-eater` - Rahul Rameshbabu
