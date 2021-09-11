# ROS RaspberryPi GPIO

This package is an experimental sandbox for getting ROS to work with Raspberry Pi Peripherals.

Some of my notes from this is on hoani.net:
* [Raspberry Pi C++ IO](https://hoani.net/posts/guides/2021-09-11-raspberryPiCpp/)
* [Raspberry Pi Python IO](https://hoani.net/posts/guides/2021-09-10-raspberryPiPython/)
* [Raspberry Pi Camera](https://hoani.net/posts/guides/2021-09-09-raspberryPiCamera/)

## Dependencies

```sh
sudo apt update
sudo apt install python3-gpiozero
sudo apt install pigpio python3-pigpio
```

This package depends on the `pigpio` daemon running:

```sh
systemctl start pigpiod
```

Optionally:
```sh
systemctl enable pigpiod # Always run
```

## Build 

This package should be cloned into your catkin workspace.

```sh
cd $CATKIN_WS
catkin_make
cd -
```

## Run

Currently, there is only:

```sh
rosrun rpi_gpio servo
# or
rosrun rpi_gpio servo.py
```

They both:
* publish a servo angle on `/servo/angle`
* can receive a service command on `/servo/command`

