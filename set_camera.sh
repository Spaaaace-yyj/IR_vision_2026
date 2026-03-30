#!/bin/bash

v4l2-ctl -d /dev/video1 -c exposure_auto=1
v4l2-ctl -d /dev/video1 -c exposure_absolute=400
v4l2-ctl -d /dev/video1 -c gain=0
v4l2-ctl -d /dev/video1 -c white_balance_temperature_auto=0
v4l2-ctl -d /dev/video1 -c white_balance_temperature=5500