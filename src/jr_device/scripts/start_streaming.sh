#!/bin/bash
echo "mjpeg streamer starting on port 9000"
echo "http://192.168.8.102:9000/?action=stream"
mjpg_streamer -o "output_http.so -p 9000" -i "input_uvc.so -r 640x480 -n -d /dev/video6"