DEVICES=""
for dev in /dev/video* /dev/media* /dev/v4l-subdev*; do
  [ -e "$dev" ] && DEVICES+=" --device $dev:$dev"
done

docker run -it \
  --group-add video \
  $DEVICES \
  --device /dev/i2c-1:/dev/i2c-1 \
  --device /dev/ttyUSB0:/dev/ttyUSB0 \
  -v vscode:/root/.vscode-server \
  --network host \
  --hostname raspberrypi \
  -p 11811:11811/udp \
  -e ROS_DOMAIN_ID=42 \
  orbslam:v1.2 \
  bash