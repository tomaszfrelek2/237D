GOPRO_IP="172.28.114.51"
GOPRO_PORT="8080"

# Wake up the GoPro & enable USB control
curl -X GET "http://$GOPRO_IP:$GOPRO_PORT/gopro/camera/control/wired_usb?p=1"

# Start the preview/live stream
curl -X GET "http://$GOPRO_IP:$GOPRO_PORT/gopro/camera/stream/start"

echo ""
echo ">>> GoPro stream should now be active on UDP port 8554."

python3 cam_record.py