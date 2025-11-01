### Open Track + LED
    ros2 topic pub /tower/udp_json std_msgs/String '{"data":"{\"ver\":1,\"light\":{\"color\":\"red\",\"mode\":\"blink\",\"period_ms\":240},\"audio\":{\"action\":\"play\",\"track\":1}}"}' -1

### Close Track + LED
    ros2 topic pub --once /tower/udp_json std_msgs/String '{data: "{\"ver\":1,\"light\":{\"mode\":\"off\"},\"audio\":{\"action\":\"stop\"}}"}'
