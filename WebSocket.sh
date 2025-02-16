#!/bin/bash

# 啟動 rosbridge_server (WebSocket server for ROS)
echo "Starting ROSBridge WebSocket server..."
roslaunch rosbridge_server rosbridge_websocket.launch address:=0.0.0.0 &
ROSBRIDGE_PID=$!
echo "ROSBridge WebSocket server started with PID $ROSBRIDGE_PID"

# 啟動 web_video_server (Video streaming server)
echo "Starting web_video_server..."
rosrun web_video_server web_video_server _port:=8080 image_transport:=compressed &
VIDEO_SERVER_PID=$!
echo "web_video_server started with PID $VIDEO_SERVER_PID"

# 啟動 Python HTTP 靜態伺服器
echo "Starting Python HTTP server on port 8000..."
python3 -m http.server 8000 &
HTTP_SERVER_PID=$!
echo "Python HTTP server started with PID $HTTP_SERVER_PID"

# 提示用戶服務已啟動
echo "All services are up and running:"
echo " - ROSBridge WebSocket: ws://0.0.0.0:9090"
echo " - Video streaming: http://0.0.0.0:8080/stream?topic=<YOUR_TOPIC>"
echo " - Static HTTP server: http://0.0.0.0:8000/index.html"

echo "Press [CTRL+C] to stop all services..."

# 設置 trap：只在收到 Ctrl+C 訊號時停止三個後台進程，但不結束 script
trap 'echo "Stopping services..."; \
      kill $ROSBRIDGE_PID $VIDEO_SERVER_PID $HTTP_SERVER_PID; \
      echo "All services stopped.";' SIGINT

# 主程序阻塞，等待所有後台程式結束
wait

# 到這裡代表所有後台程式都已結束（包含手動 Ctrl+C 或正常退出）
echo "All services have ended."

# 啟動交互式 Shell，不退出容器或終端
exec bash
