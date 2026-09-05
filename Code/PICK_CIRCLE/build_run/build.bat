@echo off
echo Dang xoa build cu...
rmdir /s /q build
rmdir /s /q dist

echo Dang build lai...
pyinstaller --noconfirm --onefile --windowed --icon=icon.ico --name "RobotDelta" --add-data "delta.png;." --add-data "robot_config.json;." --add-data "point.csv;." main.py

echo Xong!
pause
