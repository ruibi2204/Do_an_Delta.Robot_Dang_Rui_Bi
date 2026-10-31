@echo off
cd /d "C:\Users\MSI\Documents\MECHATRONICS\Tot_nghiep\Do_an_Delta.Robot_Dang_Rui_Bi\Code\DELTA_ROBOT"

echo Dang xoa build cu...
rmdir /s /q build
rmdir /s /q dist

echo Dang build lai...
"C:\Users\MSI\PyCharmMiscProject\.venv\Scripts\python.exe" -m PyInstaller --noconfirm --onefile --windowed --icon=icon.ico --name "DeltaRobot" --add-data "robot_config.json;." --add-data "calibration_result.npz;." --add-data "calibration_result.yaml;." --add-data "DeltaRobot.png;." --add-data "CT.png;." --add-data "icon.ico;." main.py

echo Xong!
pause