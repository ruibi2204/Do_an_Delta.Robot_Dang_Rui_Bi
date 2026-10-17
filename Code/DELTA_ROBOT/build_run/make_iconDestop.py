from PIL import Image

img = Image.open("../DeltaRobot.png")   # đổi tên nếu bạn đặt lại tên khác
img.save("icon.ico", format="ICO", sizes=[(16,16), (32,32), (48,48), (256,256)])