from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont
import subprocess
import socket
import time

# Initialize I2C OLED at address 0x3C
serial = i2c(port=7, address=0x3C)
oled = ssd1306(serial, width=128, height=32)

def get_ip():
    try:
        return subprocess.check_output("hostname -I", shell=True).decode().strip().split()[0]
    except:
        return "No IP"

def get_ssid():
    try:
        ssid = subprocess.check_output("iwgetid -r", shell=True).decode().strip()
        return ssid if ssid else "No WiFi"
    except:
        return "No WiFi"

def draw_info():
    hostname = socket.gethostname()
    ip = get_ip()
    ssid = get_ssid()

    image = Image.new("1", (128, 32))
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()

    draw.text((0, 0), f"Host: {hostname}", fill=255, font=font)
    draw.text((0, 10), f"IP:   {ip}", fill=255, font=font)
    draw.text((0, 20), f"SSID: {ssid}", fill=255, font=font)


    oled.display(image)

while True:
    draw_info()
    time.sleep(5)
