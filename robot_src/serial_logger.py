import serial

PORT = "/dev/cu.usbserial-0001"   # ← Bluetooth-Port
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

with open("corner_log.csv", "w") as f:
    f.write("l1,l2,l3,l4,l5,mx,my,mz,gz,speed_diff,line_pass,mag_pass,corner\n")
    print("Bluetooth-Logging läuft...")

    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            f.write(line + "\n")
            print(line)
