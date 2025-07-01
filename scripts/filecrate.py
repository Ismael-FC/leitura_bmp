import serial

with serial.Serial('COM6', 500000, timeout=1) as ser, open("test.csv", "w") as file:
    while True:
        line = ser.readline().decode("utf-8", errors = 'ignore').strip()
        if line:
            file.write(f"{line}\n")
            file.flush()
