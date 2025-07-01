import serial_tools
import sys
import os
import string
import subprocess
import platform
import time
import serial
import threading

consoleBuffer = []

file_update_time=2   #how many seconds between checking if file is update
last_update=0

baud=57600

picoport_prev=''
update_flag=0

n = len(sys.argv)  #get number of arguments
if (n==1):
    file=input("Please enter path to file to monitor")
else:
    file=sys.argv[1]


def get_volume_name(drive):
    if platform.system() == "Windows":
        try:
            result = subprocess.run(["vol", drive], shell=True, capture_output=True, text=True, check=True)
            # Extract volume name from the command output
            volume_name = result.stdout.strip().split("\n")[0]
            return volume_name
        except subprocess.CalledProcessError as e:
            print(f"Error: {e}")
            return None
    else:
        print("Volume name retrieval is supported only on Windows.")
        return None

def consoleInput(myBuffer):
  while True:
        myBuffer.append(input("Sensei command:"))
        time.sleep(1)

t=threading.Thread(target=consoleInput, args=(consoleBuffer,), daemon=True) # declare thread
t.start()

print(file)

check_time=time.time();       #timestamp of last file check

#at bootup lets check if a drive already is present
current_drives =['%s:' % d for d in string.ascii_uppercase if os.path.exists('%s:' % d)]
for drive in current_drives:
    if "RPI-RP2" in get_volume_name(drive):                                                  #check if new drive has RPI-RP2 in volume name
        print("Found Pi Pico. Loading firwmare")
        result = subprocess.run(["copy" , file , drive ],shell=True,stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT)
        update_flag=0   #so that there is not another update after this one

while(1):
    try:
        picoport=serial_tools.findport("PIPICO")[0]
        picoport_prev=picoport
        break;
    except:
        print("No Pi Pico found. Waiting...")
        time.sleep(5);

print("Pi Pico found at " +picoport)

ser=0
while(ser==0):

    current_drives =['%s:' % d for d in string.ascii_uppercase if os.path.exists('%s:' % d)]
    for drive in current_drives:
        if "RPI-RP2" in get_volume_name(drive):                                                  #check if new drive has RPI-RP2 in volume name
            print("Found Pi Pico. Loading firwmare")
            result = subprocess.run(["copy" , file , drive ],shell=True,stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT)
            break


    try:
        ser=serial.Serial(picoport, baud, timeout=0.001)
        if ser.is_open:
            ser.close()
    except:
        print("Unable to open Pico serial port! Check if already open...")
        time.sleep(5)

while(1):
    time.sleep(0.1)

    if not t.is_alive():
        try:
            ser.close()
        except:
            pass
        quit()

    #check if drive already exists (happens during crashes sometimes)
    current_drives =['%s:' % d for d in string.ascii_uppercase if os.path.exists('%s:' % d)]
    for drive in current_drives:
        if "RPI-RP2" in get_volume_name(drive):                                                  #check if new drive has RPI-RP2 in volume name
            print("Found Pi Pico. Loading firwmare")
            result = subprocess.run(["copy" , file , drive ],shell=True,stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT)
            continue

    #print(len(consoleBuffer) ," " ,time.time()) # just to demonstrate non blocking parallel processing)

    if consoleBuffer:
        #print("command received:",end='')
        command=consoleBuffer.pop(0)+"\r\n"

        ser.write(command.encode())

        print(repr(command))

    if not ser.is_open:
        print("Serial port closed. Trying to open...")
        try:
            picoport=serial_tools.findport("PIPICO")[0]             #always try to find port because we migth have changet to another board
            ser=serial.Serial(picoport, baud, timeout=0.001)
            print("Pi pico port open")
            #print(picoport_prev)
            #print(picoport)
            if (picoport_prev!=picoport):  #this means it's a new pico so lets update fw
                picoport_prev=picoport
                update_flag=1

        except:
            print("#ERR: Failure to open port:" + picoport)
            time.sleep(1)
            continue


    if ser.is_open:
        try: #ser.in_waiting can crash is port takes too long to open
            if ser and ser.in_waiting:   #double check if serial exists
                value=ser.read(1000)    #read serial port
                if value:
                    try:
                        decoded=value.decode('utf-8','ignore')
                        print(decoded,end='')
                    except:
                        print("Non ascii data")
        except:
            print("Error on Serial port! Maybe it closed?")
            ser.close()     #serial port is zombie. lets close it
            continue

        try:
            ser.flush()  # Try flushing the output buffer
        except serial.SerialException:
            ser.close()     #if port is not reponsive lets close it
            continue



    if time.time()-check_time<file_update_time:
        continue

    check_time = time.time()                                #update time of last file check
    modtime=os.path.getmtime(file)                          #get file mofication time

    if (last_update==modtime) and not update_flag:                              #wait here until file has been modified
        continue

    last_update=modtime                                     #save time of last modification

    update_flag=0
    print("############# File update detected!")

    available_drives = ['%s:' % d for d in string.ascii_uppercase if os.path.exists('%s:' % d)]   #list available drives in os


    for drive in available_drives:                      #if it froze here and user used BOOT we should program immediatly
        if "RPI-RP2" in get_volume_name(drive):
            print("Found Pi Pico. Loading firwmare")
            result = subprocess.run(["copy" , file , drive ],shell=True,stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT)
            update_flag=0   #so that there is not another update after this one
            continue


    if ser.is_open:
        print("Closing Serial port")
        ser.close()

    try:

        ser=serial.Serial(picoport, 1200, timeout=0.1)                #reboot pico into fw update mode (inside try because serial open gives error since port is closed immediatly)
    except:
        print("Failed to close Serial port (frozen?)")
        pass

##    if ser.is_open():
##        ser.close()


    newdrive=[]

    while(not newdrive):                                                                            #wait until new drive appears
        current_drives =['%s:' % d for d in string.ascii_uppercase if os.path.exists('%s:' % d)]

        newdrive = set(current_drives) - set(available_drives)
        time.sleep(0.5);

    newdrive=newdrive.pop()


    if "RPI-RP2" not in get_volume_name(newdrive):                                                  #check if new drive has RPI-RP2 in volume name
        print("Attached drive does not match Pi Pico")
        continue

    result = subprocess.run(["copy" , file , newdrive ],shell=True,stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT)

    time.sleep(2)                                                                                   # wait for 5 seconds to finish copy

    while(1):                                                                                       #wait for com port to appear again
        try:
            picoport=serial_tools.findport("PIPICO")[0]
            break;
        except:
            print("No Pi Pico found(after upload). Waiting...")
            time.sleep(1);






