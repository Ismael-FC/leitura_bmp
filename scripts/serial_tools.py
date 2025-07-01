import serial.tools.list_ports
import time
import os.path
from os import path, name

#version 16/07/23

TEENSYID    = "16C0:048"
FTDIID      = "0430:6001"
HL340ID     = "1A86:7523"
U2D2	    = "FT3M9YHMA"
MIBHUB1     = "FTSEEDMIB15100A"
USB2DYN     = "AD01UYP1A"
PDC1 	 	= "0403:6015"
PDC2 	    = "0403:6010"
PROMICRO    = "1B4F:9204"
TEENSYUSB0  = "LOCATION=1-1:x.0"
TEENSYUSB1  = "LOCATION=1-1:x.2"
TEENSYUSB2  = "LOCATION=1-1:x.4"
PIPICO      = "2E8A:000A"


names = [["PDC1" , "0403:6015"] , ["PDC2_485" , "0403:6010"] , ["PDC2_422" , "0403:6010"] , ["TEENSY", TEENSYID] , ["TEENSYUSB0", TEENSYUSB0] , ["TEENSYUSB1", TEENSYUSB1] , ["TEENSYUSB2", TEENSYUSB2] , ["FTDI","0430:6001"] , ["HL340" , "1A86:7523"] , ["U2D2" , "FT3M9YHMA"] , ["MIBHUB1" , "FTSEEDMIB15100A"] , ["USB2DYN" , "AD01UYP1A"] , ["PROMICRO" , PROMICRO] , ["PIPICO" , PIPICO]]
#PDC2 is special case. it has two ports under same hwid. They will be differenciated by serial number endind in A or B

DEFAULTBAUD= 1000000
filename= "serialsettings.txt"

#os.chdir('D:\\Google Drive\Seed Robotics\\code\\sensors')

def findport(target):								   #find port that matches passed parameter and returns it (without connecting)

    ports = serial.tools.list_ports.comports()

    comports=[]
    if any(target in sublist for sublist in names):   #if target port exists in list of valid ports

        for index, name_id in enumerate(names):
            for index2, data in enumerate(name_id):   #interate through lists to find matching port
                #print (data)
                if target == data:
                    matchindex=index                              #exit cycle here. match index corresponds to list index

        targetid=names[matchindex][1]							#gets hwid into target (even if more than one hit we only need the first one)

        for port in ports:
            if (targetid in port.hwid) and not(targetid ==TEENSYID and "LOCATION=1-1:x" in port.hwid):   #finds a match ignoring teensy serial number in
                if "PDC2" in target:
                    if (target == "PDC2_485"):					#pcd2 is special case. we detect if it is a RS485 or RS422 by A or B in serial number last letter(windows) of location ending in .0 or .1
                        if (port.hwid[-1]=="A" or port.hwid[-2:]==".0"):
                            comports.append(port.device)
                    if (target == "PDC2_422"):
                        if (port.hwid[-1]=="B" or port.hwid[-2:]==".1"):
                            comports.append(port.device)
##                elif ("TEENSYUSB" in target):
##                    if ("TEENSYUSB0" in target) and TEENSYLOC["TEENSYUSB0"] in port.hwid:
##                        comports.append(port.device)
##                    elif ("TEENSYUSB1" in target) and TEENSYLOC["TEENSYUSB1"] in port.hwid:
##                        comports.append(port.device)
##                    elif ("TEENSYUSB2" in target) and TEENSYLOC["TEENSYUSB2"] in port.hwid:
##                        comports.append(port.device)

                else:
                     comports.append(port.device)

    return(comports)

def flush(ser):   #try multiple strategies to clear buffer
    ser.flush()
    bytesToRead = ser.inWaiting()
    ser.read(bytesToRead)			#discard data in buffer
    ser.read(1000)

def print_name(hwid):  #prints nice mame pf passed port
    if TEENSYID in hwid and not("LOCATION=1-1:x" in hwid):  #identifies a teensy with a single usb serial
        print ( "(Teensy) ",end='')
    elif TEENSYUSB0 in hwid:
         print ( "(TeensyUSB0) ",end='')
    elif TEENSYUSB1 in hwid:
         print ( "(TeensyUSB1) ",end='')
    elif TEENSYUSB2 in hwid:
         print ( "(TeensyUSB2) ",end='')
    elif FTDIID in hwid:
        print ( "(FTDI) ",end='')
    elif HL340ID in hwid:
        print ( "(HL340) ",end='')
    elif U2D2 in hwid:
        print ( "(U2D2) ",end='')
    elif MIBHUB1 in hwid:
        print ( "(MIBHUB1) ",end='')
    elif USB2DYN in hwid:
        print ( "(USB2DYN) ",end='')
    elif PDC1 in hwid:
        print ( "(PDC1) ",end='')
    elif PDC2 in hwid:
        if hwid[-1]=="A" or  hwid[-2:]==".0":
            print ( "(PDC2_485) ",end='')
        else:
            print ( "(PDC2_422) ",end='')
    else:
        pass

def serialconnect(serialport=[],baudarate=DEFAULTBAUD,readtimeout=1,target=[]):

    if not serialport:
        ports = serial.tools.list_ports.comports()

        serialport=findport(target)

        if serialport:					#if not an empty list
            serialport=serialport[0]	#convert list to string (only first item) so that it connects to first match

    if not serialport:		#if no serial port is passed try to find one . if previous condition was not met a new port is asked of user

        listports() 		#print available serial ports

        serialport = input("Enter port number:")

        if serialport.isnumeric():
            serialport="COM" + serialport


    try:     #try closing port
        serial.Serial(serialport, 9600).close()
    except:
        pass


    try:
        ser = serial.Serial(serialport, baudarate, timeout=readtimeout)
        print("Connected to: " + ser.portstr)
        fh = open(filename, 'w')   #open file in write mode
        print(serialport,file=fh,end='')	#save com port used
        fh.close

    except serial.SerialException:
        print(serialport + ' port does not exist! Exiting')
        return -1

    return ser


def connectlastport(baudarate=DEFAULTBAUD,readtimeout=1):

    global filename
    if not path.exists(filename):
        print("Serial port file setting does not exist.")
        return serialconnect([],baudarate,readtimeout)

    with open(filename,'r') as fh:
        comport=fh.readline()

    try:
        ser = serial.Serial(comport, baudarate, timeout=readtimeout)
        print("Connected to: " + ser.portstr)
        return ser

    except serial.SerialException:
        print(comport + 'does not exist or is open. Querying user')
        return serialconnect([],baudarate,readtimeout)


def port_change(looping=1):  #scans for changes on serial ports. returns name of last atached serial port

    portlists=[]

    while 1:

        time.sleep(0.1)

        portlists_prev=portlists.copy()
        portlists=[]
        hwids=[]

        ports = serial.tools.list_ports.comports()
        for port, desc, hwid in sorted(ports):
           # print(port)

            portlists.append(port)
            hwids.append(hwid)   #store hwid for each port

        if not(portlists_prev): #if list is empty scan again
            continue

        lost=list(set(portlists_prev) - set(portlists)) #list of lost ports
        gain=list(set(portlists) - set(portlists_prev)) #list of gained ports


        if lost:
            print("Port disconnected: ",end='')
            for portn in lost:
                print(portn + " " ,end='')
            print("")

        if gain:
            print("Port connected: ",end='')
            for portn in gain:
                print(portn + " " ,end='')
                position=portlists.index(portn)
                print_name(hwids[position])
            print("")
            if not(looping):
                return gain[0] #return first item in list

def listports():
    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in sorted(ports):
        print(port,' ',desc,' ',hwid,' ',end='')
        print_name(hwid)
        print()


def main():
    listports()
    print("Close window to terminate")
    port_change(1)

if __name__ == "__main__":
    main()