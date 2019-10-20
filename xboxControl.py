import serial
import threading
import queue
import inputs
from inputs import get_gamepad
import time
from time import sleep
import random
sem = threading.Semaphore()

def read_and_print(ser):
    while(1):
        if ser.readable():
            print(ser.readline())

def read_gamepad(throttleQueue):
    while 1:
        gamepad = inputs.devices.gamepads[0]
        events = gamepad.read()
        for event in events:
            if (event.ev_type == "Absolute"):
                if (event.code == "ABS_RZ"):
                    throttle = int(event.state*100/255)
                    sem.acquire()
                    if (throttleQueue.qsize() > 0):
                        throttleQueue.get_nowait()
                    throttleQueue.put(throttle)
                    sem.release()

def write_to_ser(ser, throttleQueue):
    last_time = time.time()
    while 1:
        if(time.time() - last_time > 0.05):
            last_time = time.time()
            if (throttleQueue.qsize() > 0):
                throttle = throttleQueue.get()
                string = 'C ' + str(throttle) + '\n'
                ser.write(string.encode())
        else:
            sleep(0.01)

def main():
    print("\n-------Start------\n")

    #open serial port
    ser = serial.Serial()
    ser.baudrate = 115200
    ser.port =  'COM9'
    ser.timeout = 25
    ser.open()
    sleep(0.5)    
    print("Serial port opened.")

    serial_read_thread = threading.Thread(target=read_and_print, args=(ser,), daemon=True)
    serial_read_thread.start()

    throttleQueue = queue.Queue()
    throttleQueue.put(0)

    gamepad_read_thread = threading.Thread(target=read_gamepad, args=(throttleQueue,), daemon=True)
    gamepad_read_thread.start()

    write_to_ser_thread = threading.Thread(target=write_to_ser, args=(ser, throttleQueue,), daemon=True)
    write_to_ser_thread.start()

    while 1:
        sleep(10000)


if __name__ == '__main__':
    main()