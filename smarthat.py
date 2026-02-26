import RPi.GPIO as GPIO
import time,sys
import numpy as np
import amg8833_i2c
import matplotlib.pyplot as plt
from scipy import interpolate
import os

# Pin Definitions for Left and Right Sensors
TRIG_LEFT = 11
ECHO_LEFT = 12
VIBRATION_LEFT = 16

TRIG_RIGHT = 31   # New TRIG pin for Right Sensor
ECHO_RIGHT = 32   # New ECHO pin for Right Sensor
VIBRATION_RIGHT = 18  # New Vibration motor pin for Right sensor

TRIG_MIDDLE = 35
ECHO_MIDDLE = 36
VIBRATION_MIDDLE = 22

t0 = time.time()
sensor = []
while (time.time()-t0)<1: # wait 1sec for sensor to start
    try:
        # AD0 = GND, addr = 0x68 | AD0 = 5V, addr = 0x69
        sensor = amg8833_i2c.AMG8833(addr=0x69) # start AMG8833
    except:
        sensor = amg8833_i2c.AMG8833(addr=0x68)
    finally:
        pass
time.sleep(0.1) # wait for sensor to settle

# If no device is found, exit the script
if sensor==[]:
    print("No AMG8833 Found - Check Your Wiring")
    sys.exit(); # exit the app if AMG88xx is not found
    
plt.rcParams.update({'font.size':16})
fig_dims = (12,9) # figure size
fig,ax = plt.subplots(figsize=fig_dims) # start figure
pix_res = (8,8) # pixel resolution
zz = np.zeros(pix_res) # set array with zeros first
im1 = ax.imshow(zz,vmin=15,vmax=40) # plot image, with temperature bounds
cbar = fig.colorbar(im1,fraction=0.0475,pad=0.03) # colorbar
cbar.set_label('Temperature [C]',labelpad=10) # temp. label
fig.canvas.draw() # draw figure

ax_bgnd = fig.canvas.copy_from_bbox(ax.bbox) # background for speeding up runs
fig.show() # show figure

pix_to_read = 64 # read all 64 pixels

def personDetection():
    _, pixels = sensor.read_temp(64)
    avg_temp = np.mean(pixels)
    return avg_temp

def setup():
    #GPIO setup for pins and ultrasonic sensor and vibration motors.
    GPIO.setmode(GPIO.BOARD)

    # Setup for Left Ultrasonic Sensor
    GPIO.setup(TRIG_LEFT, GPIO.OUT)
    GPIO.setup(ECHO_LEFT, GPIO.IN)
    GPIO.setup(VIBRATION_LEFT, GPIO.OUT)
    GPIO.output(VIBRATION_LEFT, GPIO.LOW)  # Turn off left vibration motor initially
    
    # Setup for Right Ultrasonic Sensor
    GPIO.setup(TRIG_RIGHT, GPIO.OUT)
    GPIO.setup(ECHO_RIGHT, GPIO.IN)
    GPIO.setup(VIBRATION_RIGHT, GPIO.OUT)
    GPIO.output(VIBRATION_RIGHT, GPIO.LOW)  # Turn off right vibration motor initially
    
    
   # Setup for Middle Ultrasonic Sensor
    GPIO.setup(TRIG_MIDDLE, GPIO.OUT)
    GPIO.setup(ECHO_MIDDLE, GPIO.IN)
    GPIO.setup(VIBRATION_MIDDLE, GPIO.OUT)
    GPIO.output(VIBRATION_MIDDLE, GPIO.LOW)  # Turn off middle vibration motor initially

def distance(trigger_pin, echo_pin):
    # Measure the distance using ultrasonic sensor
    GPIO.output(trigger_pin, True)
    time.sleep(0.000002)
    GPIO.output(trigger_pin, False)
    
    StartTime = time.time()
    StopTime = time.time()
    
    while GPIO.input(echo_pin) == 0:
        StartTime = time.time()
    
    while GPIO.input(echo_pin) == 1:
        StopTime = time.time()
    
    duration = StopTime - StartTime
    return (duration * 34300) / 2  # Converts to centimeters@@@@@@@@@@@

def vibrate_for_seconds(vibration_motor_pin, seconds=0.5):
    # Activate vibration motor for specified time
    GPIO.output(vibration_motor_pin, GPIO.HIGH)  # Turn On
    time.sleep(seconds)  # Keep on for specific time
    GPIO.output(vibration_motor_pin, GPIO.LOW)  # Turn Off

def left_sensor_loop():
    # Checks distance and controls left vibration motor
    while True:
        dis_left = distance(TRIG_LEFT, ECHO_LEFT)
        print(f"Left Sensor - Object detected at {dis_left} cm.")
        
        if dis_left < 20:
            vibrate_for_seconds(VIBRATION_LEFT, 0.5)
        
        time.sleep(0.3)

def right_sensor_loop():
    # Checks distance and controls right vibration motor
    while True:
        dis_right = distance(TRIG_RIGHT, ECHO_RIGHT)
        print(f"Right Sensor - Object detected at {dis_right} cm.")
        
        if dis_right < 20:
            vibrate_for_seconds(VIBRATION_RIGHT, 0.5)
        
        time.sleep(0.3)
        
        
def middle_sensor_loop():
    # Checks distance and controls right vibration motor
    while True:
        dis_middle = distance(TRIG_MIDDLE, ECHO_MIDDLE)
        print(f"Middle Sensor - Object detected at {dis_middle} cm.")
        
        if dis_middle < 20:
            vibrate_for_seconds(VIBRATION_MIDDLE, 0.5)
        
        time.sleep(0.3)

def destroy():
    GPIO.cleanup()

if __name__ == "__main__":
    setup()
    try:
        import threading
        
        left_thread = threading.Thread(target=left_sensor_loop, daemon=True)
        right_thread = threading.Thread(target=right_sensor_loop, daemon=True)
        middle_thread = threading.Thread(target=middle_sensor_loop, daemon=True)

        left_thread.start()
        right_thread.start()
        middle_thread.start()

        spoken = False

        while True:
            status, pixels = sensor.read_temp(pix_to_read)  # Read pixels with status
            if status:  # If error in pixel, re-enter loop and try again
                continue

            #T_thermistor = sensor.read_thermistor()  # Read thermistor temp
            #fig.canvas.restore_region(ax_bgnd)  # Restore background (speeds up run)
            #im1.set_data(np.reshape(pixels, pix_res))  # Update plot with new temps
            #ax.draw_artist(im1)  # Draw image again
            #fig.canvas.blit(ax.bbox)  # Blitting - for speeding up run
            #fig.canvas.flush_events()  # For real-time plot
            #print("Thermistor Temperature: {0:2.2f}".format(T_thermistor))  # Print thermistor temp
            avg_temp = personDetection()
            print ("Average Temperature: ", avg_temp)
            
            if avg_temp > 20:
                os.system(f'espeak "Person"')
                spoken = True
                
            if avg_temp < 20:
                spoken = False
                
            time.sleep(1)

    except KeyboardInterrupt:
        print("Stopping program...")
        destroy()
    except Exception as e:
        print(f"Error: {e}")
        destroy()
