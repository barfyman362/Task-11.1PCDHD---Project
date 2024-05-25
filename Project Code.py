from tkinter import *
import tkinter.font
from gpiozero import LED
import RPi.GPIO as GPIO
import cv2
import threading
import time
import atexit
import requests

GPIO.setmode(GPIO.BCM)  # Ensure the GPIO mode is set

# Hardware setup
green_led = LED(26)
blue_led = LED(5)
red_led = LED(10)
buzzer_pin = 17
pir_pin = 4  # PIR sensor connected to GPIO 4

# Configure the buzzer for PWM
GPIO.setup(buzzer_pin, GPIO.OUT)
buzzer_pwm = GPIO.PWM(buzzer_pin, 1000)  # Initialize PWM on buzzer_pin 1000Hz frequency

# Configure the PIR sensor
GPIO.setup(pir_pin, GPIO.IN)

# IFTTT Webhook settings
IFTTT_EVENT_NAME = 'motion_detected'
IFTTT_KEY = 'cva3UjjZy5iWSJeEGjPSWD'  # Replace with your actual IFTTT Webhooks key
IFTTT_URL = f'https://maker.ifttt.com/trigger/{IFTTT_EVENT_NAME}/with/key/{IFTTT_KEY}'

# Motion detection and GUI variables
armed = False
motion_detected = False
last_pir_motion_time = time.time()  # Track last motion time for debounce

# Initialize webcam
cap = cv2.VideoCapture(0)

# GUI Setup
win = Tk()
win.title("Security System GUI")
myFont = tkinter.font.Font(family='Helvetica', size=12, weight="bold")

# Event Functions
def activate_alarm():
    green_led.on()
    blue_led.on()
    red_led.on()
    buzzer_pwm.start(50)  # Start PWM with 50% duty cycle
    send_ifttt_alert()

def deactivate_alarm():
    green_led.off()
    blue_led.off()
    red_led.off()
    buzzer_pwm.stop()  # Stop PWM

def close():
    try:
        cap.release()
        if win.winfo_exists():
            win.destroy()
        GPIO.cleanup()
    except Exception as e:
        print(f"Exception during cleanup: {e}")

def arm_system():
    global armed
    armed = True
    status_label.config(text="System Armed")

def disarm_system():
    global armed
    armed = False
    status_label.config(text="System Disarmed")
    deactivate_alarm()

stop_threads = False  # Variable to signal threads to stop

def motion_detection():
    global motion_detected
    while not stop_threads:
        try:
            ret, frame = cap.read()
            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)

            if not hasattr(motion_detection, 'avg'):
                motion_detection.avg = gray.copy().astype("float")
                continue

            cv2.accumulateWeighted(gray, motion_detection.avg, 0.5)
            frame_delta = cv2.absdiff(gray, cv2.convertScaleAbs(motion_detection.avg))

            thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
            thresh = cv2.dilate(thresh, None, iterations=2)
            contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                if cv2.contourArea(contour) < 500:
                    continue

                if armed and not motion_detected:
                    motion_detected = True
                    activate_alarm()
                    for i in range(5):
                        ret, frame = cap.read()  # Capture a new frame each iteration
                        if ret:
                            cv2.imwrite(f"motion_capture_{time.strftime('%Y%m%d_%H%M%S')}_{i}.jpg", frame)
                        time.sleep(1)
                    deactivate_alarm()
                    motion_detected = False
                    break
        except Exception as e:
            print(f"Exception in motion detection: {e}")

def pir_motion_detection():
    global motion_detected, last_pir_motion_time
    while not stop_threads:
        try:
            if GPIO.input(pir_pin):
                current_time = time.time()
                if armed and not motion_detected and (current_time - last_pir_motion_time) > 2:  # 2 seconds debounce
                    motion_detected = True
                    last_pir_motion_time = current_time
                    activate_alarm()
                    for i in range(5):
                        ret, frame = cap.read()  # Capture a new frame each iteration
                        if ret:
                            cv2.imwrite(f"pir_motion_capture_{time.strftime('%Y%m%d_%H%M%S')}_{i}.jpg", frame)
                        time.sleep(1)
                    deactivate_alarm()
                    motion_detected = False
            time.sleep(0.1)
        except Exception as e:
            print(f"Exception in PIR motion detection: {e}")

def send_ifttt_alert():
    try:
        data = {"value1": "Motion detected!", "value2": "Security system alert", "value3": ""}
        requests.post(IFTTT_URL, json=data)
    except Exception as e:
        print(f"Exception sending IFTTT alert: {e}")

# Widgets
arm_button = Button(win, text='Arm', font=myFont, command=arm_system, bg='green', height=1, width=6)
arm_button.grid(row=0, column=0)

disarm_button = Button(win, text='Disarm', font=myFont, command=disarm_system, bg='yellow', height=1, width=6)
disarm_button.grid(row=0, column=1)

exitButton = Button(win, text='Exit', font=myFont, command=close, bg='red', height=1, width=6)
exitButton.grid(row=0, column=2)

status_label = Label(win, text="System Disarmed", font=myFont)
status_label.grid(row=1, columnspan=3)

win.protocol("WM_DELETE_WINDOW", close)

# Start motion detection in a separate thread
motion_thread = threading.Thread(target=motion_detection)
motion_thread.daemon = True
motion_thread.start()

# Start PIR motion detection in a separate thread
pir_motion_thread = threading.Thread(target=pir_motion_detection)
pir_motion_thread.daemon = True
pir_motion_thread.start()

# Ensure cleanup happens on exit
def on_exit():
    global stop_threads
    stop_threads = True
    motion_thread.join()
    pir_motion_thread.join()
    close()

atexit.register(on_exit)

win.mainloop()
