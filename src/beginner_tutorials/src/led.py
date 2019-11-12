import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(38, GPIO.OUT)
GPIO.setup(40, GPIO.OUT)

GPIO.output(38, GPIO.HIGH)
GPIO.output(40, GPIO.LOW)

