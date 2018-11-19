#!/usr/bin/
import time
import sys
import os
import io
import logging
import json
import RPi.GPIO as GPIO

# This is the PIN (e.g. P15) (not the GP, e.g. GP15) that the sensor signal is plugged in to. 
GPIO_SIG = 15

# this is approximately 2 feet 9 inches; if sensor reading is less than or equal to this number,
# a visitor is present. If sensor reading is above this threshold, visitor is not present. 
VISITOR_THRESHOLD_CM = 84

# number of seconds that sensor must be below/above VISITOR_THRESHOLD_CM reading before we acknowledge
# their presence or absence, respectively. 
VISITOR_STATE_CHANGE_THRESHOLD_SECONDS = 3

# Current incremental visitor count detected in this function run. 
# we must add it to the existing IoT shadow count (if any) to avoid
# resetting our counter each time the function runs. 
incremental_visitor_count = 0

# these are timers for tracking when someone moved in/out of sensor range
# if the time between these and a subsequent reading is greater than the change threshold in seconds, 
# then a state change has occurred
sensor_range_entered_timestamp = None
sensor_range_exited_timestamp = time.time()

# this tracks the last time we had a confirmed visitor (confirmed = they were within the sensor threshold distance for the threshold time)
last_confirmed_visit_time = None

# based on whether current sensor reading has been above or below the required threshold for the required amount of time
sensor_has_visitor = False

# if sensor reading has been above/below threshold for correct amount of time, the
# perceived state is "validated" and we will increment our counters, update device state, etc. 
# The concept of state validation helps us avoid false positive/negatives because of unexpected
# random readings from the sensor or someone merely walking by the sensor without actually stopping there. 
current_sensor_state_validated = True

def getAndPrint():

    print "SeeedStudio Grove Ultrasonic get data and print"

    # run indefinitely
    while True:
        measurementInCM()

    # Reset GPIO settings
    GPIO.cleanup()


def measurementInCM():

    # setup the GPIO_SIG as output
    GPIO.setup(GPIO_SIG, GPIO.OUT)

    GPIO.output(GPIO_SIG, GPIO.LOW)
    time.sleep(0.2)
    GPIO.output(GPIO_SIG, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(GPIO_SIG, GPIO.LOW)
    start = time.time()

    # setup GPIO_SIG as input
    GPIO.setup(GPIO_SIG, GPIO.IN)

    # get duration from Ultrasonic SIG pin
    while GPIO.input(GPIO_SIG) == 0:
        start = time.time()

    while GPIO.input(GPIO_SIG) == 1:
        stop = time.time()

    measurementPulse(start, stop)


def measurementPulse(start, stop):

    # Calculate pulse length
    elapsed = stop-start

    # Distance pulse travelled in that time is time
    # multiplied by the speed of sound (cm/s)
    distance = elapsed * 34300

    # That was the distance there and back so halve the value
    distance = distance / 2

    print "Distance : %.1f CM" % distance

    calculateVisitorStatus(distance)

def calculateVisitorStatus(distance):

    global sensor_has_visitor
    global sensor_range_entered_timestamp
    global sensor_range_exited_timestamp
    global last_confirmed_visit_time

    # we use this to help calculate how long someone has / has not been in range of the sensor
    current_time = time.time()

    # logic below resets timers that we later use to determine if/when visitor state has changed
    if distance <= VISITOR_THRESHOLD_CM:
        
        # any time someone appears within sensor range, we reset the timer
        # that tracks how long the sensor range has been empty
        sensor_range_exited_timestamp = None
    
    else:
        # any time someone exits the sensor range, we reset the 
        # timer that tracks how long the sensor range has been occupied
        sensor_range_entered_timestamp = None

    # someone stepped into range of sensor
    if distance <= VISITOR_THRESHOLD_CM and sensor_has_visitor == False:
        
        # if this is the first time we've detected a potential visitor, take note of the time.                    
        if sensor_range_entered_timestamp == None:
        
            sensor_range_entered_timestamp = time.time()
            print "new potential visitor detected"

        # if we've seen the same potential visitor but not enough time has elapsed to count them...
        elif (current_time - sensor_range_entered_timestamp) < VISITOR_STATE_CHANGE_THRESHOLD_SECONDS:
            
            print "same potential visitor detected, not enough time elapsed to be sure"

        # if we've seen the same potential visitor *and* enough time has now passed to count them...
        else:

            sensor_has_visitor = True
            last_confirmed_visit_time = time.time()
            print "new visitor confirmed!"
            increment_visitor_count()

    # same person is there; nothing to do
    elif distance <= VISITOR_THRESHOLD_CM and sensor_has_visitor == True:
        
        print "same visitor, no action"

    # visitor might have left    
    elif distance > VISITOR_THRESHOLD_CM and sensor_has_visitor == True:
        
        # if this is the first time a visitor might have left, take note of the time.                    
        if sensor_range_exited_timestamp == None:
        
            sensor_range_exited_timestamp = time.time()
            print "visitor might have left"

        # if we've already seen the visitor may have left but not enough time has passed to be sure...
        elif (current_time - sensor_range_exited_timestamp) < VISITOR_STATE_CHANGE_THRESHOLD_SECONDS:
            
            print "visitor might have left, not enough time elapsed to be sure"

        # we're sure the visitor has left
        else:

            sensor_has_visitor = False
            record_total_visit_time(time.time() - last_confirmed_visit_time)
            print "visitor has left"

    elif distance > VISITOR_THRESHOLD_CM and sensor_has_visitor == False:
    # no one is there, nothing to do
    
        print "no visitor"
    
    else:
    
        print "Warning: handled condition while calculating visitor status."

def record_total_visit_time(visit_time):

    print "Total visit time was %f seconds" % visit_time

def increment_visitor_count():

    # for now, we simply update our program variable. 
    # TODO - add update / write to IoT shadow

    global incremental_visitor_count
    incremental_visitor_count += 1
    print "Running visitor count = %f " % incremental_visitor_count

if __name__ == '__main__':
    # rpi board gpio or bcm gpio
    GPIO.setmode(GPIO.BOARD)

    # loop method
    getAndPrint()
