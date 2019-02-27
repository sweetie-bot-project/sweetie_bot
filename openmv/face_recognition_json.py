# Face Eye Detection with JSON output
#
# This script uses the built-in frontalface detector to find a face and then
# the eyes within the face. If you want to determine the eye gaze please see the
# iris_detection script for an example on how to do that.
#
# Original script: kwagyeman
#   (http://forums.openmv.io/viewtopic.php?f=5&t=200&p=6177&hilit=Eye+tracking#p1089)
# Modified: XDN

import sensor, time, image, pyb, json

# Reset the sensor
sensor.reset()

# Sensor settings
sensor.set_contrast(1)
sensor.set_gainceiling(16)
sensor.set_framesize(sensor.HQVGA)
sensor.set_pixformat(sensor.GRAYSCALE)

# Load Haar Cascade
# By default this will use all stages, lower satges is faster but less accurate.
face_cascade = image.HaarCascade("frontalface", stages=25)
eyes_cascade = image.HaarCascade("eye", stages=24)
print(face_cascade, eyes_cascade)

# FPS clock
clock = time.clock()

# Setup UART
uart = pyb.UART(3, 115200)

while True:
    clock.tick()

    # Capture a snapshot
    img = sensor.snapshot()

    # Find a face
    # Note: Lower scale factor scales-down the image more and detects smaller objects.
    # Higher threshold results in a higher detection rate, with more false positives.
    objects = img.find_features(face_cascade, threshold=0.5, scale=1.5)

    # Empty UART frame
    frame = []

    # Enumerate faces
    for face in objects:
        img.draw_rectangle(face)

        # Form a face record
        face_rect = {
            'x': face[0],
            'y': face[1],
            'w': face[2],
            'h': face[3]
        }
        # Enumerate eyes within each face.
        # Note: Use a higher threshold here (more detections) and lower scale
        #   (to find small objects)
        eyes = img.find_features(eyes_cascade, threshold=0.5, scale=1.2, roi=face)

        # List of eye records for the current face
        eye_rects = []

        for eye in eyes:
            img.draw_rectangle(eye)

            # Form a new eye record and add to the list
            eye_rects.append({
                'x': eye[0],
                'y': eye[1],
                'w': eye[2],
                'h': eye[3]
            })

        # Set the list to the face record
        face_rect['eyes'] = eye_rects

        # Add a face record to the frame
        frame.append(face_rect)

    # It's not an empty frame
    if len(frame):
        # Convert to JSON
        json.dump(frame, uart)
        # Terminate the frame
        uart.write('\n')

    # Print FPS.
    # Note: Actual FPS is higher, streaming the FB makes it slower.
    print(clock.fps())
