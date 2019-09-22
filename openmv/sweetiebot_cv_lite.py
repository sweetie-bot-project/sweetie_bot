# SweetieBot CV Lite with JSON output
#
# This OpenMV script detects faces and AprilTags, prints the output as JSON.
#
# Author: XDN

import sensor, time, image, pyb, json

# Reset the sensor
sensor.reset()

# Sensor settings
sensor.set_contrast(1)
sensor.set_gainceiling(16)
sensor.set_framesize(sensor.HQVGA)
sensor.set_pixformat(sensor.RGB565)

# Load Haar Cascade
# By default this will use all stages, lower satges is faster but less accurate.
face_cascade = image.HaarCascade("frontalface", stages=25)

# FPS clock
clock = time.clock()

# Setup UART
uart = pyb.USB_VCP()
pyb.enable_irq(True)
uart.setinterrupt(3)

while True:
    # Capture a snapshot
    img = sensor.snapshot()

    try:
        # Find a face
        # HINT: Lower scale factor scales-down the image more and detects smaller objects.
        # Higher threshold results in a higher detection rate, with more false positives.
        objects = img.find_features(face_cascade, threshold=0.5, scale=1.5)
    except MemoryError:
        print('Out of memory during faces recognition!')

    # Empty UART frame
    frame = {}

    faces = []

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

        # Add a face record to the frame
        faces.append(face_rect)

    if len(faces):
        # Add faces to the dict
        frame['faces'] = faces

    try:
        # Recognize AprilTags
        objects = img.find_apriltags()
    except MemoryError:
        print('Out of memory during AprilTags recognition!')

    april_tags = {}

    # Enumerate tags
    for tag in objects:
        img.draw_rectangle(tag.rect(), color=(0, 0, 0))

        # Form a tag record
        tag_rect = {
            'id': tag.id(),
            'x': tag.x(),
            'y': tag.y(),
            'w': tag.w(),
            'h': tag.h()
        }

        # Add a face record to the frame
        april_tags[tag.id()] = tag_rect

    if len(april_tags):
        # Add tags to the dict
        frame['april_tags'] = list(april_tags.values())

    # It's not an empty frame
    if len(frame):
        # Convert to JSON
        json.dump(frame, uart)
        # Terminate the frame
        uart.write('\n')
