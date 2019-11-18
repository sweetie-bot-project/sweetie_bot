# SweetieBot CV Lite with JSON output
#
# This OpenMV script detects faces and AprilTags, prints the output as JSON.
#
# Author: XDN

import sensor, time, image, pyb, json

# Reset the sensor
sensor.reset()

# Sensor settings
sensor.set_contrast(3)
sensor.set_gainceiling(16)
sensor.set_framesize(sensor.HQVGA)
sensor.set_pixformat(sensor.GRAYSCALE)

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

    # detect objects
    try:
        # Find a face
        # HINT: Lower scale factor scales-down the image more and detects smaller objects.
        # Higher threshold results in a higher detection rate, with more false positives.
        faces = img.find_features(face_cascade, threshold=0.75, scale=1.25)
    except MemoryError:
        print('Out of memory during faces recnition!')


    try:
        # Recognize AprilTags
        tags = img.find_apriltags(families=image.TAG16H5)
    except MemoryError:
        print('Out of memory during AprilTags recognition!')

    # prepare and send data
    frame = []
    # Enumerate faces
    for face in faces:
        img.draw_rectangle(face)
        # Form a face record
        face_obj = {
            'type': 'face',
            'x': face[0],
            'y': face[1],
            'w': face[2],
            'h': face[3]
        }
        # Add a face record to the frame
        frame.append(face_obj)

    # Enumerate april tags
    for tag in tags:
        img.draw_rectangle(tag.rect(), color=(0, 0, 0))
        # Form a tag record
        tag_obj = {
            'type': 'april',
            'id': tag.id(),
            'x': tag.x(),
            'y': tag.y(),
            'w': tag.w(),
            'h': tag.h()
        }
        # Add tags to the dict
        frame.append(tag_obj)
    # Send frame
    # Convert to JSON
    json.dump(frame, uart)
    # Terminate the frame
    uart.write('\n')
    # clear send buffer
    del frame[:]


