#!/usr/bin/python

# Filter to produce servo .cpf description.

import string
from sys import stdin

servo_template = string.Template('''    <struct name="$name" type="PropertyBag">
        <description>Servo description.</description>
        <simple name="servo_id" type="ulong">
            <description>Servo hardware ID.</description>
            <value>$servo_id</value>
        </simple>
        <simple name="offset" type="ulong">
            <description>Servo zero position offset (raw).</description>
            <value>512</value>
        </simple>
        <simple name="reverse" type="boolean">
            <description>Reverse direction flag.</description>
            <value>$reverse</value>
        </simple>
    </struct>''')


for line in stdin:
    name, servo_id, reverse = string.split(line)
    if reverse == 'true':
        reverse = '1'
    else:
        reverse = '0'
    print(servo_template.substitute({ 'name': name, 'servo_id': servo_id, 'reverse': reverse }))
        

    

