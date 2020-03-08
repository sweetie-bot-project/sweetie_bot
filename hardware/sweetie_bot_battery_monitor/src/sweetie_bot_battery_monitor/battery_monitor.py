#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import BatteryState
from os.path import join

def decode_helper(value, decoder, default_value):
	if value == None:
		return default_value
	value = decoder.get(value)
	if value == None:
		value = default_value
	return value

def decode_power_supply_status(status):
	status_decoder = { 'Charging': BatteryState.POWER_SUPPLY_STATUS_CHARGING,
	                   'Discharging': BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
	                   'Not charging': BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING,
	                   'Full': BatteryState.POWER_SUPPLY_STATUS_FULL }

	return decode_helper(status, status_decoder, BatteryState.POWER_SUPPLY_STATUS_UNKNOWN)

def decode_power_supply_health(health):
	health_decoder = { "Good": BatteryState.POWER_SUPPLY_HEALTH_GOOD,
	                   "Overheat": BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT,
	                   "Dead": BatteryState.POWER_SUPPLY_HEALTH_DEAD,
	                   "Over voltage": BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE,
	                   "Unspecified failure": BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE,
	                   "Cold": BatteryState.POWER_SUPPLY_HEALTH_COLD,
	                   "Watchdog timer expire": BatteryState.POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE,
	                   "Safety timer expire": BatteryState.POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE,
	                   "Over current": BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN }

	return decode_helper(health, health_decoder, BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN)

def decode_power_supply_technology(technology):
	technology_decoder = { "NiMH": BatteryState.POWER_SUPPLY_TECHNOLOGY_NIMH,
	                       "Li-ion": BatteryState.POWER_SUPPLY_TECHNOLOGY_LION,
	                       "Li-poly": BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO,
	                       "LiFe": BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE,
	                       "NiCd": BatteryState.POWER_SUPPLY_TECHNOLOGY_NICD,
	                       "LiMn": BatteryState.POWER_SUPPLY_TECHNOLOGY_LIMN }
	return decode_helper(technology, technology_decoder, BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN)

def decode_float_noexcept(value):
	try:
		return float(value) / 1e6
	except:
		return float('NaN')

def decode_float(value):
	return float(value) / 1e6

def decode_int(value):
	return int(value)

def decode_battery_state(uevent):
	state = BatteryState()

	# timestamp
	state.header.stamp = rospy.Time.now();

	# permanent data
	state.present = decode_int(uevent['POWER_SUPPLY_PRESENT'])

	# voltage
	state.voltage = decode_float(uevent['POWER_SUPPLY_VOLTAGE_NOW'])

	# charge level 
	state.percentage = float( decode_int(uevent['POWER_SUPPLY_CAPACITY']) )

	# get battery status
	state.power_supply_status = decode_power_supply_status(uevent.get('POWER_SUPPLY_STATUS'))
	state.power_supply_health = decode_power_supply_health(uevent.get('POWER_SUPPLY_HEALTH'))
	state.power_supply_technology = decode_power_supply_technology(uevent.get('POWER_SUPPLY_TECHNOLOGY'))

	# capacity: it can be supplied as energy in uW or as charge in uAh
	if 'POWER_SUPPLY_ENERGY_NOW' in uevent:
		# battery module reporting capacity as uW via energy properties
		energy_now = decode_float_noexcept( uevent.get('POWER_SUPPLY_ENERGY_NOW') )
		energy_full = decode_float_noexcept( uevent.get('POWER_SUPPLY_ENERGY_FULL') )
		energy_full_design = decode_float_noexcept( uevent.get('POWER_SUPPLY_ENERGY_FULL_DESIGN') )
		voltage_design = decode_float_noexcept( uevent.get('POWER_SUPPLY_VOLTAGE_MIN_DESIGN') )
		# calculate capacity
		state.charge = energy_now / voltage_design
		state.capacity = energy_full / voltage_design
		state.design_capacity = energy_full_design / voltage_design
		# charge/discharge rate
		power_now = decode_float_noexcept( uevent.get('POWER_SUPPLY_POWER_NOW') )
		if state.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
			state.current = power_now / voltage_design
		elif state.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING:
			state.current = - power_now / voltage_design
		else:
			state.current = 0
	elif 'POWER_SUPPLY_CHARGE_NOW' in uevent:
		# battery module reporting capacity in uAh
		state.charge = decode_float_noexcept( uevent.get('POWER_SUPPLY_CHARGE_NOW') )
		state.capacity = decode_float_noexcept( uevent.get('POWER_SUPPLY_CHARGE_FULL') )
		state.design_capacity = decode_float_noexcept( uevent.get('POWER_SUPPLY_CHARGE_FULL_DESIGN') )
		# charge/discharge rate
		current_now = decode_float_noexcept( uevent.get('POWER_SUPPLY_POWER_NOW') )
		if state.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
			state.current = current_now
		elif state.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING:
			state.current = - current_now
		else:
			state.current = 0
	else:
		state.charge = float('NaN')
		state.capacity = float('NaN')
		state.design_capacity = float('NaN')
		state.current = float('NaN')

	return state


def get_battery_state(sysfs_path):
	# parse file into dictionary
	uevent = {}
	with open(join(sysfs_path, 'uevent')) as fd:
		lines = fd.readlines()
		for line in lines:
			line = line.strip()
			(key, value) = line.split('=')
			uevent[key] = value

	# decode dictionary
	return decode_battery_state(uevent)

def main():
	rospy.init_node('battery_monitor')

	# get node parameters
	period = rospy.get_param('~period', 1.0)
	if not isinstance(period, (int, float)) or period <= 0.0:
		rospy.logerr('"period" parameter must be positive int or float.')
		sys.exit(-1)

	sysfs_path = rospy.get_param('~sysfs_path', '/sys/class/power_supply/BAT0')
	if not isinstance(sysfs_path, str):
		rospy.logerr('"sysfs_path" parameter must be a string.')
		sys.exit(-1)

	# node interface
	state_pub = rospy.Publisher('battery_state', BatteryState, queue_size=1)

	# internal timer
	rate = rospy.Rate(period)

	# main cycle
	while not rospy.is_shutdown():
		state = get_battery_state(sysfs_path)
		state_pub.publish(state)
		rate.sleep()

if __name__ == '__main__':
	main()
