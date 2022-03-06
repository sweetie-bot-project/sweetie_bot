#!/bin/bash

bus=0
gpios=(488 489)
addrs=(20 21)

source /home/sweetie/ros/sweetie/devel/setup.bash

function check_one {
  local addr=`printf '%x' $1`
  for i in $(seq 1 ${#i2c_data[@]}); do
    line=(${i2c_data[$i]})
    #echo 0 $addr
    #echo ${line[@]:1} # | grep -q $addr > /dev/null
    #echo 0 $addr
    echo ${line[@]:1} | grep -q $addr
    if [ $? -eq 0 ]; then
        #echo "$addr is present."
        echo 1
        return
    fi
  done
  #echo "$addr is not present."
  echo 0
}

function check_all {
  mapfile -t i2c_data < <(i2cdetect -y $bus)
  local res_all=0
  # check if devices already initialised
  for i in ${!addrs[@]};
  do
    res=$(check_one ${addrs[$i]})
    res_all=$(( $res_all + $res ))
  done
  echo $res_all
}

res=`check_all`

if [ $res -eq 0 ]; then
  echo Initialising...
else
  echo Already initialised.
  #exit 0
fi

set -e

for i in ${!gpios[@]};
do
  port=${gpios[$i]}
  if [ -d "/sys/class/gpio/gpio${port}" ]; then
    echo $port > /sys/class/gpio/unexport
  fi
  echo $port > /sys/class/gpio/export
  echo out > /sys/class/gpio/gpio${port}/direction
  echo 0 > /sys/class/gpio/gpio${port}/value
done

echo All GPIO ports are exported [${gpios[@]}].

for i in ${!gpios[@]};
do
  port=${gpios[$i]}
  echo 1 > /sys/class/gpio/gpio${port}/value
  echo Device №${i} gpio${port} is enabled.
  rosrun sweetie_bot_ranging_vl53l5 configuration_tool ${addrs[$i]}
  echo New i2c address is ${addrs[$i]}.
done

set +e

res=`check_all`

if [ $res -eq ${#gpios[@]} ]; then
  echo All $res devices is initialised. New addresses is ${addrs[@]}.
else
  echo [ERROR] Something went wrong.
  exit 1
fi

exit 0
