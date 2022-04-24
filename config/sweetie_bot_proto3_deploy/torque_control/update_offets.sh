#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <offset.list> <herkulex_servos_*.cpf>"
    exit
fi

while read p; do
  a=($p)
  echo "p0=${a[0]}" "p1=${a[1]}"dd
  (set -x; xmlstarlet ed --inplace -u "/properties/struct[@name='servos']/struct[@name='${a[0]}']/simple[@name='offset']/value" -v "${a[1]}" ${2} )
done < $1

