#!/bin/bash

rm map.csv

height=$1
width=$2

for h in `seq $height`; do 
	for w in `seq $width`; do
		if [ $w = 1 ]; then
			val=1
		elif [ $w = $width ]; then
			val=1
		elif [ $h = 1 ]; then
			val=1
		elif [ $h = $height ]; then
			val=1
		else
			val=0
		fi

		if [ $w = $width ]; then
			echo "$val" >> map.csv
		else
			echo -n "$val," >> map.csv
		fi
	done
done

