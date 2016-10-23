#!/bin/bash

if [[ "$#" < 1 ]]; then
	echo "usage: upload_qurt.sh MAKE_TARGET"
	exit
fi

cd $PX4_FIRMWARE

if [ make $1 upload = 2]; then
   echo "specified make target $1 does not exist"
fi

input="$PX4_FIRMWARE/build_$1/build_qurt/qurt_make_dirs.txt"

while IFS= read -r line
do
	cd "$line" && make load
done < "$input"
