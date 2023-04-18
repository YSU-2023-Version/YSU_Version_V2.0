#!/bin/bash

source /home/zjw/intel/openvino_2022.1.0.643/setupvars.sh

sec=1

cnt=0

PROC_NAME=./YSU_2022_Sentry

Thread=ps -ef | grep $PROC_NAME | grep -v "grep"

cd /home/zjw/YSU_2022_sentry_20220605/build-YSU_2022_Sentry-Desktop_Qt_5_14_1_GCC_64bit-Debug

echo "ysu1314" | sudo -S sudo chmod +777 /dev/ttyUSB0

./YSU_2022_Sentry

while [ 1 ]

do

count=ps -ef | grep $PROC_NAME | grep -v "grep" | wc -l

echo "Thread count: $count"

if [ $count -gt 1 ];then  //-gt

        echo "The $PROC_NAME is still alive!"

        sleep $sec

else

        echo "Starting $PROC_NAME"

        cd ~

        echo "ysu1314" | sudo -S sudo chmod +777 /dev/ttyUSB0

       #echo "ysu1314" | sudo -S sudo chmod +777 /dev/ttyUSB1

        cd /home/zjw/YSU_2022_sentry_20220605/build-YSU_2022_Sentry-Desktop_Qt_5_14_1_GCC_64bit-Debug

        ./YSU_2022_Sentry

        echo "$PROC_NAME has started!"

                sleep $sec

fi

done
