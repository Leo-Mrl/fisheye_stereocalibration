#!/bin/sh

unameOut="$(uname -s)"
case "${unameOut}" in
    Linux*)     MACHINE=Linux;;
    Darwin*)    MACHINE=Mac;;
    *)          MACHINE="UNKNOWN"
esac

if [ "$MACHINE" == "Mac" ]; then
    xhost + ${hostname}
	export HOSTNAME=`hostname`
elif [ "$MACHINE" == "Linux" ]; then
    export HOSTNAME=$DISPLAY
else
	echo "Unable to identify the type of machine you are running (Linux, Mac etc.), please setup this variable manually"
	return 1
fi

export CURR_PATH=`pwd`
export WORK_PATH='/fisheye_stereocalib/'

printf "\n\n\n |> Make sure the local variables below are correct :\n"
printf "	host name (for display) : $HOSTNAME\n"
printf "	local path to the code : $CURR_PATH\n"
printf "	work path in container : $WORK_PATH\n"
printf "	you are running this script on $MACHINE"

printf "\n\n\n |> Pulling the docker image\n"
docker pull leomrl/stereocalib:latest

printf "\n\n\n |> Compiling code\n"
docker run --rm -it -d -v ${CURR_PATH}:${WORK_PATH} -v /tmp/.X11-unix:/tmp/.X11-unix \
-e DISPLAY=${HOSTNAME}:0 -w ${WORK_PATH} --name stereocalib_demo leomrl/stereocalib:latest /bin/bash
docker exec -it stereocalib_demo make -j$(nproc)

printf "\n\n\n |> Running the demo"
docker exec -it stereocalib_demo ./run_demo /data_fisheyestereo/ 5
docker stop stereocalib_demo