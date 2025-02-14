#!/bin/bash

BENCHMARK_PATH="../ISPD2025_benchmarks"
NTHU_ROUTE_PATH="../"

BENCHMARK_PATH=$(realpath "$BENCHMARK_PATH")
NTHU_ROUTE_PATH=$(realpath "$NTHU_ROUTE_PATH")

TESTCASES=(
ariane
bsg_chip
NV_NVDLA_partition_c
mempool_cluster
mempool_tile_wrap
)


echo "Select a testcase to run:"
echo "0. All testcases"
for ((i=0; i<${#TESTCASES[@]}; i++)); do
echo "$((i+1)). ${TESTCASES[$i]}"
done
read -p "Enter your choice (0-$((${#TESTCASES[@]}))): " choice


if ! [[ $choice =~ ^[0-9]+$ ]] || [ $choice -lt 0 ] || [ $choice -gt ${#TESTCASES[@]} ]; then
echo "Invalid choice. Exiting..."
exit 1
fi


GUIDE_PATH="$BENCHMARK_PATH/guide"
if [ ! -d "$GUIDE_PATH" ]; then
mkdir -p "$GUIDE_PATH"
echo "Created directory: $GUIDE_PATH"
else
echo "Directory already exists: $GUIDE_PATH"
fi


echo -e "\nInfo: Running the Makefile ..."
make -C ${NTHU_ROUTE_PATH}/src clean
make -C ${NTHU_ROUTE_PATH}/src -j

if [ $choice -eq 0 ]; then


for TESTCASE in "${TESTCASES[@]}"
do
echo "==================================================================="
echo "Running testcase: $TESTCASE"

# to specify the .cap and .net file path
CAP_FILE="$BENCHMARK_PATH/Simple_inputs/$TESTCASE.cap"
NET_FILE="$BENCHMARK_PATH/Simple_inputs/$TESTCASE.net"

# check if .cap and .net files exist
if [ ! -e "$CAP_FILE" ]; then
  echo "Error: $CAP_FILE does not exist. Skipping..."
  continue
fi
if [ ! -e "$NET_FILE" ]; then
  echo "Error: $NET_FILE does not exist. Skipping..."
  continue
fi

# create testcase folder
GUIDE_FOLDER="$GUIDE_PATH/$TESTCASE"
mkdir -p "$GUIDE_FOLDER"

# specify .guide and .log file path
GUIDE_FILE="$GUIDE_FOLDER/$TESTCASE.output"
LOG_FILE="$GUIDE_FOLDER/$TESTCASE.log"

# run NTHU Route  
echo "Running command: ./route -cap $CAP_FILE -net $NET_FILE -output $GUIDE_FILE |& tee $LOG_FILE"
./route -library XXX -def XXX.def -v XXX.v.gz -sdc XXX.sdc -cap $CAP_FILE -net $NET_FILE -output $GUIDE_FILE |& tee $LOG_FILE

if [ -e "$GUIDE_FILE" ]; then
  echo "Finished running $TESTCASE"
else
  echo "Error: Something went wrong with $TESTCASE"  
fi
done
else


TESTCASE="${TESTCASES[$choice-1]}"
echo "Running testcase: $TESTCASE"


CAP_FILE="$BENCHMARK_PATH/Simple_inputs/$TESTCASE.cap"
NET_FILE="$BENCHMARK_PATH/Simple_inputs/$TESTCASE.net"


if [ ! -e "$CAP_FILE" ]; then
echo "Error: $CAP_FILE does not exist. Exiting..."
exit 1
fi
if [ ! -e "$NET_FILE" ]; then
echo "Error: $NET_FILE does not exist. Exiting..."
exit 1

fi


GUIDE_FOLDER="$GUIDE_PATH/$TESTCASE"
mkdir -p "$GUIDE_FOLDER"


GUIDE_FILE="$GUIDE_FOLDER/$TESTCASE.output"
LOG_FILE="$GUIDE_FOLDER/$TESTCASE.log"


echo "Running command: ./route --cap $CAP_FILE --net $NET_FILE --output $GUIDE_FILE |& tee $LOG_FILE"
# valgrind ./route -cap $CAP_FILE -net $NET_FILE -output $GUIDE_FILE |& tee $LOG_FILE
./route --cap $CAP_FILE --net $NET_FILE --output $GUIDE_FILE |& tee $LOG_FILE

if [ -e "$GUIDE_FILE" ]; then
echo "Finished running $TESTCASE"
else
echo "Error: Something went wrong with $TESTCASE"
fi
fi

echo "=================================================================="
echo "Finished running testcases"
