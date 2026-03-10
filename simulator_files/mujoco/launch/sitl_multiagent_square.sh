#!/bin/bash
# Launch multiple Crazyflie SITL agents in a square formation with MuJoCo.
#
# Usage: ./sitl_multiagent_square.sh [-n <num_vehicles>] [-m <model_type>] [-d <dt>]
#
# This starts N cf2 firmware instances (ports 19950..19950+N-1) and one
# crazysim.py process with all drones in a single MuJoCo world.

function cleanup() {
	pkill -x cf2
}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
	echo "Description: Launch multiple Crazyflie SITL agents in a square formation in MuJoCo."
	echo "Usage: $0 [-n <num_vehicles>] [-m <model_type>] [-d <dt>]"
	echo ""
	echo "Model types: cf2x_T350 (default), cf2x_L250, cf2x_P250, cf21B_500"
	exit 1
fi

while getopts n:m:d: option; do
	case "${option}" in
		n) NUM_VEHICLES=${OPTARG};;
		m) MODEL_TYPE=${OPTARG};;
		d) DT=${OPTARG};;
	esac
done

num_vehicles=${NUM_VEHICLES:=3}
model_type=${MODEL_TYPE:="cf2x_T350"}
dt=${DT:=0.001}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/../../../../.."
build_path=${src_path}/sitl_make/build
crazysim_dir="$SCRIPT_DIR/.."

echo "killing running crazyflie firmware instances"
pkill -x cf2 || true
sleep 1

if [ $num_vehicles -gt 255 ]; then
	echo "Tried spawning $num_vehicles vehicles. The maximum number of supported vehicles is 255"
	exit 1
fi

# Build spawn positions in square grid and start firmware instances
spawn_args=""
n=0
while [ $n -lt $num_vehicles ]; do
	denom=$(python3 -c "from math import ceil, sqrt; print(ceil(sqrt($num_vehicles)))")
	x_cord=$(($n % $denom))
	y_cord=$(($n / $denom - ($n % $denom) / $denom))

	spawn_args="${spawn_args} ${x_cord},${y_cord}"

	working_dir="$build_path/$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"
	pushd "$working_dir" &>/dev/null
	echo "Starting firmware instance $n on port $((19950+$n)) at ($x_cord, $y_cord)"
	$build_path/cf2 $((19950+${n})) > out.log 2> error.log &
	popd &>/dev/null

	n=$(($n + 1))
done

sleep 1

trap "cleanup" SIGINT SIGTERM EXIT

# Start MuJoCo crazysim with all agents
echo "Starting MuJoCo CrazySim with ${num_vehicles} agents, model_type=${model_type}"
python3 "$crazysim_dir/crazysim.py" \
	--model-type "${model_type}" \
	--port 19950 \
	--vis \
	--dt "${dt}" \
	-- ${spawn_args}
