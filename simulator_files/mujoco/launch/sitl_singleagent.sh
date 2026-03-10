#!/bin/bash
# Launch a single Crazyflie SITL agent with MuJoCo visualization.
#
# Usage: ./sitl_singleagent.sh [-m <model_type>] [-x <x>] [-y <y>] [-d <dt>] [-M <mass_kg>]
#
# This starts one cf2 firmware instance and one crazysim.py process
# with the passive MuJoCo viewer.

function cleanup() {
	pkill -x cf2
}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
	echo "Description: Launch a single Crazyflie SITL agent in MuJoCo."
	echo "Usage: $0 [-m <model_type>] [-x <x_coordinate>] [-y <y_coordinate>] [-d <dt>] [-M <mass_kg>]"
	echo ""
	echo "Model types: cf2x_T350 (default), cf2x_L250, cf2x_P250, cf21B_500"
	exit 1
fi

while getopts m:x:y:d:M: option; do
	case "${option}" in
		m) MODEL_TYPE=${OPTARG};;
		x) X_CORD=${OPTARG};;
		y) Y_CORD=${OPTARG};;
		d) DT=${OPTARG};;
		M) MASS=${OPTARG};;
	esac
done

model_type=${MODEL_TYPE:="cf2x_T350"}
x_cord=${X_CORD:=0}
y_cord=${Y_CORD:=0}
dt=${DT:=0.001}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/../../../../.."
build_path=${src_path}/sitl_make/build
crazysim_dir="$SCRIPT_DIR/.."

echo "killing running crazyflie firmware instances"
pkill -x cf2 || true
sleep 1

# Start firmware instance
working_dir="$build_path/0"
[ ! -d "$working_dir" ] && mkdir -p "$working_dir"
pushd "$working_dir" &>/dev/null
echo "Starting firmware instance 0 on port 19950"
$build_path/cf2 19950 > out.log 2> error.log &
popd &>/dev/null

sleep 1

trap "cleanup" SIGINT SIGTERM EXIT

# Start MuJoCo crazysim
echo "Starting MuJoCo CrazySim with model_type=${model_type}"
mass_arg=""
[ -n "${MASS}" ] && mass_arg="--mass ${MASS}"
python3 "$crazysim_dir/crazysim.py" \
	--model-type "${model_type}" \
	--port 19950 \
	--vis \
	--dt "${dt}" \
	${mass_arg} \
	-- "${x_cord},${y_cord}"
