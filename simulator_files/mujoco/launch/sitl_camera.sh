#!/bin/bash
# Launch a single Crazyflie SITL agent with MuJoCo visualization,
# AI-deck camera rendering, and the CPX bridge (ESP32 emulator).
#
# Usage: ./sitl_camera.sh [-m <model_type>] [-s <scene_xml>]
#        [--sensor-noise] [--ground-effect]

function cleanup() {
	pkill -x cf2
	pkill -f "crazysim_cpx.py"
}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
	echo "Usage: $0 [-m <model_type>] [-s <scene_xml>] [--sensor-noise] [--ground-effect]"
	echo ""
	echo "Feature flags:"
	echo "  --sensor-noise        BMI088 IMU noise model (bias, scale, white noise)"
	echo "  --ground-effect       Increased thrust near ground"
	exit 1
fi

# Parse feature flags (long opts) before getopts
SENSOR_NOISE=""
GROUND_EFFECT=""
POSITIONAL=()
while [[ $# -gt 0 ]]; do
	case "$1" in
		--sensor-noise)   SENSOR_NOISE="--sensor-noise"; shift;;
		--ground-effect)  GROUND_EFFECT="--ground-effect"; shift;;
		*) POSITIONAL+=("$1"); shift;;
	esac
done
set -- "${POSITIONAL[@]}"

while getopts m:s: option; do
	case "${option}" in
		m) MODEL_TYPE=${OPTARG};;
		s) SCENE=${OPTARG};;
	esac
done

model_type=${MODEL_TYPE:="cf2x_T350"}
scene_arg=""
[ -n "${SCENE}" ] && scene_arg="--scene ${SCENE}"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/../../../../.."
build_path=${src_path}/sitl_make/build
crazysim_dir="$SCRIPT_DIR/.."

echo "killing running instances"
pkill -x cf2 || true
pkill -f "crazysim_cpx.py" || true
pkill -f "crazysim.py" || true
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

# Start CPX bridge in background
python3 "$crazysim_dir/crazysim_cpx.py" &

sleep 1

# Start MuJoCo crazysim with camera (foreground)
echo "Connect with: python3 crazyflie-lib-python/examples/aideck/fpv.py tcp://127.0.0.1:5050"
python3 "$crazysim_dir/crazysim.py" \
	--model-type "${model_type}" \
	--port 19950 \
	--vis \
	--camera \
	${SENSOR_NOISE} \
	${GROUND_EFFECT} \
	${scene_arg} \
	-- "0,0"
