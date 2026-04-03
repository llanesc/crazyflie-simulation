#!/bin/bash
# Launch a single Crazyflie SITL agent with MuJoCo visualization.
#
# Usage: ./sitl_singleagent.sh [-m <model_type>] [-x <x>] [-y <y>] [-d <dt>] [-M <mass_kg>] [-s <scene_xml>]
#        [--sensor-noise] [--ground-effect] [--wind-speed <m/s>] [--turbulence <level>]
#        [--flowdeck]
#
# This starts one cf2 firmware instance and one crazysim.py process
# with the passive MuJoCo viewer.

function cleanup() {
	pkill -x cf2
}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
	echo "Description: Launch a single Crazyflie SITL agent in MuJoCo."
	echo "Usage: $0 [-m <model_type>] [-x <x>] [-y <y>] [-d <dt>] [-M <mass_kg>] [-s <scene_xml>]"
	echo "          [--sensor-noise] [--ground-effect] [--flowdeck]"
	echo "          [--wind-speed <m/s>] [--turbulence <level>]"
	echo ""
	echo "Model types: cf2x_T350 (default), cf2x_L250, cf2x_P250, cf21B_500"
	echo "Scene files: scene.xml (default), scene_obstacles.xml"
	echo ""
	echo "Feature flags:"
	echo "  --sensor-noise        BMI088 IMU noise model (bias, scale, white noise)"
	echo "  --ground-effect       Increased thrust near ground"
	echo "  --flowdeck            Simulate flowdeck (TOF + optical flow, disables pose)"
	echo "  --wind-speed <m/s>    Constant wind speed"
	echo "  --wind-direction <deg> Wind direction (0=+X, 90=+Y, 180=-X, 270=-Y)"
	echo "  --gust-intensity <m/s> Random gust peak deviation"
	echo "  --turbulence <level>  Dryden turbulence (none, light, moderate, severe)"
	exit 1
fi

# Parse feature flags (long opts) before getopts
SENSOR_NOISE=""
GROUND_EFFECT=""
FLOWDECK=""
WIND_ARGS=""
POSITIONAL=()
while [[ $# -gt 0 ]]; do
	case "$1" in
		--sensor-noise)   SENSOR_NOISE="--sensor-noise"; shift;;
		--ground-effect)  GROUND_EFFECT="--ground-effect"; shift;;
		--flowdeck)       FLOWDECK="--flowdeck"; shift;;
		--wind-speed)     WIND_ARGS="$WIND_ARGS --wind-speed $2"; shift 2;;
		--wind-direction) WIND_ARGS="$WIND_ARGS --wind-direction $2"; shift 2;;
		--gust-intensity) WIND_ARGS="$WIND_ARGS --gust-intensity $2"; shift 2;;
		--turbulence)     WIND_ARGS="$WIND_ARGS --turbulence $2"; shift 2;;
		*) POSITIONAL+=("$1"); shift;;
	esac
done
set -- "${POSITIONAL[@]}"

while getopts m:x:y:d:M:s: option; do
	case "${option}" in
		m) MODEL_TYPE=${OPTARG};;
		x) X_CORD=${OPTARG};;
		y) Y_CORD=${OPTARG};;
		d) DT=${OPTARG};;
		M) MASS=${OPTARG};;
		s) SCENE=${OPTARG};;
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
stdbuf -oL $build_path/cf2 19950 > out.log 2> error.log &
popd &>/dev/null

sleep 1

trap "cleanup" SIGINT SIGTERM EXIT

# Start MuJoCo crazysim
echo "Starting MuJoCo CrazySim with model_type=${model_type}"
mass_arg=""
[ -n "${MASS}" ] && mass_arg="--mass ${MASS}"
scene_arg=""
[ -n "${SCENE}" ] && scene_arg="--scene ${SCENE}"
python3 "$crazysim_dir/crazysim.py" \
	--model-type "${model_type}" \
	--port 19950 \
	--vis \
	--dt "${dt}" \
	${SENSOR_NOISE} \
	${GROUND_EFFECT} \
	${FLOWDECK} \
	${WIND_ARGS} \
	${mass_arg} \
	${scene_arg} \
	-- "${x_cord},${y_cord}"
