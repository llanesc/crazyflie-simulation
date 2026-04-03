#!/bin/bash
# Launch multiple Crazyflie SITL agents from a coordinates text file with MuJoCo.
#
# Usage: ./sitl_multiagent_text.sh [-m <model_type>] [-f <file_name>] [-d <dt>] [-M <mass_kg>] [-s <scene_xml>]
#        [--sensor-noise] [--ground-effect] [--downwash] [--flowdeck]
#        [--wind-speed <m/s>] [--turbulence <level>]
#
# The coordinates file should have one X,Y pair per line (CSV format).
# Default file: single_origin.txt from the shared drone_spawn_list directory.

function cleanup() {
	pkill -x cf2
}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
	echo "Description: Launch multiple Crazyflie SITL agents from a coordinates file in MuJoCo."
	echo "Usage: $0 [-m <model_type>] [-f <file_name>] [-d <dt>] [-M <mass_kg>] [-s <scene_xml>]"
	echo "          [--sensor-noise] [--ground-effect] [--downwash] [--flowdeck]"
	echo "          [--wind-speed <m/s>] [--turbulence <level>]"
	echo ""
	echo "Model types: cf2x_T350 (default), cf2x_L250, cf2x_P250, cf21B_500"
	echo "Scene files: scene.xml (default), scene_obstacles.xml"
	echo "Coordinates files are in: tools/crazyflie-simulation/drone_spawn_list/"
	echo ""
	echo "Feature flags:"
	echo "  --sensor-noise        BMI088 IMU noise model (bias, scale, white noise)"
	echo "  --ground-effect       Increased thrust near ground"
	echo "  --downwash            Aerodynamic interaction between drones"
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
DOWNWASH=""
FLOWDECK=""
WIND_ARGS=""
POSITIONAL=()
while [[ $# -gt 0 ]]; do
	case "$1" in
		--sensor-noise)   SENSOR_NOISE="--sensor-noise"; shift;;
		--ground-effect)  GROUND_EFFECT="--ground-effect"; shift;;
		--downwash)       DOWNWASH="--downwash"; shift;;
		--flowdeck)       FLOWDECK="--flowdeck"; shift;;
		--wind-speed)     WIND_ARGS="$WIND_ARGS --wind-speed $2"; shift 2;;
		--wind-direction) WIND_ARGS="$WIND_ARGS --wind-direction $2"; shift 2;;
		--gust-intensity) WIND_ARGS="$WIND_ARGS --gust-intensity $2"; shift 2;;
		--turbulence)     WIND_ARGS="$WIND_ARGS --turbulence $2"; shift 2;;
		*) POSITIONAL+=("$1"); shift;;
	esac
done
set -- "${POSITIONAL[@]}"

while getopts m:f:d:M:s: option; do
	case "${option}" in
		m) MODEL_TYPE=${OPTARG};;
		f) COORDINATES_FILE=${OPTARG};;
		d) DT=${OPTARG};;
		M) MASS=${OPTARG};;
		s) SCENE=${OPTARG};;
	esac
done

model_type=${MODEL_TYPE:="cf2x_T350"}
coordinates_file=${COORDINATES_FILE:="single_origin.txt"}
dt=${DT:=0.001}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/../../../../.."
build_path=${src_path}/sitl_make/build
crazysim_dir="$SCRIPT_DIR/.."
spawn_list_dir="${src_path}/tools/crazyflie-simulation/drone_spawn_list"

if [ ! -f "${spawn_list_dir}/${coordinates_file}" ]; then
	echo "ERROR: Coordinates file not found: ${spawn_list_dir}/${coordinates_file}"
	echo "Available files:"
	ls "${spawn_list_dir}/"
	exit 1
fi

echo "killing running crazyflie firmware instances"
pkill -x cf2 || true
sleep 1

# Read spawn positions and start firmware instances
spawn_args=""
n=0
while IFS= read -r line || [ -n "$line" ]; do
	fields=($(printf "%s" "$line" | cut -d',' --output-delimiter=' ' -f1-))
	x_cord=${fields[0]}
	y_cord=${fields[1]}

	spawn_args="${spawn_args} ${x_cord},${y_cord}"

	working_dir="$build_path/$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"
	pushd "$working_dir" &>/dev/null
	echo "Starting firmware instance $n on port $((19950+$n)) at ($x_cord, $y_cord)"
	stdbuf -oL $build_path/cf2 $((19950+${n})) > out.log 2> error.log &
	popd &>/dev/null

	n=$(($n + 1))
done < "${spawn_list_dir}/${coordinates_file}"

sleep 1

trap "cleanup" SIGINT SIGTERM EXIT

# Start MuJoCo crazysim with all agents
echo "Starting MuJoCo CrazySim with ${n} agents, model_type=${model_type}"
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
	${DOWNWASH} \
	${FLOWDECK} \
	${WIND_ARGS} \
	${mass_arg} \
	${scene_arg} \
	-- ${spawn_args}
