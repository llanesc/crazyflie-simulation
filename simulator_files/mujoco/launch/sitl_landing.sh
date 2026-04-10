#!/bin/bash
# Source ROS2 if available (needed for rover ROS2 bridge)
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash

# Use Cyclone DDS for cross-distro compatibility (Humble ↔ Jazzy) unless overridden
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

# Launch single Crazyflie SITL + X3 rover for landing experiments.
#
# Usage: ./sitl_landing.sh [-m <model_type>] [-d <dt>] [-M <mass_kg>]
#        [--rover-pos X,Y] [--rover-heading <rad>]
#        [--sensor-noise] [--ground-effect]
#        [--wind-speed <m/s>] [--turbulence <level>]

function cleanup() {
	pkill -x cf2
}

# Defaults
MODEL_TYPE="cf21B_500"
DRONE_POS="0,0"
DT=0.001
MASS=""
ROVER_POS="1.5,0"
ROVER_HEADING=0
ROVER_MIRROR=""
VIS="--vis"
SENSOR_NOISE=""
GROUND_EFFECT=""
WIND_ARGS=""

# Parse args
POSITIONAL=()
while [[ $# -gt 0 ]]; do
	case "$1" in
		-m) MODEL_TYPE="$2"; shift 2;;
		--drone-pos) DRONE_POS="$2"; shift 2;;
		-d) DT="$2"; shift 2;;
		-M) MASS="$2"; shift 2;;
		--rover-pos) ROVER_POS="$2"; shift 2;;
		--rover-heading) ROVER_HEADING="$2"; shift 2;;
		--rover-mirror) ROVER_MIRROR="--rover-mirror"; shift;;
		--no-vis) VIS=""; shift;;
		--sensor-noise) SENSOR_NOISE="--sensor-noise"; shift;;
		--ground-effect) GROUND_EFFECT="--ground-effect"; shift;;
		--wind-speed) WIND_ARGS="$WIND_ARGS --wind-speed $2"; shift 2;;
		--wind-direction) WIND_ARGS="$WIND_ARGS --wind-direction $2"; shift 2;;
		--gust-intensity) WIND_ARGS="$WIND_ARGS --gust-intensity $2"; shift 2;;
		--turbulence) WIND_ARGS="$WIND_ARGS --turbulence $2"; shift 2;;
		-h|--help)
			echo "Launch single Crazyflie + X3 rover for landing experiments."
			echo "Usage: $0 [-m model] [-d dt] [-M mass] [--drone-pos X,Y] [--rover-pos X,Y] [--rover-heading rad]"
			echo ""
			echo "Options:"
			echo "  -m MODEL_TYPE       Drone model (default: cf21B_500)"
			echo "  -d DT               Physics timestep (default: 0.001)"
			echo "  -M MASS             Override drone mass [kg]"
			echo "  --drone-pos X,Y     Drone spawn position (default: 0,0)"
			echo "  --rover-pos X,Y     Rover spawn position (default: 1.5,0)"
			echo "  --rover-heading RAD  Rover initial heading (default: 0)"
			echo "  --sensor-noise      Enable IMU noise model"
			echo "  --ground-effect     Enable ground effect"
			echo "  --wind-speed M/S    Constant wind"
			echo "  --turbulence LEVEL  none/light/moderate/severe"
			exit 0;;
		*) POSITIONAL+=("$1"); shift;;
	esac
done

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

# Build crazysim args
mass_arg=""
[ -n "${MASS}" ] && mass_arg="--mass ${MASS}"

echo "Starting MuJoCo CrazySim with drone=${MODEL_TYPE} + X3 rover at ${ROVER_POS}"
python3 "$crazysim_dir/crazysim.py" \
	--model-type "${MODEL_TYPE}" \
	--port 19950 \
	${VIS} \
	--dt "${DT}" \
	--rover-model rosmaster_x3.xml \
	--rover-pos "${ROVER_POS}" \
	--rover-heading "${ROVER_HEADING}" \
	${ROVER_MIRROR} \
	${SENSOR_NOISE} \
	${GROUND_EFFECT} \
	${WIND_ARGS} \
	${mass_arg} \
	-- "${DRONE_POS}"
