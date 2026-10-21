#!/usr/bin/env bash
# Start the billet cell in Isaac Sim 6 (pip install in ~/isaacsim-env-6).
#
#   run_isaac.sh [--headless]
#
# 1. With ROS 2 sourced: expand the robot xacro (hw:=isaac) and make mesh paths absolute,
#    since Isaac's URDF importer reads plain URDF only.
# 2. In a clean environment (no /opt/ros: Isaac runs Python 3.12, Humble is 3.10): point
#    Isaac's ROS 2 bridge at its bundled Humble libraries and run isaac/billet_cell.py.
# ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY are passed through: use the same values as the ROS side.
set -euo pipefail

ISAAC_ENV="${ISAAC_ENV:-$HOME/isaacsim-env-6}"
ROS_SETUP="${ROS_SETUP:-$HOME/ws_fanuc/install/setup.bash}"
WORK="${XDG_CACHE_HOME:-$HOME/.cache}/fanuc_pick_drop/isaac"
mkdir -p "$WORK"

read -r SHARE < <(
  set +u
  source "$ROS_SETUP"
  share=$(ros2 pkg prefix --share fanuc_pick_drop)
  sed_args=()
  for pkg in $(xacro "$share/urdf/crx10ia_l_schmalz.urdf.xacro" hw:=isaac | grep -o 'package://[^/]*' | sort -u); do
    name=${pkg#package://}
    sed_args+=(-e "s#package://$name#$(ros2 pkg prefix --share "$name")#g")
  done
  xacro "$share/urdf/crx10ia_l_schmalz.urdf.xacro" hw:=isaac | sed "${sed_args[@]}" > "$WORK/crx10ia_l_schmalz.urdf"
  echo "$share"
)

ROS_LIBS=$(echo "$ISAAC_ENV"/lib/python3*/site-packages/isaacsim/exts/isaacsim.ros2.core/humble/lib)
exec env -i HOME="$HOME" USER="${USER:-}" PATH=/usr/local/bin:/usr/bin:/bin \
  DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" \
  XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
  OMNI_KIT_ACCEPT_EULA="${OMNI_KIT_ACCEPT_EULA:-YES}" \
  ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp LD_LIBRARY_PATH="$ROS_LIBS" \
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" \
  "$ISAAC_ENV/bin/python" "$SHARE/isaac/billet_cell.py" \
  --urdf "$WORK/crx10ia_l_schmalz.urdf" --share "$SHARE" --cache "$WORK" "$@"
