#!/usr/bin/env bash
# Old name of `ros2 run jax_bringup up` (jax_bringup/scripts/up):
#   tools/bringup.sh camera [name:=value ...]            -> up [name:=value ...]
#   tools/bringup.sh fixed "[x, y, yaw]" [name:=value]   -> up "[x, y, yaw]" [name:=value]
# BRINGUP_YES=1 / BRINGUP_CAN=skip / BRINGUP_SETUP still work (JAX_YES / JAX_CAN / JAX_SETUP).
up="$(cd "$(dirname "$0")/.." && pwd)/jax_bringup/scripts/up"
export JAX_YES="${JAX_YES:-${BRINGUP_YES:-}}" JAX_CAN="${JAX_CAN:-${BRINGUP_CAN:-}}"
[ -n "${BRINGUP_SETUP:-}" ] && export JAX_SETUP="$BRINGUP_SETUP"
case "${1:-}" in
    camera) shift; exec "$up" "$@" ;;
    fixed) shift; [ $# -ge 1 ] || { echo "usage: tools/bringup.sh fixed \"[x, y, yaw]\""; exit 2; }; exec "$up" "$@" ;;
    *) sed -n '2,4p' "$0" | sed 's/^# //'; exit 2 ;;
esac
