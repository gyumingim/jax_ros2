#!/usr/bin/env bash
# Old name of `ros2 run jax_bringup zero` (jax_bringup/scripts/zero): AK45s to sim 0. THE ARM MOVES.
exec "$(cd "$(dirname "$0")/.." && pwd)/jax_bringup/scripts/zero" "$@"
