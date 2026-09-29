#!/usr/bin/env bash
set -e
source "$RCLNET_SETUP"
while IFS= read -r overlay; do
    if [ -n "$overlay" ]; then
        source "$overlay"
    fi
done < "$RCLNET_OVERLAYS"
export RMW_IMPLEMENTATION="$RCLNET_RMW"
exec "$RCLNET_DOTNET" "$(dirname "$0")/Rcl.NET.TestWorker.dll" "$RCLNET_REQUEST"
