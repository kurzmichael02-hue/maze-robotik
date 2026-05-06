# startet container mit GUI support (gazebo, rviz) via WSLg
# der workspace-ordner wird ins container gemounted, dh deine files bleiben erhalten

$projectRoot = $PSScriptRoot
$workspace = Join-Path $projectRoot "workspace"

if (-not (Test-Path $workspace)) {
    New-Item -ItemType Directory -Path $workspace | Out-Null
    New-Item -ItemType Directory -Path (Join-Path $workspace "src") | Out-Null
}

docker run -it --rm `
    --name maze_robotik `
    --net=host `
    -e DISPLAY=:0 `
    -e WAYLAND_DISPLAY=wayland-0 `
    -e XDG_RUNTIME_DIR=/tmp `
    -e PULSE_SERVER=unix:/mnt/wslg/PulseServer `
    -e LIBGL_ALWAYS_SOFTWARE=1 `
    -v /tmp/.X11-unix:/tmp/.X11-unix `
    -v /mnt/wslg:/mnt/wslg `
    -v "${workspace}:/root/ros2_ws" `
    maze-robotik:latest
