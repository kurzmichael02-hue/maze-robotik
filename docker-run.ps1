# startet container mit GUI support (gazebo, rviz) via VcXsrv
# voraussetzung: VcXsrv läuft mit "disable access control"

$projectRoot = $PSScriptRoot
$workspace = Join-Path $projectRoot "workspace"

if (-not (Test-Path $workspace)) {
    New-Item -ItemType Directory -Path $workspace | Out-Null
    New-Item -ItemType Directory -Path (Join-Path $workspace "src") | Out-Null
}

# auto-detect host ip für DISPLAY (vcxsrv läuft auf windows host)
$hostIp = (Get-NetIPAddress -AddressFamily IPv4 | Where-Object {$_.InterfaceAlias -like "*WSL*" -or $_.InterfaceAlias -like "*vEthernet*"} | Select-Object -First 1).IPAddress
if (-not $hostIp) { $hostIp = "host.docker.internal" }

Write-Host "Display target: ${hostIp}:0.0"

docker run -it --rm `
    --name maze_robotik `
    -e DISPLAY="${hostIp}:0.0" `
    -e LIBGL_ALWAYS_SOFTWARE=1 `
    -e MESA_GL_VERSION_OVERRIDE=3.3 `
    -e QT_X11_NO_MITSHM=1 `
    -v "${workspace}:/root/ros2_ws" `
    maze-robotik:latest
