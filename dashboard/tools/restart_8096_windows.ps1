# 8096 대시보드(noros, 가상 모드) 다시 띄우기 — 2026-11-20
Get-NetTCPConnection -LocalPort 8096 -State Listen -ErrorAction SilentlyContinue | ForEach-Object { Stop-Process -Id $_.OwningProcess -Force -ErrorAction SilentlyContinue }
Start-Sleep -Milliseconds 800
$env:JAX_IK_DIR = "C:\Users\a3162\Documents\GitHub\IK"
$env:JAX_DASH_PYTHONPATH = "C:\Users\a3162\Documents\GitHub\IK\.pydeps"
$log = "C:\Users\a3162\Documents\GitHub\jax_ros2_noros\dashboard\logs"
New-Item -ItemType Directory -Force $log | Out-Null
$p = Start-Process -WindowStyle Hidden -PassThru -FilePath python -ArgumentList @("dashboard\server.py","--port","8096","--virtual") -WorkingDirectory "C:\Users\a3162\Documents\GitHub\jax_ros2_noros" -RedirectStandardOutput "$log\s8096.out" -RedirectStandardError "$log\s8096.err"
Start-Sleep -Seconds 4
"pid $($p.Id)"
