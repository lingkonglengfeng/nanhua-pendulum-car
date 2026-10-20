param([switch]$NoBrowser)
$ErrorActionPreference = 'Stop'
$taskRoot = $PSScriptRoot
$taskPort = $null
foreach ($taskCandidate in 8768..8788) {
    $taskSocket = [Net.Sockets.TcpClient]::new()
    $taskBusy = $false
    try { $taskPending=$taskSocket.ConnectAsync('127.0.0.1',$taskCandidate); $taskBusy=$taskPending.Wait(250) -and $taskSocket.Connected } catch {} finally { $taskSocket.Dispose() }
    if (!$taskBusy) { $taskPort=$taskCandidate; break }
    try {
        $taskInfo=Invoke-RestMethod -Uri "http://127.0.0.1:$taskCandidate/monitor-info.json" -TimeoutSec 1
        if ($taskInfo.id -eq 'f103-as5600-monitor-v1') { $taskPort=$taskCandidate; break }
    } catch {}
}
if ($null -eq $taskPort) { throw 'No free local monitor port in 8768..8788.' }
$taskRunning=$false
try { $taskInfo=Invoke-RestMethod -Uri "http://127.0.0.1:$taskPort/monitor-info.json" -TimeoutSec 1; $taskRunning=$taskInfo.id -eq 'f103-as5600-monitor-v1' } catch {}
if (!$taskRunning) {
    $taskPython='D:\blance_car\.cad-venv\Scripts\python.exe'
    if (!(Test-Path -LiteralPath $taskPython)) {
        $taskRuntime=Get-Command python.exe -ErrorAction SilentlyContinue
        if (!$taskRuntime) { throw 'Python runtime unavailable. Open as5600-monitor.html directly in Edge or Chrome.' }
        $taskPython=$taskRuntime.Source
    }
    $taskArgs=@('-m','http.server',[string]$taskPort,'--bind','127.0.0.1','--directory',('"'+$taskRoot+'"'))
    $taskProcess=Start-Process -FilePath $taskPython -ArgumentList $taskArgs -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $taskRoot 'server-output.log') -RedirectStandardError (Join-Path $taskRoot 'server-error.log')
    [IO.File]::WriteAllText((Join-Path $taskRoot 'server.pid'),[string]$taskProcess.Id)
    $taskReady=$false
    for ($taskAttempt=0;$taskAttempt -lt 30;$taskAttempt++) {
        try { $taskInfo=Invoke-RestMethod -Uri "http://127.0.0.1:$taskPort/monitor-info.json" -TimeoutSec 1; if ($taskInfo.id -eq 'f103-as5600-monitor-v1') { $taskReady=$true;break } } catch {}
        Start-Sleep -Milliseconds 100
    }
    if (!$taskReady) { throw 'Local monitor server did not become ready.' }
}
$taskUrl="http://127.0.0.1:$taskPort/as5600-monitor.html"
[IO.File]::WriteAllText((Join-Path $taskRoot 'last-url.txt'),$taskUrl)
if (!$NoBrowser) {
    $taskEdge=@("${env:ProgramFiles(x86)}\Microsoft\Edge\Application\msedge.exe","$env:ProgramFiles\Microsoft\Edge\Application\msedge.exe") | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
    if ($taskEdge) { Start-Process -FilePath $taskEdge -ArgumentList @($taskUrl) | Out-Null }
    else { Start-Process $taskUrl | Out-Null }
}
Write-Output $taskUrl
