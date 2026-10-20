$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$taskTestDir = Join-Path $PSScriptRoot 'host_tests\balance'
New-Item -ItemType Directory -Force -Path $taskTestDir | Out-Null
$taskVcvars = & 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe' -utf8 -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -find VC\Auxiliary\Build\vcvars64.bat
if (!$taskVcvars) { throw 'MSVC unavailable' }
$taskEnvironmentLines = & cmd.exe /d /c ('call "'+$taskVcvars+'" >nul && set')
foreach ($taskEnvironmentLine in $taskEnvironmentLines) {
    $taskSeparator=$taskEnvironmentLine.IndexOf('=')
    if ($taskSeparator -gt 0) { [Environment]::SetEnvironmentVariable($taskEnvironmentLine.Substring(0,$taskSeparator),$taskEnvironmentLine.Substring($taskSeparator+1),'Process') }
}
& cl.exe /nologo /W3 /WX /utf-8 /TC /std:c11 ('/I'+(Join-Path $taskRoot 'Core\Inc')) (Join-Path $PSScriptRoot 'test_balance.c') (Join-Path $taskRoot 'Core\Src\balance_control.c') (Join-Path $taskRoot 'Core\Src\pendulum_angle.c') ('/Fo:'+$taskTestDir+'\') ('/Fe:'+(Join-Path $taskTestDir 'balance_test.exe'))
if ($LASTEXITCODE -ne 0) { throw 'Balance host compile failed' }
& (Join-Path $taskTestDir 'balance_test.exe')
if ($LASTEXITCODE -ne 0) { throw 'Balance host regression failed' }
& cl.exe /nologo /W3 /WX /utf-8 /TC /std:c11 ('/I'+(Join-Path $taskRoot 'Core\Inc')) (Join-Path $PSScriptRoot 'test_balance_velocity.c') (Join-Path $taskRoot 'Core\Src\balance_control.c') ('/Fo:'+$taskTestDir+'\') ('/Fe:'+(Join-Path $taskTestDir 'balance_velocity_test.exe'))
if ($LASTEXITCODE -ne 0) { throw 'Balance velocity host compile failed' }
& (Join-Path $taskTestDir 'balance_velocity_test.exe')
if ($LASTEXITCODE -ne 0) { throw 'Balance velocity host regression failed' }
& cl.exe /nologo /W3 /WX /utf-8 /TC /std:c11 ('/I'+(Join-Path $taskRoot 'Core\Inc')) (Join-Path $PSScriptRoot 'test_balance_motion.c') (Join-Path $taskRoot 'Core\Src\balance_control.c') ('/Fo:'+$taskTestDir+'\') ('/Fe:'+(Join-Path $taskTestDir 'balance_motion_test.exe'))
if ($LASTEXITCODE -ne 0) { throw 'Balance motion host compile failed' }
& (Join-Path $taskTestDir 'balance_motion_test.exe')
if ($LASTEXITCODE -ne 0) { throw 'Balance motion regression failed' }
& cl.exe /nologo /W3 /WX /utf-8 /TC /std:c11 ('/I'+(Join-Path $taskRoot 'Core\Inc')) (Join-Path $PSScriptRoot 'test_balance_position.c') (Join-Path $taskRoot 'Core\Src\balance_control.c') ('/Fo:'+$taskTestDir+'\') ('/Fe:'+(Join-Path $taskTestDir 'balance_position_test.exe'))
if ($LASTEXITCODE -ne 0) { throw 'Position hold host compile failed' }
& (Join-Path $taskTestDir 'balance_position_test.exe')
if ($LASTEXITCODE -ne 0) { throw 'Position hold regression failed' }
& cl.exe /nologo /W3 /WX /utf-8 /TC /std:c11 ('/I'+(Join-Path $taskRoot 'Core\Inc')) (Join-Path $PSScriptRoot 'test_balance_exit.c') (Join-Path $taskRoot 'Core\Src\balance_control.c') ('/Fo:'+$taskTestDir+'\') ('/Fe:'+(Join-Path $taskTestDir 'balance_exit_test.exe'))
if ($LASTEXITCODE -ne 0) { throw 'Exit diagnostic host compile failed' }
& (Join-Path $taskTestDir 'balance_exit_test.exe')
if ($LASTEXITCODE -ne 0) { throw 'Exit diagnostic regression failed' }
& cl.exe /nologo /W3 /WX /utf-8 /TC /std:c11 ('/I'+(Join-Path $taskRoot 'Core\Inc')) (Join-Path $PSScriptRoot 'test_balance_spin.c') (Join-Path $taskRoot 'Core\Src\balance_control.c') ('/Fo:'+$taskTestDir+'\') ('/Fe:'+(Join-Path $taskTestDir 'balance_spin_test.exe'))
if ($LASTEXITCODE -ne 0) { throw 'Spin hold host compile failed' }
& (Join-Path $taskTestDir 'balance_spin_test.exe')
if ($LASTEXITCODE -ne 0) { throw 'Spin hold regression failed' }
& cl.exe /nologo /W3 /WX /utf-8 /TC /std:c11 ('/I'+(Join-Path $taskRoot 'Core\Inc')) (Join-Path $PSScriptRoot 'test_balance_swing.c') (Join-Path $taskRoot 'Core\Src\balance_control.c') ('/Fo:'+$taskTestDir+'\') ('/Fe:'+(Join-Path $taskTestDir 'balance_swing_test.exe'))
if ($LASTEXITCODE -ne 0) { throw 'Swing-up host compile failed' }
& (Join-Path $taskTestDir 'balance_swing_test.exe')
if ($LASTEXITCODE -ne 0) { throw 'Swing-up regression failed' }
& cl.exe /nologo /W3 /WX /utf-8 /TC /std:c11 ('/I'+(Join-Path $taskRoot 'Core\Inc')) (Join-Path $PSScriptRoot 'test_balance_step.c') (Join-Path $taskRoot 'Core\Src\balance_control.c') ('/Fo:'+$taskTestDir+'\') ('/Fe:'+(Join-Path $taskTestDir 'balance_step_test.exe'))
if ($LASTEXITCODE -ne 0) { throw 'Metre sequence host compile failed' }
& (Join-Path $taskTestDir 'balance_step_test.exe')
if ($LASTEXITCODE -ne 0) { throw 'Metre sequence regression failed' }
