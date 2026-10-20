$ErrorActionPreference = 'Stop'
$taskProjectDir = Split-Path -Parent $PSScriptRoot
$taskOutDir = $PSScriptRoot
$taskCompilerDir = 'C:\Keil_v5\ARM\ARMCC\bin'
[xml]$taskProject = Get-Content -LiteralPath (Join-Path $taskProjectDir 'f103_bringup.uvprojx') -Raw
$taskTarget = $taskProject.Project.Targets.Target
$taskIncludeFlags = @($taskTarget.TargetOption.TargetArmAds.Cads.VariousControls.IncludePath.Split(';') | ForEach-Object { '-I' + [IO.Path]::GetFullPath((Join-Path $taskProjectDir $_)) })
$taskDefineFlags = @($taskTarget.TargetOption.TargetArmAds.Cads.VariousControls.Define.Split(',') | ForEach-Object { '-D' + $_ })
$taskObjects = @()
# Keil also permits header entries in groups; only C/assembly are compiled.
$taskFiles = @($taskTarget.Groups.Group.Files.File | Where-Object { ([string]$_.FilePath) -match '\.(c|s)$' })
foreach ($taskFile in $taskFiles) {
    $taskSource = [IO.Path]::GetFullPath((Join-Path $taskProjectDir ([string]$taskFile.FilePath)))
    $taskObject = Join-Path $taskOutDir ([IO.Path]::GetFileNameWithoutExtension($taskSource) + '.o')
    if ($taskSource.EndsWith('.c')) {
        $taskOptions = @('--cpu','Cortex-M3','--c99','--apcs=interwork','-O3','--split_sections','--debug','--diag_error=warning') + $taskDefineFlags + $taskIncludeFlags + @('-c',$taskSource,'-o',$taskObject)
        & (Join-Path $taskCompilerDir 'armcc.exe') @taskOptions
    } elseif ($taskSource.EndsWith('.s')) {
        & (Join-Path $taskCompilerDir 'armasm.exe') --cpu Cortex-M3 --debug $taskSource -o $taskObject
    } else { throw "Unsupported source $taskSource" }
    if ($LASTEXITCODE -ne 0) { throw "Build failed: $taskSource" }
    $taskObjects += $taskObject
}
$taskAxf = Join-Path $taskOutDir 'f103_speed_pid.axf'
$taskMap = Join-Path $taskOutDir 'f103_speed_pid.map'
$taskHex = Join-Path $taskOutDir 'f103_speed_pid.hex'
$taskScatter = Join-Path $taskProjectDir 'f103_bringup\f103_bringup.sct'
$taskLinkOptions = @('--cpu','Cortex-M3','--strict','--scatter',$taskScatter,'--summary_stderr','--info','summarysizes','--map','--symbols','--callgraph','--info','sizes','--info','totals','--list',$taskMap,'-o',$taskAxf) + $taskObjects
& (Join-Path $taskCompilerDir 'armlink.exe') @taskLinkOptions
if ($LASTEXITCODE -ne 0) { throw 'Link failed' }
& (Join-Path $taskCompilerDir 'fromelf.exe') --i32combined --output $taskHex $taskAxf
if ($LASTEXITCODE -ne 0) { throw 'HEX generation failed' }
# Keep the normal Keil download/debug paths aligned with this source version.
foreach ($taskExtension in @('axf','hex')) {
    $taskNormalOutput = Join-Path $taskProjectDir ("f103_bringup\f103_bringup." + $taskExtension)
    $taskBackup = Join-Path $taskOutDir ("backup\before_speed_pid." + $taskExtension)
    if ((Test-Path -LiteralPath $taskNormalOutput) -and !(Test-Path -LiteralPath $taskBackup)) {
        Copy-Item -LiteralPath $taskNormalOutput -Destination $taskBackup
    }
    Copy-Item -LiteralPath (Join-Path $taskOutDir ("f103_speed_pid." + $taskExtension)) -Destination $taskNormalOutput
}
Write-Output "Built $($taskFiles.Count) source files with warnings treated as errors."
Get-Item -LiteralPath $taskAxf,$taskHex | Select-Object Name,Length,LastWriteTime
