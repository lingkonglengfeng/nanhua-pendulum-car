$ErrorActionPreference = 'Stop'
$taskProjectDir = Split-Path -Parent $PSScriptRoot
$taskOutDir = $PSScriptRoot
$taskCompilerDir = 'C:\Keil_v5\ARM\ARMCC\bin'
[xml]$taskProject = Get-Content -LiteralPath (Join-Path $taskProjectDir 'f103_bringup.uvprojx') -Raw
$taskTarget = $taskProject.Project.Targets.Target
$taskIncludeFlags = @($taskTarget.TargetOption.TargetArmAds.Cads.VariousControls.IncludePath.Split(';') | ForEach-Object { '-I' + [IO.Path]::GetFullPath((Join-Path $taskProjectDir $_)) })
$taskDefineFlags = @($taskTarget.TargetOption.TargetArmAds.Cads.VariousControls.Define.Split(',') | ForEach-Object { '-D' + $_ })
$taskObjects = @()
$taskFiles = @($taskTarget.Groups.Group.Files.File)
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
$taskAxf = Join-Path $taskOutDir 'f103_wheel_sequence.axf'
$taskMap = Join-Path $taskOutDir 'f103_wheel_sequence.map'
$taskHex = Join-Path $taskOutDir 'f103_wheel_sequence.hex'
$taskScatter = Join-Path $taskProjectDir 'f103_bringup\f103_bringup.sct'
$taskLinkOptions = @('--cpu','Cortex-M3','--strict','--scatter',$taskScatter,'--summary_stderr','--info','summarysizes','--map','--symbols','--info','sizes','--info','totals','--list',$taskMap,'-o',$taskAxf) + $taskObjects
& (Join-Path $taskCompilerDir 'armlink.exe') @taskLinkOptions
if ($LASTEXITCODE -ne 0) { throw 'Link failed' }
& (Join-Path $taskCompilerDir 'fromelf.exe') --i32combined --output $taskHex $taskAxf
if ($LASTEXITCODE -ne 0) { throw 'HEX generation failed' }
Write-Output "Built $($taskFiles.Count) source files with warnings treated as errors."
Get-Item -LiteralPath $taskAxf,$taskHex | Select-Object Name,Length,LastWriteTime
