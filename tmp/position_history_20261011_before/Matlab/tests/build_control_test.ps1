param(
    [string]$KeilRoot = 'D:\Applications\Keil',
    [string]$Python = 'python'
)
$ErrorActionPreference = 'Stop'
$taskRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$taskOutput = Join-Path $taskRoot 'tmp\control_audit_build'
New-Item -ItemType Directory -Path $taskOutput -Force | Out-Null
[xml]$taskProject = Get-Content -LiteralPath (Join-Path $taskRoot 'MDK-ARM\WheelLeg.uvprojx')
$taskControls = $taskProject.SelectSingleNode('//TargetArmAds/Cads/VariousControls')
$taskProjectDir = Join-Path $taskRoot 'MDK-ARM'
$taskIncludes = @($taskControls.IncludePath.Split(';') | Where-Object { $_ } | ForEach-Object {
    '-I' + [IO.Path]::GetFullPath((Join-Path $taskProjectDir $_))
})
$taskDefines = @($taskControls.Define.Split(',') | Where-Object { $_ } | ForEach-Object { '-D' + $_.Trim() })
$taskBin = Join-Path $KeilRoot 'ARM\ARMCC\Bin'
$taskObjects = @()
foreach ($taskSource in @('Algorithm\balance.c','Algorithm\VMC.c',
                          'Kinematics\forward_solution.c','Matlab\tests\control_pair_test.c',
                          'Drivers\CMSIS\DSP\Source\FastMathFunctions\arm_sin_f32.c',
                          'Drivers\CMSIS\DSP\Source\FastMathFunctions\arm_cos_f32.c',
                          'Drivers\CMSIS\DSP\Source\CommonTables\arm_common_tables.c')) {
    $taskObject = Join-Path $taskOutput ([IO.Path]::GetFileNameWithoutExtension($taskSource) + '.o')
    & (Join-Path $taskBin 'armcc.exe') '--cpu=Cortex-M7.fp.dp' '--c99' '--gnu' '-g' '-O0' `
        '--apcs=interwork' '--split_sections' '--diag_suppress=1041' @taskIncludes @taskDefines `
        '-c' (Join-Path $taskRoot $taskSource) '-o' $taskObject
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $taskSource" }
    $taskObjects += $taskObject
}
$taskStartupObject = Join-Path $taskOutput 'control_test_startup.o'
& (Join-Path $taskBin 'armasm.exe') '--cpu=Cortex-M7.fp.dp' `
    (Join-Path $PSScriptRoot 'control_test_startup.s') '-o' $taskStartupObject
if ($LASTEXITCODE -ne 0) { throw 'Test startup assembly failed' }
$taskImage = Join-Path $taskOutput 'control_pair_test.axf'
& (Join-Path $taskBin 'armlink.exe') '--cpu=Cortex-M7.fp.dp' '--entry=Reset_Handler' `
    '--scatter' (Join-Path $PSScriptRoot 'control_test.sct') '--output' $taskImage `
    $taskStartupObject @taskObjects
if ($LASTEXITCODE -ne 0) { throw 'Offline test link failed' }
& $Python (Join-Path $PSScriptRoot 'run_arm_control_test.py') $taskImage
if ($LASTEXITCODE -ne 0) { throw 'Offline control test failed' }
