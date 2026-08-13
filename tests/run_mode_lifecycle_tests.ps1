$ErrorActionPreference = 'Stop'

$compiler = Get-Command gcc -ErrorAction Stop
$projectRoot = Split-Path -Parent $PSScriptRoot
$controlSource = Get-Content -Raw (Join-Path $projectRoot 'Control/control.c')
$encoderSource = Get-Content -Raw (Join-Path $projectRoot 'Encoder/encoder.c')

$sourceChecks = [ordered]@{
    LifecycleStepRunsInTimerIsr = $controlSource.Contains(
        'lifecycle_event = mode_lifecycle_step(&mode_lifecycle, mode, begin);')
    EnterResetsRuntime = [regex]::IsMatch(
        $controlSource, `
        'MODE_LIFECYCLE_ENTER\) \{\s*Control_ResetRuntime\(\);')
    AbortExitInvalidBrakeAndStop = [regex]::IsMatch(
        $controlSource, `
        'MODE_LIFECYCLE_EXIT.*?MODE_LIFECYCLE_ABORT.*?MODE_LIFECYCLE_INVALID.*?\) \{\s*brake\(\);\s*Control_ResetRuntime\(\);\s*begin = 0u;', `
        [System.Text.RegularExpressions.RegexOptions]::Singleline)
    InvalidModeResetsSelection = $controlSource.Contains(
        'if (lifecycle_event == MODE_LIFECYCLE_INVALID) {')
    RuntimeResetClearsModeTwo = $controlSource.Contains('mode2_bee = 0u;')
    RuntimeResetClearsModeThree = $controlSource.Contains('mode3_1 = 0u;')
    RuntimeResetClearsModeFour = $controlSource.Contains('mode4_circle_nums = 0u;')
    RuntimeResetClearsDistance = $controlSource.Contains('path_line = 0u;')
    RuntimeResetClearsSensorStatus = [regex]::IsMatch(
        $controlSource, 'huidu_read_status = 1;.*?mode4_huidu_read_status = 1;', `
        [System.Text.RegularExpressions.RegexOptions]::Singleline)
    RuntimeResetClearsSensorSums = [regex]::IsMatch(
        $controlSource, 'huidu_data_sum = 0;.*?mode4_huidu_data_sum = 0;', `
        [System.Text.RegularExpressions.RegexOptions]::Singleline)
    RuntimeResetClearsSensorArrays = [regex]::IsMatch(
        $controlSource, 'for \(sensor_index = 0u; sensor_index < 8u; \+\+sensor_index\) \{\s*huidu_data\[sensor_index\] = 0;\s*mode4_huidu_data\[sensor_index\] = 0;\s*\}')
    RuntimeResetClearsPidState = $controlSource.Contains(
        'position_pid_state = (ControlPidState){0};')
    EncoderResetExists = $encoderSource.Contains('void encoder_reset(void)')
    EncoderReadUsesReset = $encoderSource.Contains('encoder_reset();')
}
foreach ($check in $sourceChecks.GetEnumerator()) {
    if (-not $check.Value) {
        throw "Mode lifecycle source check failed: $($check.Key)"
    }
}

$source = Join-Path $PSScriptRoot 'test_mode_lifecycle.c.txt'
$output = Join-Path $env:TEMP (
    'mspm0-mode-lifecycle-' + [guid]::NewGuid().ToString() + '.exe')

& $compiler.Source -x c -std=c11 -Wall -Wextra -Werror `
    -I $projectRoot $source -o $output
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

& $output
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

Write-Output "Test binary retained at: $output"
