$ErrorActionPreference = 'Stop'

$compiler = Get-Command gcc -ErrorAction Stop
$projectRoot = Split-Path -Parent $PSScriptRoot
$controlSource = Get-Content -Raw (Join-Path $projectRoot 'Control/control.c')

$sourceChecks = [ordered]@{
    SchedulerHeaderIncluded = $controlSource.Contains(
        '#include "control_scheduler.h"')
    LoadUsesTransaction = $controlSource.Contains(
        'control_scheduler_load((motor1), (motor2), (motor3), (motor4))')
    BrakeLocksTransaction = $controlSource.Contains(
        '#define brake() control_scheduler_brake()')
    CommitUsesUnderlyingMotorApi = $controlSource.Contains(
        '(Load)(control_scheduler_output.motor1') -and $controlSource.Contains(
        '(brake)();')
    TickInitializesTransaction = $controlSource.Contains(
        'control_scheduler_init(&control_scheduler_output);')
    NormalTickCommitsOutput = $controlSource.IndexOf(
        'control_scheduler_commit_output();') -lt $controlSource.IndexOf('default:')
    QueuedSignalCommitsBrake = $controlSource.Contains(
        'if (!signal_state_is_idle(&signal_state)) {') -and
        $controlSource.Contains('control_scheduler_commit_output();')
    StaleImuCommitsBrake = $controlSource.Contains(
        'if (begin && !uart_gyro_is_fresh()) {') -and
        $controlSource.Contains('control_scheduler_commit_output();')
    Mode4MicroTurnUsesFourDegreeTarget = $controlSource.Contains(
        'control_heading(), mode4_angle1_1 + 4.0f) <= 0.5f')
}
foreach ($check in $sourceChecks.GetEnumerator()) {
    if (-not $check.Value) {
        throw "Scheduler source check failed: $($check.Key)"
    }
}

$source = Join-Path $PSScriptRoot 'test_control_scheduler.c.txt'
$output = Join-Path $env:TEMP (
    'mspm0-control-scheduler-' + [guid]::NewGuid().ToString() + '.exe')
& $compiler.Source -x c -std=c11 -Wall -Wextra -Werror `
    -I $projectRoot $source -o $output
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $output
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
Write-Output "Test binary retained at: $output"
