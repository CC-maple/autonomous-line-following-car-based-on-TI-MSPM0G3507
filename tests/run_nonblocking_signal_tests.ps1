$ErrorActionPreference = 'Stop'

$compiler = Get-Command gcc -ErrorAction Stop
$projectRoot = Split-Path -Parent $PSScriptRoot
$controlSource = Get-Content -Raw (Join-Path $projectRoot 'Control/control.c')
$syscfgSource = Get-Content -Raw (Join-Path $projectRoot 'main.syscfg')

$sourceChecks = [ordered]@{
    TimerPeriodIs20ms = $syscfgSource.Contains('TIMER1.timerPeriod        = "20ms";')
    SignalDurationIs25Ticks = $controlSource.Contains(
        '#define SIGNAL_DURATION_TICKS 25u')
    ControlHasNoBlockingSignalDelay = -not $controlSource.Contains(
        'delay_cycles(16000000)')
    QueuedSignalsBrakeAndSkipModeControl = [regex]::IsMatch(
        $controlSource, `
        'case DL_TIMER_IIDX_ZERO:\s*Sign_LED_Bee_Tick\(\);.*?if \(!signal_state_is_idle\(&signal_state\)\) \{\s*brake\(\);\s*break;\s*\}', `
        [System.Text.RegularExpressions.RegexOptions]::Singleline)
    SignalGateStillPrecedesModeControl = [regex]::IsMatch(
        $controlSource, `
        'if \(!signal_state_is_idle\(&signal_state\)\).*?break;.*?if\(begin\)', `
        [System.Text.RegularExpressions.RegexOptions]::Singleline)
    SignalRequestsUseQueue = $controlSource.Contains(
        'signal_state_request(&signal_state, SIGNAL_DURATION_TICKS)')
}
foreach ($check in $sourceChecks.GetEnumerator()) {
    if (-not $check.Value) {
        throw "Nonblocking signal source check failed: $($check.Key)"
    }
}

$source = Join-Path $PSScriptRoot 'test_nonblocking_signals.c.txt'
$output = Join-Path $env:TEMP (
    'mspm0-nonblocking-signals-' + [guid]::NewGuid().ToString() + '.exe')

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
