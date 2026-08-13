$ErrorActionPreference = 'Stop'

$compiler = Get-Command gcc -ErrorAction Stop
$projectRoot = Split-Path -Parent $PSScriptRoot
$controlSource = Get-Content -Raw (Join-Path $projectRoot 'Control/control.c')
$gyroSource = Get-Content -Raw (Join-Path $projectRoot 'UART_Gyro/uart_gyro.c')
$syscfgSource = Get-Content -Raw (Join-Path $projectRoot 'main.syscfg')

$sourceChecks = [ordered]@{
    UartChecksFrameErrors = $gyroSource.Contains(
        'DL_UART_Main_getErrorStatus(UART_gyro_INST,')
    UartHandlesTimeout = $gyroSource.Contains(
        'DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR')
    UartChecksChecksum = $gyroSource.Contains(
        'IMU_FRAME_CHECKSUM_FAILED')
    SyscfgEnablesUartErrors = [regex]::IsMatch(
        $syscfgSource, `
        'UART2\.enabledInterrupts\s*=.*BREAK_ERROR.*FRAMING_ERROR.*NOISE_ERROR.*OVERRUN_ERROR.*PARITY_ERROR.*RX.*RX_TIMEOUT_ERROR')
    ControlUsesFreshnessGate = [regex]::IsMatch(
        $controlSource, `
        'uart_gyro_tick\(\);.*?if \(begin && !uart_gyro_is_fresh\(\)\)', `
        [System.Text.RegularExpressions.RegexOptions]::Singleline)
    ControlDoesNotOwnImuAngle = -not $controlSource.Contains('Angle[2]')
    ControlUsesWrappedCompletion = $controlSource.Contains(
        'control_heading_abs_error_degrees')
}
foreach ($check in $sourceChecks.GetEnumerator()) {
    if (-not $check.Value) {
        throw "IMU heading source check failed: $($check.Key)"
    }
}

$source = Join-Path $PSScriptRoot 'test_imu_heading.c.txt'
$output = Join-Path $env:TEMP (
    'mspm0-imu-heading-' + [guid]::NewGuid().ToString() + '.exe')

& $compiler.Source -x c -std=c11 -Wall -Wextra -Werror `
    -I $projectRoot $source -lm -o $output
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

& $output
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

Write-Output "Test binary retained at: $output"
