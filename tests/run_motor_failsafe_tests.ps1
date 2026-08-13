$ErrorActionPreference = 'Stop'

$compiler = Get-Command gcc -ErrorAction Stop
$projectRoot = Split-Path -Parent $PSScriptRoot
$controlSource = Get-Content -Raw (Join-Path $projectRoot 'Control/control.c')
$motorSource = Get-Content -Raw (Join-Path $projectRoot 'Motor/motor.c')

$sourceChecks = [ordered]@{
    BalancedBraces = ([regex]::Matches($controlSource, '\{')).Count -eq `
        ([regex]::Matches($controlSource, '\}')).Count
    InvalidActiveModeBrakes = [regex]::IsMatch(
        $controlSource, 'else \{\s*brake\(\);\s*begin=0')
    InvalidIdleModeBrakes = [regex]::IsMatch(
        $controlSource, 'if\(mode>7\) \{\s*brake\(\);\s*mode=0')
    BrakeClearsFourPwmChannels = `
        ([regex]::Matches($motorSource, `
        'DL_TimerG_setCaptureCompareValue\(TIMA0, 0')).Count -eq 4
    Mode2TransitionDoesNotFallThrough = $controlSource.Contains(
        'else if (mode2_1==0&&mode2_2==1)')
    Mode2SignalCycleDoesNotDrive = [regex]::IsMatch(
        $controlSource, `
        'if \(mode2_bee==0\).*?Sign_LED_Bee\(\);.*?mode2_bee=1;.*?\}\s*else \{', `
        [System.Text.RegularExpressions.RegexOptions]::Singleline)
    Mode2TurnCompletionBrakes = [regex]::IsMatch(
        $controlSource, `
        'Angle\[2\] - 179.*?\) \{\s*brake\(\);\s*mode2_1=1,mode2_2=1;.*?\}\s*else \{\s*Load', `
        [System.Text.RegularExpressions.RegexOptions]::Singleline)
    Mode3TurnCompletionBrakes = [regex]::IsMatch(
        $controlSource, `
        'if \(angle_error <= 1u\) \{\s*brake\(\);.*?\}\s*else \{\s*Load', `
        [System.Text.RegularExpressions.RegexOptions]::Singleline)
    TerminalDriveGuards = `
        ([regex]::Matches($controlSource, `
        'if \(huidu_data_sum!=6\)')).Count -ge 3
}
foreach ($check in $sourceChecks.GetEnumerator()) {
    if (-not $check.Value) {
        throw "Failsafe source check failed: $($check.Key)"
    }
}

$mockRoot = Join-Path $env:TEMP ('mspm0-motor-mock-' + [guid]::NewGuid().ToString())
$mockDriverLib = Join-Path $mockRoot 'ti/driverlib'
New-Item -ItemType Directory -Path $mockDriverLib -Force | Out-Null
Copy-Item (Join-Path $PSScriptRoot 'mock/dl_gpio.h.txt') (Join-Path $mockDriverLib 'dl_gpio.h')
Copy-Item (Join-Path $PSScriptRoot 'mock/dl_timerg.h.txt') (Join-Path $mockDriverLib 'dl_timerg.h')
$source = Join-Path $PSScriptRoot 'test_motor_failsafe.c.txt'
$sourceC = Join-Path $mockRoot 'test_motor_failsafe.c'
Copy-Item $source $sourceC
$output = Join-Path $env:TEMP (
    'mspm0-motor-failsafe-' + [guid]::NewGuid().ToString() + '.exe')

& $compiler.Source -std=c11 -Wall -Wextra -Werror `
    -I $mockRoot -I $projectRoot $sourceC (Join-Path $projectRoot 'Motor/motor.c') `
    -o $output
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

& $output
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

Write-Output "Test binary retained at: $output"
Write-Output "Mock source retained at: $mockRoot"
