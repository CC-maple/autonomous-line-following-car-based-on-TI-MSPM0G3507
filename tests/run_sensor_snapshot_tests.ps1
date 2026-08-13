$ErrorActionPreference = 'Stop'

$compiler = Get-Command gcc -ErrorAction Stop
$projectRoot = Split-Path -Parent $PSScriptRoot
$controlSource = Get-Content -Raw (Join-Path $projectRoot 'Control/control.c')

$sixFunction = [regex]::Match(
    $controlSource, `
    'void huidu_updata\(void\)\s*\{(?<body>.*?)\n\}', `
    [System.Text.RegularExpressions.RegexOptions]::Singleline).Groups['body'].Value
$eightFunction = [regex]::Match(
    $controlSource, `
    'void mode4_huidu_updata\(void\)\s*\{(?<body>.*?)\n\}', `
    [System.Text.RegularExpressions.RegexOptions]::Singleline).Groups['body'].Value
$mode3Function = [regex]::Match(
    $controlSource, `
    'void mode3_only_go_line\(float mode3_line_angle\)\{(?<body>.*?)\n\}', `
    [System.Text.RegularExpressions.RegexOptions]::Singleline).Groups['body'].Value

$sourceChecks = [ordered]@{
    SixFunctionFound = $sixFunction.Length -gt 0
    EightFunctionFound = $eightFunction.Length -gt 0
    Mode3FunctionFound = $mode3Function.Length -gt 0
    SixReadsGpioOnce = ([regex]::Matches(
        $sixFunction, 'DL_GPIO_readPins\(')).Count -eq 1
    EightReadsGpioOnce = ([regex]::Matches(
        $eightFunction, 'DL_GPIO_readPins\(')).Count -eq 1
    SixDecodesCurrentSnapshot = $sixFunction.Contains(
        'huidu_data_sum = sensor_snapshot_decode(')
    EightDecodesCurrentSnapshot = $eightFunction.Contains(
        'mode4_huidu_data_sum = sensor_snapshot_decode(')
    NoOldArraySumLoops = -not [regex]::IsMatch(
        $controlSource, '(huidu_data_sum|mode4_huidu_data_sum)\s*\+=')
    Mode3ReadsOneSnapshot = ([regex]::Matches(
        $mode3Function, 'huidu_updata\(\);')).Count -eq 1
    SixUsesOffsetOne = $sixFunction.Contains(
        'pin_masks, &huidu_data[1], 6u')
    EightUsesAllChannels = $eightFunction.Contains(
        'mode4_huidu_data, 8u')
}
foreach ($check in $sourceChecks.GetEnumerator()) {
    if (-not $check.Value) {
        throw "Sensor snapshot source check failed: $($check.Key)"
    }
}

$source = Join-Path $PSScriptRoot 'test_sensor_snapshot.c.txt'
$output = Join-Path $env:TEMP (
    'mspm0-sensor-snapshot-' + [guid]::NewGuid().ToString() + '.exe')

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
