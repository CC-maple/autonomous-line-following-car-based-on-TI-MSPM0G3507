$ErrorActionPreference = 'Stop'

$compiler = Get-Command gcc -ErrorAction Stop
$projectRoot = Split-Path -Parent $PSScriptRoot
$source = Join-Path $PSScriptRoot 'test_absolute_and_distance.c.txt'
$output = Join-Path $env:TEMP (
    'mspm0-absolute-distance-' + [guid]::NewGuid().ToString() + '.exe')

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
