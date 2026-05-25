$ErrorActionPreference = 'Stop'

$repo = Resolve-Path (Join-Path $PSScriptRoot '..')
Set-Location $repo

$commonIncludes = @(
    '-IUserLib\FOC',
    '-IUserLib\FOC\base_classes',
    '-IUserLib\PID',
    '-IApplications\Inc',
    '-IBSP'
)

function Invoke-TestBinary {
    param(
        [string] $Name,
        [string[]] $CompileArgs,
        [string] $Exe
    )

    Write-Host "== $Name =="
    & gcc @CompileArgs -o $Exe
    & (Join-Path $repo $Exe)
    Remove-Item -LiteralPath $Exe -ErrorAction SilentlyContinue
}

Invoke-TestBinary `
    -Name 'PID original behavior' `
    -CompileArgs @('-std=c11', '-Wall', '-Wextra', '-Werror', '-IUserLib\PID',
                   'tests\test_pid_original_behavior.c', '-lm') `
    -Exe 'tests\test_pid_original_behavior.exe'

Invoke-TestBinary `
    -Name 'FOC original behavior' `
    -CompileArgs (@('-std=c11', '-Wall', '-Wextra', '-Werror', '-DSYS_PUBLIC_H',
                    '-Ddelay(x)=((void)(x))', '-DFOC_MAP_LEN=1') +
                  $commonIncludes +
                  @('tests\test_foc_original_behavior.c', 'UserLib\FOC\FOC.c', '-lm')) `
    -Exe 'tests\test_foc_original_behavior.exe'

Invoke-TestBinary `
    -Name 'QD4310 original layout' `
    -CompileArgs @('-std=c11', '-Wall', '-Wextra', '-Werror',
                   '-Itests\stubs', '-IUserLib\QD4310', '-IUserLib\FOC',
                   '-IUserLib\FOC\base_classes', '-IUserLib\PID',
                   'tests\test_qd4310_original_layout.c') `
    -Exe 'tests\test_qd4310_original_layout.exe'

Invoke-TestBinary `
    -Name 'QD4310 storage behavior' `
    -CompileArgs (@('-std=c11', '-Wall', '-Wextra', '-Werror', '-DSYS_PUBLIC_H',
                    '-Ddelay(x)=((void)(x))', '-Itests\stubs', '-IUserLib\QD4310') +
                  $commonIncludes +
                  @('tests\test_qd4310_storage_behavior.c',
                    'UserLib\FOC\FOC.c',
                    'UserLib\QD4310\QD4310.c',
                    '-lm')) `
    -Exe 'tests\test_qd4310_storage_behavior.exe'

$env:Path = 'D:\Tools\cmake-4.3.3-windows-x86_64\bin;D:\Tools\arm-gnu-toolchain-15.2.rel1\bin;D:\MinGW\bin;' + $env:Path

Write-Host '== Firmware configure =='
cmake -S . -B build -G 'MinGW Makefiles' -DCMAKE_BUILD_TYPE=Debug -DSTM32CUBE_G4_PATH='D:/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.2'

Write-Host '== Firmware build =='
$previousErrorActionPreference = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
$buildOutput = & cmake --build build --clean-first --config Debug -- -j4 2>&1
$buildExitCode = $LASTEXITCODE
$ErrorActionPreference = $previousErrorActionPreference
$buildOutput
$buildText = $buildOutput -join "`n"
if ($buildExitCode -ne 0) {
    exit $buildExitCode
}
if ($buildText -match 'cast between incompatible function types') {
    throw 'Firmware build has incompatible function pointer cast warnings.'
}
