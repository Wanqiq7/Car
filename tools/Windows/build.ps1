#Requires -Version 7.0
# Clock 工程编译入口。用法见 -h。
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$global:LASTEXITCODE = 0

$script:UsageName = 'tools/Windows/build.ps1'
$script:DefaultConfigPrimary = 'User/xrobot.yaml'
$script:DefaultConfigFallback = 'User/xrobot.yaml'
$script:DefaultPreset = 'RelWithDebInfo'
# 留空 = 走 CMakePresets（构建目录即 build/<Preset>）。需要自定义目录时用 -b。
$script:DefaultBuildDir = ''
$script:ToolchainFile = 'cmake/gcc-arm-none-eabi.cmake'

try {
  Push-Location $PSScriptRoot
  . "$PSScriptRoot\build_firmware.ps1" @args
} catch {
  [Console]::Error.WriteLine($_.Exception.Message)
  exit 1
} finally {
  Pop-Location
}
exit $global:LASTEXITCODE
