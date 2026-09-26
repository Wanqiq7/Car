#Requires -Version 7.0
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $false
$global:LASTEXITCODE = 0

# 禁止在本脚本内调用 Set-Location：它由 build_firmware.ps1 以 & 的方式在同一 runspace
# 内调用，切换工作目录会泄漏到调用者会话且不会恢复。所有路径一律基于 $RepoRoot 拼接。
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path

# 逐字节保留上游 InvenSense DMP 源码的模块目录：重排格式会破坏与上游的逐字节可对比性，
# 且其中 inv_mpu.c / inv_mpu.h / MPU6050.c / MPU6050.h 为 GBK 编码，clang-format
# 处理无效 UTF-8 会报错或损坏文件。因此整体排除。
$ExcludedModuleDirs = @('MPU6050')

function Show-Usage {
  @'
Usage:
  tools/Windows/format_code.ps1 [--check]

Description:
  Format C/C++ files under Modules/ using clang-format.
  Requires clang-format version 21.1.8 by default.
  Requires a .clang-format at the repository root.

Options:
  --check   Run clang-format in dry-run mode with --Werror.
  -h, --help
'@ | Write-Host
}

function Get-HostPlatform {
  if ($IsWindows) { return 'win32' }
  if ($IsMacOS) { return 'darwin' }
  if ($IsLinux) { return 'linux' }
  return 'unknown'
}

function Get-HostArch {
  $arch = if ($IsWindows) {
    if ($env:PROCESSOR_ARCHITECTURE) { $env:PROCESSOR_ARCHITECTURE } else { [System.Runtime.InteropServices.RuntimeInformation]::OSArchitecture.ToString() }
  } else {
    [System.Runtime.InteropServices.RuntimeInformation]::OSArchitecture.ToString()
  }

  switch -Regex ($arch) {
    '^(amd64|x64|X64)$' { return 'x86_64' }
    '^(x86_64)$' { return 'x86_64' }
    '^(arm64|ARM64|aarch64)$' { return 'arm64' }
    default { return $arch.ToLowerInvariant() }
  }
}

function Test-CommandExists([string]$Name) {
  return [bool](Get-Command $Name -ErrorAction SilentlyContinue)
}

function Get-PythonForTools {
  if ($IsWindows -and (Test-CommandExists 'py')) {
    & py -3 --version 1>$null 2>$null
    if ($global:LASTEXITCODE -eq 0) {
      return @('py', '-3')
    }
  }

  foreach ($candidate in @('python3', 'python')) {
    if (Test-CommandExists $candidate) {
      return @($candidate)
    }
  }

  if ($IsWindows -and (Test-CommandExists 'py')) {
    return @('py')
  }

  return @()
}

function Invoke-Python {
  param(
    [Parameter(Mandatory)][string[]]$PythonCmd,
    [Parameter(Mandatory)][string[]]$ArgumentList
  )

  $exe = $PythonCmd[0]
  $prefix = @()
  if ($PythonCmd.Count -gt 1) {
    $prefix = $PythonCmd[1..($PythonCmd.Count - 1)]
  }
  & $exe @prefix @ArgumentList 2>&1 | ForEach-Object { Write-Host $_ }
  if ($global:LASTEXITCODE -ne 0) {
    $rendered = @($prefix + $ArgumentList) -join ' '
    throw "Python 命令失败 (exit $global:LASTEXITCODE): $exe $rendered"
  }
}

function Get-ClangFormatVersion([string]$Bin) {
  if ([string]::IsNullOrWhiteSpace($Bin) -or -not (Test-Path -LiteralPath $Bin -PathType Leaf)) {
    return ''
  }
  $output = & $Bin --version 2>$null
  if ($global:LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($output)) {
    return ''
  }
  $text = if ($output -is [array]) { $output -join "`n" } else { [string]$output }
  $match = [regex]::Match($text, '\d+\.\d+\.\d+')
  if ($match.Success) { return $match.Value }
  return ''
}

function Test-IsExecutable([string]$Path) {
  if ([string]::IsNullOrWhiteSpace($Path)) { return $false }
  return (Test-Path -LiteralPath $Path -PathType Leaf)
}

function Find-ExistingClangFormat {
  param(
    [string]$ToolRoot,
    [string]$RepoRootPath
  )

  $candidates = @(
    (Join-Path $ToolRoot 'bin/clang-format')
    (Join-Path $ToolRoot 'bin/clang-format.exe')
    (Join-Path $RepoRootPath '.venv-clang-format/bin/clang-format')
    (Join-Path $RepoRootPath '.venv-clang-format/Scripts/clang-format.exe')
    (Join-Path $RepoRootPath '../.venv-clang-format/bin/clang-format')
    (Join-Path $RepoRootPath '../.venv-clang-format/Scripts/clang-format.exe')
  )

  foreach ($candidate in $candidates) {
    if (Test-IsExecutable $candidate) {
      return $candidate
    }
  }

  foreach ($name in @('clang-format.exe', 'clang-format')) {
    $cmd = Get-Command $name -ErrorAction SilentlyContinue
    if ($cmd) {
      return $cmd.Source
    }
  }

  return ''
}

function Get-LlvmAssetPattern {
  param(
    [string]$Platform,
    [string]$Arch,
    [string]$Version
  )

  switch ("${Platform}:${Arch}") {
    'win32:x86_64' { return "^clang\+llvm-$Version-x86_64-pc-windows-msvc\.tar\.xz$" }
    'win32:arm64' { return "^clang\+llvm-$Version-aarch64-pc-windows-msvc\.tar\.xz$" }
    'linux:x86_64' { return "^LLVM-$Version-Linux-X64\.tar\.xz$" }
    'linux:arm64' { return "^LLVM-$Version-Linux-ARM64\.tar\.xz$" }
    'darwin:arm64' { return "^LLVM-$Version-macOS-ARM64\.tar\.xz$" }
    'darwin:x86_64' { return "^LLVM-$Version-macOS-X64\.tar\.xz$|^clang\+llvm-$Version-x86_64-apple-darwin.*\.tar\.xz$" }
    default { return '' }
  }
}

function Install-OfficialClangFormat {
  param(
    [string]$Platform,
    [string]$Arch,
    [string]$Version,
    [string]$CacheDir,
    [string]$ToolRoot
  )

  $pythonCmd = Get-PythonForTools
  if ($pythonCmd.Count -eq 0) {
    throw '未找到 Python。请安装 Python 3，或手动设置 CLANG_FORMAT_BIN。'
  }

  $pattern = Get-LlvmAssetPattern -Platform $Platform -Arch $Arch -Version $Version
  if ([string]::IsNullOrWhiteSpace($pattern)) {
    throw "没有适用于 $Platform/$Arch 的官方 LLVM clang-format 包映射。"
  }

  New-Item -ItemType Directory -Force -Path $CacheDir | Out-Null
  $archivePath = Join-Path $CacheDir "clang-format-$Version-$Platform-$Arch.tar.xz"

  if (-not (Test-Path -LiteralPath $archivePath -PathType Leaf)) {
    Write-Host "Downloading official LLVM clang-format $Version for $Platform/$Arch..."
    $py = @'
import json
import re
import sys
import urllib.request

version, pattern, archive_path = sys.argv[1:4]
api_url = f"https://api.github.com/repos/llvm/llvm-project/releases/tags/llvmorg-{version}"

with urllib.request.urlopen(api_url) as response:
    release = json.load(response)

asset = next(
    (candidate for candidate in release.get("assets", []) if re.fullmatch(pattern, candidate["name"])),
    None,
)

if asset is None:
    raise SystemExit(f"No LLVM clang-format archive matches pattern: {pattern}")

urllib.request.urlretrieve(asset["browser_download_url"], archive_path)
print(f"Downloaded {asset['name']}")
'@
    Invoke-Python -PythonCmd $pythonCmd -ArgumentList @('-c', $py, $Version, $pattern, $archivePath)
  }

  if (Test-Path -LiteralPath $ToolRoot) {
    Remove-Item -LiteralPath $ToolRoot -Recurse -Force
  }
  New-Item -ItemType Directory -Force -Path $ToolRoot | Out-Null

  $list = & tar -tf $archivePath
  if ($global:LASTEXITCODE -ne 0) {
    throw "无法列出 LLVM 压缩包内容: $archivePath"
  }

  $entryPattern = if ($Platform -eq 'win32') {
    '(^|/)bin/(clang-format\.exe|.*\.dll)$'
  } elseif ($Platform -eq 'darwin') {
    '(^|/)bin/clang-format$|(^|/)(bin|lib)/.*\.dylib$'
  } else {
    '(^|/)bin/clang-format$|(^|/)(bin|lib)/.*\.so(\..*)?$'
  }

  $entries = @($list | Where-Object { $_ -match $entryPattern })
  if ($entries.Count -eq 0) {
    throw "未能在 $archivePath 中定位 clang-format 文件。"
  }

  & tar -xf $archivePath -C $ToolRoot --strip-components=1 -- @entries
  if ($global:LASTEXITCODE -ne 0) {
    throw "解压 LLVM clang-format 失败: $archivePath"
  }
  Remove-Item -LiteralPath $archivePath -Force -ErrorAction SilentlyContinue

  if ($Platform -eq 'win32') {
    return (Join-Path $ToolRoot 'bin/clang-format.exe')
  }
  return (Join-Path $ToolRoot 'bin/clang-format')
}

function Install-LocalClangFormatVenv {
  param(
    [string]$RepoRootPath,
    [string]$Version
  )

  $pythonCmd = Get-PythonForTools
  if ($pythonCmd.Count -eq 0) {
    throw '未找到 Python。请安装 Python 3，或手动设置 CLANG_FORMAT_BIN。'
  }

  $venvDir = Join-Path $RepoRootPath '.venv-clang-format'
  Write-Host "Preparing local clang-format $Version in $venvDir..."

  if (-not (Test-Path -LiteralPath $venvDir -PathType Container)) {
    Invoke-Python -PythonCmd $pythonCmd -ArgumentList @('-m', 'venv', $venvDir)
  }

  $venvPythonUnix = Join-Path $venvDir 'bin/python'
  $venvPythonWin = Join-Path $venvDir 'Scripts/python.exe'
  $clangUnix = Join-Path $venvDir 'bin/clang-format'
  $clangWin = Join-Path $venvDir 'Scripts/clang-format.exe'

  $venvPython = ''
  $clangBin = ''
  if (Test-IsExecutable $venvPythonUnix) {
    $venvPython = $venvPythonUnix
    $clangBin = $clangUnix
  } elseif (Test-IsExecutable $venvPythonWin) {
    $venvPython = $venvPythonWin
    $clangBin = $clangWin
  } else {
    throw "创建 Python venv 失败: $venvDir"
  }

  & $venvPython -m pip install --upgrade pip 2>&1 | ForEach-Object { Write-Host $_ }
  if ($global:LASTEXITCODE -ne 0) {
    throw "pip 升级失败 (exit $global:LASTEXITCODE)"
  }
  & $venvPython -m pip install "clang-format==$Version" 2>&1 | ForEach-Object { Write-Host $_ }
  if ($global:LASTEXITCODE -ne 0) {
    throw "安装 clang-format==$Version 失败 (exit $global:LASTEXITCODE)"
  }

  if (-not (Test-IsExecutable $clangBin)) {
    throw "pip 安装 clang-format 后未找到可执行文件: $clangBin"
  }
  return $clangBin
}

function Get-SourceFiles {
  $modulesDir = Join-Path $RepoRoot 'Modules'
  $files = [System.Collections.Generic.List[string]]::new()
  if (-not (Test-Path -LiteralPath $modulesDir -PathType Container)) {
    return [string[]]@()
  }

  $exts = @('.c', '.cc', '.cpp', '.cxx', '.h', '.hh', '.hpp', '.hxx')
  Get-ChildItem -LiteralPath $modulesDir -Recurse -File |
    Where-Object { $exts -contains $_.Extension.ToLowerInvariant() } |
    Where-Object {
      $relative = $_.FullName.Substring($modulesDir.Length).TrimStart('\', '/')
      $topDir = ($relative -split '[\\/]')[0]
      return $ExcludedModuleDirs -notcontains $topDir
    } |
    ForEach-Object { [void]$files.Add($_.FullName) }
  return [string[]]$files.ToArray()
}

function Invoke-ClangFormatOnFiles {
  param(
    [string]$Bin,
    [string[]]$ClangArgs,
    [string[]]$Files
  )

  if ($null -eq $Files -or $Files.Count -eq 0) {
    return
  }

  $batchSize = 80
  for ($i = 0; $i -lt $Files.Count; $i += $batchSize) {
    $end = [Math]::Min($i + $batchSize - 1, $Files.Count - 1)
    $batch = [string[]]$Files[$i..$end]
    $formatArgs = [string[]]($ClangArgs + $batch)
    & $Bin @formatArgs
    if ($global:LASTEXITCODE -ne 0) {
      throw "clang-format 失败 (exit $global:LASTEXITCODE)"
    }
  }
}

$Mode = 'format'
$RequiredVersion = if ($env:CLANG_FORMAT_REQUIRED_VERSION) { $env:CLANG_FORMAT_REQUIRED_VERSION } else { '21.1.8' }

if ($args.Count -gt 0) {
  switch ($args[0]) {
    '--check' { $Mode = 'check' }
    { $_ -in @('-h', '--help') } { Show-Usage; exit 0 }
    default {
      [Console]::Error.WriteLine("Unknown option: $($args[0])")
      Show-Usage
      exit 2
    }
  }
}

# --style=file 依赖仓库根的 .clang-format；缺失时会退化为 LLVM 默认风格并大范围改动代码。
$StyleFile = Join-Path $RepoRoot '.clang-format'
if (-not (Test-Path -LiteralPath $StyleFile -PathType Leaf)) {
  throw "未找到 $StyleFile。--style=file 将退化为 LLVM 默认风格，请先添加 .clang-format。"
}

$HostPlatform = Get-HostPlatform
$HostArch = Get-HostArch
$ClangFormatCacheDir = Join-Path $RepoRoot '.cache/clang-format'
$ClangFormatToolRoot = Join-Path $ClangFormatCacheDir "llvm-$RequiredVersion-$HostPlatform-$HostArch"

$ClangFormatBin = if ($env:CLANG_FORMAT_BIN) { $env:CLANG_FORMAT_BIN } else { '' }
if ([string]::IsNullOrWhiteSpace($ClangFormatBin)) {
  $ClangFormatBin = Find-ExistingClangFormat -ToolRoot $ClangFormatToolRoot -RepoRootPath $RepoRoot
}

$CfVersion = ''
if (-not [string]::IsNullOrWhiteSpace($ClangFormatBin)) {
  $CfVersion = Get-ClangFormatVersion $ClangFormatBin
}

if ([string]::IsNullOrWhiteSpace($ClangFormatBin) -or $CfVersion -ne $RequiredVersion) {
  $ClangFormatBin = Install-LocalClangFormatVenv -RepoRootPath $RepoRoot -Version $RequiredVersion
  $CfVersion = Get-ClangFormatVersion $ClangFormatBin
  if ($CfVersion -ne $RequiredVersion) {
    $ClangFormatBin = Install-OfficialClangFormat -Platform $HostPlatform -Arch $HostArch -Version $RequiredVersion -CacheDir $ClangFormatCacheDir -ToolRoot $ClangFormatToolRoot
    $CfVersion = Get-ClangFormatVersion $ClangFormatBin
  }
}

if ([string]::IsNullOrWhiteSpace($CfVersion)) {
  throw "无法从 $ClangFormatBin 解析 clang-format 版本。"
}

if ($CfVersion -ne $RequiredVersion) {
  throw "未能准备 clang-format $RequiredVersion；在 $ClangFormatBin 找到 $CfVersion。"
}

$files = [string[]](Get-SourceFiles)
if ($Mode -eq 'check') {
  Invoke-ClangFormatOnFiles -Bin $ClangFormatBin -ClangArgs @('--dry-run', '--Werror', '--style=file') -Files $files
  Write-Host 'clang-format check passed for Modules/ (excluded: Modules/MPU6050).'
} else {
  Invoke-ClangFormatOnFiles -Bin $ClangFormatBin -ClangArgs @('-i', '--style=file') -Files $files
  Write-Host 'Formatted C/C++ files under Modules/ (excluded: Modules/MPU6050).'
}
