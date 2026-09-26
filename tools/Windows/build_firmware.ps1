#Requires -Version 7.0
# 由 build.ps1 dot-source。请勿直接调用。
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $false
$global:LASTEXITCODE = 0

if (-not $script:UsageName) {
  throw '请通过 tools/Windows/build.ps1 调用。'
}

$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Set-Location $RepoRoot

function Show-Usage {
  @"
Usage:
  $($script:UsageName) [options]

Description:
  1) 可选：对 Modules/ 运行 clang-format（默认跳过，见 --format）
  2) 用 cube-cmake / cmake 配置工程（配置期由 CMake 自动重跑 xrobot_gen_main）
  3) 用 cube-cmake / cmake 编译固件

Options:
  -c, --config <path>     XRobot YAML 路径（默认: $($script:DefaultConfigPrimary)）
  -p, --preset <name>     CMake preset 名（默认: `$CMAKE_BUILD_PRESET 或 $($script:DefaultPreset)）
                          本工程可用: Debug / Release / RelWithDebInfo
  -b, --build-dir <dir>   直接指定构建目录（覆盖 --preset 的默认目录）
  -j, --jobs <n>          传给构建器的并行度（默认交给构建器自行决定）
      --format            运行 clang-format（默认跳过）
      --check-format      只做 clang-format 检查，不修改文件
  -h, --help              显示本帮助

Notes:
  * 默认 preset 为 RelWithDebInfo（-Os -g3）：机器码与 Release(-Os -g0) 逐字节相同，
    但 .elf 自带完整调试信息（Ozone 可直接查看变量）。Debug(-O0/-Og) 在 64 KB 上无法链接。
  * 默认不跑 clang-format：本工程 Modules/MPU6050 内含上游 InvenSense DMP 源码
    （逐字节保留，其中 4 个文件为 GBK 编码），重排格式会破坏与上游的一致性。
  * 配置前请确认 arm-none-eabi-gcc 与 ninja 可用；脚本会自动从 STM32CubeCLT 探测。

Examples:
  $($script:UsageName)
  $($script:UsageName) -p Debug
  $($script:UsageName) -p Release -j 2
  $($script:UsageName) --check-format
"@ | Write-Host
}

function Exit-FromBuild([int]$Code) {
  $global:LASTEXITCODE = $Code
}

function Test-CommandExists([string]$Name) {
  return [bool](Get-Command $Name -ErrorAction SilentlyContinue)
}

function Get-CommandPath([string]$Name) {
  $cmd = Get-Command $Name -ErrorAction SilentlyContinue
  if ($cmd) { return $cmd.Source }
  return ''
}

function Add-PathFront([string]$Dir) {
  if ([string]::IsNullOrWhiteSpace($Dir)) { return }
  if (-not (Test-Path -LiteralPath $Dir -PathType Container)) { return }

  $resolved = [System.IO.Path]::GetFullPath($Dir)
  $parts = @($env:PATH -split [IO.Path]::PathSeparator)
  if ($parts | Where-Object { $_ -eq $resolved }) { return }
  $env:PATH = $resolved + [IO.Path]::PathSeparator + $env:PATH
}

# ---------------------------------------------------------------------------
# 工具解析
# ---------------------------------------------------------------------------

function Resolve-CubeCMake {
  foreach ($name in @('cube-cmake', 'cube-cmake.exe')) {
    $fromPath = Get-CommandPath $name
    if ($fromPath) {
      $script:CubeCMakeBin = $fromPath
      return $true
    }
  }

  foreach ($name in @('cmake', 'cmake.exe')) {
    $cmake = Get-CommandPath $name
    if ($cmake) {
      $script:CubeCMakeBin = $cmake
      Write-Host "[preflight] cube-cmake 未找到，改用 cmake: $($script:CubeCMakeBin)"
      return $true
    }
  }

  return $false
}

function Get-LatestSubdir([string]$ParentDir) {
  if (-not (Test-Path -LiteralPath $ParentDir -PathType Container)) {
    return ''
  }
  $dirs = @(Get-ChildItem -LiteralPath $ParentDir -Directory | Sort-Object Name)
  if ($dirs.Count -eq 0) {
    return ''
  }
  return $dirs[-1].FullName
}

function Get-Stm32BundleRoots {
  $roots = [System.Collections.Generic.List[string]]::new()
  if ($env:CUBE_BUNDLE_PATH) {
    $roots.Add($env:CUBE_BUNDLE_PATH)
  }

  $candidates = @(
    (Join-Path $HOME 'AppData/Local/stm32cube/bundles')
    (Join-Path $HOME '.stm32cube/bundles')
    (Join-Path $HOME '.local/share/stm32cube/bundles')
    (Join-Path $HOME '.config/stm32cube/bundles')
    (Join-Path $HOME 'Library/Application Support/stm32cube/bundles')
  )
  if ($env:LOCALAPPDATA) {
    $candidates += (Join-Path $env:LOCALAPPDATA 'stm32cube/bundles')
  }

  foreach ($root in $candidates) {
    $roots.Add($root)
  }

  return @($roots | Where-Object { Test-Path -LiteralPath $_ -PathType Container } | Select-Object -Unique)
}

function Get-CubeCltRoots {
  if (-not $IsWindows) { return @() }
  $stRoot = 'C:\ST'
  if (-not (Test-Path -LiteralPath $stRoot -PathType Container)) { return @() }
  return @(Get-ChildItem -LiteralPath $stRoot -Directory -Filter 'STM32CubeCLT_*' |
      Sort-Object Name | ForEach-Object { $_.FullName })
}

function Resolve-GccToolchain {
  if ($env:GCC_TOOLCHAIN_ROOT -and -not (Test-Path -LiteralPath $env:GCC_TOOLCHAIN_ROOT -PathType Container)) {
    Remove-Item Env:GCC_TOOLCHAIN_ROOT
  }

  $detectedGccRoot = ''

  foreach ($bundleRoot in Get-Stm32BundleRoots) {
    if (-not $env:GCC_TOOLCHAIN_ROOT -and -not $detectedGccRoot) {
      $latest = Get-LatestSubdir (Join-Path $bundleRoot 'gnu-tools-for-stm32')
      if ($latest) {
        $binDir = Join-Path $latest 'bin'
        $detectedGccRoot = if (Test-Path -LiteralPath $binDir -PathType Container) { $binDir } else { $latest }
      }
    }
    if ($env:GCC_TOOLCHAIN_ROOT -or $detectedGccRoot) { break }
  }

  if (-not $env:GCC_TOOLCHAIN_ROOT -and -not $detectedGccRoot) {
    foreach ($cltRoot in Get-CubeCltRoots) {
      $cltGcc = Join-Path $cltRoot 'GNU-tools-for-STM32/bin'
      if (Test-Path -LiteralPath $cltGcc -PathType Container) {
        $detectedGccRoot = $cltGcc
        break
      }
    }
  }

  if (-not $env:GCC_TOOLCHAIN_ROOT -and $detectedGccRoot) {
    $env:GCC_TOOLCHAIN_ROOT = $detectedGccRoot
    Write-Host "[preflight] GCC_TOOLCHAIN_ROOT 未设置，自动使用 $($env:GCC_TOOLCHAIN_ROOT)"
  }

  if ($env:GCC_TOOLCHAIN_ROOT) {
    Add-PathFront $env:GCC_TOOLCHAIN_ROOT
  }
}

function Resolve-Ninja {
  if ((Test-CommandExists 'ninja') -or (Test-CommandExists 'ninja.exe')) {
    return $true
  }

  foreach ($bundleRoot in Get-Stm32BundleRoots) {
    $ninjaRoot = Get-LatestSubdir (Join-Path $bundleRoot 'ninja')
    if (-not $ninjaRoot) { continue }
    foreach ($candidate in @((Join-Path $ninjaRoot 'bin/ninja.exe'), (Join-Path $ninjaRoot 'bin/ninja'))) {
      if (Test-Path -LiteralPath $candidate -PathType Leaf) {
        Add-PathFront (Join-Path $ninjaRoot 'bin')
        Write-Host "[preflight] ninja 未在 PATH 中，改用 $candidate"
        return $true
      }
    }
  }

  foreach ($cltRoot in Get-CubeCltRoots) {
    $cltNinjaDir = Join-Path $cltRoot 'Ninja/bin'
    $cltNinja = Join-Path $cltNinjaDir 'ninja.exe'
    if (Test-Path -LiteralPath $cltNinja -PathType Leaf) {
      Add-PathFront $cltNinjaDir
      Write-Host "[preflight] ninja 未在 PATH 中，改用 $cltNinja"
      return $true
    }
  }

  [Console]::Error.WriteLine('Error: 未找到 ninja（PATH 或 STM32CubeCLT/Cube bundle 目录）。')
  return $false
}

function Get-PresetBuildType([string]$Preset) {
  switch -Regex ($Preset) {
    '^(?i)debug$' { return 'Debug' }
    '^(?i)release$' { return 'Release' }
    '^(?i)relwithdebinfo$' { return 'RelWithDebInfo' }
    '^(?i)minsizerel$' { return 'MinSizeRel' }
    default { return 'Debug' }
  }
}

function Invoke-Native {
  param(
    [Parameter(Mandatory)][string]$FilePath,
    [string[]]$ArgumentList = @()
  )

  & $FilePath @ArgumentList
  if ($global:LASTEXITCODE -ne 0) {
    throw "命令失败 (exit $global:LASTEXITCODE): $FilePath $($ArgumentList -join ' ')"
  }
}

function Reset-IncompatibleCMakeCache([string]$BuildPath) {
  $cache = Join-Path $BuildPath 'CMakeCache.txt'
  if (-not (Test-Path -LiteralPath $cache -PathType Leaf)) {
    return
  }

  $homeDir = ''
  foreach ($line in Get-Content -LiteralPath $cache) {
    if ($line -match '^CMAKE_HOME_DIRECTORY:INTERNAL=(.*)$') {
      $homeDir = $Matches[1].Trim()
      break
    }
  }
  if (-not $homeDir) {
    return
  }

  $normalize = {
    param([string]$PathValue)
    try {
      return [System.IO.Path]::GetFullPath($PathValue).TrimEnd('\', '/').ToLowerInvariant()
    } catch {
      return $PathValue.Replace('\', '/').TrimEnd('/').ToLowerInvariant()
    }
  }

  $cachedRoot = & $normalize $homeDir
  $currentRoot = & $normalize $RepoRoot
  if ($cachedRoot -eq $currentRoot) {
    return
  }

  Write-Host "[preflight] CMake 缓存来自 $homeDir，为 $RepoRoot 重置 $BuildPath"
  Remove-Item -LiteralPath $cache -Force
  $cmakeFiles = Join-Path $BuildPath 'CMakeFiles'
  if (Test-Path -LiteralPath $cmakeFiles) {
    Remove-Item -LiteralPath $cmakeFiles -Recurse -Force
  }
}

function Invoke-ConfigureBuildTree {
  param(
    [string]$BuildDir,
    [string]$BuildPath,
    [string]$Preset,
    [string]$ConfigPath
  )

  if ($BuildDir) {
    $buildType = Get-PresetBuildType $Preset
    Invoke-Native -FilePath $script:CubeCMakeBin -ArgumentList @(
      '-S', $RepoRoot
      '-B', $BuildPath
      '-G', 'Ninja'
      '--toolchain', (Join-Path $RepoRoot $script:ToolchainFile)
      '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
      "-DCMAKE_BUILD_TYPE=$buildType"
      "-DXROBOT_CONFIG:FILEPATH=$ConfigPath"
    )
  } else {
    Invoke-Native -FilePath $script:CubeCMakeBin -ArgumentList @(
      '--preset', $Preset
      "-DXROBOT_CONFIG:FILEPATH=$ConfigPath"
    )
  }
}

function Get-RequiredOptionValue {
  param(
    [string]$Option,
    [System.Collections.IList]$AllArgs,
    [int]$Index,
    [string]$Kind
  )

  if ($Index + 1 -ge $AllArgs.Count) {
    throw "Error: $Option requires a $Kind value."
  }
  $value = [string]$AllArgs[$Index + 1]
  if ([string]::IsNullOrWhiteSpace($value) -or $value.StartsWith('-')) {
    throw "Error: $Option requires a $Kind value."
  }
  return $value
}

# ---------------------------------------------------------------------------
# 参数解析
# ---------------------------------------------------------------------------

$ConfigPath = ''
$Preset = if ($env:CMAKE_BUILD_PRESET) {
  $env:CMAKE_BUILD_PRESET
} elseif ($env:CMAKE_PRESET) {
  $env:CMAKE_PRESET
} else {
  ''
}
$BuildDir = $script:DefaultBuildDir
$RunFormat = $false
$CheckFormat = $false
$Jobs = ''
$script:CubeCMakeBin = ''
$AllArgs = @($args)

try {
  $i = 0
  while ($i -lt $AllArgs.Count) {
    $current = [string]$AllArgs[$i]
    if ($current -in @('-c', '--config')) {
      $ConfigPath = Get-RequiredOptionValue -Option $current -AllArgs $AllArgs -Index $i -Kind 'path'
      $i += 2
    } elseif ($current -in @('-p', '--preset')) {
      $Preset = Get-RequiredOptionValue -Option $current -AllArgs $AllArgs -Index $i -Kind 'preset name'
      $i += 2
    } elseif ($current -in @('-b', '--build-dir')) {
      $BuildDir = Get-RequiredOptionValue -Option $current -AllArgs $AllArgs -Index $i -Kind 'directory'
      $i += 2
    } elseif ($current -in @('-j', '--jobs')) {
      $Jobs = Get-RequiredOptionValue -Option $current -AllArgs $AllArgs -Index $i -Kind 'number'
      $i += 2
    } elseif ($current -eq '--format') {
      $RunFormat = $true
      $i += 1
    } elseif ($current -eq '--check-format') {
      $CheckFormat = $true
      $i += 1
    } elseif ($current -in @('-h', '--help')) {
      Show-Usage
      Exit-FromBuild 0
      return
    } else {
      [Console]::Error.WriteLine("Unknown option: $current")
      Show-Usage
      Exit-FromBuild 2
      return
    }
  }
} catch {
  [Console]::Error.WriteLine($_.Exception.Message)
  Show-Usage
  Exit-FromBuild 2
  return
}

# 归一化 preset 名，使其与本工程 CMakePresets.json 中实际存在的名字一致。
if (-not [string]::IsNullOrWhiteSpace($Preset)) {
  $Preset = Get-PresetBuildType $Preset
}

if ($BuildDir) {
  if ([System.IO.Path]::IsPathRooted($BuildDir)) {
    $BuildPath = $BuildDir
  } else {
    $BuildPath = Join-Path $RepoRoot $BuildDir
  }
  $BuildTargetDesc = "directory: $BuildPath"
} else {
  if (-not $Preset) {
    $Preset = $script:DefaultPreset
  }
  $BuildPath = Join-Path $RepoRoot "build/$Preset"
  $BuildTargetDesc = "preset: $Preset (dir: $BuildPath)"
}

if (-not $ConfigPath) {
  if (Test-Path -LiteralPath $script:DefaultConfigPrimary -PathType Leaf) {
    $ConfigPath = $script:DefaultConfigPrimary
  } elseif (Test-Path -LiteralPath $script:DefaultConfigFallback -PathType Leaf) {
    $ConfigPath = $script:DefaultConfigFallback
  } else {
    $ConfigPath = $script:DefaultConfigPrimary
  }
}

if (-not [System.IO.Path]::IsPathRooted($ConfigPath)) {
  $ConfigPath = Join-Path $RepoRoot $ConfigPath
}

if (-not (Test-Path -LiteralPath $ConfigPath -PathType Leaf)) {
  throw "Error: XRobot YAML 配置不存在: $ConfigPath"
}

# ---------------------------------------------------------------------------
# 预检
# ---------------------------------------------------------------------------

if (-not (Test-CommandExists 'xrobot_gen_main') -and -not (Test-CommandExists 'xrobot_gen_main.exe')) {
  throw 'Error: 未找到 xrobot_gen_main（请先安装 XRobot 工具链，pip install xrobot）。'
}

if (-not (Resolve-CubeCMake)) {
  throw 'Error: 未找到 cube-cmake 或 cmake。'
}

Resolve-GccToolchain

if (-not (Resolve-Ninja)) {
  Exit-FromBuild 1
  return
}

if (-not $env:GCC_TOOLCHAIN_ROOT -or -not (Test-Path -LiteralPath $env:GCC_TOOLCHAIN_ROOT -PathType Container)) {
  throw 'Error: 未配置 GCC_TOOLCHAIN_ROOT，且无法自动探测。'
}

if (-not (Test-CommandExists 'arm-none-eabi-gcc') -and -not (Test-CommandExists 'arm-none-eabi-gcc.exe')) {
  throw 'Error: 探测工具链后仍未找到 arm-none-eabi-gcc。'
}

if (-not (Test-CommandExists 'cube') -and -not (Test-CommandExists 'cube.exe')) {
  Write-Host '[preflight] 提示: 未找到 CubeMX 命令行 cube，跳过 CubeMX 代码生成检查（不影响编译）。'
}

# ---------------------------------------------------------------------------
# 步骤
# ---------------------------------------------------------------------------

if ($CheckFormat) {
  Write-Host '[1/1] clang-format 检查...'
  & (Join-Path $RepoRoot 'tools/Windows/format_code.ps1') --check
  if ($global:LASTEXITCODE -ne 0) {
    throw "clang-format 检查失败 (exit $global:LASTEXITCODE)"
  }
  Write-Host 'Done.'
  $global:LASTEXITCODE = 0
  return
}

if ($RunFormat) {
  Write-Host '[1/3] 运行 clang-format...'
  & (Join-Path $RepoRoot 'tools/Windows/format_code.ps1')
  if ($global:LASTEXITCODE -ne 0) {
    throw "clang-format 失败 (exit $global:LASTEXITCODE)"
  }
} else {
  Write-Host '[1/3] 跳过 clang-format（如需格式化请加 --format）。'
}

Write-Host "[2/3] 配置工程 ($BuildTargetDesc)..."
if ($BuildDir) {
  Reset-IncompatibleCMakeCache $BuildPath
}
Invoke-ConfigureBuildTree -BuildDir $BuildDir -BuildPath $BuildPath -Preset $Preset -ConfigPath $ConfigPath

Write-Host "[3/3] 编译固件 ($BuildTargetDesc)..."
$buildArgs = @('--build', $BuildPath)
if ($Jobs) {
  $buildArgs += @('--parallel', $Jobs)
}
Invoke-Native -FilePath $script:CubeCMakeBin -ArgumentList $buildArgs

Write-Host 'Done.'
$global:LASTEXITCODE = 0
