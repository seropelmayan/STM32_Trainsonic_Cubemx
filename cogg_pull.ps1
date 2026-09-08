<#
.SYNOPSIS
  Pull the anti-cogging map off the connected STM32 over SWD, compile it into
  Inc/cogg_table.h, archive the raw flash record, and commit + push.

.DESCRIPTION
  Wraps the flow documented in cogg_read.py:
    1. STM32_Programmer_CLI (HOTPLUG, no reset) reads page 63 (0x0801F800, 1048 B)
    2. cogg_read.py validates it (magic/version/nbins/CRC) and emits Inc/cogg_table.h
    3. the raw record is archived as cogg_maps/cogg_<UTC stamp>.bin (keeps clamp/gain,
       which the header does not carry)
    4. git commit + push (skip with -NoPush; -NoCommit leaves the tree dirty)

  Requires an ST-LINK on SWD. The USB-CDC console cannot dump the map without
  running a calibration sweep, so this is the only read-only path.

  PRECEDENCE REMINDER: Ropetow_CoggInit() loads COGG_TABLE_INIT and then lets a
  valid FLASH map override it. The compiled-in copy is a backup that survives a
  chip erase; it only becomes active after console 'n' erases the saved map.

.EXAMPLE
  .\cogg_pull.ps1
  .\cogg_pull.ps1 -NoPush
#>
param(
  [switch]$NoCommit,
  [switch]$NoPush,
  [string]$Cli = "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
)

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

if (-not (Test-Path $Cli)) {
  $alt = Get-ChildItem "C:\ST" -Recurse -Filter STM32_Programmer_CLI.exe -ErrorAction SilentlyContinue | Select-Object -First 1
  if ($null -eq $alt) { throw "STM32_Programmer_CLI.exe not found; pass -Cli <path>" }
  $Cli = $alt.FullName
}

$stamp  = (Get-Date).ToUniversalTime().ToString("yyyyMMdd-HHmmss")
$binDir = Join-Path $PSScriptRoot "cogg_maps"
$bin    = Join-Path $binDir "cogg_$stamp.bin"
New-Item -ItemType Directory -Force $binDir | Out-Null

Write-Host "== reading cogging record from flash (SWD hot-plug, no reset) =="
& $Cli -c port=SWD mode=HOTPLUG -u 0x0801F800 1048 $bin
if ($LASTEXITCODE -ne 0 -or -not (Test-Path $bin)) {
  throw "flash read failed (is the ST-LINK plugged in?)"
}

Write-Host "`n== validating + emitting Inc/cogg_table.h =="
python cogg_read.py $bin --emit-header Inc/cogg_table.h
if ($LASTEXITCODE -ne 0) {
  Remove-Item $bin -Force
  throw "cogg_read.py rejected the record; nothing changed"
}

if ($NoCommit) { Write-Host "`n-NoCommit: Inc/cogg_table.h and $bin left uncommitted"; exit 0 }

git add Inc/cogg_table.h $bin
$staged = git diff --cached --name-only
if (-not $staged) { Write-Host "`nno change vs. committed table; nothing to commit"; exit 0 }

$msg = @"
Cogging map pulled from board $stamp

Read over SWD from 0x0801F800 by cogg_pull.ps1, compiled into Inc/cogg_table.h,
raw record archived as cogg_maps/cogg_$stamp.bin.

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
"@
git commit -m $msg
if ($LASTEXITCODE -ne 0) { throw "git commit failed" }

if (-not $NoPush) {
  git push
  if ($LASTEXITCODE -ne 0) { throw "git push failed" }
}
Write-Host "`ndone: $bin -> Inc/cogg_table.h committed$(if (-not $NoPush) {' and pushed'})"
