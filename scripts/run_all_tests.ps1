#!/usr/bin/env pwsh
# =============================================================
# TCAS Full Build & Test Script
# Runs all test suites for Modules 1-5 plus integration tests.
# Usage: .\scripts\run_all_tests.ps1
# =============================================================

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$ProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$BuildDir    = Join-Path $ProjectRoot "build"

Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host " TCAS Build & Test Runner" -ForegroundColor Cyan
Write-Host "========================================`n" -ForegroundColor Cyan

# --- CMake Configure ---
Write-Host "[1/3] Configuring CMake..." -ForegroundColor Yellow
cmake -S $ProjectRoot -B $BuildDir -G "MinGW Makefiles" `
    -DCMAKE_BUILD_TYPE=RelWithDebInfo
if ($LASTEXITCODE -ne 0) { throw "CMake configuration failed." }

# --- Build ---
Write-Host "`n[2/3] Building..." -ForegroundColor Yellow
cmake --build $BuildDir --parallel
if ($LASTEXITCODE -ne 0) { throw "Build failed." }

# --- CTest ---
Write-Host "`n[3/3] Running tests..." -ForegroundColor Yellow
ctest --test-dir $BuildDir --output-on-failure -V

$exitCode = $LASTEXITCODE

Write-Host "`n========================================"
if ($exitCode -eq 0)
{
    Write-Host " ALL TESTS PASSED" -ForegroundColor Green
}
else
{
    Write-Host " SOME TESTS FAILED (exit code $exitCode)" -ForegroundColor Red
}
Write-Host "========================================`n"

# --- Cppcheck ---
Write-Host "[OPTIONAL] Running cppcheck..." -ForegroundColor Yellow
$CppCheck = (Get-Command cppcheck -ErrorAction SilentlyContinue)
if ($CppCheck)
{
    cppcheck --enable=warning,style,performance,portability `
             --suppress=missingIncludeSystem `
             --suppress=unusedFunction `
             --suppress=syntaxError `
             --std=c++23 `
             -I (Join-Path $ProjectRoot "include") `
             (Join-Path $ProjectRoot "src") `
             (Join-Path $ProjectRoot "tests")
    Write-Host "cppcheck complete." -ForegroundColor Green
}
else
{
    Write-Host "cppcheck not found — skipping." -ForegroundColor DarkYellow
}

exit $exitCode
