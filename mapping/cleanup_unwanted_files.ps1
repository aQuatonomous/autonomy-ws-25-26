# Cleanup Script - Remove Unwanted Files from Repository Migration
# Run this script to remove unwanted packages and build artifacts

Write-Host "Starting cleanup of unwanted files..." -ForegroundColor Cyan

# Remove unwanted source packages
$unwantedPackages = @(
    "src\my_lidar_pkg",
    "src\ros_numpy",
    "src\unilidar_sdk"
)

foreach ($package in $unwantedPackages) {
    if (Test-Path $package) {
        Write-Host "Removing $package..." -ForegroundColor Yellow
        Remove-Item -Path $package -Recurse -Force
        Write-Host "  [OK] Removed $package" -ForegroundColor Green
    } else {
        Write-Host "  [SKIP] $package not found (already removed)" -ForegroundColor Gray
    }
}

# Remove build artifacts
$buildDirs = @("build", "install", "log")

foreach ($dir in $buildDirs) {
    if (Test-Path $dir) {
        Write-Host "Removing $dir/..." -ForegroundColor Yellow
        Remove-Item -Path $dir -Recurse -Force
        Write-Host "  [OK] Removed $dir/" -ForegroundColor Green
    } else {
        Write-Host "  [SKIP] $dir/ not found (already removed)" -ForegroundColor Gray
    }
}

# Remove unitree_lidar_ros2.h if it exists in root directory
if (Test-Path "unitree_lidar_ros2.h") {
    Write-Host "Removing unitree_lidar_ros2.h..." -ForegroundColor Yellow
    Remove-Item -Path "unitree_lidar_ros2.h" -Force
    Write-Host "  [OK] Removed unitree_lidar_ros2.h" -ForegroundColor Green
}

Write-Host ""
Write-Host "Cleanup complete!" -ForegroundColor Cyan
Write-Host "Remaining packages in src/:" -ForegroundColor Cyan
Get-ChildItem -Directory src | Select-Object Name | Format-Table -AutoSize

Write-Host "Next steps:" -ForegroundColor Yellow
Write-Host "1. Run: git add ." -ForegroundColor White
Write-Host "2. Run: git commit -m 'Clean up unwanted packages and build artifacts'" -ForegroundColor White
Write-Host "3. Verify with: git status" -ForegroundColor White
