$ErrorActionPreference = "Stop"

$root = Get-Location
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$archive = Join-Path $root "_old_before_dynamic_calib_$timestamp"

Write-Host "Root folder: $root"
Write-Host "Archive folder: $archive"

New-Item -ItemType Directory -Path $archive -Force | Out-Null

$itemsToArchive = @(
    "posv2",
    "scripts",
    "legacy"
)

foreach ($item in $itemsToArchive) {
    $path = Join-Path $root $item
    if (Test-Path $path) {
        Write-Host "Archiving $item ..."
        Move-Item -Path $path -Destination $archive -Force
    }
}

# Keep these main folders
New-Item -ItemType Directory -Path (Join-Path $root "config") -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $root "data") -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $root "data\captures") -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $root "docs") -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $root "firmware") -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $root "tools") -Force | Out-Null

Write-Host ""
Write-Host "Clean structure ready."
Write-Host "Old files were moved to: $archive"
Write-Host ""
Write-Host "Current structure:"
Get-ChildItem $root
