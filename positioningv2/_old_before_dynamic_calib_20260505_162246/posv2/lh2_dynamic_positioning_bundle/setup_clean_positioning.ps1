# Run this from PowerShell.
# It creates a clean positioningv2 folder and archives old files instead of deleting them.

$ROOT = "$env:USERPROFILE\lh2_positioning\positioningv2"
$ARCHIVE = "$ROOT\_old_before_dynamic_calib_$(Get-Date -Format yyyyMMdd_HHmmss)"

Write-Host "Project root: $ROOT"

if (!(Test-Path $ROOT)) {
    New-Item -ItemType Directory -Force -Path $ROOT | Out-Null
}

New-Item -ItemType Directory -Force -Path "$ROOT\config" | Out-Null
New-Item -ItemType Directory -Force -Path "$ROOT\data\captures" | Out-Null
New-Item -ItemType Directory -Force -Path "$ROOT\tools" | Out-Null
New-Item -ItemType Directory -Force -Path "$ROOT\docs" | Out-Null

# Archive common old generated files, but do not delete.
$oldPatterns = @(
    "config\origin_angles*.json",
    "config\lighthouse_geometry*.json",
    "data\captures\origin*.json",
    "data\captures\origin*.csv",
    "data\captures\*.tmp",
    "tools\estimate_lighthouse_geometry*.py",
    "tools\live_relative_position*.py"
)

$toMove = @()
foreach ($pat in $oldPatterns) {
    $toMove += Get-ChildItem -Path $ROOT -Filter (Split-Path $pat -Leaf) -Recurse -ErrorAction SilentlyContinue | Where-Object {
        $_.FullName -like (Join-Path $ROOT $pat).Replace("*", "*")
    }
}

$toMove = $toMove | Sort-Object FullName -Unique
if ($toMove.Count -gt 0) {
    New-Item -ItemType Directory -Force -Path $ARCHIVE | Out-Null
    foreach ($f in $toMove) {
        $rel = Resolve-Path -Path $f.FullName -Relative
        $dest = Join-Path $ARCHIVE $f.Name
        Write-Host "Archiving $($f.FullName) -> $dest"
        Move-Item -Force $f.FullName $dest
    }
    Write-Host "Old files archived in: $ARCHIVE"
} else {
    Write-Host "No old generated files found to archive."
}

Write-Host "Clean structure ready. Now copy the new config/tools files into $ROOT"
