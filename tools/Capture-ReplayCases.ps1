param(
    [string[]]$Cases = @("near_single_object", "far_cabinet", "depth_hole_black_object"),
    [string]$DatasetRoot = "datasets",
    [string]$Exe = ".\x64\Release\D455.exe",
    [int]$Frames = 120,
    [int]$Warmup = 30,
    [switch]$Force,
    [switch]$NoPrompt,
    [switch]$RequireIr = $true
)

$ErrorActionPreference = "Stop"

function Resolve-RepoPath([string]$PathText) {
    if ([System.IO.Path]::IsPathRooted($PathText)) {
        return $PathText
    }
    return Join-Path (Get-Location) $PathText
}

function Get-CasePrompt([string]$CaseId) {
    switch ($CaseId) {
        "near_single_object" {
            return "Place one clear near object on a desk, with stable background and no hand motion."
        }
        "far_cabinet" {
            return "Aim at the far cabinet or far indoor objects. Keep near foreground minimal."
        }
        "depth_hole_black_object" {
            return "Place a dark or depth-hole-prone object in view, with visible color contour."
        }
        default {
            return "Prepare the scene for case '$CaseId'."
        }
    }
}

if (-not (Test-Path -LiteralPath $Exe)) {
    throw "D455 executable not found: $Exe"
}

$datasetRootPath = Resolve-RepoPath $DatasetRoot
New-Item -ItemType Directory -Force -Path $datasetRootPath | Out-Null

foreach ($caseId in $Cases) {
    $caseDir = Join-Path $datasetRootPath $caseId
    $framesDir = Join-Path $caseDir "frames"
    if ((Test-Path -LiteralPath $framesDir) -and -not $Force) {
        throw "Case already has frames: $caseDir. Re-run with -Force to overwrite after manual review."
    }

    if ((Test-Path -LiteralPath $caseDir) -and $Force) {
        Remove-Item -LiteralPath $caseDir -Recurse -Force
    }

    Write-Host ""
    Write-Host "Preparing replay case: $caseId"
    Write-Host (Get-CasePrompt $caseId)
    Write-Host "Target: $caseDir"
    if (-not $NoPrompt) {
        Read-Host "Press Enter when the camera view is ready"
    }

    $captureArgs = @(
        "--capture-replay-dir=$caseDir",
        "--capture-replay-frames=$Frames",
        "--capture-replay-warmup=$Warmup",
        "--quality-segmentation",
        "--stereo-contour-distance",
        "--no-display"
    )
    Write-Host "capture: $Exe $($captureArgs -join ' ')"
    & $Exe @captureArgs
    if ($LASTEXITCODE -ne 0) {
        throw "D455 capture failed for $caseId with exit code $LASTEXITCODE"
    }

    $validateArgs = @(
        "scripts\validate_replay_dataset.py",
        "--case-dir", $caseDir,
        "--min-frames", "$Frames"
    )
    if ($RequireIr) {
        $validateArgs += @("--require-ir-left", "--require-ir-right")
    }
    Write-Host "validate: python $($validateArgs -join ' ')"
    & python @validateArgs
    if ($LASTEXITCODE -ne 0) {
        throw "Replay validation failed for $caseId with exit code $LASTEXITCODE"
    }
}

Write-Host ""
Write-Host "Replay capture complete. Review each case_manifest.json before scoring."
