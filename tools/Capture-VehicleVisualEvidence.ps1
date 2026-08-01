[CmdletBinding()]
param(
    [ValidateRange(60, 20000)]
    [int]$Frames = 900,

    [ValidateRange(1, 300)]
    [int]$RecordEveryN = 1,

    [ValidateRange(10, 100)]
    [int]$RecordScalePercent = 100,

    [string]$OutputRoot = "recordings",
    [string]$Exe = ".\x64\Release\D455.exe",
    [string]$SessionLabel = "vehicle_visual",

    [switch]$NoPrompt,
    [switch]$PlanOnly,
    [switch]$SkipDeviceProbe
)

$ErrorActionPreference = "Stop"

function Resolve-FromRepo([string]$PathText, [string]$RepoRoot) {
    if ([System.IO.Path]::IsPathRooted($PathText)) {
        return [System.IO.Path]::GetFullPath($PathText)
    }
    return [System.IO.Path]::GetFullPath((Join-Path $RepoRoot $PathText))
}

function Get-GitText([string[]]$Arguments) {
    $text = & git @Arguments 2>$null
    if ($LASTEXITCODE -ne 0) {
        return "unknown"
    }
    return ($text | Select-Object -First 1).Trim()
}

function Write-SessionManifest([string]$Path, [System.Collections.IDictionary]$Manifest) {
    $Manifest | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $Path -Encoding utf8
}

$repoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
Push-Location $repoRoot
try {
    $exePath = Resolve-FromRepo $Exe $repoRoot
    if (-not (Test-Path -LiteralPath $exePath -PathType Leaf)) {
        throw "D455 executable not found: $exePath"
    }

    $safeLabel = ($SessionLabel -replace '[^A-Za-z0-9_-]', '_').Trim('_')
    if ([string]::IsNullOrWhiteSpace($safeLabel)) {
        $safeLabel = "vehicle_visual"
    }
    $timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $outputRootPath = Resolve-FromRepo $OutputRoot $repoRoot
    $sessionDir = Join-Path $outputRootPath ("{0}_{1}" -f $safeLabel, $timestamp)
    $videoPath = Join-Path $sessionDir "dashboard.avi"
    $acceptancePath = Join-Path $sessionDir "acceptance.csv"
    $profilePath = Join-Path $sessionDir "profile.csv"
    $consolePath = Join-Path $sessionDir "console.log"
    $manifestPath = Join-Path $sessionDir "session_manifest.json"

    $captureArgs = @(
        "--acceptance-baseline",
        "--acceptance-label=$safeLabel",
        "--record-video=$videoPath",
        "--acceptance-csv=$acceptancePath",
        "--profile-csv=$profilePath",
        "--record-fps=30",
        "--record-every-n=$RecordEveryN",
        "--record-scale-percent=$RecordScalePercent",
        "--no-record-command-control",
        "--pose-read",
        "--pose-overlay",
        "--gravity-line",
        "--pose-log",
        "--pose-log-every-n=30",
        "--motion-diagnostics",
        "--motion-overlay",
        "--motion-log",
        "--motion-log-every-n=30",
        "--cluster-map",
        "--quality-segmentation",
        "--color-segmentation",
        "--color-refine-depth-masks",
        "--stereo-contour-distance",
        "--realtime-30",
        "--max-frames=$Frames"
    )
    $displayCommand = '"{0}" {1}' -f $exePath, ($captureArgs -join ' ')

    Write-Host "Vehicle visual-evidence capture plan"
    Write-Host "  repository: $repoRoot"
    Write-Host "  output:     $sessionDir"
    Write-Host "  frames:     $Frames processed frames"
    Write-Host "  command:    $displayCommand"

    if ($PlanOnly) {
        Write-Host "Plan only: no device query and no files were created."
        return
    }

    if (-not $SkipDeviceProbe) {
        Write-Host "Checking connected RealSense device..."
        & $exePath --probe-only
        if ($LASTEXITCODE -ne 0) {
            throw "RealSense preflight failed. Connect the D455 and retry."
        }
    }

    $outputDriveName = ([System.IO.Path]::GetPathRoot($sessionDir)).TrimEnd('\').TrimEnd(':')
    $outputDrive = Get-PSDrive -Name $outputDriveName -ErrorAction SilentlyContinue
    if ($outputDrive -and $outputDrive.Free -lt 10GB) {
        throw "Less than 10 GiB free on output drive $outputDriveName."
    }

    if (-not $NoPrompt) {
        Write-Host ""
        Write-Host "Mount the D455 rigidly and keep the vehicle parked."
        Write-Host "After capture starts, keep still briefly, move normally, then stop and remain still."
        Read-Host "Press Enter to start the fixed-frame capture"
    }

    New-Item -ItemType Directory -Force -Path $sessionDir | Out-Null
    $manifest = [ordered]@{
        format = "d455_vehicle_visual_evidence_v1"
        status = "prepared"
        session_id = (Split-Path -Leaf $sessionDir)
        created_at = (Get-Date).ToString("o")
        scheme_reference = "visual-system Fish Nest feasibility scheme dated 2026-06-25; see docs/VEHICLE_VISUAL_CAPTURE.md"
        repository = $repoRoot
        branch = Get-GitText @("rev-parse", "--abbrev-ref", "HEAD")
        commit = Get-GitText @("rev-parse", "HEAD")
        frame_budget = $Frames
        record_every_n = $RecordEveryN
        record_scale_percent = $RecordScalePercent
        command_line = $displayCommand
        outputs = [ordered]@{
            dashboard_video = $videoPath
            acceptance_csv = $acceptancePath
            profile_csv = $profilePath
            console_log = $consolePath
        }
        evidence_scope = @(
            "five-panel processed video",
            "per-processed-frame acceptance metrics with latest IMU pose sample",
            "per-processed-frame performance profile",
            "visual and IMU motion diagnostics"
        )
        limitations = @(
            "This session does not save deterministic color/depth/left-IR/right-IR replay frames.",
            "The yaw value is relative gyro integration and may drift.",
            "Vehicle acceleration contaminates accelerometer-only gravity estimates during motion.",
            "No GPS or absolute vehicle pose is captured."
        )
    }
    Write-SessionManifest $manifestPath $manifest

    Write-Host "Starting capture. Do not operate the computer while the vehicle is moving."
    & $exePath @captureArgs 2>&1 | Tee-Object -FilePath $consolePath
    $exitCode = $LASTEXITCODE

    $manifest.status = if ($exitCode -eq 0) { "complete" } else { "failed" }
    $manifest.completed_at = (Get-Date).ToString("o")
    $manifest.exit_code = $exitCode
    $manifest.output_sizes_bytes = [ordered]@{}
    foreach ($name in @("dashboard_video", "acceptance_csv", "profile_csv", "console_log")) {
        $path = $manifest.outputs[$name]
        $manifest.output_sizes_bytes[$name] = if (Test-Path -LiteralPath $path) {
            (Get-Item -LiteralPath $path).Length
        } else {
            0
        }
    }
    Write-SessionManifest $manifestPath $manifest

    if ($exitCode -ne 0) {
        throw "D455 capture failed with exit code $exitCode. See $consolePath"
    }
    foreach ($requiredPath in @($videoPath, $acceptancePath, $profilePath)) {
        if (-not (Test-Path -LiteralPath $requiredPath) -or (Get-Item -LiteralPath $requiredPath).Length -eq 0) {
            throw "Capture finished but required output is missing or empty: $requiredPath"
        }
    }

    Write-Host ""
    Write-Host "Vehicle visual-evidence capture complete: $sessionDir"
    Write-Host "Keep this whole ignored directory together for later analysis."
}
finally {
    Pop-Location
}
