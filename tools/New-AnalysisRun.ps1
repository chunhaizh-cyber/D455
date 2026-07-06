param(
    [Parameter(Mandatory = $true)]
    [string]$RunId,

    [string]$CaseName = $RunId,
    [string]$Purpose = "",
    [string]$Scene = "",
    [string]$CommandLine = ".\x64\Release\D455.exe --cluster-map --acceptance-baseline --max-frames=600",
    [string]$BuildType = "Release",
    [int]$FrameCount = 0,
    [switch]$Force
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$templateDir = Join-Path $repoRoot "analysis_runs\_template"
$runDir = Join-Path $repoRoot ("analysis_runs\" + $RunId)

if (!(Test-Path $templateDir)) {
    throw "Template directory not found: $templateDir"
}

if ((Test-Path $runDir) -and !$Force) {
    throw "Run directory already exists: $runDir. Use -Force to overwrite template files."
}

New-Item -ItemType Directory -Force -Path $runDir | Out-Null
Copy-Item -Path (Join-Path $templateDir "*") -Destination $runDir -Recurse -Force

$branch = ""
$commit = ""
try {
    $branch = (& git -C $repoRoot rev-parse --abbrev-ref HEAD 2>$null).Trim()
    $commit = (& git -C $repoRoot rev-parse --short HEAD 2>$null).Trim()
} catch {
    $branch = ""
    $commit = ""
}

$manifestPath = Join-Path $runDir "run_manifest.json"
$manifest = Get-Content -Raw -Path $manifestPath | ConvertFrom-Json
$configPath = Join-Path $runDir "config_snapshot.json"
$configHash = ""
if (Test-Path $configPath) {
    $configHash = (Get-FileHash -Algorithm SHA256 -Path $configPath).Hash.ToLowerInvariant()
}
$manifest.run_id = $RunId
$manifest.case_name = $CaseName
$manifest.purpose = $Purpose
$manifest.git_commit = $commit
$manifest.config_hash = $configHash
$manifest.branch = $branch
$manifest.build_type = $BuildType
$manifest.command_line = $CommandLine
$manifest.frame_count = $FrameCount
$manifest.scene.description = $Scene
$manifest | ConvertTo-Json -Depth 8 | Set-Content -Path $manifestPath -Encoding UTF8

$notesPath = Join-Path $runDir "notes.md"
$notes = Get-Content -Raw -Path $notesPath
$notes = $notes -replace "YYYYMMDD_HHMMSS_case01", $RunId
Set-Content -Path $notesPath -Value $notes -Encoding UTF8

$scorePath = Join-Path $runDir "run_score.json"
if (Test-Path $scorePath) {
    $score = Get-Content -Raw -Path $scorePath | ConvertFrom-Json
    $score.run_id = $RunId
    $score | ConvertTo-Json -Depth 8 | Set-Content -Path $scorePath -Encoding UTF8
}

Write-Host "Created analysis run: $runDir"
Write-Host "Update docs\RUN_INDEX.md before syncing."
