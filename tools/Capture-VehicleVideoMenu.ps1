[CmdletBinding()]
param(
    [string]$Choice,

    [ValidateRange(0, 20000)]
    [int]$FramesOverride = 0,

    [ValidateRange(0, 30)]
    [int]$CountdownSeconds = 3,

    [ValidateRange(1, 300)]
    [int]$RecordEveryN = 1,

    [ValidateRange(10, 100)]
    [int]$RecordScalePercent = 100,

    [string]$OutputRoot = "recordings",
    [string]$Exe = ".\x64\Release\D455.exe",

    [switch]$PlanOnly,
    [switch]$SkipDeviceProbe
)

$ErrorActionPreference = "Stop"

try {
    [Console]::OutputEncoding = New-Object System.Text.UTF8Encoding($false)
}
catch {
}

$menuTextPath = Join-Path $PSScriptRoot "VehicleCaptureMenu.zh-CN.json"
if (-not (Test-Path -LiteralPath $menuTextPath -PathType Leaf)) {
    throw "Chinese menu resource not found: $menuTextPath"
}
$menuText = Get-Content -LiteralPath $menuTextPath -Raw -Encoding UTF8 | ConvertFrom-Json

$cases = [ordered]@{
    "1" = [ordered]@{
        label = "vehicle_01_parked_static"
        title = "Parked static baseline"
        frames = 300
        scenario = "Vehicle parked; camera and scene remain static for stability and IMU baseline."
        guidance = "Keep the vehicle parked and do not touch the camera."
    }
    "2" = [ordered]@{
        label = "vehicle_02_slow_straight"
        title = "Slow straight motion"
        frames = 600
        scenario = "Rigidly mounted camera during safe slow straight vehicle translation."
        guidance = "Start while parked, then use a safe slow straight segment."
    }
    "3" = [ordered]@{
        label = "vehicle_03_turn_pose_change"
        title = "Turn and pose change"
        frames = 600
        scenario = "Rigidly mounted camera during a safe turn or heading change."
        guidance = "Use a normal safe turn; do not manipulate the camera."
    }
    "4" = [ordered]@{
        label = "vehicle_04_road_vibration"
        title = "Normal road vibration"
        frames = 600
        scenario = "Normal road vibration with rigid camera mounting; no intentional harsh maneuver."
        guidance = "Use an ordinary road segment; do not seek bumps or unsafe vibration."
    }
    "5" = [ordered]@{
        label = "vehicle_05_occlusion_reappear"
        title = "Occlusion and reappearance"
        frames = 450
        scenario = "Vehicle parked; a passenger briefly occludes a known visible target and reveals it."
        guidance = "Vehicle must be parked. A passenger may occlude the target; the driver must not participate."
    }
    "6" = [ordered]@{
        label = "vehicle_06_new_occupancy"
        title = "New occupied region"
        frames = 450
        scenario = "Vehicle parked; a passenger places and removes an object in a previously free visible region."
        guidance = "Vehicle must be parked. A passenger places/removes the object after recording starts."
    }
    "7" = [ordered]@{
        label = "vehicle_07_near_far_transition"
        title = "Near/far transition"
        frames = 600
        scenario = "Safe approach to or departure from a large visible object, covering near and far distance modes."
        guidance = "Use a normal safe approach/departure; keep the camera fixed."
    }
    "8" = [ordered]@{
        label = "vehicle_08_lighting_transition"
        title = "Natural lighting transition"
        frames = 600
        scenario = "Natural shade-to-light or light-to-shade transition without touching the camera."
        guidance = "Use natural lighting change only; do not cover the lens while driving."
    }
    "9" = [ordered]@{
        label = "vehicle_09_mixed_route"
        title = "Mixed route sequence"
        frames = 900
        scenario = "Parked baseline, gentle start, straight motion, safe turn, stop, and parked ending."
        guidance = "Begin parked, include normal motion and a safe turn, then stop and remain still."
    }
}

$captureScript = Join-Path $PSScriptRoot "Capture-VehicleVisualEvidence.ps1"
if (-not (Test-Path -LiteralPath $captureScript -PathType Leaf)) {
    throw "Capture script not found: $captureScript"
}

function Show-CaptureMenu {
    Write-Host ""
    Write-Host $menuText.header
    foreach ($key in 1..9) {
        $line = $menuText.lines.PSObject.Properties[[string]$key].Value
        Write-Host ("  {0}" -f $line)
    }
    Write-Host ("  {0}" -f $menuText.quit)
    Write-Host ""
}

function Start-CaptureCase([string]$SelectedChoice) {
    if (-not $cases.Contains($SelectedChoice)) {
        throw ($menuText.invalid_choice -f $SelectedChoice)
    }

    $case = $cases[$SelectedChoice]
    $localizedCase = $menuText.cases.PSObject.Properties[$SelectedChoice].Value
    $frames = if ($FramesOverride -gt 0) { $FramesOverride } else { [int]$case.frames }
    Write-Host ""
    Write-Host ($menuText.selected -f $SelectedChoice, $localizedCase.title)
    Write-Host ($menuText.guidance -f $localizedCase.guidance)
    Write-Host ($menuText.auto_stop -f $frames)

    if (-not $PlanOnly -and $CountdownSeconds -gt 0) {
        for ($second = $CountdownSeconds; $second -ge 1; --$second) {
            Write-Host ($menuText.countdown -f $second)
            Start-Sleep -Seconds 1
        }
    }

    $captureParameters = @{
        Frames = $frames
        RecordEveryN = $RecordEveryN
        RecordScalePercent = $RecordScalePercent
        OutputRoot = $OutputRoot
        Exe = $Exe
        SessionLabel = [string]$case.label
        Scenario = [string]$case.scenario
        NoPrompt = $true
        PlanOnly = [bool]$PlanOnly
        SkipDeviceProbe = [bool]$SkipDeviceProbe
    }
    & $captureScript @captureParameters
}

if (-not [string]::IsNullOrWhiteSpace($Choice)) {
    Start-CaptureCase $Choice.Trim()
    return
}

while ($true) {
    Show-CaptureMenu
    $selected = (Read-Host $menuText.prompt).Trim()
    if ($selected -in @("q", "Q")) {
        break
    }
    if (-not $cases.Contains($selected)) {
        Write-Warning $menuText.invalid
        continue
    }
    Start-CaptureCase $selected
}
