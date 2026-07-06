param(
    [string[]]$Cases = @("near_single_object", "far_cabinet", "depth_hole_black_object"),
    [string]$DatasetRoot = "datasets",
    [string]$CasesYaml = "eval\cases.yaml",
    [switch]$Force
)

$ErrorActionPreference = "Stop"

$argsList = @(
    "scripts\finalize_replay_manifests.py",
    "--dataset-root", $DatasetRoot,
    "--cases-yaml", $CasesYaml
)
foreach ($caseId in $Cases) {
    $argsList += @("--case-id", $caseId)
}
if ($Force) {
    $argsList += "--force"
}

& python @argsList
if ($LASTEXITCODE -ne 0) {
    throw "Replay manifest finalization failed with exit code $LASTEXITCODE"
}
