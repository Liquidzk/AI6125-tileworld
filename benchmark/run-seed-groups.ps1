param(
    [string]$WorkspaceRoot = ".",
    [string]$SeedFile = "benchmark/seed-groups.json",
    [string]$Profile = "config1",
    [string[]]$Groups = @(),
    [string[]]$ExtraJavaArgs = @(),
    [string]$Label = "benchmark",
    [string]$OutputPath = "",
    [switch]$Compile
)

$ErrorActionPreference = "Stop"

function Get-FinalReward {
    param([string[]]$OutputLines)

    foreach ($line in $OutputLines) {
        if ($line -match "The final reward is:\s*(\d+)") {
            return [int]$Matches[1]
        }
    }

    throw "Could not parse final reward from Tileworld output."
}

$workspace = (Resolve-Path $WorkspaceRoot).Path
$seedPath = if ([System.IO.Path]::IsPathRooted($SeedFile)) { $SeedFile } else { Join-Path (Get-Location) $SeedFile }
$seedGroupsObject = Get-Content $seedPath -Raw | ConvertFrom-Json
$seedGroups = @{}
foreach ($property in $seedGroupsObject.PSObject.Properties) {
    $seedGroups[$property.Name] = @($property.Value)
}

if ($Groups.Count -eq 0) {
    $selectedGroups = @($seedGroups.Keys | Sort-Object)
} else {
    $selectedGroups = @($Groups)
}

if ($Compile) {
    Push-Location $workspace
    try {
        & javac -cp "libs\MASON_14.jar" -d out (Get-ChildItem -Recurse src -Filter *.java | ForEach-Object FullName)
    } finally {
        Pop-Location
    }
}

$groupResults = @()
$overallRewards = New-Object System.Collections.Generic.List[int]

foreach ($groupName in $selectedGroups) {
    if (-not $seedGroups.ContainsKey($groupName)) {
        throw "Unknown seed group '$groupName'."
    }

    $rewards = New-Object System.Collections.Generic.List[int]
    Push-Location $workspace
    try {
        foreach ($seed in $seedGroups[$groupName]) {
            $javaArgs = @(
                "-Dtileworld.seed=$seed",
                "-Dtileworld.useConfiguredSeed=true",
                "-Dtileworld.iterations=1"
            )
            if ($Profile -ne "config1") {
                $javaArgs += "-Dtileworld.profile=$Profile"
            }
            if ($ExtraJavaArgs.Count -gt 0) {
                $javaArgs += $ExtraJavaArgs
            }
            $javaArgs += @("-cp", "out;libs\MASON_14.jar", "tileworld.TileworldMain")

            $outputLines = & java @javaArgs 2>&1
            $reward = Get-FinalReward -OutputLines $outputLines
            $rewards.Add($reward)
            $overallRewards.Add($reward)
        }
    } finally {
        Pop-Location
    }

    $groupAverage = if ($rewards.Count -gt 0) { [Math]::Round((($rewards | Measure-Object -Average).Average), 2) } else { 0.0 }
    $groupResults += [ordered]@{
        group = $groupName
        seeds = @($seedGroups[$groupName])
        rewards = @($rewards)
        average = $groupAverage
        min = ($rewards | Measure-Object -Minimum).Minimum
        max = ($rewards | Measure-Object -Maximum).Maximum
    }
}

$overallAverage = if ($overallRewards.Count -gt 0) {
    [Math]::Round((($overallRewards | Measure-Object -Average).Average), 2)
} else {
    0.0
}

$result = [ordered]@{
    label = $Label
    workspace = $workspace
    profile = $Profile
    groups = $groupResults
    overallAverage = $overallAverage
}

if ($OutputPath) {
    $resolvedOutput = if ([System.IO.Path]::IsPathRooted($OutputPath)) { $OutputPath } else { Join-Path (Get-Location) $OutputPath }
    $outputDir = Split-Path -Parent $resolvedOutput
    if ($outputDir) {
        New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
    }
    $result | ConvertTo-Json -Depth 6 | Set-Content -Path $resolvedOutput
}

$result | ConvertTo-Json -Depth 6
