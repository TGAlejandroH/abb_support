# The VC campaign (readiness plan 6.4 x 6.5) on a virtual controller: every fixture built by
# make_fixtures.py, run once with the real HMI in the loop (..\hmi_case.ps1), then the results
# table (summary.py).
#
#   .\run_campaign.ps1 -Dir C:\abb_readiness\vc_campaign [-Station 1] [-Only M1_mixed,F3_P9_presets]
#                      [-Strategy CAPTURE_CAPTURE_WELD_WELD]
#
# -Station 2 (default) runs part 9022 against <name>_expect.json. -Station 1 runs part 9011
# against <name>_stn1_expect.json (station_variant.py); fixtures without one are skipped. Either
# way the station is presented first with vc_modules\TGIndexStn<n>.mod (Production Manager's own
# index hook) and the positioner is homed after every case.
# -Strategy sets the HMI's HMI_SEQUENCE_WELDING_STRATEGY for this pass only (restored after);
# the labels get its name.
param(
    [Parameter(Mandatory)][string]$Dir,
    [ValidateSet(1, 2)][int]$Station = 2,
    [string[]]$Only,
    [string]$Strategy,
    [string]$Python = "C:\Users\AlejandroHernandez\anaconda3\envs\pyocc_release\python.exe",
    [string]$Hmi = "C:\Users\AlejandroHernandez\source\repos\TGAlejandroH\hmi-endstatus-wt\TGuideWeldingHMI\x64\Release"
)
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$fixtures = "$Dir\fixtures"; $results = "$Dir\results"
$part = if ($Station -eq 1) { "9011" } else { "9022" }
$suffix = if ($Station -eq 1) { "_stn1" } else { "" }
$tag = $suffix + $(if ($Strategy) { "_" + $Strategy.ToLower() } else { "" })

$config = "$Hmi\ProgramData\config\config.json"
$configText = [System.IO.File]::ReadAllText($config)
if ($Strategy) {
    $edited = [regex]::Replace($configText, '"HMI_SEQUENCE_WELDING_STRATEGY"\s*:\s*"[A-Z_]+"',
                               ('"HMI_SEQUENCE_WELDING_STRATEGY":"' + $Strategy + '"'))
    [System.IO.File]::WriteAllText($config, $edited)
}
try {
    & $Python "$here\..\bench_cycle.py" --mod "$here\vc_modules\TGIndexStn$Station.mod" --part $part --timeout 180 *> $null
    $names = Get-ChildItem "$fixtures\*_expect.json" | ForEach-Object { $_.Name -replace "_expect\.json$", "" } |
        Where-Object { $_ -notmatch "_(stn1|run)$" -and (Test-Path "$fixtures\$_.tgs") -and
                       (Test-Path "$fixtures\$_${suffix}_expect.json") -and (-not $Only -or $Only -contains $_) }
    foreach ($n in $names) {
        $caseArgs = @{ Fixture = "$fixtures\$n.tgs"; Expect = "$fixtures\$n${suffix}_expect.json"
                       Label = "HMI_$n$tag"; OutDir = $results; Part = $part; Hmi = $Hmi }
        # The VC shares the PC's CPU: when another job loads the PC the VC runs slower than real
        # time, and every timing here (PC clock) reads slow. A lag can only make a weld look
        # slower, never faster, and a real speed fault comes back on every run (wrong weld data
        # also fails A2). So a case whose only failures are "too slow" timing rows is re-run, up
        # to twice; it counts when a run passes everything.
        for ($try = 1; $try -le 3; $try++) {
            & "$here\..\hmi_case.ps1" @caseArgs *> $null
            $code = $LASTEXITCODE
            $fails = @(Select-String -Path "$results\HMI_$n$tag.txt" -Pattern "^FAIL" -ErrorAction SilentlyContinue)
            $tooSlow = { param($line)
                if ($line -match "seam speed ([0-9.]+) mm/s vs weld_speed ([0-9.]+)") { return [double]$Matches[1] -lt [double]$Matches[2] }
                return $line -match "^FAIL A1 .*(weld-on output high|TCP travel)" }
            $timingOnly = $fails.Count -gt 0 -and -not ($fails | Where-Object { -not (& $tooSlow $_.Line) })
            if ($code -eq 0 -or -not $timingOnly -or $try -eq 3) { break }
            Copy-Item "$results\HMI_$n$tag.txt" "$results\HMI_$n${tag}_slow_try$try.txt" -Force
            & $Python "$here\..\bench_cycle.py" --mod "$here\vc_modules\TGHomeStn.mod" --part $part --timeout 120 *> $null
        }
        Write-Host ("{0,-44} exit {1} (run {2})" -f "$n$tag", $code, $try)
        # A run that ends at abort_end leaves the positioner where it was, and the HMI then refuses
        # every part ("station not at load position"). Home it before the next case.
        & $Python "$here\..\bench_cycle.py" --mod "$here\vc_modules\TGHomeStn.mod" --part $part --timeout 120 *> $null
    }
} finally {
    [System.IO.File]::WriteAllText($config, $configText)
}
& $Python "$here\summary.py" --results $results --fixtures $fixtures --markdown "$results\summary.md"
