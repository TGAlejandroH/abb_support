# One VC case with the real HMI in the loop (readiness plan 6.3), on a virtual controller only:
#   1. the work selection map sends part $Part to $Fixture (in the HMI build's own ProgramData),
#   2. the HMI starts, Simulation mode on, Connect = armed for socket start,
#   3. hmi_case.py records the run while it starts the robot's TG_Part$Part over RWS,
#   4. the HMI log of the run is kept next to the recording, and the HMI is closed.
# -PlantedOffset "x,y,z,rz,ry,rx" runs the case in the camera test mode (readiness plan F6): the
# HMI's HMI_SIMULATION_PART_OFFSET is set for this case only (config.json restored after), and the
# checks expect every served frame moved by it.
# The GUI is driven with posted messages and UI Automation, never the real mouse (gui_lib.ps1).
#
#   .\hmi_case.ps1 -Fixture C:\...\F4.tgs -Expect C:\...\F4_expect.json -Label F4 -OutDir C:\...\results
param(
    [Parameter(Mandatory)][string]$Fixture,
    [Parameter(Mandatory)][string]$Expect,
    [Parameter(Mandatory)][string]$Label,
    [Parameter(Mandatory)][string]$OutDir,
    [string]$Part = "9022",
    [string]$Hmi = "C:\Users\AlejandroHernandez\source\repos\TGAlejandroH\hmi-endstatus-wt\TGuideWeldingHMI\x64\Release",
    [string]$Python = "C:\Users\AlejandroHernandez\anaconda3\envs\pyocc_release\python.exe",
    [int]$MaxSeconds = 600,
    [string]$DisarmOnLog,
    [string]$PlantedOffset,
    [switch]$KeepOpen
)
$ErrorActionPreference = "Continue"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
. "$here\gui_lib.ps1"
$cfg = "$Hmi\ProgramData\config"
New-Item -ItemType Directory -Force $OutDir | Out-Null
$console = "$OutDir\$Label.console.txt"
Remove-Item $console -ErrorAction SilentlyContinue
function Say($m) { $s = "[{0}] {1}" -f (Get-Date -Format HH:mm:ss), $m; Write-Host $s; Add-Content $console $s }

# Only an HMI from this build counts: another session may run another build's HMI on the same PC.
if (Get-Process TGuideWeldingHMI -ErrorAction SilentlyContinue | Where-Object { $_.Path -like "$Hmi*" }) { Say "FAIL: an HMI from $Hmi is already running"; exit 2 }

$configFile = "$cfg\config.json"
$configText = $null
if ($PlantedOffset) {
    $configText = [System.IO.File]::ReadAllText($configFile)
    $value = '"HMI_SIMULATION_PART_OFFSET": [' + (($PlantedOffset -split ",") -join ", ") + ']'
    $pattern = '"HMI_SIMULATION_PART_OFFSET"\s*:\s*\[[^\]]*\]'
    if ($configText -match $pattern) { $edited = [regex]::Replace($configText, $pattern, $value) }
    else { $edited = $configText -replace '(\r?\n)(\s*)"CONFIG_FILE_HEALTH"', ('$1$2' + $value + ',$1$2"CONFIG_FILE_HEALTH"') }
    [System.IO.File]::WriteAllText($configFile, $edited)
}
function Restore-Config { if ($configText) { [System.IO.File]::WriteAllText($configFile, $configText) } }

$map = @{ version = "1.0"; entries = @(@{ id = $Part; kind = "project"; path = $Fixture }) } | ConvertTo-Json -Depth 4
[System.IO.File]::WriteAllText("$cfg\work_selection_map.json", $map)
Say "case $Label : part $Part -> $Fixture"
if ($PlantedOffset) { Say "camera test mode: simulated part planted at [$PlantedOffset]" }

$before = @{}
Get-ChildItem "$Hmi\logs\*.log" -ErrorAction SilentlyContinue | ForEach-Object { $before[$_.FullName] = Log-Size $_.FullName }
$psi = New-Object System.Diagnostics.ProcessStartInfo "$Hmi\TGuideWeldingHMI.exe"
$psi.WorkingDirectory = $Hmi
$psi.UseShellExecute = $false
$psi.EnvironmentVariables["QT_QPA_PLATFORM"] = "windows:nowmpointer"
$proc = [System.Diagnostics.Process]::Start($psi)
Say "HMI started, pid $($proc.Id)"

# The HMI's log rotates between log_0.log and log_1.log at about 1 MB, so this run's text can be
# in either file, or both: read each from where it stood before the launch (from 0 if it was
# restarted since).
function Read-RunText {
    $text = ""
    foreach ($f in Get-ChildItem "$Hmi\logs\*.log" | Sort-Object LastWriteTime) {
        $from = if ($before.ContainsKey($f.FullName)) { $before[$f.FullName] } else { 0 }
        if ((Log-Size $f.FullName) -lt $from) { $from = 0 }
        $text += Read-LogFrom $f.FullName $from
    }
    return $text
}
function Wait-RunLog([string]$pattern, [int]$timeoutS) {
    $t0 = Get-Date
    while (((Get-Date) - $t0).TotalSeconds -lt $timeoutS) {
        if ((Read-RunText) -match $pattern) { return $true }
        Start-Sleep -Milliseconds 500
    }
    return $false
}
function Close-Hmi {
    for ($try = 0; $try -lt 3 -and -not $proc.HasExited; $try++) {
        $m = Find-Top $proc.Id "GUIMainWindow"
        if ($m) { [void][W]::PostMessage((Get-Hwnd $m), 0x0010, [IntPtr]0, [IntPtr]0) }
        [void]$proc.WaitForExit(20000)
        Dismiss-Boxes
    }
    if (-not $proc.HasExited) { $proc.Kill(); Remove-Item "$cfg\app_running.flag" -ErrorAction SilentlyContinue; Say "killed the HMI (crash flag removed)" }
    else { Say "HMI closed, exit $($proc.ExitCode)" }
}
# A case that fails before the run still closes its HMI: a leftover one blocks every later case.
function Fail-Case([string]$why) { Say "FAIL: $why"; Set-Content "$OutDir\$Label.hmi.log" (Read-RunText); Close-Hmi; Restore-Config; exit 1 }

$main = $null
for ($i = 0; $i -lt 240 -and $main -eq $null; $i++) { Start-Sleep -Milliseconds 500; $main = Find-Top $proc.Id "GUIMainWindow" }
if ($main -eq $null) { Fail-Case "no main window" }
Start-Sleep -Seconds 3

function Dismiss-Boxes {
    foreach ($e in $AE::RootElement.FindAll($TS::Children, (New-Object System.Windows.Automation.PropertyCondition($AE::ProcessIdProperty, $proc.Id)))) {
        if ($e.Current.ClassName -eq "QMessageBox") {
            $texts = @()
            foreach ($d in $e.FindAll($TS::Descendants, [System.Windows.Automation.Condition]::TrueCondition)) { if ($d.Current.Name) { $texts += $d.Current.Name } }
            Say ("message box: " + ($texts -join " | "))
            $ok = Find-Desc $e "OK" "QPushButton"
            if ($ok) { Post-Click $ok } else { [void][W]::PostMessage((Get-Hwnd $e), 0x0010, [IntPtr]0, [IntPtr]0) }
            Start-Sleep -Seconds 1
        }
    }
}
Dismiss-Boxes

# Simulation mode: the check box in the menu bar's corner (not exposed to UIA), 33 px from its right end.
$menubar = $null
foreach ($e in $main.FindAll($TS::Descendants, [System.Windows.Automation.Condition]::TrueCondition)) {
    if ($e.Current.ControlType -eq [System.Windows.Automation.ControlType]::MenuBar) { $menubar = $e; break }
}
$simOn = $false
for ($try = 0; $try -lt 4 -and -not $simOn; $try++) {
    $r = $menubar.Current.BoundingRectangle
    Post-ClickAt (Get-Hwnd $main) ([int]($r.Right - 33)) ([int]($r.Y + $r.Height / 2))
    $simOn = Wait-RunLog "Simulation mode turned ON" 5
}
if (-not $simOn) { Fail-Case "simulation mode did not turn on" }
Say "simulation mode ON"

$armed = $false
for ($try = 0; $try -lt 4 -and -not $armed; $try++) {
    $btn = Find-Desc $main "Connect" "QPushButton"
    if ($btn) { Post-Click $btn }
    $armed = Wait-RunLog "Socket start: armed" 20
    Dismiss-Boxes
}
if (-not $armed) { Fail-Case "not armed" }
Say "armed"

# The fixture gives the checks the part's pose in the station frame (coordinated welds).
$plantedArgs = if ($PlantedOffset) { @("--planted", $PlantedOffset, "--fixture", $Fixture) } else { @() }
if (-not $DisarmOnLog) {
    & $Python "$here\hmi_case.py" --label $Label --expect $Expect --part $Part --out-dir $OutDir --max-seconds $MaxSeconds @plantedArgs 2>&1 |
        ForEach-Object { Say "case: $_" }
    $caseExit = $LASTEXITCODE
} else {
    # The operator disarms during the run (readiness plan F8 / H12): Disconnect is clicked as
    # soon as the HMI log matches $DisarmOnLog, while the run goes on.
    $caseOut = "$OutDir\$Label.case_out.txt"
    $case = Start-Process -FilePath $Python -NoNewWindow -PassThru -RedirectStandardOutput $caseOut -RedirectStandardError "$caseOut.err" `
        -ArgumentList @("`"$here\hmi_case.py`"", "--label", $Label, "--expect", "`"$Expect`"", "--part", $Part,
                        "--out-dir", "`"$OutDir`"", "--max-seconds", $MaxSeconds) +
                        $(if ($PlantedOffset) { @("--planted", $PlantedOffset, "--fixture", "`"$Fixture`"") } else { @() })
    $disarmed = $false
    while (-not $case.HasExited) {
        if (-not $disarmed -and (Read-RunText) -match $DisarmOnLog) {
            $btn = Find-Desc $main "Disconnect" "QPushButton"
            if ($btn) { Post-Click $btn; $disarmed = $true; Say "operator disarm: Disconnect clicked (log matched '$DisarmOnLog')" }
        }
        Start-Sleep -Milliseconds 300
    }
    $case.WaitForExit(); $caseExit = $case.ExitCode
    Get-Content $caseOut, "$caseOut.err" -ErrorAction SilentlyContinue | ForEach-Object { Say "case: $_" }
    if (-not $disarmed) { Say "operator disarm: the log never matched '$DisarmOnLog'" }
}
Dismiss-Boxes
[void](Wait-RunLog "Socket start: run ended|Socket start: refused|disarmed" 30)
Start-Sleep -Seconds 2
Set-Content "$OutDir\$Label.hmi.log" (Read-RunText)
Say "HMI log saved to $OutDir\$Label.hmi.log"

if (-not $KeepOpen) { Close-Hmi }
Restore-Config
Say "case exit $caseExit"
exit $caseExit
