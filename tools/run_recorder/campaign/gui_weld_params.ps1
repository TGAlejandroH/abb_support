# The operator's weld-parameter edit on a running HMI, between runs (readiness plan P3/P4/P7):
# Tools > Manual touch up mode, select the weld, "Modify weld parameters", tick or untick
# "Use Custom Parameters", type values on the keypad, Save, leave Manual touch up mode.
# Driven with posted messages and UI Automation, never the real mouse (..\gui_lib.ps1).
# -Set types any field the panel lets the operator change, by its caption: "Travel Speed (IPM)",
# "Wire Speed (IPM)", "Arc Length"; "Proc #=+1" presses the procedure's up arrow once (it has no
# keypad). Job mode greys all but the travel speed.
#
#   .\gui_weld_params.ps1 -Weld Weld1 -Custom on -Travel 15
#   .\gui_weld_params.ps1 -Weld Weld1 -Index 0 -Custom on -Set "Travel Speed (IPM)=13.75","Arc Length=21.45"
# -KeepManual leaves Manual touch up mode on (several welds in a row; the Tools menu does not
# always open on the first click).
param(
    [Parameter(Mandatory)][string]$Weld,
    [ValidateSet("on", "off", "keep")][string]$Custom = "keep",
    [string]$Travel,
    [string[]]$Set = @(),
    [int]$Index = 0,
    [switch]$NoSave,
    [switch]$KeepManual,
    [string]$Hmi = "C:\Users\AlejandroHernandez\source\repos\TGAlejandroH\hmi-endstatus-wt\TGuideWeldingHMI\x64\Release"
)
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
. "$here\..\gui_lib.ps1"
$p = Get-Process TGuideWeldingHMI | Where-Object { $_.Path -like "$Hmi*" } | Select-Object -First 1
if (-not $p) { "FAIL: no HMI running from $Hmi"; exit 2 }
$main = Find-Top $p.Id "GUIMainWindow"
function All { $main.FindAll($TS::Descendants, [System.Windows.Automation.Condition]::TrueCondition) }
function Tops {
    $cond = New-Object System.Windows.Automation.PropertyCondition($AE::ProcessIdProperty, $p.Id)
    foreach ($top in $AE::RootElement.FindAll($TS::Children, $cond)) { @($top) + @($top.FindAll($TS::Descendants, [System.Windows.Automation.Condition]::TrueCondition)) }
}
function Boxes {
    foreach ($e in Tops) {
        if ($e.Current.ClassName -eq "QMessageBox") {
            $txt = @(); foreach ($d in $e.FindAll($TS::Descendants, [System.Windows.Automation.Condition]::TrueCondition)) { if ($d.Current.Name) { $txt += $d.Current.Name } }
            $r = $e.Current.BoundingRectangle
            "message box ({0}x{1} at {2},{3}): " -f [int]$r.Width, [int]$r.Height, [int]$r.X, [int]$r.Y
            ($txt -join " | ")
            Post-Click (Find-Desc $e "OK" "QPushButton"); Start-Sleep 1
        }
    }
}
function Panel {
    $cb = Find-Desc $main "Use Custom Parameters" "QCheckBox"
    if (-not $cb -or $cb.Current.IsOffscreen) { return "panel not shown" }
    $st = $cb.GetCurrentPattern([System.Windows.Automation.TogglePattern]::Pattern).Current.ToggleState
    $rows = @(); $caption = $null
    foreach ($e in All) {
        if ($e.Current.IsOffscreen) { continue }
        $n = $e.Current.Name
        # Job mode greys the caption with its value (the value labels themselves read as disabled).
        if ($e.Current.ClassName -eq "QLabel" -and $n -match "^(Proc #|Sched #|Wire Speed \(IPM\)|Travel Speed \(IPM\)|Arc Length):$") { $caption = $n; $grey = -not $e.Current.IsEnabled }
        elseif ($caption -and $e.Current.ClassName -eq "QLabel" -and $n -match "^-?[0-9.]+$") {
            $rows += "$caption $n" + $(if ($grey) { " (greyed)" } else { "" }); $caption = $null
        }
    }
    "custom=$st " + ($rows -join ", ")
}
function Toggle-ManualMode {
    $tools = $null; foreach ($e in All) { if ($e.Current.Name -match "^\s*Tools\s*$") { $tools = $e } }
    $menu = $null
    for ($try = 0; $try -lt 4 -and $menu -eq $null; $try++) {
        Post-Click $tools; Start-Sleep 1.5
        foreach ($e in All) { if ($e.Current.ClassName -eq "QMenu" -and -not $e.Current.IsOffscreen) { $menu = $e; break } }
    }
    if ($menu -eq $null) { "FAIL: the Tools menu did not open"; exit 1 }
    Post-Click @($menu.FindAll($TS::Children, [System.Windows.Automation.Condition]::TrueCondition))[0]; Start-Sleep 2
    foreach ($e in All) { if ($e.Current.ClassName -eq "QMenu") { $h = Get-Hwnd $e; [void][W]::PostMessage($h, 0x0100, [IntPtr]0x1B, [IntPtr]0); [void][W]::PostMessage($h, 0x0101, [IntPtr]0x1B, [IntPtr]0) } }
    Start-Sleep 1.5
    Boxes
}

if (-not (Find-Desc $main "Modify weld parameters" "QPushButton")) { Toggle-ManualMode; "manual touch up mode on" }
# The weld list's items are custom widgets with no accessible name: matched by name when they
# have one, otherwise by position (-Index, in list order).
$items = @(); $item = $null
foreach ($e in All) {
    if ($e.Current.ControlType.ProgrammaticName -ne "ControlType.ListItem" -or $e.Current.IsOffscreen) { continue }
    $items += $e
    if ($e.Current.Name -match "\b$([regex]::Escape($Weld))\b") { $item = $e }
}
if (-not $item -and $Index -lt $items.Count) { $item = $items[$Index]; "weld list items have no names: using item $Index of $($items.Count)" }
if (-not $item) { "FAIL: weld $Weld not in the list"; exit 1 }
Post-Click $item; Start-Sleep 1
Post-Click (Find-Desc $main "Modify weld parameters" "QPushButton"); Start-Sleep 2
"before: " + (Panel)

$cb = Find-Desc $main "Use Custom Parameters" "QCheckBox"
$st = $cb.GetCurrentPattern([System.Windows.Automation.TogglePattern]::Pattern).Current.ToggleState
if (($Custom -eq "on" -and $st -ne "On") -or ($Custom -eq "off" -and $st -eq "On")) {
    if (-not $cb.Current.IsEnabled) { "custom check box is disabled (a weld with no preset)" }
    Post-Click $cb; Start-Sleep 1.5
}

if ($Travel) { $Set = @("Travel Speed (IPM)=$Travel") + $Set }
foreach ($pair in $Set) {
    # A value opens the keypad when clicked (only while its buttons are enabled: custom on, and
    # not greyed by Job mode).
    $caption, $typed = $pair -split "=", 2
    if ($typed -match "^\+(\d+)$") {
        # "+n": press the field's up arrow n times (the procedure has no keypad).
        $presses = [int]$Matches[1]
        $up = $null; $after = $false
        foreach ($e in All) {
            if ($e.Current.Name -eq "${caption}:") { $after = $true; continue }
            if ($after -and $e.Current.ClassName -eq "QPushButton" -and $e.Current.Name -match [char]0x2191) { $up = $e; break }
        }
        if (-not $up) { "FAIL: no up arrow for $caption"; exit 1 }
        for ($k = 0; $k -lt $presses; $k++) { Post-Click $up; Start-Sleep 0.5 }
        "pressed $caption up $presses time(s)"
        continue
    }
    $v = $null; $after = $false
    foreach ($e in All) {
        if ($e.Current.Name -eq "${caption}:") { $after = $true; continue }
        if ($after -and $e.Current.ClassName -eq "QLabel" -and $e.Current.Name -match "^-?[0-9.]+$") { $v = $e; break }
    }
    if (-not $v) { "FAIL: no value shown for $caption"; exit 1 }
    Post-Click $v; Start-Sleep 1.5
    $keypad = $null; foreach ($e in Tops) { if ($e.Current.ClassName -eq "NumericKeypadWidget") { $keypad = $e; break } }
    if (-not $keypad) { "FAIL: the keypad did not open for $caption"; exit 1 }
    foreach ($ch in $typed.ToCharArray()) { Post-Click (Find-Desc $keypad ([string]$ch) "QPushButton"); Start-Sleep 0.3 }
    Post-Click (Find-Desc $keypad "Accept" "QPushButton"); Start-Sleep 1.5
    "typed $caption = $typed"
}
"after:  " + (Panel)
if ($NoSave) {
    # Look only: what the operator sees after the tick, then Cancel.
    Post-Click (Find-Desc $main " Cancel " "QPushButton"); Start-Sleep 1.5
    Boxes
    if (-not $KeepManual) { Toggle-ManualMode; "manual touch up mode off (not saved)" }
    return
}
Post-Click (Find-Desc $main " Save " "QPushButton"); Start-Sleep 2
Boxes
if (-not $KeepManual) { Toggle-ManualMode; "manual touch up mode off" }
