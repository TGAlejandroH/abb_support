# Click one Tools menu entry of the running HMI and answer the message boxes that follow
# (Yes / OK), printing their text. Posted messages and UI Automation only (..\gui_lib.ps1).
#
#   .\gui_tools_menu.ps1 -Item "Retrieve robot program from controller"
param(
    [Parameter(Mandatory)][string]$Item,
    [int]$WaitBoxesS = 20,
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

$tools = $null; foreach ($e in All) { if ($e.Current.Name -match "^\s*Tools\s*$") { $tools = $e } }
$entry = $null
for ($try = 0; $try -lt 4 -and $entry -eq $null; $try++) {
    Post-Click $tools; Start-Sleep 1.5
    foreach ($e in Tops) { if ($e.Current.Name -eq $Item -and -not $e.Current.IsOffscreen) { $entry = $e; break } }
}
if ($entry -eq $null) { "FAIL: '$Item' not in the Tools menu"; exit 1 }
Post-Click $entry
"clicked: $Item"
$t0 = Get-Date; $quiet = 0
while (((Get-Date) - $t0).TotalSeconds -lt $WaitBoxesS -and $quiet -lt 4) {
    Start-Sleep 1
    $box = $null; foreach ($e in Tops) { if ($e.Current.ClassName -eq "QMessageBox") { $box = $e; break } }
    if (-not $box) { $quiet++; continue }
    $quiet = 0
    $txt = @(); foreach ($d in $box.FindAll($TS::Descendants, [System.Windows.Automation.Condition]::TrueCondition)) { if ($d.Current.Name) { $txt += $d.Current.Name } }
    "message box: " + ($txt -join " | ")
    $btn = Find-Desc $box "Yes" "QPushButton"; if (-not $btn) { $btn = Find-Desc $box "&Yes" "QPushButton" }
    if (-not $btn) { $btn = Find-Desc $box "OK" "QPushButton" }
    if ($btn) { $btn.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern).Invoke() }
    else { [void][W]::PostMessage((Get-Hwnd $box), 0x0010, [IntPtr]0, [IntPtr]0) }
}
