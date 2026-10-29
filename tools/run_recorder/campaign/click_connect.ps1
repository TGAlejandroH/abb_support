# Click the HMI's Connect / Disconnect button (posted message), e.g. to re-arm after a disarm.
param([string]$Hmi = "C:\Users\AlejandroHernandez\source\repos\TGAlejandroH\hmi-endstatus-wt\TGuideWeldingHMI\x64\Release",
      [ValidateSet("Connect", "Disconnect")][string]$Button = "Connect")
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
. "$here\..\gui_lib.ps1"
$p = Get-Process TGuideWeldingHMI | Where-Object { $_.Path -like "$Hmi*" } | Select-Object -First 1
if (-not $p) { "FAIL: no HMI running from $Hmi"; exit 2 }
$main = Find-Top $p.Id "GUIMainWindow"
$btn = Find-Desc $main $Button "QPushButton"
if (-not $btn) { "FAIL: no $Button button"; exit 1 }
# UI Automation's Invoke: a posted click on this button is sometimes not taken once the HMI has
# been running a while (seen 2026-10-28, two posted clicks lost; Invoke took at once).
$btn.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern).Invoke()
"clicked $Button"
