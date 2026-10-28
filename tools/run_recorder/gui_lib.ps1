# Drive the HMI's Qt GUI with posted messages + UIA (never the real mouse).
Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
Add-Type @"
using System;
using System.Runtime.InteropServices;
public static class W {
  [DllImport("user32.dll")] public static extern bool PostMessage(IntPtr h, uint m, IntPtr w, IntPtr l);
  [DllImport("user32.dll")] public static extern bool ScreenToClient(IntPtr h, ref POINT p);
  [DllImport("user32.dll")] public static extern IntPtr SetProcessDpiAwarenessContext(IntPtr v);
  [StructLayout(LayoutKind.Sequential)] public struct POINT { public int X; public int Y; }
}
"@
[void][W]::SetProcessDpiAwarenessContext([IntPtr](-4))
$AE = [System.Windows.Automation.AutomationElement]
$TS = [System.Windows.Automation.TreeScope]

function Find-Top([int]$procId, [string]$class) {
    $cond = New-Object System.Windows.Automation.PropertyCondition($AE::ProcessIdProperty, $procId)
    foreach ($e in $AE::RootElement.FindAll($TS::Children, $cond)) {
        if (-not $class -or $e.Current.ClassName -eq $class) { return $e }
    }
    return $null
}

function Find-Desc($root, [string]$name, [string]$class) {
    foreach ($e in $root.FindAll($TS::Descendants, [System.Windows.Automation.Condition]::TrueCondition)) {
        if (($name -eq "" -or $e.Current.Name -eq $name) -and ($class -eq "" -or $e.Current.ClassName -eq $class)) { return $e }
    }
    return $null
}

function Get-Hwnd($el) {
    $walker = [System.Windows.Automation.TreeWalker]::ControlViewWalker
    $cur = $el
    while ($cur -ne $null) {
        $h = $cur.Current.NativeWindowHandle
        if ($h -ne 0) { return [IntPtr]$h }
        $cur = $walker.GetParent($cur)
    }
    return [IntPtr]::Zero
}

function Post-ClickAt([IntPtr]$hwnd, [int]$sx, [int]$sy) {
    $p = New-Object W+POINT; $p.X = $sx; $p.Y = $sy
    [void][W]::ScreenToClient($hwnd, [ref]$p)
    $l = [IntPtr](($p.Y -shl 16) -bor ($p.X -band 0xFFFF))
    [void][W]::PostMessage($hwnd, 0x0200, [IntPtr]0, $l)   # WM_MOUSEMOVE
    Start-Sleep -Milliseconds 60
    [void][W]::PostMessage($hwnd, 0x0201, [IntPtr]1, $l)   # WM_LBUTTONDOWN
    Start-Sleep -Milliseconds 60
    [void][W]::PostMessage($hwnd, 0x0202, [IntPtr]0, $l)   # WM_LBUTTONUP
}

function Post-Click($el) {
    $r = $el.Current.BoundingRectangle
    Post-ClickAt (Get-Hwnd $el) ([int]($r.X + $r.Width / 2)) ([int]($r.Y + $r.Height / 2))
}

# Log reading: the HMI holds the file open; read through a shared stream.
function Read-LogFrom([string]$path, [long]$offset) {
    $fs = [System.IO.File]::Open($path, 'Open', 'Read', 'ReadWrite')
    try {
        [void]$fs.Seek($offset, 'Begin')
        $sr = New-Object System.IO.StreamReader($fs)
        return $sr.ReadToEnd()
    } finally { $fs.Close() }
}
function Log-Size([string]$path) {
    if (-not (Test-Path $path)) { return 0 }
    $fs = [System.IO.File]::Open($path, 'Open', 'Read', 'ReadWrite')
    try { return $fs.Length } finally { $fs.Close() }
}
function Wait-Log([string]$path, [long]$offset, [string]$pattern, [int]$timeoutS) {
    $t0 = Get-Date
    while (((Get-Date) - $t0).TotalSeconds -lt $timeoutS) {
        if ((Read-LogFrom $path $offset) -match $pattern) { return $true }
        Start-Sleep -Milliseconds 500
    }
    return $false
}
