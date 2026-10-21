# ZIPY serial logger for Windows - use it instead of the Arduino Serial Monitor.
#
# Shows everything the Teensy prints and saves it to logs\<date-time>.txt, so
# the whole session can be read back afterwards - nothing to copy and paste.
# Type commands exactly as in the Serial Monitor (h, z, c, m, d, u, x ...).
# Every key you press is marked in the log as  >> h  so it is clear what
# happened when.
#
# Esc quits. It sends x first, so the motor is never left running.
# Close the Arduino Serial Monitor first: only one program can use the port.
#
#   powershell -ExecutionPolicy Bypass -File tools\serial_log.ps1
#   powershell -ExecutionPolicy Bypass -File tools\serial_log.ps1 -Port COM5

param([string]$Port = '')

if (-not $Port) {
    # PJRC's USB vendor id is 16C0 - that picks the Teensy out of any other ports
    $dev = Get-CimInstance Win32_PnPEntity |
        Where-Object { $_.DeviceID -match 'VID_16C0' -and $_.Name -match '\(COM\d+\)' } |
        Select-Object -First 1
    if (-not $dev) { Write-Host "No Teensy found. Is it plugged in?"; exit 1 }
    $Port = [regex]::Match($dev.Name, 'COM\d+').Value
}

$logDir = Join-Path (Split-Path $PSScriptRoot -Parent) 'logs'
New-Item -ItemType Directory -Force $logDir | Out-Null
$log = Join-Path $logDir ((Get-Date -Format 'yyyyMMdd-HHmmss') + '.txt')

$p = New-Object System.IO.Ports.SerialPort $Port, 115200
$p.DtrEnable = $true
try { $p.Open() }
catch {
    Write-Host "Could not open $Port - close the Arduino Serial Monitor first, then try again."
    exit 1
}

$w = New-Object System.IO.StreamWriter($log, $true)
$w.AutoFlush = $true
Write-Host "Connected to $Port. Saving to $log" -ForegroundColor Cyan
Write-Host "Type commands as usual (h, z, c, x ...). Esc quits." -ForegroundColor Cyan

$running = $true
try {
    while ($running) {
        $s = $p.ReadExisting()
        if ($s) { [Console]::Write($s); $w.Write($s) }
        while ([Console]::KeyAvailable) {
            $k = [Console]::ReadKey($true)
            if ($k.Key -eq 'Escape') { $running = $false; break }
            $ch = $k.KeyChar
            if ([int]$ch -gt 32) {
                $p.Write([string]$ch)
                Write-Host "`n>> $ch" -ForegroundColor Yellow
                $w.Write("`n>> $ch`n")
            }
        }
        Start-Sleep -Milliseconds 20
    }
}
finally {
    if ($p.IsOpen) { $p.Write('x'); Start-Sleep -Milliseconds 150; $w.Write($p.ReadExisting()); $p.Close() }
    $w.Close()
    Write-Host "`nStopped (sent x). Log saved: $log" -ForegroundColor Cyan
}
