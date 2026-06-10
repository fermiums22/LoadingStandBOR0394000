<#
  console.ps1 - open the BORK loading-stand console and reset the board so the
  BORK logo is shown on connect.

  The ST-Link Virtual COM Port cannot tell the MCU when the host opens the port,
  so we trigger the logo by pulsing the target reset (over SWD) right after the
  terminal is opened. The terminal opens first, then the reset is issued, so the
  boot banner lands in the already-open terminal.

  Usage:   powershell -ExecutionPolicy Bypass -File tools\console.ps1
           tools\console.ps1 -Port COM14
#>
param(
    [string]$Port = "COM14",
    [int]   $Baud = 921600
)

$putty = "C:\Program Files\PuTTY\putty.exe"
$cli   = "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"

Write-Host "Opening $Port @ $Baud (8N1)..."
Start-Process $putty -ArgumentList '-serial', $Port, '-sercfg', "$Baud,8,n,1,N"

Start-Sleep -Milliseconds 800
Write-Host "Resetting target so the BORK logo is shown..."
& $cli -c port=SWD mode=HOTPLUG -rst | Out-Null
Write-Host "Done. Type 'help' in the PuTTY window."
