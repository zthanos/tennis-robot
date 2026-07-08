param(
  [string]$Port = "COM8",
  [int]$Baud = 9600,
  [int]$ListenPort = 8091,
  [int]$InitialPwm = 75
)

$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud, 'None', 8, 'One')
$serial.WriteTimeout = 1000
$serial.Open()
Start-Sleep -Milliseconds 1800

$listener = [System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Any, $ListenPort)
$listener.Start()
$running = $false
# The validated diagnostic sketch resets to PWM 255. Step it down before any
# start command; full PWM caused the TB6612/power path to cycle in pulses.
$pwm = 255
while ($pwm -gt $InitialPwm) {
  $serial.Write("-")
  $pwm = [Math]::Max(0, $pwm - 10)
  Start-Sleep -Milliseconds 20
}

try {
  while ($true) {
    if ($listener.Pending()) {
      $client = $listener.AcceptTcpClient()
      try {
        $stream = $client.GetStream()
        $reader = [System.IO.StreamReader]::new($stream)
        $writer = [System.IO.StreamWriter]::new($stream)
        $writer.AutoFlush = $true
        $request = $reader.ReadLine() | ConvertFrom-Json
        switch ($request.action) {
          # The hardware-validated sketch latches f/r until it receives s.
          # Send exactly one command; repeated f bytes are not motor pulses.
          "start"      { $serial.Write("f"); $running = $true }
          "stop"       { $serial.Write("s"); $running = $false }
          "speed_up"   { $serial.Write("+"); $pwm = [Math]::Min(255, $pwm + 10) }
          "speed_down" { $serial.Write("-"); $pwm = [Math]::Max(0, $pwm - 10) }
        }
        $writer.WriteLine((@{ok=$true; running=$running; speed=$pwm; port=$Port} | ConvertTo-Json -Compress))
      } catch {
        try { $writer.WriteLine((@{ok=$false; message=$_.Exception.Message} | ConvertTo-Json -Compress)) } catch {}
      } finally {
        $client.Close()
      }
    }
    Start-Sleep -Milliseconds 40
  }
} finally {
  $listener.Stop()
  if ($serial.IsOpen) { $serial.Write("s"); $serial.Close() }
}
