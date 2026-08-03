param([switch]$Open)
$env:PATH = [System.Environment]::GetEnvironmentVariable('Path','User') + ';' + $env:PATH
$exe = Join-Path $PSScriptRoot 'target\release\brim-lineups.exe'
Start-Process -WindowStyle Hidden $exe -ArgumentList 'serve'
# no browser tab by default (focus-stealing mid-game); pass -Open to launch one
if ($Open) {
    Start-Sleep 1
    Start-Process 'http://localhost:8777/picker.html'
}
