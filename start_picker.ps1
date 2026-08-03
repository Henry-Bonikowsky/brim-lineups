$env:PATH = [System.Environment]::GetEnvironmentVariable('Path','User') + ';' + $env:PATH
$exe = Join-Path $PSScriptRoot 'target\release\brim-lineups.exe'
Start-Process -WindowStyle Hidden $exe -ArgumentList 'serve'
Start-Sleep 1
Start-Process 'http://localhost:8777/picker.html'
