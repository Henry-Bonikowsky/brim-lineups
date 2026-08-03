# Renders results/*.json onto the extracted minimap textures as browsable
# lineup cards: cards/index.html. Re-run after regenerating results.
$root = $PSScriptRoot
$cams = Get-Content "$root\cards\cameras.json" -Raw | ConvertFrom-Json
$TOP = 12

function Project($cam, $x, $y) {
    $th = $cam.yaw * [math]::PI / 180.0
    $ux = [math]::Cos($th); $uy = [math]::Sin($th)      # image-up axis in world
    $rx = -$uy; $ry = $ux                                # image-right axis
    $dx = $x - $cam.cx; $dy = $y - $cam.cy
    $u = 0.5 + ($dx * $rx + $dy * $ry) / $cam.ortho
    $v = 0.5 - ($dx * $ux + $dy * $uy) / $cam.ortho
    @($u, $v)
}

$html = @"
<!doctype html><meta charset="utf-8"><title>Brim molly lineups</title>
<style>
body { background:#14181c; color:#dde; font:14px/1.4 system-ui; margin:20px }
.map { margin-bottom:40px } .wrap { position:relative; display:inline-block }
.wrap img { width:768px; image-rendering:auto; opacity:.9 }
svg { position:absolute; inset:0; width:100%; height:100% }
h2 { color:#7fd } table { border-collapse:collapse; margin:8px 0 } td,th { padding:2px 10px; text-align:right }
th { color:#8ac } .n { fill:#fff; font-size:9px; text-anchor:middle }
</style>
<h1>Brim molly lineups (patch 13.02 file physics; knobs at file defaults)</h1>
<p>Dot = stand spot, ring = target, line = throw direction. Hover a dot for aim numbers.
Columns: crosshair yaw/pitch deg, flight time, landing error, aim forgiveness.</p>
"@

foreach ($mapFile in (Get-ChildItem "$root\results\*.json" | Group-Object { ($_.BaseName -split "-")[0] })) {
    $map = $mapFile.Name
    $cam = $cams.$map
    if ($null -eq $cam -or -not (Test-Path "$root\cards\$map.png")) { continue }
    $html += "<div class=map><h2>$map</h2><div class=wrap><img src='$map.png'><svg viewBox='0 0 1000 1000'>"
    $tables = ""
    $siteIdx = 0
    $colors = @("#ffd166", "#66d1ff", "#ff8fa3", "#9dff8f")
    foreach ($f in $mapFile.Group) {
        $lineups = Get-Content $f.FullName -Raw | ConvertFrom-Json
        if ($lineups.Count -eq 0) { $siteIdx++; continue }
        $col = $colors[$siteIdx % 4]
        # target = infer from filename tag written by the batch runner
        $tag = ($f.BaseName -replace "^$map-", "") -replace "m", "-"
        $tc = $tag -split "_"
        $tuv = Project $cam ([double]$tc[0]) ([double]$tc[1])
        $html += ("<circle cx='{0:F1}' cy='{1:F1}' r='14' fill='none' stroke='{2}' stroke-width='3'/>" -f ($tuv[0]*1000), ($tuv[1]*1000), $col)
        $tables += "<h3 style='color:$col'>$map target ($($tc -join ', '))</h3><table><tr><th>#</th><th>stand</th><th>yaw</th><th>pitch</th><th>time</th><th>err</th><th>forgive</th><th>crosshair on</th></tr>"
        $i = 1
        foreach ($l in ($lineups | Sort-Object time | Select-Object -First $TOP)) {
            $suv = Project $cam $l.stand[0] $l.stand[1]
            $x = $suv[0]*1000; $y = $suv[1]*1000
            $html += ("<line x1='{0:F1}' y1='{1:F1}' x2='{2:F1}' y2='{3:F1}' stroke='{4}' stroke-width='1' opacity='.35'/>" -f $x, $y, ($tuv[0]*1000), ($tuv[1]*1000), $col)
            $html += ("<circle cx='{0:F1}' cy='{1:F1}' r='7' fill='{2}' opacity='.9'><title>#{3}: yaw {4:F1} pitch {5:F1}, {6:F2}s, err {7:F0}u, forgive {8:P0}</title></circle>" -f $x, $y, $col, $i, $l.yaw, $l.pitch, $l.time, $l.err, $l.forgive)
            $html += ("<text class=n x='{0:F1}' y='{1:F1}'>{2}</text>" -f $x, ($y+3), $i)
            $aim = if ($l.aim_ref) { "({0:F0}, {1:F0}, {2:F0}) at {3:F0}u" -f $l.aim_ref[0], $l.aim_ref[1], $l.aim_ref[2], $l.aim_ref[3] } else { "open sky" }
            $tables += ("<tr><td>{0}</td><td>({1:F0}, {2:F0}, {3:F0})</td><td>{4:F1}</td><td>{5:F1}</td><td>{6:F2}s</td><td>{7:F0}u</td><td>{8:P0}</td><td>{9}</td></tr>" -f $i, $l.stand[0], $l.stand[1], $l.stand[2], $l.yaw, $l.pitch, $l.time, $l.err, $l.forgive, $aim)
            $i++
        }
        $tables += "</table>"
        # synthetic first-person screenshots, if rendered for this site
        $renders = Get-ChildItem "$root\cards\renders2\$($f.BaseName)_r*.bmp" -ErrorAction SilentlyContinue | Sort-Object Name
        if ($renders) {
            $tables += "<details><summary>aim screenshots (match your screen to the image; green cross = crosshair)</summary>"
            $i = 1
            foreach ($r in $renders) {
                $tables += "<div>#$i</div><img src='renders2/$($r.Name)?v=$($r.LastWriteTime.Ticks)' style='width:640px;margin:4px 0'>"
                $i++
            }
            $tables += "</details>"
        }
        $siteIdx++
    }
    $html += "</svg></div>$tables</div>"
}
$html | Out-File "$root\cards\index.html" -Encoding utf8
Write-Host "wrote cards/index.html"

