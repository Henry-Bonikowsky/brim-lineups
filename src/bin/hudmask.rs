//! One-off: extract the static HUD (ability bar band) from game frames.
//! Pixels whose color is near-constant across frames from DIFFERENT clips are
//! UI; everything else is world. Writes cards/hud.bmp (mean color) and
//! cards/hudmask.bmp (white = HUD pixel), both 1152x720.
//!
//! Usage: hudmask <framesDir with h*.bmp>

fn read_bmp(path: &std::path::Path) -> Option<(usize, usize, Vec<u8>)> {
    let d = std::fs::read(path).ok()?;
    if &d[0..2] != b"BM" {
        return None;
    }
    let off = u32::from_le_bytes(d[10..14].try_into().unwrap()) as usize;
    let w = i32::from_le_bytes(d[18..22].try_into().unwrap()) as usize;
    let h = i32::from_le_bytes(d[22..26].try_into().unwrap()) as usize;
    let bpp = u16::from_le_bytes(d[28..30].try_into().unwrap());
    if bpp != 24 {
        return None;
    }
    let row = (w * 3 + 3) & !3;
    let mut px = vec![0u8; w * h * 3];
    for y in 0..h {
        let src = off + (h - 1 - y) * row; // bottom-up -> top-down
        px[y * w * 3..y * w * 3 + w * 3].copy_from_slice(&d[src..src + w * 3]);
    }
    Some((w, h, px))
}

fn main() {
    let dir = std::env::args().nth(1).expect("frames dir");
    let frames: Vec<_> = std::fs::read_dir(&dir)
        .expect("dir")
        .filter_map(|e| e.ok())
        .filter(|e| e.file_name().to_string_lossy().ends_with(".bmp"))
        .filter_map(|e| read_bmp(&e.path()))
        .collect();
    assert!(frames.len() >= 8, "need frames, got {}", frames.len());
    let (w, h, _) = frames[0];
    let n = frames.len() as f32;

    let mut mean = vec![0f32; w * h * 3];
    for (_, _, px) in &frames {
        for (m, &p) in mean.iter_mut().zip(px.iter()) {
            *m += p as f32 / n;
        }
    }
    let mut var = vec![0f32; w * h]; // max channel stddev per pixel
    for (_, _, px) in &frames {
        for i in 0..w * h {
            for c in 0..3 {
                let d = px[i * 3 + c] as f32 - mean[i * 3 + c];
                var[i] += d * d / n;
            }
        }
    }

    // ability-bar band only: x 28%..72%, y 86%..99.5%
    let (x0, x1) = ((w as f32 * 0.28) as usize, (w as f32 * 0.72) as usize);
    let (y0, y1) = ((h as f32 * 0.86) as usize, (h as f32 * 0.995) as usize);
    let mut hud = vec![0u8; w * h * 3];
    let mut mask = vec![0u8; w * h * 3];
    let mut count = 0usize;
    for y in y0..y1 {
        for x in x0..x1 {
            let i = y * w + x;
            if var[i].sqrt() < 16.0 {
                for c in 0..3 {
                    hud[i * 3 + c] = mean[i * 3 + c] as u8;
                    mask[i * 3 + c] = 255;
                }
                count += 1;
            }
        }
    }
    eprintln!("{count} static HUD pixels in the band");

    let write = |path: &str, px: &[u8]| {
        let row = w * 3;
        let mut bmp = Vec::with_capacity(54 + row * h);
        bmp.extend_from_slice(b"BM");
        bmp.extend_from_slice(&((54 + row * h) as u32).to_le_bytes());
        bmp.extend_from_slice(&[0; 4]);
        bmp.extend_from_slice(&54u32.to_le_bytes());
        bmp.extend_from_slice(&40u32.to_le_bytes());
        bmp.extend_from_slice(&(w as i32).to_le_bytes());
        bmp.extend_from_slice(&(h as i32).to_le_bytes());
        bmp.extend_from_slice(&1u16.to_le_bytes());
        bmp.extend_from_slice(&24u16.to_le_bytes());
        bmp.extend_from_slice(&[0; 24]);
        for y in (0..h).rev() {
            bmp.extend_from_slice(&px[y * row..y * row + row]);
        }
        std::fs::write(path, bmp).unwrap();
    };
    write("cards/hud.bmp", &hud);
    write("cards/hudmask.bmp", &mask);
    eprintln!("wrote cards/hud.bmp + cards/hudmask.bmp ({w}x{h})");
}
