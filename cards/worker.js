// Solver worker for the static (GitHub Pages) build: the wasm solve can run
// for a while, so it must never block the page. Same JSON as the native
// server's /solve (see src/api.rs).
import init, { load_map, solve_json, render_lineup, traj_json, flight_stills } from './pkg/brim_lineups.js';
const ready = init();
onmessage = async e => {
  const { id, cmd, args } = e.data;
  try {
    await ready;
    let result, transfer = [];
    if (cmd === 'load') { load_map(new Uint8Array(args[0])); result = true; }
    else if (cmd === 'solve') result = solve_json(...args);
    else if (cmd === 'render') { result = render_lineup(...args).map(u => u.buffer); transfer = result; }
    else if (cmd === 'traj') result = traj_json(...args);
    else if (cmd === 'stills') { result = flight_stills(...args).map(u => u.buffer); transfer = result; }
    postMessage({ id, result }, transfer);
  } catch (err) {
    postMessage({ id, error: String(err) });
  }
};
