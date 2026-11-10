# Ski HUD phone page

Sends speed, altitude and vertical drop from the phone's GPS to the Ski HUD goggles over Bluetooth.

Open it at https://mkuener.github.io/ski-hud/ in a browser with Web Bluetooth. On iPhone that means Bluefy (free, on the App Store); Safari can't use Bluetooth from a web page.

1. **Connect goggles** and pick **SkiHUD**.
2. **Start GPS** and allow location. Indoors, use **Demo values** instead.
3. Keep the screen on (Auto-Lock → Never if the page can't do it itself).

The goggles show `--` for anything unknown, and blank everything if the page stops sending for 5 seconds.

## Optics bench

https://mkuener.github.io/ski-hud/optics/ is a real-scale 3D model of the HUD optics in the goggles: panel, f30 doublet, 45° mirror and film combiner, with live ray tracing, an eye view and a build sheet. Source: [optics/index.html](optics/index.html).

## Private pages

https://mkuener.github.io/ski-hud/harness/ (wiring harness) and https://mkuener.github.io/ski-hud/housing/ (3D-printed housings) are password protected. The files in this repo are encrypted (AES-256-GCM, key from the password via PBKDF2-SHA256); the browser decrypts them after the password is entered. Their sources are not in this repo.
