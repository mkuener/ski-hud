# Ski HUD phone page

Sends speed, altitude and vertical drop from the phone's GPS to the Ski HUD goggles over Bluetooth.

Open it at https://mkuener.github.io/ski-hud/ in a browser with Web Bluetooth. On iPhone that means Bluefy (free, on the App Store); Safari can't use Bluetooth from a web page.

1. **Connect goggles** and pick **SkiHUD**.
2. **Start GPS** and allow location. Indoors, use **Demo values** instead.
3. Keep the screen on (Auto-Lock → Never if the page can't do it itself).

The goggles show `--` for anything unknown, and blank everything if the page stops sending for 5 seconds.

## Optics bench

https://mkuener.github.io/ski-hud/optics/ is a real-scale 3D model of the HUD optics in the goggles: panel, f30 doublet, 45° mirror and film combiner, with live ray tracing, an eye view and a build sheet. Source: [optics/index.html](optics/index.html).

## Harness and housings

https://mkuener.github.io/ski-hud/harness/ is the wiring harness and https://mkuener.github.io/ski-hud/housing/ the 3D-printed housings. Sources: [harness/index.html](harness/index.html), [housing/index.html](housing/index.html) and [housing/cad.js](housing/cad.js).
