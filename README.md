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

https://mkuener.github.io/ski-hud/harness/ is the wiring harness. The 3D-printed housings come in two versions, one per display being tested: https://mkuener.github.io/ski-hud/housing/ (variant A, 0.96" colour LCD) and https://mkuener.github.io/ski-hud/housing-oled/ (variant B, 0.96" OLED, with the bench test clip). Sources: [harness/index.html](harness/index.html), [housing/](housing/) and [housing-oled/](housing-oled/).
