# Ski HUD phone page

Sends speed, altitude and vertical drop from the phone's GPS to the Ski HUD goggles over Bluetooth.

Open it at https://mkuener.github.io/ski-hud/ in a browser with Web Bluetooth. On iPhone that means Bluefy (free, on the App Store); Safari can't use Bluetooth from a web page.

1. **Connect goggles** and pick **SkiHUD**.
2. **Start GPS** and allow location. Indoors, use **Demo values** instead.
3. Keep the screen on (Auto-Lock → Never if the page can't do it itself).

The goggles show `--` for anything unknown, and blank everything if the page stops sending for 5 seconds.
