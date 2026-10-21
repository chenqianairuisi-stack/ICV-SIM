"""Desktop/mobile playback verification and screenshots with Playwright."""
from pathlib import Path
import json
from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "output" / "screenshots"
OUT.mkdir(exist_ok=True)
with sync_playwright() as p:
    browser = p.chromium.launch(channel="msedge", headless=True)
    errors = []
    page = browser.new_page(viewport={"width": 1440, "height": 900}, device_scale_factor=1)
    page.on("pageerror", lambda error: errors.append(error.stack))
    page.goto((ROOT / "web" / "index.html").as_uri())
    page.wait_for_timeout(500)
    page.click("#play")
    first = page.locator("#time").inner_text()
    page.click("#play")
    page.wait_for_timeout(500)
    assert page.locator("#time").inner_text() != first, "Playback did not advance"
    assert page.locator(".scenario").count() == 9
    for key, button, fraction in [("traffic", 0, .38), ("parking", 3, .64), ("city", 4, .42)]:
        page.locator(".scenario").nth(button).click()
        page.evaluate("playing=false;clock=replay.result.time_s*" + str(fraction) + ";draw()")
        assert page.evaluate("ctx.getImageData(0,0,canvas.width,canvas.height).data.some((v,i)=>i%4===0&&v>60)")
        page.screenshot(path=str(OUT / (key + "_desktop.png")), full_page=True)
    for size in [(390, 844), (768, 1024)]:
        page.set_viewport_size({"width": size[0], "height": size[1]})
        page.wait_for_timeout(150)
        assert page.evaluate("document.documentElement.scrollWidth<=innerWidth"), "Horizontal overflow"
        page.screenshot(path=str(OUT / ("mobile.png" if size[0] == 390 else "tablet.png")), full_page=True)
    assert not errors, errors
    (OUT / "verification.json").write_text(json.dumps({"page_errors": errors, "viewports": [1440, 390, 768], "playback_advanced": True, "canvas_nonblank": True}, indent=2), encoding="utf-8")
    browser.close()
print("Browser verification passed")
