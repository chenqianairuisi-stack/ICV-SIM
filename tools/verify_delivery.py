"""Validate generated handoff, browser resource loading, and animation frames."""
from pathlib import Path
from PIL import Image
from playwright.sync_api import sync_playwright
ROOT = Path(__file__).resolve().parents[1]
for key in ("traffic_red", "obstacle_start", "parking", "city"):
    image = Image.open(ROOT / "output" / (key + ".gif"))
    assert image.n_frames > 100
    image.seek(image.n_frames // 2)
    assert len(image.convert("RGB").getcolors(800 * 440)) > 10
with sync_playwright() as p:
    browser = p.chromium.launch(channel="msedge", headless=True)
    page = browser.new_page(viewport={"width": 1280, "height": 900})
    errors = []
    page.on("pageerror", lambda error: errors.append(error.stack))
    page.goto((ROOT / "README.html").as_uri())
    page.wait_for_timeout(300)
    assert page.locator("#run-results-table tbody tr").count() == 9
    assert page.locator(".toc a").count() >= 10
    assert page.locator("h3").count() >= 5
    assert "当前不是 MPC" in page.locator("main").inner_text()
    assert page.locator("img").evaluate_all("items=>items.every(i=>i.complete&&i.naturalWidth>0)")
    page.screenshot(path=str(ROOT / "output" / "screenshots" / "handoff.png"))
    page.set_viewport_size({"width": 390, "height": 844})
    assert page.evaluate("document.documentElement.scrollWidth<=innerWidth")
    assert not errors, errors
    browser.close()
print("Handoff and 4 GIFs verified")
