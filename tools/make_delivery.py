"""Build handoff docs and animations; --docs-only preserves simulation assets."""
import argparse
import json
import html
import math
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from markdown_it import MarkdownIt

ROOT = Path(__file__).resolve().parents[1]
DATA = json.loads((ROOT / "web/replays.js").read_text(encoding="utf-8").removeprefix("window.REPLAYS=").removesuffix(";"))
OUT = ROOT / "output"
FONT = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 17)
SMALL = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 12)


def gif(replay):
    frames = []
    rows = replay["trace"]
    all_points = np.array([p for polygon in replay["road"] for p in polygon])
    for idx in np.linspace(0, len(rows) - 1, 180).astype(int):
        row = rows[idx]
        lo, hi = all_points.min(axis=0), all_points.max(axis=0)
        if replay["result"]["key"] == "city":
            lo, hi = np.array([row[1] - 45, row[2] - 35]), np.array([row[1] + 85, row[2] + 35])
        scale = min(740 / (hi[0] - lo[0]), 320 / (hi[1] - lo[1]))
        center = (lo + hi) / 2
        transform = lambda p: (400 + (p[0] - center[0]) * scale, 215 - (p[1] - center[1]) * scale)
        im = Image.new("RGB", (800, 440), "#20282b")
        draw = ImageDraw.Draw(im)
        draw.text((20, 12), replay["result"]["title"], font=FONT, fill="#e1ecec")
        draw.text((570, 16), "离线 / 自建场景 / 非官方成绩", font=SMALL, fill="#9cacb0")
        for polygon in replay["road"]:
            draw.polygon([transform(p) for p in polygon], fill="#3c474b", outline="#7b8b90")
        if replay["slot"]:
            draw.polygon([transform(p) for p in replay["slot"]], outline="#e0c578", width=2)
        for route in replay["routes"]:
            draw.line([transform(p) for p in route], fill="#b1a06a", width=1)
        draw.line([transform([r[1], r[2]]) for r in rows[:idx + 1]] or [(0, 0)], fill="#25c9b9", width=2)

        def body(x, y, yaw, length, width, color, rear=None):
            f, b = (length / 2, length / 2) if rear is None else (length - rear, rear)
            pts = [[f, width / 2], [f, -width / 2], [-b, -width / 2], [-b, width / 2]]
            draw.polygon([transform([x + a * math.cos(yaw) - c * math.sin(yaw),
                                    y + a * math.sin(yaw) + c * math.cos(yaw)]) for a, c in pts], fill=color)

        for o in replay["obstacles"]:
            body(o[0] + row[0] * o[5] * math.cos(o[2]), o[1] + row[0] * o[5] * math.sin(o[2]),
                 o[2], o[3], o[4], "#d0d7d9")
        for n, light in enumerate(replay["lights"]):
            x, y = transform(light["point"])
            color = "#52d59e" if row[12][n] == "green" else "#f47d72"
            draw.ellipse((x - 4, y - 15, x + 4, y - 7), fill=color)
        body(row[1], row[2], row[3], 4.6, 1.8, "#36d6c7", rear=1)
        draw.rectangle((0, 393, 800, 440), fill="#172024")
        draw.text((20, 405), f"{row[0]:06.1f} s    {abs(row[4])*3.6:04.1f} km/h    {'R' if row[4]<-.01 else 'D'}", font=FONT, fill="#d9e6e5")
        draw.text((485, 409), "轨迹来自闭环计算 · 已加速回放", font=SMALL, fill="#8da4a8")
        frames.append(im.convert("P", palette=Image.Palette.ADAPTIVE, colors=96))
    path = OUT / (replay["result"]["key"] + ".gif")
    frames[0].save(path, save_all=True, append_images=frames[1:], loop=0, duration=70, optimize=False)


def docs():
    rows = []
    for d in DATA:
        r = d["result"]
        rows.append(f"<tr><td>{html.escape(r['title'])}</td><td>{'通过' if r['passed'] else '未通过'}</td><td>{r['time_s']:.2f}</td><td>{r['local_score_estimate'] if r['local_score_estimate'] is not None else '附加测试'}</td></tr>")
    text = (ROOT / "IMPLEMENTATION.md").read_text(encoding="utf-8")
    md = MarkdownIt("default", {"html": False}).enable("table")
    md.renderer.rules["table_open"] = lambda tokens, idx, options, env: '<div class="table-scroll"><table>\n'
    md.renderer.rules["table_close"] = lambda tokens, idx, options, env: '</table></div>\n'
    tokens = md.parse(text)
    headings = []
    for i, token in enumerate(tokens):
        if token.type == "heading_open" and token.tag in ("h2", "h3"):
            anchor = "section-" + str(len(headings) + 1)
            token.attrSet("id", anchor)
            headings.append((anchor, tokens[i + 1].content, token.tag))
    toc = '<nav class="toc" aria-label="说明目录"><strong>说明目录</strong><ol>'
    toc += ''.join('<li class="' + tag + '"><a href="#' + anchor + '">' + html.escape(label) + '</a></li>' for anchor, label, tag in headings if tag == "h2")
    toc += '<li><a href="#run-results">从当前文件读取的运行汇总</a></li></ol></nav>'
    intro_end = next(i for i, token in enumerate(tokens) if token.type == "heading_open" and token.tag == "h2")
    body = md.renderer.render(tokens[:intro_end], md.options, {}) + toc + md.renderer.render(tokens[intro_end:], md.options, {})
    style = "body{font:15px/1.9 'Microsoft YaHei',sans-serif;color:#263338;background:#f4f6f6;margin:0}main{max-width:1040px;margin:auto;padding:35px 24px 70px}h1{font-size:27px;line-height:1.5}h2{font-size:20px;border-top:1px solid #d7e0e0;padding-top:25px;margin-top:36px}h3{font-size:17px;margin-top:27px}h2,h3{scroll-margin-top:20px}p,li{overflow-wrap:anywhere}a{color:#087d70}code,pre{font-family:Consolas,'Microsoft YaHei',monospace;background:#e6ecec;border-radius:3px}code{font-size:13px;padding:2px 4px;overflow-wrap:anywhere}pre{padding:18px;overflow:auto;font-size:12px;line-height:1.8}pre code{font-size:inherit;padding:0;white-space:pre;overflow-wrap:normal}table{width:100%;border-collapse:collapse;font-size:13px}td,th{text-align:left;padding:10px;border-bottom:1px solid #d7dfdf;vertical-align:top;min-width:100px;overflow-wrap:anywhere}th{background:#e7eeed}.table-scroll{max-width:100%;overflow:auto;margin:18px 0}img{max-width:100%;height:auto}small{color:#697b80}.actions{display:flex;gap:20px;flex-wrap:wrap}.toc{padding:15px 0;border-bottom:1px solid #d7dfdf}.toc ol{columns:2;padding-left:24px}.toc li{break-inside:avoid;font-size:13px;padding:3px 10px 3px 0}@media(max-width:650px){main{padding:20px 14px 40px}h1{font-size:23px}h2{font-size:18px}.toc ol{columns:1}.actions{gap:12px;font-size:13px}td,th{padding:8px}pre{padding:12px}}"
    content = '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>ICV 代码结构与控制方法说明</title><style>' + style + '</style><main><div class="actions"><a href="web/index.html">打开交互回放</a><a href="output/parking.gif">泊车动画</a><a href="output/report.json">原始检查报告</a><a href="IMPLEMENTATION.md">Markdown 原稿</a></div>'
    content += body
    content += '<h2 id="run-results">从当前文件读取的运行汇总</h2><small>全部为自建场景本地结果，官方平台未运行。城市估分未计未建模的官方实线扣分。</small><div class="table-scroll"><table id="run-results-table"><thead><tr><th>场景</th><th>检查</th><th>用时 / 秒</th><th>本地估分</th></tr></thead><tbody>' + ''.join(rows) + '</tbody></table></div>'
    by_key = {d["result"]["key"]: d["result"] for d in DATA}
    if "traffic_red" in by_key:
        content += '<p>红灯车头停车间距：' + html.escape(str(by_key["traffic_red"]["red_stop_gaps_m"])) + ' m。</p>'
    if "parking" in by_key:
        content += '<p>泊车停稳：' + str(by_key["parking"]["parked_seconds"]) + ' s。</p>'
    content += '<img src="output/parking.gif" alt="实际计算的泊车轨迹动画"></main></html>'
    (ROOT / "README.html").write_text(content, encoding="utf-8")
    summary = ["# 离线运行结果", "", "以下不是 NEVC 官方成绩。", "", "| 场景 | 本地检查 | 时间 s | 本地估分 |", "|---|---|---:|---:|"]
    for d in DATA:
        r = d["result"]
        summary.append(f"| {r['title']} | {'通过' if r['passed'] else '未通过'} | {r['time_s']} | {r['local_score_estimate']} |")
    summary += ["", "城市估分假设未建模的官方实线扣分为零。轨迹和指标以各场景 CSV/JSON 为准。"]
    (OUT / "RESULTS.md").write_text("\n".join(summary), encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--docs-only", action="store_true", help="Regenerate HTML and summary without changing GIFs")
    args = parser.parse_args()
    if not args.docs_only:
        for replay in DATA:
            if replay["result"]["key"] in ("traffic_red", "obstacle_start", "parking", "city"):
                gif(replay)
    docs()
    print("Built handoff HTML and summary" if args.docs_only else "Built GIF animations, handoff HTML and summary")
