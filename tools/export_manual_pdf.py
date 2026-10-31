#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/export_manual_pdf.py — Генерація автономного HTML та PDF з USER_MANUAL.md
Використання:
    python3 tools/export_manual_pdf.py
"""

import os
import sys
import subprocess

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
MD_PATH = os.path.join(ROOT_DIR, "USER_MANUAL.md")
HTML_PATH = os.path.join(ROOT_DIR, "USER_MANUAL.html")
PDF_PATH = os.path.join(ROOT_DIR, "USER_MANUAL.pdf")

CSS_STYLES = """
body {
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
    line-height: 1.5;
    color: #1f2328;
    max-width: 900px;
    margin: 0 auto;
    padding: 24px;
}
h1, h2, h3, h4, h5, h6 {
    color: #0f172a;
    margin-top: 1.2em;
    margin-bottom: 0.5em;
    border-bottom: 1px solid #d1d9e0;
    padding-bottom: 0.25em;
    break-after: avoid !important;
    page-break-after: avoid !important;
}
h1 {
    font-size: 22px;
    border-bottom: 2.5px solid #0969da;
    color: #0969da;
}
h2 {
    font-size: 17px;
    margin-top: 1.4em;
    border-bottom: 1.5px solid #d1d9e0;
}
h3 {
    font-size: 14.5px;
    border-bottom: none;
    color: #334155;
}
h4 {
    font-size: 13.5px;
    border-bottom: none;
    color: #1e293b;
    margin-top: 1em;
}
.table-card, .diagram-card {
    break-inside: avoid !important;
    page-break-inside: avoid !important;
    margin: 14px 0;
}
.table-card > h1, .table-card > h2, .table-card > h3, .table-card > h4, .table-card > h5, .table-card > h6,
.diagram-card > h1, .diagram-card > h2, .diagram-card > h3, .diagram-card > h4, .diagram-card > h5, .diagram-card > h6 {
    margin-top: 0;
    margin-bottom: 6px;
    break-after: avoid !important;
    page-break-after: avoid !important;
}
.table-card > p, .diagram-card > p {
    margin-top: 0;
    margin-bottom: 6px;
    font-weight: 500;
}
table {
    border-collapse: collapse;
    width: 100%;
    margin: 6px 0 0 0;
    font-size: 11.5px;
    break-inside: avoid !important;
    page-break-inside: avoid !important;
}
thead {
    display: table-header-group;
}
th, td {
    border: 1px solid #cbd5e1;
    padding: 6px 8px;
    text-align: left;
}
tr {
    break-inside: avoid !important;
    page-break-inside: avoid !important;
}
th {
    background-color: #f1f5f9;
    font-weight: 600;
    color: #0f172a;
}
tr:nth-child(even) {
    background-color: #f8fafc;
}
pre {
    background-color: #0f172a;
    color: #f8fafc;
    padding: 10px 12px;
    border-radius: 6px;
    overflow-x: auto;
    font-size: 10.5px;
    font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
    border: 1px solid #334155;
    break-inside: avoid !important;
    page-break-inside: avoid !important;
    margin: 8px 0;
}
code {
    font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
    background-color: #f1f5f9;
    color: #0969da;
    padding: 2px 4px;
    border-radius: 4px;
    font-size: 88%;
}
pre code {
    background: none;
    color: inherit;
    padding: 0;
}
blockquote {
    margin: 12px 0;
    padding: 10px 14px;
    color: #1e293b;
    background-color: #f0fdf4;
    border-left: 4px solid #22c55e;
    border-radius: 4px;
    break-inside: avoid !important;
    page-break-inside: avoid !important;
}
blockquote h3 {
    margin-top: 0;
    border: none;
    color: #dc2626;
}
ul, ol {
    padding-left: 22px;
}
li {
    margin-bottom: 4px;
    break-inside: avoid !important;
    page-break-inside: avoid !important;
}
hr {
    border: none;
    border-top: 1px solid #e2e8f0;
    margin: 18px 0;
}
@media print {
    body {
        max-width: 100%;
        padding: 0;
        font-size: 10pt;
    }
    @page {
        margin: 12mm 14mm;
        size: A4;
    }
    h1, h2, h3, h4, h5, h6 {
        break-after: avoid !important;
        page-break-after: avoid !important;
    }
    .table-card, .diagram-card, pre, blockquote, tr, li {
        break-inside: avoid !important;
        page-break-inside: avoid !important;
    }
}
"""

def main():
    if not os.path.exists(MD_PATH):
        print(f"Помилка: файл {MD_PATH} не знайдено.")
        sys.exit(1)

    try:
        import markdown
    except ImportError:
        print("Встановіть бібліотеку markdown: pip install markdown")
        sys.exit(1)

    try:
        from bs4 import BeautifulSoup
    except ImportError:
        print("Встановіть бібліотеку beautifulsoup4: pip install beautifulsoup4")
        sys.exit(1)

    with open(MD_PATH, "r", encoding="utf-8") as f:
        md_text = f.read()

    html_content = markdown.markdown(
        md_text,
        extensions=["tables", "fenced_code", "toc"]
    )

    soup = BeautifulSoup(html_content, "html.parser")

    # 1. Атомарне зв'язування кожного <table> з його заголовком у неподільний контейнер .table-card
    # Це гарантує, що заголовок і таблиця НІКОЛИ не розриваються на різні сторінки!
    for t in soup.find_all("table"):
        to_wrap = [t]
        prev = t.previous_sibling
        while prev and (prev.name is None and str(prev).strip() == ""):
            prev = prev.previous_sibling

        if prev and prev.name in ["h1", "h2", "h3", "h4", "h5", "h6"]:
            to_wrap.insert(0, prev)
        elif prev and prev.name == "p":
            to_wrap.insert(0, prev)
            # Якщо перед параграфом є заголовок h і параграф короткий (вступ до таблиці)
            prev2 = prev.previous_sibling
            while prev2 and (prev2.name is None and str(prev2).strip() == ""):
                prev2 = prev2.previous_sibling
            if prev2 and prev2.name in ["h1", "h2", "h3", "h4", "h5", "h6"] and len(prev.get_text()) < 200:
                to_wrap.insert(0, prev2)

        container = soup.new_tag("div", attrs={"class": "table-card"})
        to_wrap[0].insert_before(container)
        for el in to_wrap:
            container.append(el.extract())

    # 2. Атомарне зв'язування графічних схем/діаграм <pre> із ввідним заголовком
    for p in soup.find_all("pre"):
        if p.find_parent(class_=["table-card", "diagram-card"]) or p.find_parent("blockquote"):
            continue
        to_wrap = [p]
        prev = p.previous_sibling
        while prev and (prev.name is None and str(prev).strip() == ""):
            prev = prev.previous_sibling

        if prev and prev.name in ["h1", "h2", "h3", "h4", "h5", "h6"]:
            to_wrap.insert(0, prev)
        elif prev and prev.name == "p":
            prev2 = prev.previous_sibling
            while prev2 and (prev2.name is None and str(prev2).strip() == ""):
                prev2 = prev2.previous_sibling
            if prev2 and prev2.name in ["h1", "h2", "h3", "h4", "h5", "h6"] and len(prev.get_text()) < 150:
                to_wrap.insert(0, prev)
                to_wrap.insert(0, prev2)

        if len(to_wrap) > 1:
            container = soup.new_tag("div", attrs={"class": "diagram-card"})
            to_wrap[0].insert_before(container)
            for el in to_wrap:
                container.append(el.extract())

    # 3. Чистий перехід для фінальних розділів: Розділ 11 (PDF експорт) та Додаток 12 (стікер-пам'ятка)
    sec_pdf = soup.find(id="11-pdf") or soup.find(id="10-pdf")
    if sec_pdf:
        sec_pdf["style"] = "break-before: page; page-break-before: always;"
    sec12 = soup.find(id="12")
    if sec12:
        sec12["style"] = "break-before: page; page-break-before: always;"

    full_html = f"""<!DOCTYPE html>
<html lang="uk">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>COM-Box V1.3 — Посібник користувача та монтажника</title>
<style>
{CSS_STYLES}
</style>
</head>
<body>
{soup}
</body>
</html>
"""

    with open(HTML_PATH, "w", encoding="utf-8") as f:
        f.write(full_html)
    print(f"✓ HTML згенеровано: {HTML_PATH}")

    # Пошук браузера Chrome для експорту в PDF
    chrome_paths = [
        "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
        "/Applications/Chromium.app/Contents/MacOS/Chromium",
        "google-chrome",
        "chromium",
    ]
    chrome_bin = None
    for cp in chrome_paths:
        if os.path.exists(cp) or subprocess.run(["which", cp], capture_output=True).returncode == 0:
            chrome_bin = cp
            break

    if chrome_bin:
        cmd = [
            chrome_bin,
            "--headless",
            "--disable-gpu",
            f"--print-to-pdf={PDF_PATH}",
            "--no-pdf-header-footer",
            HTML_PATH,
        ]
        res = subprocess.run(cmd, capture_output=True)
        if res.returncode == 0 and os.path.exists(PDF_PATH):
            size_kb = os.path.getsize(PDF_PATH) / 1024
            print(f"✓ PDF успішно створено: {PDF_PATH} ({size_kb:.1f} КБ)")
        else:
            print("Не вдалося запустити автоматичний друк через Chrome. Відкрийте USER_MANUAL.html у браузері та натисніть Cmd+P -> Зберегти як PDF.")
    else:
        print("Браузер Chrome не знайдено. Відкрийте USER_MANUAL.html у браузері та натисніть Cmd+P -> Зберегти як PDF.")

if __name__ == "__main__":
    main()
