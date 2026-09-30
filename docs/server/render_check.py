#!/usr/bin/env python3
"""Render every docs page in a headless browser and report Mermaid diagrams that fail to render.

    python3 -m venv /tmp/pw && /tmp/pw/bin/pip install playwright && /tmp/pw/bin/playwright install chromium-headless-shell
    /tmp/pw/bin/python docs/server/render_check.py [--base http://localhost:8080] [--screenshots DIR]

Checks light and dark theme; exits 1 if any diagram shows an error, stays unrendered, or the page throws.
"""
import argparse
import json
import sys
import urllib.request

from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--base', default='http://localhost:8080')
    parser.add_argument('--screenshots', help='directory for a screenshot of the status page')
    args = parser.parse_args()
    pages = json.load(urllib.request.urlopen(args.base + '/api/pages'))
    problems = 0
    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={'width': 1400, 'height': 900})
        errors = []
        page.on('pageerror', lambda e: errors.append(str(e)))
        for theme in ('light', 'dark'):
            for entry in pages:
                errors.clear()
                page.goto(f"{args.base}/#/{entry['name']}")
                page.evaluate(f"document.documentElement.dataset.theme = '{theme}'")
                page.wait_for_function("document.querySelector('main h1') !== null", timeout=15000)
                page.wait_for_timeout(1500)
                unrendered = page.evaluate("document.querySelectorAll('pre.mermaid:not([data-processed])').length")
                rendered = page.evaluate("document.querySelectorAll('.mermaid-wrap svg').length")
                diagram_errors = page.evaluate(
                    "[...document.querySelectorAll('.diagram-error')].map(e => e.textContent.slice(0, 300))")
                if theme == 'light':
                    print(f"{entry['name']:26s} diagrams {rendered:2d}, unrendered {unrendered}, "
                          f"errors {len(diagram_errors)}{'  page error: ' + errors[0] if errors else ''}")
                    for e in diagram_errors:
                        print('   !!', e)
                problems += len(diagram_errors) + unrendered + len(errors)
        if args.screenshots:
            page.goto(args.base + '/#/status')
            page.wait_for_timeout(2500)
            page.screenshot(path=f'{args.screenshots}/status.png')
        browser.close()
    print('TOTAL problems:', problems)
    sys.exit(1 if problems else 0)


if __name__ == '__main__':
    main()
