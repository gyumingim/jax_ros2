#!/usr/bin/env python3
"""휴대폰 카메라 탭의 인쇄물을 만든다 → dashboard/phone_markers/phone_markers.pdf (A4 가로 2쪽, 300 dpi) + 쪽마다 PNG.

    /usr/bin/python3 dashboard/tools/make_phone_markers.py

1쪽 = 기준 종이 (ArUco 5x5 ID 0~5, 32 mm): 책상에 붙인다.  2쪽 = 큐브 윗면 마커 (ArUco 4x4 ID 0, 40 mm) 6장.
탑 쌓기용 stack_markers.pdf (1쪽, 큐브마다 다른 마커 4x4 ID 0~4) 도 같이 만든다 (--stack-only 면 그것만).
크기·자리는 phone_detect.py 의 상수 그대로 (여기서 숫자를 따로 두지 않는다). 인쇄는 꼭 '실제 크기 100%'.
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import phone_detect as PD   # noqa: E402

A4 = (297.0, 210.0)
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"


def _marker(mid, mm, dpi, dict_name=PD.REF_DICT_NAME):
    import cv2
    from PIL import Image
    px = round(mm / 25.4 * dpi)
    return Image.fromarray(cv2.aruco.drawMarker(PD._dictionary(cv2, dict_name), mid, px))


def _font(mm, dpi):
    from PIL import ImageFont
    try:
        return ImageFont.truetype(FONT, round(mm / 25.4 * dpi))
    except OSError:
        return ImageFont.load_default()


def _page(dpi):
    from PIL import Image, ImageDraw
    im = Image.new("L", (round(A4[0] / 25.4 * dpi), round(A4[1] / 25.4 * dpi)), 255)
    return im, ImageDraw.Draw(im)


def sheet_image(dpi=300):
    """1쪽: 기준 종이. 종이 좌표 (mm, 가운데 원점, +y 위) → 그림 픽셀."""
    im, d = _page(dpi)
    k = dpi / 25.4
    P = lambda x, y: (round((A4[0] / 2 + x) * k), round((A4[1] / 2 - y) * k))   # noqa: E731
    for mid, (mx, my) in PD.REF_MM.items():
        m = _marker(mid, PD.REF_SIZE_MM, dpi)
        x, y = P(mx, my)
        im.paste(m, (x - m.width // 2, y - m.height // 2))
        d.text(P(mx + (-24 if mx > 0 else 24), my), str(mid), fill=150, font=_font(4, dpi), anchor="mm")
    lw = max(1, round(0.3 * k))
    d.line([P(-8, 0), P(8, 0)], fill=0, width=lw); d.line([P(0, -8), P(0, 8)], fill=0, width=lw)   # 가운데 십자
    d.ellipse([P(-1.2, 1.2), P(1.2, -1.2)], outline=0, width=lw)
    f = _font(4.2, dpi)
    c = PD.DEFAULT_CFG
    d.text(P(0, -13), f"가운데 십자 = 어깨 축 바로 아래에서 앞으로 {abs(c['y']) * 100:.0f} cm, 로봇 {'오른쪽' if c['x'] < 0 else '왼쪽'}으로 {abs(c['x']) * 100:.0f} cm",
           fill=110, font=f, anchor="mm")
    d.text(P(0, 92), "▲ 로봇 쪽 (+y)   ·   이 변을 로봇 좌우와 나란히", fill=0, font=_font(5.5, dpi), anchor="mm")
    d.text(P(0, 80), "JAX 휴대폰 카메라 — 기준 종이 (ArUco 5x5, ID 0~5, 32 mm). 책상에 평평하게 붙인다. 큐브는 종이 위·옆 어디든.",
           fill=110, font=f, anchor="mm")
    d.text(P(70, 60), "+x = 로봇 왼쪽 ▶", fill=110, font=f, anchor="mm")
    d.line([P(-50, -88), P(50, -88)], fill=0, width=lw)   # 인쇄 확인 자
    for x in (-50, 0, 50):
        d.line([P(x, -86), P(x, -90)], fill=0, width=lw)
    d.text(P(0, -95), "인쇄 확인: 위 선이 정확히 100 mm (실제 크기 100% 로 인쇄)", fill=110, font=f, anchor="mm")
    return im


def cube_image(dpi=300):
    """2쪽: 큐브 윗면 마커 6장 (50 mm 점선을 따라 오린다)."""
    im, d = _page(dpi)
    k = dpi / 25.4
    P = lambda x, y: (round((A4[0] / 2 + x) * k), round((A4[1] / 2 - y) * k))   # noqa: E731
    d.text(P(0, 90), f"JAX 휴대폰 카메라 — 큐브 윗면 마커 (ArUco 4x4, ID {PD.CUBE_ID}, {PD.CUBE_MARKER_MM:.0f} mm)", fill=0, font=_font(5.5, dpi), anchor="mm")
    d.text(P(0, 80), "바깥 네모(50 mm)를 따라 오려 큐브 윗면에 반듯하게 붙인다. 한 장만 쓰면 된다 (나머지는 여분).", fill=110, font=_font(4.2, dpi), anchor="mm")
    for j, cy in enumerate((30, -45)):
        for i, cx in enumerate((-80, 0, 80)):
            m = _marker(PD.CUBE_ID, PD.CUBE_MARKER_MM, dpi, PD.CUBE_DICT_NAME)
            x, y = P(cx, cy)
            im.paste(m, (x - m.width // 2, y - m.height // 2))
            d.rectangle([P(cx - 25, cy + 25), P(cx + 25, cy - 25)], outline=150, width=max(1, round(0.25 * k)))
    return im


def stack_image(dpi=300):
    """탑 쌓기 쪽: 큐브마다 다른 마커 (4x4 ID PD.CUBE_IDS = 0~4, 6칸 중 앞 5칸, 40 mm, 50 mm 선을 따라 오린다). 번호는 오리는 선 밖에 적는다."""
    im, d = _page(dpi)
    k = dpi / 25.4
    P = lambda x, y: (round((A4[0] / 2 + x) * k), round((A4[1] / 2 - y) * k))   # noqa: E731
    ids = list(PD.CUBE_IDS)
    d.text(P(0, 92), f"JAX 탑 쌓기 — 큐브 윗면 마커 (ArUco 4x4, ID {ids[0]}~{ids[-1]}, {PD.CUBE_MARKER_MM:.0f} mm)", fill=0, font=_font(5.5, dpi), anchor="mm")
    d.text(P(0, 83), "큐브마다 다른 번호 한 장씩. 모서리 표시(50 mm 네모)에 맞춰 오려 큐브 윗면에 반듯하게 붙인다. 기준 종이는 phone_markers.pdf 1쪽 그대로.",
           fill=110, font=_font(4.2, dpi), anchor="mm")
    lw = max(1, round(0.25 * k))
    for j, cy in enumerate((38, -32)):
        for i, cx in enumerate((-80, 0, 80)):
            if j * 3 + i >= len(ids):
                continue
            mid = ids[j * 3 + i]
            m = _marker(mid, PD.CUBE_MARKER_MM, dpi, PD.CUBE_DICT_NAME)
            x, y = P(cx, cy)
            im.paste(m, (x - m.width // 2, y - m.height // 2))
            for sx in (-1, 1):   # 50 mm 네모는 모서리 표시(ㄱ자 8 mm)만: 닫힌 네모 선이 오린 뒤에도 남으면 ArUco 가 그 네모를 후보로 잡아 안쪽 마커를 버린다
                for sy in (-1, 1):
                    d.line([P(cx + sx * 25, cy + sy * 25), P(cx + sx * 17, cy + sy * 25)], fill=150, width=lw)
                    d.line([P(cx + sx * 25, cy + sy * 25), P(cx + sx * 25, cy + sy * 17)], fill=150, width=lw)
            d.text(P(cx, cy - 31), f"ID {mid}", fill=110, font=_font(4.2, dpi), anchor="mm")
    d.line([P(-50, -84), P(50, -84)], fill=0, width=max(1, round(0.3 * k)))   # 인쇄 확인 자
    for x in (-50, 0, 50):
        d.line([P(x, -82), P(x, -86)], fill=0, width=max(1, round(0.3 * k)))
    d.text(P(0, -91), "인쇄 확인: 위 선이 정확히 100 mm (실제 크기 100% 로 인쇄)", fill=110, font=_font(4.2, dpi), anchor="mm")
    return im


def main():
    out = HERE.parent / "phone_markers"
    out.mkdir(exist_ok=True)
    c = stack_image(300)
    c.save(out / "stack_markers.pdf", "PDF", resolution=300.0)
    c.save(out / "sheet_stack_cubes.png", dpi=(300, 300), optimize=True)
    print("만듦:", out / "stack_markers.pdf")
    if "--stack-only" in sys.argv:
        return
    a, b = sheet_image(300), cube_image(300)
    a.save(out / "phone_markers.pdf", "PDF", resolution=300.0, save_all=True, append_images=[b])
    a.save(out / "sheet_p1_reference.png", dpi=(300, 300), optimize=True)
    b.save(out / "sheet_p2_cube.png", dpi=(300, 300), optimize=True)
    print("만듦:", out / "phone_markers.pdf")


if __name__ == "__main__":
    main()
