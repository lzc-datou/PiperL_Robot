"""Generate an Original ArUco marker as a vector A4 print page and preview PNG."""
from pathlib import Path
import cv2
import numpy as np


def generate(directory: Path, marker_id=582):
    directory.mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    if not 0 <= marker_id < len(dictionary.bytesList):
        raise ValueError('Marker ID outside Original ArUco dictionary.')
    if hasattr(cv2.aruco, 'generateImageMarker'):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 700)
    else:
        marker = cv2.aruco.drawMarker(dictionary, marker_id, 700)
    preview = np.full((900, 900), 255, np.uint8)
    preview[100:800, 100:800] = marker
    cv2.imwrite(str(directory / f'original_{marker_id}_preview.png'), preview)
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="210mm" height="297mm" viewBox="0 0 2100 2970">',
             '<rect width="2100" height="2970" fill="white"/>',
             f'<text x="150" y="220" font-family="sans-serif" font-size="50">Original ArUco / ID {marker_id}</text>',
             '<text x="150" y="310" font-family="sans-serif" font-size="36">Print A4 at 100%; disable fit-to-page.</text>']
    # 5x5 bits + one black border cell on each edge = 7x7 cells.
    for row in range(7):
        for col in range(7):
            if marker[row * 100 + 50, col * 100 + 50] == 0:
                parts.append(f'<rect x="{700+100*col}" y="{700+100*row}" width="100" height="100" fill="black"/>')
    parts += ['<text x="150" y="1750" font-family="sans-serif" font-size="36">Nominal black square: 70 mm. Measure after printing!</text>',
              '<text x="150" y="1830" font-family="sans-serif" font-size="36">Set marker_size_m to the measured black outer edge.</text>',
              '<path d="M 700 2030 v 40 m 0 -20 h 500 m 0 -20 v 40" stroke="black" stroke-width="3" fill="none"/>',
              '<text x="850" y="2150" font-family="sans-serif" font-size="36">50 mm scale</text>', '</svg>']
    page = directory / f'original_{marker_id}_A4.svg'
    page.write_text('\n'.join(parts) + '\n')
    print('打印文件:', page)


if __name__ == '__main__':
    generate(Path(__file__).resolve().parent / 'assets')
