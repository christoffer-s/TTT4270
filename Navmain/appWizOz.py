from flask import Flask, render_template, jsonify
import json
from threading import Lock
from pathlib import Path

app = Flask(__name__)

# Sett True for demo-modus på nettsiden
USE_MOCK_GPS = True
MOCK_FID = 1
MOCK_STEP = 0.03  # Hvor fort markøren går (0.01-0.08 er typisk fint)

_mock_lock = Lock()
_mock_line = []
_mock_seg = 0
_mock_t = 0.0
_mock_dir = 1  # 1 fremover, -1 bakover (ping-pong i endene)

def _load_mock_line():
    global _mock_line
    geo_path = Path(app.static_folder) / "LinjemapGløsV2.geojson"
    with geo_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    chosen = None
    for ft in data.get("features", []):
        if ft.get("properties", {}).get("fid") == MOCK_FID:
            chosen = ft
            break

    if not chosen:
        for ft in data.get("features", []):
            if ft.get("geometry", {}).get("type") == "LineString":
                chosen = ft
                break

    if not chosen:
        return []

    coords = chosen.get("geometry", {}).get("coordinates", [])
    # GeoJSON er [lon, lat]
    return [(float(c[1]), float(c[0])) for c in coords if len(c) >= 2]

def _get_mock_gps():
    global _mock_seg, _mock_t, _mock_dir

    with _mock_lock:
        if len(_mock_line) < 2:
            return (63.41, 10.40)

        # Klipp indekser i gyldig område
        if _mock_seg < 0:
            _mock_seg = 0
        if _mock_seg > len(_mock_line) - 2:
            _mock_seg = len(_mock_line) - 2

        lat1, lon1 = _mock_line[_mock_seg]
        lat2, lon2 = _mock_line[_mock_seg + 1]

        lat = lat1 + (lat2 - lat1) * _mock_t
        lon = lon1 + (lon2 - lon1) * _mock_t

        _mock_t += MOCK_STEP
        while _mock_t >= 1.0:
            _mock_t -= 1.0
            _mock_seg += _mock_dir

            # Ping-pong i endene
            if _mock_seg >= len(_mock_line) - 1:
                _mock_seg = len(_mock_line) - 2
                _mock_dir = -1
            elif _mock_seg < 0:
                _mock_seg = 0
                _mock_dir = 1

        return (lat, lon)

def _get_gps_safe():
    if USE_MOCK_GPS:
        return _get_mock_gps()

    try:
        from gps_to_csv_call import get_gps  # lazy import
        return get_gps()
    except Exception:
        return (63.41, 10.40)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/gps")
def gps():
    lat, lon = _get_gps_safe()
    return jsonify({"lat": lat, "lon": lon})

if __name__ == "__main__":
    _mock_line = _load_mock_line()
    app.run(host="0.0.0.0", port=5000)