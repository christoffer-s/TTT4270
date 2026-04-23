from flask import Flask, render_template, jsonify

app = Flask(__name__)

def _get_gps_safe():
    try:
        from Navmain.mock_gps import get_gps  # lazy import
        return get_gps()
    except Exception:
        # Fallback så webserveren fortsatt kjører
        return (63.41, 10.40)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/gps")
def gps():
    lat, lon = _get_gps_safe()
    return jsonify({"lat": lat, "lon": lon})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)