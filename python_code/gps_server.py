from flask import Flask, jsonify, render_template
import gps
import threading
import time

app = Flask(__name__)

# Vi setter startposisjonen til Torvet i Trondheim, siden 
# HTML-en din starter der. Denne oppdateres så fort vi får GPS-fix.
nyeste_posisjon = {"lat": 63.4305, "lon": 10.3951}

def lytt_til_gps():
    """Kjører i bakgrunnen og oppdaterer posisjonen når vi har signal."""
    try:
        # Kobler til gpsd
        session = gps.gps(mode=gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
        
        while True:
            pakke = session.next()
            if getattr(pakke, 'class', '') == 'TPV':
                lat = getattr(pakke, 'lat', None)
                lon = getattr(pakke, 'lon', None)
                
                # Bare oppdater hvis vi faktisk får gyldige tall fra satellittene
                if lat is not None and lon is not None:
                    nyeste_posisjon['lat'] = lat
                    nyeste_posisjon['lon'] = lon
                    
    except Exception as e:
        print(f"Feil med GPS-lesing: {e}")

# --- Webserver-ruter ---

@app.route('/')
def vis_kart():
    """Sender index.html til nettleseren når du går til hovedsiden."""
    return render_template('index.html')

@app.route('/gps')
def send_gps_data():
    """Sender de nyeste koordinatene som JSON når Leaflet spør etter dem."""
    return jsonify(nyeste_posisjon)

# --- Start programmet ---
if __name__ == '__main__':
    # Start GPS-lyttingen i en egen tråd
    gps_trad = threading.Thread(target=lytt_til_gps, daemon=True)
    gps_trad.start()
    
    print("Starter kartserver på port 5000...")
    print("Åpne nettleseren og gå til: http://<IP-til-Raspberry-Pi>:5000")
    
    # Kjør webserveren åpen for alle på lokalnettet
    app.run(host='0.0.0.0', port=5000)