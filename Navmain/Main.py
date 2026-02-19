import json
import math
import time
import networkx as nx
import sys
import os

# Get the path of the parent directory (project_root)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Now you can import
from python_code import gps_to_csv_call

# ==========================================
# 1. MATEMATIKK OG KART-HÅNDTERING
# ==========================================

def beregn_avstand(lon1, lat1, lon2, lat2):
    """Beregner avstand i meter mellom to GPS-koordinater (Haversine-formelen)."""
    R = 6371000  # Jordens radius i meter
    phi_1 = math.radians(lat1)
    phi_2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi / 2.0)**2 + math.cos(phi_1) * math.cos(phi_2) * math.sin(delta_lambda / 2.0)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def beregn_vinkel_til_maal(lon1, lat1, lon2, lat2):
    """Beregner kompassretning (bearing) fra punkt 1 til punkt 2 i grader (0-360)."""
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lon = math.radians(lon2 - lon1)

    x = math.sin(delta_lon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - (math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))
    
    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    kompass_vinkel = (initial_bearing + 360) % 360
    return kompass_vinkel

def bygg_graf(geojson_data):
    """Konverterer GeoJSON-linjer til en navigerbar graf."""
    G = nx.Graph()
    for feature in geojson_data['features']:
        if feature['geometry']['type'] == 'LineString':
            coords = feature['geometry']['coordinates']
            for i in range(len(coords) - 1):
                p1 = tuple(coords[i])   
                p2 = tuple(coords[i+1]) 
                avstand = beregn_avstand(p1[0], p1[1], p2[0], p2[1])
                G.add_edge(p1, p2, weight=avstand)
    return G

def finn_naermeste_node(G, posisjon):
    """Finner det punktet på grafen som er nærmest bilens GPS-posisjon."""
    naermeste_node = None
    min_avstand = float('inf')
    for node in G.nodes():
        avstand = beregn_avstand(posisjon[0], posisjon[1], node[0], node[1])
        if avstand < min_avstand:
            min_avstand = avstand
            naermeste_node = node
    return naermeste_node

def finn_korteste_vei(G, start_pos, slutt_pos):
    """Finner korteste vei fra start til slutt ved hjelp av Dijkstras algoritme."""
    start_node = finn_naermeste_node(G, start_pos)
    slutt_node = finn_naermeste_node(G, slutt_pos)
    
    try:
        vei = nx.shortest_path(G, source=start_node, target=slutt_node, weight='weight')
        return vei
    except nx.NetworkXNoPath:
        print("Feil: Fant ingen vei mellom disse punktene.")
        return None

# ==========================================
# 2. HARDWARE / SENSORER (Bytt ut med din egen kode)
# ==========================================

def les_sensorer_og_kalman():
    """Henter bilens estimerte posisjon og retning (fra f.eks. Kalman-filter)."""
    # Dummy-data for testing
    pos = gps_to_csv_call.get_gps()
    estimert_lon = pos[1]
    estimert_lat = pos[0]
    estimert_retning = 90.0 # Peker mot Øst
    return (estimert_lon, estimert_lat), estimert_retning

def les_tof_sensor():
    """Leser TOF-sensor og returnerer avstand til hindring i meter."""
    # Dummy-data: 10 meter betyr fri vei. Bytt ut med ekte avlesning.
    return 10.0 

def styr_motorer(fart, sving_vinkel):
    """Sender fart og styrevinkel til motorkontrolleren."""
    print(f"[MOTOR] Fart: {fart} | Styrevinkel: {sving_vinkel:.1f} grader")

def brems_bilen():
    """Stopper motorene fullstendig."""
    print("[MOTOR] 🛑 Bremsene aktivert. Bilen har stoppet.")

# ==========================================
# 3. DYNAMISK RUTEPLANLEGGING (Hindringer)
# ==========================================

def beregn_ny_rute(G, naa_pos, neste_waypoint, slutt_maal):
    """Klipper den blokkerte veien ut av grafen og finner ny rute."""
    naermeste_node_naa = finn_naermeste_node(G, naa_pos)
    neste_node = tuple(neste_waypoint) 
    
    # 1. Fjern den blokkerte strekningen fra kartet (G)
    try:
        if G.has_edge(naermeste_node_naa, neste_node):
            G.remove_edge(naermeste_node_naa, neste_node)
            print(f"[RUTING] ✂️ Slettet blokkert vei mellom {naermeste_node_naa} og {neste_node}")
    except Exception as e:
        print(f"[RUTING] Kunne ikke fjerne vei fra grafen: {e}")
        
    # 2. Finn ny rute
    slutt_node = finn_naermeste_node(G, slutt_maal)
    try:
        ny_vei = nx.shortest_path(G, source=naermeste_node_naa, target=slutt_node, weight='weight')
        return ny_vei
    except nx.NetworkXNoPath:
        print("[RUTING] 🚨 KRITISK: Ingen andre veier til målet! Bilen er innestengt.")
        return None

# ==========================================
# 4. HOVEDKONTROLLØKKEN (Path Tracking)
# ==========================================

def kjor_bil_til_maal(G, waypoints, slutt_maal):
    """Kjører bilen langs ruten, sjekker hindringer og styrer mot målet."""
    naavaerende_waypoint_indeks = 1 
    
    print("\n--- STARTER SELVKJØRING ---")
    
    while naavaerende_waypoint_indeks < len(waypoints):
        
        # 1. Hent posisjon og retning
        estimert_pos, estimert_retning = les_sensorer_og_kalman()
        
        # 2. Sjekk for hindringer
        hindring_avstand = les_tof_sensor()
        
        if hindring_avstand < 0.5: # 50 cm grense
            print("\n🚨 HINDRING OPPDAGET! Stopper bilen.")
            brems_bilen()
            time.sleep(1) 
            
            print("Planlegger ny rute rundt hindringen...")
            neste_punkt = waypoints[naavaerende_waypoint_indeks]
            nye_waypoints = beregn_ny_rute(G, estimert_pos, neste_punkt, slutt_maal)
            
            if nye_waypoints:
                print(f"✅ Fant ny rute med {len(nye_waypoints)} punkter!")
                waypoints = nye_waypoints 
                naavaerende_waypoint_indeks = 1 
                continue # Start på nytt med ny rute
            else:
                print("❌ Bilen kan ikke fortsette. Ruten er totalt blokkert.")
                break 

        # 3. Sjekk progresjon
        maal_pos = waypoints[naavaerende_waypoint_indeks]
        avstand_til_maal = beregn_avstand(estimert_pos[0], estimert_pos[1], maal_pos[0], maal_pos[1])
        
        if avstand_til_maal < 2.0: # Vi godtar at vi er "framme" hvis vi er under 2 meter unna punktet
            print(f"📍 Nådd waypoint {naavaerende_waypoint_indeks}! Bytter til neste.")
            naavaerende_waypoint_indeks += 1
            
            if naavaerende_waypoint_indeks >= len(waypoints):
                print("🏁 Mål nådd! Bilen parkerer.")
                brems_bilen()
                break
            
            maal_pos = waypoints[naavaerende_waypoint_indeks]
            
        # 4. Regn ut styrevinkel
        maal_vinkel = beregn_vinkel_til_maal(estimert_pos[0], estimert_pos[1], maal_pos[0], maal_pos[1])
        vinkel_feil = maal_vinkel - estimert_retning
        
        # Normaliser for å finne korteste vei å svinge
        if vinkel_feil > 180:
            vinkel_feil -= 360
        elif vinkel_feil < -180:
            vinkel_feil += 360
            
        # 5. Send til motor
        styr_motorer("Normal", vinkel_feil)
        
        # 6. Kontroller hastigheten på løkken (10 Hz)
        time.sleep(0.1) 

# ==========================================
# 5. START AV PROGRAMMET
# ==========================================

if __name__ == "__main__":
    print("Initialiserer systemet...")
    
    # 1. Last inn kart
    try:
        with open('LinjemapGløsV2.geojson', 'r') as fil:
            kart_data = json.load(fil)
    except FileNotFoundError:
        print("Kritisk feil: Fant ikke 'LinjemapGløsV2.geojson'. Ligger den i samme mappe som scriptet?")
        exit()
        
    # 2. Definer ruten (Eksempel-koordinater på Gløshaugen)
    min_start = (10.402332799157428, 63.41809573255258) 
    mitt_maal = (10.405400716816052, 63.41672421102855)
    
    # 3. Bygg det matematiske kartet (G)
    G_kart = bygg_graf(kart_data)
    
    # 4. Finn den første ruten
    waypoints = finn_korteste_vei(G_kart, min_start, mitt_maal)
    
    if waypoints:
        print(f"✅ Rute planlagt vellykket! Ruten består av {len(waypoints)} punkter.")
        # 5. Start bilen
        kjor_bil_til_maal(G_kart, waypoints, mitt_maal)
    else:
        print("Klarte ikke å planlegge ruten. Avslutter.")