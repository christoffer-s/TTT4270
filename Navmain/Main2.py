import json
import math
import time
import networkx as nx
import sys
import os
import numpy as np


# Get the path of the parent directory (project_root)
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Now you can import
import gps_to_csv_call
from Fossen_euler import updateKalmanFilter
import acc
import motor_ctrl
import tof

# ==========================================
# 0. DEFINISJON AV ORIGO (LOKALT KOORDINATSYSTEM)
# ==========================================
# Vi setter origo (0,0) til å være startpunktet på Gløshaugen.
# X-aksen går Øst/Vest. Y-aksen går Nord/Sør.
ORIGO_LON = 10.402332799157428
ORIGO_LAT = 63.41809573255258

def lon_lat_til_xy(lon, lat):
    """Konverterer GPS-koordinater til X (meter mot øst) og Y (meter mot nord) fra Origo."""
    R = 6371000  # Jordens radius i meter
    
    lat_rad = math.radians(lat)
    origo_lat_rad = math.radians(ORIGO_LAT)
    delta_lon = math.radians(lon - ORIGO_LON)
    delta_lat = math.radians(lat - ORIGO_LAT)
    
    # Flat-jord tilnærming for små avstander
    x = R * delta_lon * math.cos(origo_lat_rad)
    y = R * delta_lat
    
    # Runder av til 3 desimaler (millimeter-presisjon) for å unngå flyttalls-rot i grafen
    return round(x, 3), round(y, 3)

# ==========================================
# 1. MATEMATIKK OG KART-HÅNDTERING (NÅ I X/Y METER!)
# ==========================================

def beregn_avstand(x1, y1, x2, y2):
    """Beregner avstand i meter (vanlig Pytagoras i et flatt rutenett)."""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def beregn_vinkel_til_maal(x1, y1, x2, y2):
    """Beregner vinkel til mål i grader (0=Nord, 90=Øst, 180=Sør, 270=Vest)."""
    dx = x2 - x1
    dy = y2 - y1
    
    # math.atan2(dx, dy) gir vinkel med 0 grader rett opp (Y-aksen/Nord).
    vinkel = math.degrees(math.atan2(dx, dy))
    return (vinkel + 360) % 360

def bygg_graf(geojson_data):
    """Konverterer GeoJSON til en navigerbar graf basert på X og Y meter."""
    G = nx.Graph()
    for feature in geojson_data['features']:
        if feature['geometry']['type'] == 'LineString':
            coords = feature['geometry']['coordinates']
            for i in range(len(coords) - 1):
                # Hent GPS, konverter til (X, Y)
                lon1, lat1 = coords[i]
                lon2, lat2 = coords[i+1]
                
                p1 = lon_lat_til_xy(lon1, lat1)
                p2 = lon_lat_til_xy(lon2, lat2)
                
                avstand = beregn_avstand(p1[0], p1[1], p2[0], p2[1])
                
                # Vi kan også forhåndsregne ideell retning for linjen og lagre det!
                ideell_vinkel = beregn_vinkel_til_maal(p1[0], p1[1], p2[0], p2[1])
                
                G.add_edge(p1, p2, weight=avstand, ideal_heading=ideell_vinkel)
    return G

def finn_naermeste_node(G, posisjon_xy):
    """Finner det X/Y-punktet på grafen som er nærmest bilen."""
    naermeste_node = None
    min_avstand = float('inf')
    for node in G.nodes():
        avstand = beregn_avstand(posisjon_xy[0], posisjon_xy[1], node[0], node[1])
        if avstand < min_avstand:
            min_avstand = avstand
            naermeste_node = node
    return naermeste_node

def finn_korteste_vei(G, start_xy, slutt_xy):
    """Finner korteste vei fra start til slutt ved hjelp av Dijkstras algoritme."""
    start_node = finn_naermeste_node(G, start_xy)
    slutt_node = finn_naermeste_node(G, slutt_xy)
    
    try:
        vei = nx.shortest_path(G, source=start_node, target=slutt_node, weight='weight')
        return vei
    except nx.NetworkXNoPath:   
        print("Feil: Fant ingen vei mellom disse punktene.")
        return None

# ==========================================
# 2. HARDWARE / SENSORER
# ==========================================

def les_sensorer_og_kalman():
    """Henter bilens estimerte posisjon i meter (X,Y) og retning."""
    global x_ins 
    global P_prd
    #AKSELEROMETER DATA:
    f_imu, w_imu = acc.IMU()

    # Henter rå-GPS fra modulen din
    pos = gps_to_csv_call.get_gps()
    if pos[0] == 0:
        # print("NO GPS")
        x_ins, P_prd = updateKalmanFilter(x_ins, P_prd, h, Qd, Rd, f_imu, w_imu, gps_read=False)
    else:
        raw_lon = pos[1]
        raw_lat = pos[0]
        gps_x, gps_y = lon_lat_til_xy(raw_lon, raw_lat)
        y_pos = np.array([gps_x, gps_y, 0]).T
        # print(f"ypos: {y_pos}")
        x_ins, P_prd = updateKalmanFilter(x_ins, P_prd, h, Qd, Rd, f_imu, w_imu, gps_read=True, y_pos=y_pos)

    # x_ins, P_prd = Fossen_euler.updateKalmanFilter(x_ins, P_prd, h, Qd, Rd, f_imu, w_imu, y_pos)
    # estimert_retning = 90.0 # Bilen peker mot Øst
    # print(f"x_ins returned from les sensorer: {x_ins[0][0]} & {x_ins[0][1]}")
    return (x_ins[0][0], x_ins[0][1]), x_ins[3][2]

def les_tof_sensor():
    """Leser TOF-sensor og returnerer avstand til hindring i meter."""
    distance = tof.read_tof()
    print(distance)
    if distance > 0:
        return 10.0 # 10 meter = fri vei
    return 10 




def styr_motorer(fart, sving_vinkel):
    if sving_vinkel > 10:
        turn_rate = 0.20
    elif sving_vinkel < -10:
        turn_rate = -0.20
    else:
        turn_rate = 0
    # turn_rate = 0 # For testing uten at den svinger
    motor_ctrl.drive.drive(forward_speed=fart,turn_rate=turn_rate)

    # """Sender fart og styrevinkel til motorkontrolleren."""
    # print(f"[MOTOR] Fart: {fart} | Styrevinkel: {sving_vinkel:.1f} grader")
    #TRENGER MER TESTING MED OVERSETTE MELLOM MOTOR_CTRL SKRIPT OG MAIN
    #MOTOR_CTRL GIR -1 TIL 1 FOR BEGGE DELER, MAIN GIR GRADER FEIL FRA ØNSKET COURSE







def brems_bilen():
    motor_ctrl.drive.stop()
    """Stopper motorene fullstendig."""
    print("[MOTOR] 🛑 Bremsene aktivert. Bilen har stoppet.")

# ==========================================
# 3. DYNAMISK RUTEPLANLEGGING (Hindringer)
# ==========================================

def beregn_ny_rute(G, naa_pos_xy, neste_waypoint_xy, slutt_maal_xy):
    """Klipper den blokkerte veien ut av grafen og finner ny rute."""
    naermeste_node_naa = finn_naermeste_node(G, naa_pos_xy)
    neste_node = tuple(neste_waypoint_xy) 
    
    # 1. Fjern strekningen fra grafen
    try:
        if G.has_edge(naermeste_node_naa, neste_node):
            G.remove_edge(naermeste_node_naa, neste_node)
            print(f"[RUTING] ✂️ Slettet blokkert vei mellom {naermeste_node_naa} og {neste_node}")
    except Exception as e:
        print(f"[RUTING] Kunne ikke fjerne vei fra grafen: {e}")
        
    # 2. Finn ny rute
    slutt_node = finn_naermeste_node(G, slutt_maal_xy)
    try:
        ny_vei = nx.shortest_path(G, source=naermeste_node_naa, target=slutt_node, weight='weight')
        return ny_vei
    except nx.NetworkXNoPath:
        print("[RUTING] 🚨 KRITISK: Ingen andre veier til målet! Bilen er innestengt.")
        return None

# ==========================================
# 4. HOVEDKONTROLLØKKEN (Path Tracking)
# ==========================================

def kjor_bil_til_maal(G, waypoints_xy, slutt_maal_xy):
    """Kjører bilen langs ruten, sjekker hindringer og styrer mot målet."""
    naavaerende_waypoint_indeks = 1 
    global prev_tof_check
    global noTofRead
    print("\n--- STARTER SELVKJØRING ---")
    while naavaerende_waypoint_indeks < len(waypoints_xy):
        # 1. Hent posisjon (NÅ I X,Y METER) og retning
        estimert_pos_xy, estimert_retning = les_sensorer_og_kalman()
        # print("NY SENSOR LESNING!!!!!!!!!!!!!!!!")
        next_time = time.time()
        
        # 2. Sjekk for hindringer
        hindring_avstand = les_tof_sensor()
        
        
        if (hindring_avstand < 0.5 and prev_tof_check[0] - estimert_pos_xy[0] > 1 and prev_tof_check[1] - estimert_pos_xy[1] > 1) or (hindring_avstand < 0.5 and noTofRead): # 50 cm grense
            print("\n🚨 HINDRING OPPDAGET! Stopper bilen.")
            brems_bilen()
            time.sleep(1) 
            
            noTofRead = False
            prev_tof_check = estimert_pos_xy

            
            print("Planlegger ny rute rundt hindringen...")
            neste_punkt_xy = waypoints_xy[naavaerende_waypoint_indeks]
            nye_waypoints = beregn_ny_rute(G, estimert_pos_xy, neste_punkt_xy, slutt_maal_xy)
            
            if nye_waypoints:
                print(f"✅ Fant ny rute med {len(nye_waypoints)} punkter!")
                waypoints_xy = nye_waypoints 
                naavaerende_waypoint_indeks = 1 
                continue 
            else:
                print("❌ Bilen kan ikke fortsette. Ruten er totalt blokkert.")
                break 

        # 3. Sjekk progresjon mot neste (X, Y)-punkt
        maal_pos_xy = waypoints_xy[naavaerende_waypoint_indeks]
        avstand_til_maal = beregn_avstand(estimert_pos_xy[0], estimert_pos_xy[1], maal_pos_xy[0], maal_pos_xy[1])
        
        if avstand_til_maal < 2.0:
            print(f"📍 Nådd waypoint {naavaerende_waypoint_indeks}! Bytter til neste.")
            naavaerende_waypoint_indeks += 1
            
            if naavaerende_waypoint_indeks >= len(waypoints_xy):
                print("🏁 Mål nådd! Bilen parkerer.")
                brems_bilen()
                break
            
            maal_pos_xy = waypoints_xy[naavaerende_waypoint_indeks]
            
        # 4. Regn ut styrevinkel mot (X, Y)
        maal_vinkel = beregn_vinkel_til_maal(estimert_pos_xy[0], estimert_pos_xy[1], maal_pos_xy[0], maal_pos_xy[1])
        vinkel_feil = maal_vinkel - estimert_retning
        
        # Normaliser for å finne korteste vei å svinge
        if vinkel_feil > 180:
            vinkel_feil -= 360
        elif vinkel_feil < -180:
            vinkel_feil += 360
            
        # 5. Send til motor
        print(f"Vinkel feil til motor: {vinkel_feil} og mål posisjon: {maal_pos_xy[0]},{maal_pos_xy[1]}")
        print(f"Nåværenede posisjon og retning: {x_ins[0][0]}, {x_ins[0][1]} og {x_ins[3][2]}")
        styr_motorer(0.5, vinkel_feil)
        
        # # 6. Kontroller hastigheten på løkken (1000 Hz)
        next_time += 1/1000
        sleep_time = next_time - time.time()
        # print(f"Sleep time: {sleep_time}")
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            pass


# ==========================================
# 5. START AV PROGRAMMET
# ==========================================

# Initialize ins
p_ins = np.array([0, 0, 0]).T
v_ins = np.array([0, 0, 0]).T
b_acc_ins = np.array([0.01, 0.01, 0.01]).T # Added small starting bias
theta_ins = np.array([0, 0, 0]).T
b_ars_ins = np.array([0.01, 0.01, 0.01]).T # Added small starting bias
x_ins = [p_ins, v_ins, b_acc_ins, theta_ins, b_ars_ins]

Rd = np.diag([1, 1, 1,  1, 1, 1, 1]) #pos, euler_angles
Qd = np.diag([1, 1, 1,  1, 1, 1,  10, 10, 10,  10, 10, 10])

P_prd = np.eye(15) # EYE NOT ZEROS 

f_fast = 1000
f_slow = 10

h = 1/f_fast
h_slow = 1/f_slow



if __name__ == "__main__":
    print("Initialiserer systemet (Kartesisk XY)...")
    
    # 1. Last inn kart
    try:
        with open('LinjemapGløsV2.geojson', 'r') as fil:
            kart_data = json.load(fil)
    except FileNotFoundError:
        print("Kritisk feil: Fant ikke 'LinjemapGløsV2.geojson'.")
        exit()
        
    # 2. Definer ruten med rå GPS, og konverter til Lokalt X/Y
    # start_lon_lat = (10.402332799157428, 63.41809573255258) 
    maal_lon_lat = (10.405400716816052, 63.41672421102855)
    prev_tof_check = (0,0)
    noTofRead = True
    # min_start_xy = lon_lat_til_xy(start_lon_lat[0], start_lon_lat[1]) 
    min_start_xy = (x_ins[0][0], x_ins[0][1])
    mitt_maal_xy = lon_lat_til_xy(maal_lon_lat[0], maal_lon_lat[1])
    
    # 3. Bygg det matematiske kartet (Nodes er nå X, Y meter fra Origo!)
    G_kart = bygg_graf(kart_data)
    
    # 4. Finn den første ruten
    waypoints_xy = finn_korteste_vei(G_kart, min_start_xy, mitt_maal_xy)
    
    if waypoints_xy:
        print(f"✅ Rute planlagt vellykket! Ruten består av {len(waypoints_xy)} punkter.")
        # Print gjerne ut waypoints for å sjekke at de er i X/Y meter:
        # print("Waypoints (X, Y):", waypoints_xy)
        
        # 5. Start bilen
        kjor_bil_til_maal(G_kart, waypoints_xy, mitt_maal_xy)
    else:
        print("Klarte ikke å planlegge ruten. Avslutter.")