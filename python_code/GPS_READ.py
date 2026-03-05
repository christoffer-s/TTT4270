import gps
import time

def hent_lon_alt(gps_pakke):
    """
    Tar inn en datapakke fra gpsd og returnerer longitude og altitude.
    """
    # gpsd sender mange ulike pakker. Vi ser etter 'TPV' (Time-Position-Velocity)
    if getattr(gps_pakke, 'class', '') == 'TPV':
        # Hent longitude
        lon = getattr(gps_pakke, 'lon', None)
        
        # Hent altitude. Nyere versjoner av gpsd bruker 'altMSL' (Mean Sea Level) 
        # eller 'altHAE' i stedet for bare 'alt'. Vi sjekker altMSL først.
        alt = getattr(gps_pakke, 'altMSL', getattr(gps_pakke, 'alt', None))
        
        return lon, alt
        
    return None, None

# --- Her starter selve programmet som lytter på GPS-en ---
def start_gps_lytting():
    # Koble til gpsd-tjenesten som kjører i bakgrunnen på Pi-en
    session = gps.gps(mode=gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
    
    print("Lytter etter GPS-data... (Trykk Ctrl+C for å avslutte)")
    
    try:
        while True:
            # Hent neste datapakke fra gpsd
            pakke = session.next()
            
            # Send pakken inn i funksjonen vår
            longitude, altitude = hent_lon_alt(pakke)
            
            # Skriv ut resultatet bare hvis vi faktisk fikk tall (altså at vi har "fix")
            if longitude is not None:
                print(f"Longitude: {longitude} | Altitude: {altitude} meter")
            
            time.sleep(1) # Vent litt før vi leser neste for å ikke spamme terminalen
            
    except KeyboardInterrupt:
        print("\nAvslutter GPS-lytting.")

# Kjør programmet
if __name__ == '__main__':
    start_gps_lytting()