import folium
import webbrowser
import os
import pandas as pd

# Create a map centered at a specific latitude and longitude
map_center = [6.267059, -75.567722]  # San Francisco
my_map = folium.Map(location=map_center, zoom_start=20)
datos = pd.read_csv('datos.csv')
lat = datos['Latitude'].tolist()
lon = datos['Longitude'].tolist()

def convert_coordinates(latitude, longitude):
    latitude = float(lat[i])/100
    longitude = float(lon[i])/100
    # Convert latitude and longitude values to degrees nb
    latitude_degrees = int(latitude)
    latitude_minutes = int((latitude - latitude_degrees) * 100)
    seconds1 = ((latitude - latitude_degrees) * 100 - latitude_minutes) * 60
    longitude_degrees = int(longitude)
    longitude_minutes = int((longitude - longitude_degrees) * 100)
    seconds2 = ((longitude - longitude_degrees) * 100 - longitude_minutes) * 60
    a = latitude_degrees + latitude_minutes/60 + seconds1/3600
    b = longitude_degrees + longitude_minutes/60 + seconds2/3600
    print(a, b)
    return a, b

# Add a marker for each latitude and longitude
for i in range(len(lat)):
    # Extract latitude and longitude values
    latitude_decimal, longitude_decimal = convert_coordinates(lat[i],lon[i])
    print(latitude_decimal, longitude_decimal)
    folium.Marker(location=(latitude_decimal, longitude_decimal),icon=folium.Icon(color='red', icon='ok-circle', prefix='fa')).add_to(my_map)
    

# Save the map to an HTML file and open it in a web browser
my_map.save('recorrido.html')
webbrowser.open('file://' + os.path.realpath('my_map.html'))