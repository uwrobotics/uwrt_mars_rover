from math import radians, sin, cos, sqrt, atan2
import sys

def haversine_distance(lat1, lon1, lat2, lon2):
    # Convert latitude and longitude from degrees to radians
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    # Radius of the Earth in meters (mean value)
    earth_radius = 6371000

    # Calculate the distance
    distance = earth_radius * c

    return distance

# Define the fixed point
fixed_point = (43.54245135, -80.51818219)

# Read input from stdin
for line in sys.stdin:
    timestamp, latitude, longitude = map(float, line.split())
    
    # Calculate distance using the haversine_distance function
    distance = haversine_distance(latitude, longitude, *fixed_point)
    
    print(f"{timestamp} {distance}")
